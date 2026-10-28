"""Kavrama yaklaşması durum makinesi düğümü (kavrama_dugumu.py).

ICP'siz, nesneye göreli kapalı çevrim son yaklaşmalı ve kurtarmalı durum makinesi.
Mimari kısıtlar: Lidar odometrisi / ICP kullanılmaz; odom görsel+tekerlek+IMU füzyonundan gelir.
Map->odom lokalizasyon katmanından gelir. Lidar yalnızca engel algısı ve güvenlik mesafesi içindir.
Kavrama doğruluğu, nesneye göreli kapalı çevrim son yaklaşmadan (SonYaklasma) elde edilir.

Düğüm Akışı:
  PLAN -> NAV_Q -> SON_YAKLASMA -> HAZIR  |  hata -> KURTARMA -> (sonraki aday) NAV_Q ... | HATA

Konular:
  Giriş:
    /kavrama/hedef            geometry_msgs/PointStamped  (frame map; z = nesne yüksekliği)
    /kavrama/nesne_algi       geometry_msgs/PointStamped  (herhangi bir frame, 2-10 Hz)
    /kavrama/iptal            std_msgs/Empty              (görevi durdurur ve iptal eder)
    odom                      nav_msgs/Odometry           (varsayılan 'odom')
    scan                      sensor_msgs/LaserScan       (varsayılan 'scan')
  Çıkış:
    /kavrama/durum            std_msgs/String             ('DURUM ayrıntı' biçiminde)
    /kavrama/nesne_robotta    geometry_msgs/PointStamped  (frame base_footprint) — kola bildirilen nokta
    /kavrama/plan             geometry_msgs/PoseArray     ([Q, P] görselleştirme)
    /cmd_vel                  geometry_msgs/Twist         (parametre cmd_konu)

Eylem Sunucusu:
  /kavra                      limo_interfaces/action/Kavra
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
import re
import threading
import time
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

from limo_autonomy.kavrama_yaklasim import Ayar, Harita, yaklasim_bul
from limo_autonomy.son_yaklasma import (
    SonYaklasma,
    SonYaklasmaAyar,
    hedef_pozu,
    nesne_robotta_son,
)

try:
    from limo_interfaces.action import Kavra

    LIMO_INTERFACES_VAR = True
except ImportError:
    Kavra = None
    LIMO_INTERFACES_VAR = False


def yaw_q(q) -> float:
    """Quaternion geometrisinden 2B yaw açısını (radyan) hesaplar."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a: float) -> float:
    """Açıyı (-pi, pi] aralığına sarar (atan2)."""
    return math.atan2(math.sin(a), math.cos(a))


def poz(x: float, y: float, th: float) -> Pose:
    """(x, y, th) 2B pozundan Pose mesajı üretir."""
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.orientation.z = math.sin(th / 2.0)
    p.orientation.w = math.cos(th / 2.0)
    return p


def guvenlik_mesafesi(
    scan_noktalari_robotta,
    nesne_robotta: tuple[float, float] | None = None,
    koni: float = math.radians(25),
    haric_r: float = 0.08,
    tampon: float = 0.161,
) -> float:
    """Robotun önündeki ±koni içinde, nesnenin haric_r çevresi hariç en yakın noktanın tampona mesafesi (m).

    Parametreler:
        scan_noktalari_robotta: base_footprint çerçevesindeki (x, y) tarama noktaları.
        nesne_robotta: Nesnenin base_footprint'teki tahmini (x, y) konumu (bu noktanın çevresi hariç tutulur).
        koni: Ön koni yarı açısı (radyan, varsayılan ±25°).
        haric_r: Nesne çevresinde yok sayılacak yarıçap (m, varsayılan 0.08 m).
        tampon: Ön tampon mesafesi (m, varsayılan LIMO için 0.161 m).

    Döndürür:
        Tampona olan en yakın engel mesafesi (m). Koni içinde engel yoksa float('inf').
    """
    min_d = float('inf')
    for pt in scan_noktalari_robotta:
        x, y = float(pt[0]), float(pt[1])
        # Yalnızca robotun önündeki noktalar
        if x <= 0:
            continue
        aci = math.atan2(y, x)
        if abs(aci) > koni:
            continue
        # Nesnenin çevresi hariç tutulur (kavranacak nesne güvenlik durdurmasını tetiklemesin)
        if nesne_robotta is not None:
            dx = x - nesne_robotta[0]
            dy = y - nesne_robotta[1]
            if math.hypot(dx, dy) <= haric_r:
                continue
        # En yakın noktanın ön tampona mesafesi
        d = math.hypot(x, y) - tampon
        if d < min_d:
            min_d = d
    return min_d


def quaternion_nokta_dondur(
    q: tuple[float, float, float, float],
    pt: tuple[float, float, float],
) -> tuple[float, float, float]:
    """3B noktayı quaternion (x, y, z, w) ile döndürür (Euler-Rodrigues formülü)."""
    qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    px, py, pz = float(pt[0]), float(pt[1]), float(pt[2])
    # r = (qx, qy, qz), v = (px, py, pz)
    # v' = v + 2 * r x (qw * v + r x v)
    rx = qy * pz - qz * py
    ry = qz * px - qx * pz
    rz = qx * py - qy * px

    cx = qw * px + rx
    cy = qw * py + ry
    cz = qw * pz + rz

    fx = qy * cz - qz * cy
    fy = qz * cx - qx * cz
    fz = qx * cy - qy * cx

    return (px + 2.0 * fx, py + 2.0 * fy, pz + 2.0 * fz)


def nokta_donustur_3d(
    trans: tuple[float, float, float],
    rot: tuple[float, float, float, float],
    pt: tuple[float, float, float],
) -> tuple[float, float, float]:
    """3B noktayı öteleme ve dönme ile dönüştürür."""
    rx, ry, rz = quaternion_nokta_dondur(rot, pt)
    return (trans[0] + rx, trans[1] + ry, trans[2] + rz)


def map_noktasi_odoma(
    pt_map: tuple[float, float],
    trans_map_to_odom: tuple[float, float],
    yaw_map_to_odom: float,
) -> tuple[float, float]:
    """2B harita noktasını map->odom TF dönüşümü ile odom çerçevesine çevirir."""
    mx, my = float(pt_map[0]), float(pt_map[1])
    tx, ty = float(trans_map_to_odom[0]), float(trans_map_to_odom[1])
    c, s = math.cos(yaw_map_to_odom), math.sin(yaw_map_to_odom)
    ox = tx + c * mx - s * my
    oy = ty + s * mx + c * my
    return (ox, oy)


def map_yonu_odoma(yaw_map: float, yaw_map_to_odom: float) -> float:
    """Harita çerçevesindeki yön açısını odom çerçevesine çevirir."""
    return wrap(yaw_map + yaw_map_to_odom)


def scan_to_base_footprint(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    tf_trans: tuple[float, float],
    tf_yaw: float,
) -> np.ndarray:
    """LaserScan ölçümlerini base_footprint çerçevesindeki (N, 2) kartezyen noktalara çevirir."""
    r = np.array(ranges)
    angles = angle_min + np.arange(len(r)) * angle_increment
    valid = np.isfinite(r) & (r > range_min) & (r < range_max)
    if not np.any(valid):
        return np.empty((0, 2))
    r_val = r[valid]
    a_val = angles[valid]
    x_l = r_val * np.cos(a_val)
    y_l = r_val * np.sin(a_val)
    c, s = math.cos(tf_yaw), math.sin(tf_yaw)
    lx, ly = tf_trans[0], tf_trans[1]
    return np.c_[lx + c * x_l - s * y_l, ly + s * x_l + c * y_l]


def q_yakin_mi(
    robot: tuple[float, ...] | list[float],
    q: tuple[float, ...] | list[float],
    xy: float = 0.35,
    yaw: float = math.radians(35.0),
) -> tuple[bool, float, float]:
    """Robotun harita pozunun hedef Q pozuna toleranslar içinde olup olmadığını denetler.

    Parametreler:
        robot: Robotun haritadaki (x, y, yaw) pozu.
        q: Hedef Q (x, y, yaw) pozu.
        xy: Mesafe kabul toleransı (metre, varsayılan 0.35 m).
        yaw: Yönelme kabul toleransı (radyan veya derece; varsayılan 35° / ~0.61 rad).

    Döndürür:
        (yakin_mi, d, a):
            yakin_mi (bool): Robot Q'ya kabul sınırları içindeyse True.
            d (float): Robot ile Q arasındaki xy Öklid mesafesi (metre).
            a (float): Robot ile Q arasındaki mutlak açısal fark (radyan).
    """
    rx, ry = float(robot[0]), float(robot[1])
    qx, qy = float(q[0]), float(q[1])
    d = float(math.hypot(rx - qx, ry - qy))

    ryaw = float(robot[2]) if len(robot) >= 3 else 0.0
    qyaw = float(q[2]) if len(q) >= 3 else 0.0
    a = float(abs(wrap(ryaw - qyaw)))

    xy_tol = float(xy) / 100.0 if float(xy) > 10.0 else float(xy)
    yaw_tol = math.radians(float(yaw)) if abs(float(yaw)) > math.pi else abs(float(yaw))

    yakin = (d <= xy_tol) and (a <= yaw_tol)
    return yakin, d, a


def kurtarma_durum_gecisi(
    durum_kodu: str,
    kat_edilen: float,
    kurtarma_sayisi: int,
    aktif_aday_idx: int,
    hedef_mesafe: float = 0.20,
) -> tuple[str, int, int, bool]:
    """KURTARMA durumundayken mesafe tamamlandığında durumu atomik/korumalı olarak GECIS'e alır.

    İki ardışık çağrıda durum 'GECIS' olacağı için sayaçlar yalnızca bir kez artar.

    Döndürür:
        (yeni_durum, yeni_kurtarma_sayisi, yeni_aktif_aday_idx, tamamlandi_mi)
    """
    if durum_kodu != 'KURTARMA':
        return durum_kodu, kurtarma_sayisi, aktif_aday_idx, False

    if kat_edilen < hedef_mesafe:
        return 'KURTARMA', kurtarma_sayisi, aktif_aday_idx, False

    return 'GECIS', kurtarma_sayisi + 1, aktif_aday_idx + 1, True


def erken_kabul_durumu(
    gecmis: list[tuple[float, float, float]] | list[Any],
    xy: float = 0.20,
    yaw: float = math.radians(20.0),
    sure: float = 1.0,
) -> bool:
    """Robotun Q pozuna erken kabul eşikleri içinde yeterli süre kalıp kalmadığını belirler (ROS'suz saf fonksiyon).

    Parametreler:
        gecmis: Zaman damgalı (t, d, a) kayıtları listesi:
                t: saniye (float),
                d: xy mesafesi (metre veya cm, float),
                a: yaw açısal farkı (radyan veya derece, float).
        xy: Mesafe eşiği (metre veya cm, varsayılan 0.20 m).
        yaw: Açısal fark eşiği (radyan veya derece, varsayılan 20° / ~0.349 rad).
        sure: Eşik içinde kesintisiz kalınması gereken süre (saniye, varsayılan 1.0 s).

    Döndürür:
        bool: Son durum kesintisiz olarak en az 'sure' saniye eşikler içindeyse True, aksi halde False.
    """
    if not gecmis:
        return False

    xy_tol = float(xy) / 100.0 if float(xy) > 10.0 else float(xy)
    yaw_tol = math.radians(float(yaw)) if abs(float(yaw)) > math.pi else abs(float(yaw))
    sure_val = float(sure)

    # Son kayıt eşik içinde değilse kesintisiz süre sıfırlanmıştır (sayaç sıfırlanır)
    son_t, son_d, son_a = float(gecmis[-1][0]), float(gecmis[-1][1]), float(gecmis[-1][2])
    if not (son_d <= xy_tol and son_a <= yaw_tol):
        return False

    # Sondan geriye doğru kesintisiz eşik içinde kalınan başlangıç zamanını bul
    t_baslangic = son_t
    for kayit in reversed(gecmis):
        t, d, a = float(kayit[0]), float(kayit[1]), float(kayit[2])
        if d <= xy_tol and a <= yaw_tol:
            t_baslangic = t
        else:
            break

    return (son_t - t_baslangic) >= (sure_val - 1e-9)


def nav_q_durum_gecisi(
    durum_kodu: str,
    hedef_durum: str = 'SON_YAKLASMA',
) -> tuple[str, bool]:
    """NAV_Q durumundan çıkışta tek geçiş koruması sağlar (N13 mantığı).

    İki ardışık çağrıda durum ilk çağrıda 'GECIS'e alınacağı için geçiş
    aynı tikte veya yarış durumunda yalnızca bir kez tetiklenir.

    Parametreler:
        durum_kodu: Mevcut durum makinesi kodu ('NAV_Q', 'GECIS', 'SON_YAKLASMA', vb.).
        hedef_durum: Geçilmek istenen durum ('SON_YAKLASMA' veya 'SONRAKI_ADAY').

    Döndürür:
        (yeni_durum, gecis_izni):
            yeni_durum: 'GECIS' (durum NAV_Q ise) veya mevcut durum_kodu.
            gecis_izni: Yalnızca mevcut durum 'NAV_Q' ise True, aksi halde False.
    """
    if durum_kodu != 'NAV_Q':
        return durum_kodu, False
    return 'GECIS', True


# ---------------------------------------------------------------------------
# Hareketli Engel Kalkanı Entegrasyonu İçin ROS'suz Saf Fonksiyonlar (N24)
# ---------------------------------------------------------------------------


def kalkan_dur_mu(durum_metni: str | None) -> bool:
    """Durum metninin 'DUR' kararı içerip içermediğini döndürür (ROS'suz saf fonksiyon)."""
    if not durum_metni:
        return False
    s_upper = str(durum_metni).strip().upper()
    return s_upper == 'DUR' or s_upper.startswith(('DUR ', 'DUR:', 'DUR='))


def kalkan_yavas_mi(durum_metni: str | None) -> bool:
    """Durum metninin 'YAVAS' kararı içerip içermediğini döndürür (ROS'suz saf fonksiyon)."""
    if not durum_metni:
        return False
    s_upper = str(durum_metni).strip().upper()
    return s_upper == 'YAVAS' or s_upper.startswith(('YAVAS ', 'YAVAS:', 'YAVAS='))


def kalkan_uygula(
    vx: float,
    wz: float,
    durum_metni: str | None,
) -> tuple[float, float]:
    """Kalkan durum metnine göre hız komutunu filtreler (ROS'suz saf fonksiyon).

    Parametreler:
        vx: Doğrusal hız komutu (m/s).
        wz: Açısal hız komutu (rad/s).
        durum_metni: /hareketli_kalkan/durum metni (örn. 'DUR ttc=1.2s v=0.5',
                     'YAVAS ttc=2.0s', 'serbest', 'ODOM_YOK', None).

    Kural:
        - 'DUR': vx = 0.0 yapılır, wz korunur (yerinde dönüş serbest).
          Geri çekilmede (vx < 0) de DUR durumunda vx = 0 yapılır; çünkü
          hareketli engelin robota göre konumu/yönü (önde mi arkada mı olduğu)
          bu durum metninden bilinmemektedir ve engel robotun arkasında
          olabilir.
        - 'YAVAS': vx = vx * 0.5 yapılır, wz korunur.
        - diğer / None: vx ve wz aynen korunur.

    Döndürür:
        (vx_yeni, wz_yeni)
    """
    vx_val = float(vx)
    wz_val = float(wz)
    if not durum_metni:
        return (vx_val, wz_val)

    s = str(durum_metni).strip()
    s_upper = s.upper()

    if s_upper == 'DUR' or s_upper.startswith(('DUR ', 'DUR:', 'DUR=')):
        # Geri çekilmede (vx < 0) de DUR'da vx=0 (engel arkada olabilir; kalkan yönü bilinmiyor)
        return (0.0, wz_val)
    elif s_upper == 'YAVAS' or s_upper.startswith(('YAVAS ', 'YAVAS:', 'YAVAS=')):
        return (vx_val * 0.5, wz_val)
    else:
        return (vx_val, wz_val)


def kalkan_bekleme_kontrolu(
    dur_baslangic_t: float | None,
    simdi_t: float,
    kalkan_durum: str | None,
    maks_bekleme: float = 15.0,
) -> tuple[bool, float | None, float]:
    """Kalkan DUR durumunun süresini ve zaman aşımını denetler (ROS'suz saf fonksiyon).

    Parametreler:
        dur_baslangic_t: DUR durumunun başladığı zaman damgası (s) veya None.
        simdi_t: Geçerli zaman damgası (s).
        kalkan_durum: Kalkan durum dizesi (örn. 'DUR ttc=1.2s v=0.4', 'serbest', None).
        maks_bekleme: İzin verilen azami bekleme süresi (s, varsayılan 15.0).

    Döndürür:
        (asildi_mi, yeni_dur_baslangic_t, gecen_sure_s):
            asildi_mi (bool): DUR süresi maks_bekleme'yi aştıysa True.
            yeni_dur_baslangic_t (float | None): Güncel başlangıç zamanı (DUR değilse None).
            gecen_sure_s (float): Kesintisiz DUR durumunda geçen toplam süre (s).
    """
    if not kalkan_dur_mu(kalkan_durum):
        return (False, None, 0.0)

    baslangic = float(simdi_t) if dur_baslangic_t is None else float(dur_baslangic_t)
    gecen = float(simdi_t) - baslangic
    asildi = gecen > float(maks_bekleme)
    return (asildi, baslangic, gecen)


# ---------------------------------------------------------------------------
# Eylem (Action) İçin ROS'suz Saf Yardımcı Fonksiyonlar ve Sınıflar
# ---------------------------------------------------------------------------


def yon_derece_radyana(yon_derece: float | int | None) -> float | None:
    """Derece cinsinden yaklaşma yönünü radyana çevirir; NaN veya None ise None döner."""
    if yon_derece is None:
        return None
    val = float(yon_derece)
    if not math.isfinite(val):
        return None
    return math.radians(val)


def eylem_hedef_coz(
    hedef: Any = None,
    *,
    yon_derece: float | None = float('nan'),
    algisiz_izinli: bool = True,
    ayar: Ayar | None = None,
) -> dict[str, Any]:
    """Kavra eylem hedefini (veya sözlük/tuple'ı) düğüm içi parametrelere dönüştürür (ROS'suz saf fonksiyon).

    Parametreler:
        hedef: Kavra.Goal benzeri nesne (.nesne, .yon_derece, .algisiz_izinli),
               PointStamped benzeri nesne (.point, .header), sözlük veya (x, y[, z]) dizisi.
        yon_derece: hedef içinde yon_derece alanı yoksa kullanılacak derece açısı (NaN -> serbest/None).
        algisiz_izinli: hedef içinde algisiz_izinli alanı yoksa kullanılacak değer.
        ayar: Verilirse ayar.yon alanı hesaplanan radyan değeri (veya None) ile güncellenir.

    Döndürür:
        {
            'nesne_xyz': (x, y, z),
            'frame_id': str,
            'yon_rad': float | None,
            'yon': float | None,
            'algisiz_izinli': bool,
            'ayar': Ayar | None,
        }
    """
    nx, ny, nz = 0.0, 0.0, 0.0
    frame_id = 'map'
    hedef_yon = yon_derece
    hedef_algisiz = algisiz_izinli

    if hedef is not None:
        if hasattr(hedef, 'nesne'):
            nesne = hedef.nesne
            if hasattr(nesne, 'point'):
                nx = float(nesne.point.x)
                ny = float(nesne.point.y)
                nz = float(getattr(nesne.point, 'z', 0.0))
            elif isinstance(nesne, (tuple, list)):
                nx = float(nesne[0])
                ny = float(nesne[1])
                nz = float(nesne[2]) if len(nesne) > 2 else 0.0
            header = getattr(nesne, 'header', None)
            if header is not None and getattr(header, 'frame_id', ''):
                frame_id = str(header.frame_id)
            if hasattr(hedef, 'yon_derece'):
                hedef_yon = getattr(hedef, 'yon_derece')
            if hasattr(hedef, 'algisiz_izinli'):
                hedef_algisiz = getattr(hedef, 'algisiz_izinli')
        elif hasattr(hedef, 'point'):
            nx = float(hedef.point.x)
            ny = float(hedef.point.y)
            nz = float(getattr(hedef.point, 'z', 0.0))
            header = getattr(hedef, 'header', None)
            if header is not None and getattr(header, 'frame_id', ''):
                frame_id = str(header.frame_id)
        elif isinstance(hedef, dict):
            if 'nesne' in hedef:
                n = hedef['nesne']
                if isinstance(n, dict):
                    pt = n.get('point', n)
                    nx = float(pt.get('x', 0.0))
                    ny = float(pt.get('y', 0.0))
                    nz = float(pt.get('z', 0.0))
                    hdr = n.get('header', {})
                    if isinstance(hdr, dict) and hdr.get('frame_id'):
                        frame_id = str(hdr['frame_id'])
                elif isinstance(n, (tuple, list)):
                    nx = float(n[0])
                    ny = float(n[1])
                    nz = float(n[2]) if len(n) > 2 else 0.0
                elif hasattr(n, 'point'):
                    nx = float(n.point.x)
                    ny = float(n.point.y)
                    nz = float(getattr(n.point, 'z', 0.0))
                    header = getattr(n, 'header', None)
                    if header is not None and getattr(header, 'frame_id', ''):
                        frame_id = str(header.frame_id)
            else:
                nx = float(hedef.get('x', 0.0))
                ny = float(hedef.get('y', 0.0))
                nz = float(hedef.get('z', 0.0))
                if hedef.get('frame_id'):
                    frame_id = str(hedef['frame_id'])
            if 'yon_derece' in hedef:
                hedef_yon = hedef['yon_derece']
            if 'algisiz_izinli' in hedef:
                hedef_algisiz = hedef['algisiz_izinli']
        elif isinstance(hedef, (tuple, list)):
            nx = float(hedef[0])
            ny = float(hedef[1])
            nz = float(hedef[2]) if len(hedef) > 2 else 0.0

    yon_rad = yon_derece_radyana(hedef_yon)
    algisiz_bool = bool(hedef_algisiz)

    if ayar is not None:
        ayar.yon = yon_rad

    return {
        'nesne_xyz': (nx, ny, nz),
        'frame_id': frame_id,
        'yon_rad': yon_rad,
        'yon': yon_rad,
        'algisiz_izinli': algisiz_bool,
        'ayar': ayar,
    }


def durum_metni_ayristir(metin: str) -> tuple[str, str]:
    """'DURUM ayrıntı' biçimindeki durum metnini (durum, ayrinti) olarak ikiye ayırır."""
    s = str(metin or '').strip()
    if not s:
        return ('', '')
    if s.startswith('HATA:'):
        parcalar = s.split(None, 1)
        durum = 'HATA'
        ayrinti_1 = parcalar[0][5:].strip()
        ayrinti_2 = parcalar[1].strip() if len(parcalar) > 1 else ''
        ayrinti = f'{ayrinti_1} {ayrinti_2}'.strip()
        return (durum, ayrinti)
    parcalar = s.split(None, 1)
    durum = parcalar[0]
    ayrinti = parcalar[1].strip() if len(parcalar) > 1 else ''
    return (durum, ayrinti)


def durum_metninden_sonuc(durum_metni: str) -> tuple[bool, str]:
    """Durum metninden ('HAZIR ...' veya 'HATA ...') eylem sonucu (basarili, neden) üretir."""
    s = str(durum_metni or '').strip()
    if s.startswith('HAZIR'):
        return (True, 'HAZIR')
    if s.startswith('HATA'):
        return (False, s)
    if s:
        return (False, f'HATA {s}')
    return (False, 'HATA bilinmeyen neden')


@dataclass
class KavraSonucBilgi:
    """ROS'suz ortamlarda Kavra.Result alanlarını taşıyan saf veri yapısı."""

    basarili: bool = False
    neden: str = 'HATA'
    nesne_robotta_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tahmini_hata_cm: float = 0.0
    tahmini_yon_hatasi_derece: float = 0.0
    sure_s: float = 0.0
    nesne_robotta: Any = None

    def __getitem__(self, anahtar: Any) -> Any:
        if anahtar == 0 or anahtar == 'basarili':
            return self.basarili
        if anahtar == 1 or anahtar == 'neden':
            return self.neden
        if anahtar in ('nesne_robotta_xyz', 'nesne_robotta', 'tahmini_hata_cm', 'tahmini_yon_hatasi_derece', 'sure_s'):
            return getattr(self, anahtar)
        raise KeyError(anahtar)


_RE_NESNE_ROBOTTA = re.compile(
    r'nesne\s+robotta\s*\(\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*\)'
)
_RE_P_HATASI = re.compile(
    r'P\s+hatas[ıi]\s*([+-]?\d+(?:\.\d+)?)\s*cm\s*,\s*([+-]?\d+(?:\.\d+)?)\s*°'
)
_RE_SURE = re.compile(r'sure\s+([+-]?\d+(?:\.\d+)?)\s*s')


def sonuc_doldur(
    durum_metni: str,
    sonuc_nesnesi: Any = None,
    *,
    nesne_robotta_xyz: tuple[float, float, float] | None = None,
    tahmini_hata_cm: float | None = None,
    tahmini_yon_hatasi_derece: float | None = None,
    sure_s: float | None = None,
    frame_id: str = 'base_footprint',
) -> Any:
    """HAZIR veya HATA durum metninden eylem sonuç nesnesini doldurur (ROS'suz saf fonksiyon).

    Eğer metin 'HAZIR nesne robotta (...); P hatası X cm, Y°; sure Z s' biçimindeyse ve
    ilgili sayısal argümanlar None bırakılmışsa metinden otomatik ayrıştırılır.
    """
    basarili, neden = durum_metninden_sonuc(durum_metni)
    s = str(durum_metni or '')

    if nesne_robotta_xyz is None:
        m_nesne = _RE_NESNE_ROBOTTA.search(s)
        if m_nesne:
            nesne_robotta_xyz = (
                float(m_nesne.group(1)),
                float(m_nesne.group(2)),
                float(m_nesne.group(3)),
            )
        else:
            nesne_robotta_xyz = (0.0, 0.0, 0.0)
    else:
        nesne_robotta_xyz = (
            float(nesne_robotta_xyz[0]),
            float(nesne_robotta_xyz[1]),
            float(nesne_robotta_xyz[2]) if len(nesne_robotta_xyz) > 2 else 0.0,
        )

    if tahmini_hata_cm is None or tahmini_yon_hatasi_derece is None:
        m_hata = _RE_P_HATASI.search(s)
        if m_hata:
            if tahmini_hata_cm is None:
                tahmini_hata_cm = float(m_hata.group(1))
            if tahmini_yon_hatasi_derece is None:
                tahmini_yon_hatasi_derece = float(m_hata.group(2))

    if sure_s is None:
        m_sure = _RE_SURE.search(s)
        if m_sure:
            sure_s = float(m_sure.group(1))

    hata_cm_val = float(tahmini_hata_cm) if tahmini_hata_cm is not None else 0.0
    yon_hata_val = float(tahmini_yon_hatasi_derece) if tahmini_yon_hatasi_derece is not None else 0.0
    sure_val = float(sure_s) if sure_s is not None else 0.0

    ps = PointStamped()
    ps.header.frame_id = str(frame_id)
    ps.point.x = float(nesne_robotta_xyz[0])
    ps.point.y = float(nesne_robotta_xyz[1])
    ps.point.z = float(nesne_robotta_xyz[2])

    if sonuc_nesnesi is None:
        return KavraSonucBilgi(
            basarili=basarili,
            neden=neden,
            nesne_robotta_xyz=nesne_robotta_xyz,
            tahmini_hata_cm=hata_cm_val,
            tahmini_yon_hatasi_derece=yon_hata_val,
            sure_s=sure_val,
            nesne_robotta=ps,
        )

    sonuc_nesnesi.basarili = bool(basarili)
    sonuc_nesnesi.neden = str(neden)
    sonuc_nesnesi.tahmini_hata_cm = float(hata_cm_val)
    sonuc_nesnesi.tahmini_yon_hatasi_derece = float(yon_hata_val)
    sonuc_nesnesi.sure_s = float(sure_val)
    if hasattr(sonuc_nesnesi, 'nesne_robotta'):
        if getattr(sonuc_nesnesi, 'nesne_robotta') is None:
            sonuc_nesnesi.nesne_robotta = ps
        else:
            sonuc_nesnesi.nesne_robotta.header.frame_id = str(frame_id)
            sonuc_nesnesi.nesne_robotta.point.x = float(nesne_robotta_xyz[0])
            sonuc_nesnesi.nesne_robotta.point.y = float(nesne_robotta_xyz[1])
            sonuc_nesnesi.nesne_robotta.point.z = float(nesne_robotta_xyz[2])
    return sonuc_nesnesi


def geri_bildirim_gonderilmeli_mi(
    simdi_s: float,
    son_gonderim_s: float | None,
    yeni_durum: str | None = None,
    onceki_durum: str | None = None,
    frekans_hz: float = 5.0,
) -> bool:
    """Geri bildirimin bu adımda yayınlanıp yayınlanmayacağını belirler (ROS'suz saf fonksiyon).

    Durum değişimlerinde anında True döner; aynı durumda ise en az (1 / frekans_hz) saniye
    geçtiğinde True döner.
    """
    if frekans_hz <= 0:
        raise ValueError('frekans_hz pozitif olmalıdır')
    if yeni_durum is not None and onceki_durum is not None and yeni_durum != onceki_durum:
        return True
    if son_gonderim_s is None:
        return True
    periyot = 1.0 / float(frekans_hz)
    return (float(simdi_s) - float(son_gonderim_s)) >= (periyot - 1e-9)


class GeriBildirimKisitlayici:
    """Durum değişimlerinde anında, aynı durumda ise belirli frekansta (vars. 5 Hz) izin veren kısıtlayıcı."""

    def __init__(self, frekans_hz: float = 5.0) -> None:
        if frekans_hz <= 0:
            raise ValueError('frekans_hz pozitif olmalıdır')
        self.frekans_hz: float = float(frekans_hz)
        self.periyot_s: float = 1.0 / self.frekans_hz
        self.son_zaman_s: float | None = None
        self.son_durum: str | None = None

    def sifirla(self) -> None:
        """Kısıtlayıcı zaman ve durum geçmişini sıfırlar."""
        self.son_zaman_s = None
        self.son_durum = None

    def izin_var_mi(self, t_s: float, durum: str | None = None) -> bool:
        """Verilen zaman (s) ve durumda geri bildirim gönderimine izin verilip verilmediğini döndürür."""
        gonder = geri_bildirim_gonderilmeli_mi(
            simdi_s=float(t_s),
            son_gonderim_s=self.son_zaman_s,
            yeni_durum=durum,
            onceki_durum=self.son_durum,
            frekans_hz=self.frekans_hz,
        )
        if gonder:
            self.son_zaman_s = float(t_s)
            if durum is not None:
                self.son_durum = str(durum)
            return True
        return False


@dataclass
class KavraGeriBildirimBilgi:
    """ROS'suz ortamlarda Kavra.Feedback alanlarını taşıyan saf veri yapısı."""

    durum: str = ''
    ayrinti: str = ''
    aday_no: int = 0
    aday_sayisi: int = 0
    kalan_mesafe_m: float = 0.0


def geri_bildirim_doldur(
    geri_bildirim_nesnesi: Any = None,
    *,
    durum: str = '',
    ayrinti: str = '',
    aday_no: int = 0,
    aday_sayisi: int = 0,
    kalan_mesafe_m: float = 0.0,
) -> Any:
    """Geri bildirim mesaj nesnesini (veya KavraGeriBildirimBilgi'yi) doldurur."""
    hedef = geri_bildirim_nesnesi if geri_bildirim_nesnesi is not None else KavraGeriBildirimBilgi()
    hedef.durum = str(durum)
    hedef.ayrinti = str(ayrinti)
    hedef.aday_no = int(aday_no)
    hedef.aday_sayisi = int(aday_sayisi)
    hedef.kalan_mesafe_m = float(kalan_mesafe_m)
    return hedef


class KavramaDugumu(Node):
    """ICP'siz, nesneye göreli son yaklaşmalı ve kurtarmalı kavrama durum makinesi."""

    AKTIF_DURUMLAR = ('PLAN', 'NAV_Q', 'SON_YAKLASMA', 'KURTARMA', 'GECIS')

    def __init__(self) -> None:
        super().__init__('kavrama')
        p = self.declare_parameter

        # Parametreler
        harita_yolu = p('harita', '').value
        self.harita = Harita.yaml_oku(harita_yolu) if harita_yolu else None

        yon = p('yon_derece', float('nan')).value
        d_on_val = p('d_on', 0.35).value
        if math.isnan(d_on_val):
            d_on_val = None

        d_kavrama_val = p('d_kavrama', 0.22).value
        yanal_val = p('yanal', 0.0).value

        varsayilan_yon_rad = yon_derece_radyana(yon)
        self._varsayilan_yon_rad: float | None = varsayilan_yon_rad

        self.ayar = Ayar(
            d_kavrama=d_kavrama_val,
            yanal=yanal_val,
            d_on=d_on_val,
            pay=p('pay', 0.05).value,
            r_nesne=p('r_nesne', 0.12).value,
            yon=varsayilan_yon_rad,
        )

        self.son_ayar = SonYaklasmaAyar(
            d_kavrama=d_kavrama_val,
            yanal=yanal_val,
            v_max=p('son_hiz', 0.10).value,
            w_max=p('donus_hiz', 0.6).value,
        )

        # Kol erişimi sınırları (myCobot 280)
        self.z_min = float(p('z_min', 0.0).value)
        self.z_max = float(p('z_max', 0.45).value)

        # Q yakın kabul parametreleri (Nav2 MPPI hassas hedefi tutturamazsa)
        self.q_kabul_xy = float(p('q_kabul_xy', 0.35).value)
        q_yaw_val = float(p('q_kabul_yaw', 35.0).value)
        self.q_kabul_yaw = math.radians(q_yaw_val) if abs(q_yaw_val) > math.pi else q_yaw_val

        # Q erken kabul ve NAV_Q zaman aşımı parametreleri (N15)
        q_erken_xy_val = float(p('q_erken_xy', 0.20).value)
        self.q_erken_xy = q_erken_xy_val / 100.0 if q_erken_xy_val > 10.0 else q_erken_xy_val
        q_erken_yaw_val = float(p('q_erken_yaw', 20.0).value)
        self.q_erken_yaw = math.radians(q_erken_yaw_val) if abs(q_erken_yaw_val) > math.pi else q_erken_yaw_val
        self.q_erken_sure = float(p('q_erken_sure', 1.0).value)
        self.nav_q_zaman_asimi = float(p('nav_q_zaman_asimi', 90.0).value)

        # Varsayılan hassas BT XML yolu
        try:
            from ament_index_python.packages import get_package_share_directory
            varsayilan_bt = os.path.join(
                get_package_share_directory('limo_bringup'),
                'behavior_trees',
                'navigate_to_pose_hassas.xml',
            )
        except Exception:
            varsayilan_bt = ''

        self.declare_parameter('bt_hassas', varsayilan_bt)

        # cmd_vel konusu
        cmd_konu = p('cmd_konu', 'cmd_vel').value
        # Geriye dönük uyumluluk: son_cmd_topic parametresi verilmişse kontrol et
        if self.has_parameter('son_cmd_topic'):
            cmd_konu = self.get_parameter('son_cmd_topic').value

        scan_konu = p('scan_topic', 'scan').value
        odom_konu = p('odom_topic', 'odom').value

        # Hareketli engel kalkanı entegrasyon parametreleri (N24)
        kalkan_konu = str(p('kalkan_konu', '/hareketli_kalkan/durum').value)
        self.kalkan_uy = bool(p('kalkan_uy', True).value)
        self.kalkan_maks_bekleme = float(p('kalkan_maks_bekleme', 15.0).value)

        # Eşzamanlılık ve TF
        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        TransformListener(self.tf, self)

        # Nav2 Action Client
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)

        # Yayıncılar
        self.p_cmd = self.create_publisher(Twist, cmd_konu, 10)
        self.p_durum = self.create_publisher(String, '/kavrama/durum', 10)
        self.p_nesne = self.create_publisher(PointStamped, '/kavrama/nesne_robotta', 10)
        self.p_plan = self.create_publisher(PoseArray, '/kavrama/plan', 10)

        # Abonelikler
        self.create_subscription(Odometry, odom_konu, self._odom_cb, 50, callback_group=self.cb)
        self.create_subscription(LaserScan, scan_konu, self._scan_cb, qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(PointStamped, '/kavrama/hedef', self._hedef_cb, 10, callback_group=self.cb)
        self.create_subscription(PointStamped, '/kavrama/nesne_algi', self._nesne_algi_cb, 10, callback_group=self.cb)
        self.create_subscription(Empty, '/kavrama/iptal', self._iptal_cb, 10, callback_group=self.cb)
        self.create_subscription(String, kalkan_konu, self._kalkan_cb, 10, callback_group=self.cb)

        # Durum Değişkenleri
        self.durum_kodu: str = 'BOSTA'  # 'BOSTA', 'PLAN', 'NAV_Q', 'SON_YAKLASMA', 'HAZIR', 'KURTARMA', 'HATA'
        self.odom: tuple[float, float, float] | None = None
        self.tarama_noktalari: np.ndarray | None = None
        self.son_algi_odom: tuple[float, float] | None = None
        self.yeni_algi_var: bool = False
        self.aktif_nav_goal = None

        # Hareketli engel kalkanı durum değişkenleri (N24)
        self.kalkan_durum: str | None = None
        self.kalkan_durum_zamani: float | None = None
        self._kalkan_dur_baslangic_t: float | None = None
        self._kalkan_bekleme_bildirildi: bool = False

        self.nesne_hedef_map: tuple[float, float, float] | None = None
        self.nesne_yedek_odom: tuple[float, float] | None = None
        self.adaylar: list = []
        self.aktif_aday_idx: int = 0
        self.kurtarma_sayisi: int = 0
        self.kurtarma_baslangic_odom: tuple[float, float, float] | None = None

        self.son_denetleyici: SonYaklasma | None = None
        self.algisiz: bool = False
        self.algisiz_izinli: bool = True
        self.tek_olcum_verildi: bool = False
        self.t_son_yaklasma_baslangic: rclpy.time.Time | None = None
        self.son_durum_log_zamani: float = 0.0

        # NAV_Q erken kabul ve zaman aşımı durum değişkenleri (N15)
        self.t_nav_q_baslangic: float = 0.0
        self.nav_q_gecmis: list[tuple[float, float, float]] = []
        self._nav_q_kalan_mesafe_m: float = 0.0
        self._durum_kilit = threading.RLock()

        # Eylem Sunucusu Durum ve Eşzamanlılık Değişkenleri
        self._eylem_kilit = threading.RLock()
        self._eylem_aktif: bool = False
        self._aktif_eylem_gh: Any = None
        self._eylem_bitti_event = threading.Event()
        self._fb_kisitlayici = GeriBildirimKisitlayici(frekans_hz=5.0)
        self._son_durum_metni: str = ''
        self._son_nesne_robotta_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._son_nesne_robotta_msg: PointStamped | None = None
        self._son_hata_cm: float = 0.0
        self._son_yon_hatasi_deg: float = 0.0
        self._son_sure_s: float = 0.0
        self._t_hedef_baslangic_s: float = 0.0

        if LIMO_INTERFACES_VAR and Kavra is not None:
            self._eylem_sunucu = ActionServer(
                self,
                Kavra,
                'kavra',
                execute_callback=self._kavra_execute_cb,
                goal_callback=self._kavra_goal_cb,
                cancel_callback=self._kavra_cancel_cb,
                callback_group=self.cb,
            )
        else:
            self._eylem_sunucu = None
            self.get_logger().warn(
                'limo_interfaces paketi bulunamadı (derlenmemiş olabilir); '
                'kavra eylem sunucusu atlandı, yalnız konu arayüzü çalışıyor.'
            )

        # 50 Hz Denetim Döngüsü Timer'ı
        self.timer_50hz = self.create_timer(1.0 / 50.0, self._dongu_50hz, callback_group=self.cb)

        self.get_logger().info(f'Kavrama düğümü hazır (ICP kaldırıldı, kapalı çevrim aktif): {self.ayar}')

    @property
    def durum_adi(self) -> str:
        """Durum koduna durum_adi üzerinden de erişim sağlar."""
        return self.durum_kodu

    @durum_adi.setter
    def durum_adi(self, val: str) -> None:
        self.durum_kodu = val

    def _simdi_s(self) -> float:
        """ROS saatinden saniye cinsinden zaman döndürür."""
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _mesgul_mu(self) -> bool:
        """Düğümün aktif bir kavrama görevi yürütüp yürütmediğini döndürür."""
        return (self.durum_kodu in self.AKTIF_DURUMLAR) or self._eylem_aktif

    def durum(self, s: str) -> None:
        """Durum bilgisini konsola, /kavrama/durum konusuna ve aktifse eylem geri bildirimine yayınlar."""
        self.get_logger().info(s)
        self._son_durum_metni = s
        self.p_durum.publish(String(data=s))

        durum_on_eki, ayrinti = durum_metni_ayristir(s)
        if durum_on_eki in ('PLAN', 'NAV_Q', 'KURTARMA'):
            if durum_on_eki == 'KURTARMA':
                kalan = 0.20
            elif durum_on_eki == 'NAV_Q':
                kalan = getattr(self, '_nav_q_kalan_mesafe_m', 0.0)
            else:
                kalan = 0.0
            self._eylem_geri_bildirim_yayinla(
                durum=durum_on_eki,
                ayrinti=ayrinti,
                kalan_mesafe_m=kalan,
                zorla=True,
            )
        elif durum_on_eki == 'SON_YAKLASMA':
            if self._fb_kisitlayici.son_durum != 'SON_YAKLASMA':
                self._eylem_geri_bildirim_yayinla(
                    durum='SON_YAKLASMA',
                    ayrinti=ayrinti,
                    kalan_mesafe_m=0.0,
                    zorla=False,
                )
        elif durum_on_eki in ('HAZIR', 'HATA'):
            self._eylem_bitti_event.set()

    def _eylem_geri_bildirim_yayinla(
        self,
        durum: str,
        ayrinti: str,
        kalan_mesafe_m: float = 0.0,
        zorla: bool = False,
    ) -> None:
        """Aktif eylem hedefi varsa geri bildirim mesajı yayınlar."""
        gh = self._aktif_eylem_gh
        if gh is None or not LIMO_INTERFACES_VAR or Kavra is None:
            return

        t_s = self._simdi_s()
        izin = self._fb_kisitlayici.izin_var_mi(t_s, durum=durum)
        if not zorla and not izin:
            return
        if zorla:
            self._fb_kisitlayici.son_zaman_s = t_s
            self._fb_kisitlayici.son_durum = durum

        aday_sayisi = len(self.adaylar)
        aday_no = (self.aktif_aday_idx + 1) if (aday_sayisi > 0 and self.aktif_aday_idx < aday_sayisi) else 0

        fb = Kavra.Feedback()
        geri_bildirim_doldur(
            fb,
            durum=durum,
            ayrinti=ayrinti,
            aday_no=aday_no,
            aday_sayisi=aday_sayisi,
            kalan_mesafe_m=kalan_mesafe_m,
        )
        try:
            gh.publish_feedback(fb)
        except Exception as e:
            self.get_logger().debug(f'Geri bildirim yayınlama hatası: {e}')

    def tf2d(self, hedef: str, kaynak: str, timeout_sec: float = 2.0) -> tuple[float, float, float]:
        """TF ağacından kaynak->hedef 2B (x, y, yaw) dönüşümünü çeker."""
        t = self.tf.lookup_transform(
            hedef,
            kaynak,
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=timeout_sec),
        ).transform
        return t.translation.x, t.translation.y, yaw_q(t.rotation)

    def _durdur(self) -> None:
        """Robotun tekerlek hızlarını sıfırlar."""
        self.p_cmd.publish(Twist())

    # --- Eylem Sunucusu Geri Çağrıları ---

    def _kavra_goal_cb(self, _goal_request: Any) -> GoalResponse:
        """Yeni eylem hedefini değerlendirir; düğüm meşgulse reddeder."""
        with self._eylem_kilit:
            if self._mesgul_mu():
                self.get_logger().warn('Yeni kavra eylem hedefi reddedildi: önceki görev sürüyor')
                return GoalResponse.REJECT
            self._eylem_aktif = True
            self._eylem_bitti_event.clear()
            return GoalResponse.ACCEPT

    def _kavra_cancel_cb(self, _goal_handle: Any) -> CancelResponse:
        """Eylem iptal isteğini kabul eder."""
        self.get_logger().info('Kavra eylem iptal isteği alındı')
        return CancelResponse.ACCEPT

    def _kavra_execute_cb(self, goal_handle: Any) -> Any:
        """Kavra eylem hedefini mevcut durum makinesi akışıyla yürütür ve sonucu döndürür."""
        self._aktif_eylem_gh = goal_handle
        self._t_hedef_baslangic_s = self._simdi_s()
        self._son_durum_metni = ''
        self._son_nesne_robotta_xyz = (0.0, 0.0, 0.0)
        self._son_nesne_robotta_msg = None
        self._son_hata_cm = 0.0
        self._son_yon_hatasi_deg = 0.0
        self._son_sure_s = 0.0
        self._fb_kisitlayici.sifirla()

        try:
            cozulen = eylem_hedef_coz(goal_handle.request, ayar=self.ayar)
            self.algisiz_izinli = cozulen['algisiz_izinli']
            nx, ny, nz = cozulen['nesne_xyz']
            frame_id = cozulen['frame_id']

            hedef_gecerli = True
            if frame_id and frame_id != 'map':
                try:
                    t_fm = self.tf.lookup_transform(
                        'map',
                        frame_id,
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=1.0),
                    )
                    trans = (
                        t_fm.transform.translation.x,
                        t_fm.transform.translation.y,
                        t_fm.transform.translation.z,
                    )
                    rot = (
                        t_fm.transform.rotation.x,
                        t_fm.transform.rotation.y,
                        t_fm.transform.rotation.z,
                        t_fm.transform.rotation.w,
                    )
                    nx, ny, nz = nokta_donustur_3d(trans, rot, (nx, ny, nz))
                except Exception as e:
                    self.durum_kodu = 'HATA'
                    self.durum(f'HATA hedef TF map dönüşümü başarısız ({frame_id}): {e}')
                    hedef_gecerli = False

            if hedef_gecerli:
                if self.harita is None:
                    self.durum_kodu = 'HATA'
                    self.durum('HATA harita parametresi boş veya yüklenmedi')
                else:
                    self.nesne_hedef_map = (nx, ny, nz)
                    self.kurtarma_sayisi = 0
                    self.aktif_aday_idx = 0
                    self.durum_kodu = 'PLAN'
                    self._planla()

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._iptal_cb(Empty())
                    break
                if self.durum_kodu in ('HAZIR', 'HATA', 'BOSTA'):
                    break
                self._eylem_bitti_event.wait(timeout=0.02)

            toplam_sure = max(0.0, self._simdi_s() - self._t_hedef_baslangic_s)
            sure_s = self._son_sure_s if self._son_sure_s > 0.0 else toplam_sure

            sonuc = Kavra.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                sonuc_doldur(
                    'HATA iptal',
                    sonuc,
                    nesne_robotta_xyz=self._son_nesne_robotta_xyz,
                    tahmini_hata_cm=0.0,
                    tahmini_yon_hatasi_derece=0.0,
                    sure_s=sure_s,
                )
            elif self.durum_kodu == 'HAZIR':
                goal_handle.succeed()
                sonuc_doldur(
                    'HAZIR',
                    sonuc,
                    nesne_robotta_xyz=self._son_nesne_robotta_xyz,
                    tahmini_hata_cm=self._son_hata_cm,
                    tahmini_yon_hatasi_derece=self._son_yon_hatasi_deg,
                    sure_s=sure_s,
                )
                if self._son_nesne_robotta_msg is not None:
                    sonuc.nesne_robotta = self._son_nesne_robotta_msg
            else:
                goal_handle.abort()
                hata_metni = (
                    self._son_durum_metni
                    if self._son_durum_metni.startswith('HATA')
                    else 'HATA kavrama başarısız'
                )
                sonuc_doldur(
                    hata_metni,
                    sonuc,
                    nesne_robotta_xyz=self._son_nesne_robotta_xyz,
                    tahmini_hata_cm=0.0,
                    tahmini_yon_hatasi_derece=0.0,
                    sure_s=sure_s,
                )
            return sonuc
        finally:
            with self._eylem_kilit:
                self._aktif_eylem_gh = None
                self._eylem_aktif = False

    # --- Abonelik Geri Çağrıları ---

    def _odom_cb(self, m: Odometry) -> None:
        self.odom = (
            float(m.pose.pose.position.x),
            float(m.pose.pose.position.y),
            float(yaw_q(m.pose.pose.orientation)),
        )

    def _kalkan_cb(self, msg: String) -> None:
        """/hareketli_kalkan/durum konusundan gelen son kalkan kararını saklar (N24)."""
        with self._durum_kilit:
            self.kalkan_durum = str(msg.data)
            self.kalkan_durum_zamani = self._simdi_s()

    def _scan_cb(self, m: LaserScan) -> None:
        try:
            t = self.tf.lookup_transform(
                'base_footprint',
                m.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1),
            )
            lx = t.transform.translation.x
            ly = t.transform.translation.y
            lyaw = yaw_q(t.transform.rotation)

            self.tarama_noktalari = scan_to_base_footprint(
                m.ranges,
                m.angle_min,
                m.angle_increment,
                m.range_min,
                m.range_max,
                (lx, ly),
                lyaw,
            )
        except Exception:
            pass

    def _nesne_algi_cb(self, m: PointStamped) -> None:
        """/kavrama/nesne_algi ölçümünü TF ile odom çerçevesine çevirir."""
        try:
            if m.header.frame_id == 'odom':
                self.son_algi_odom = (float(m.point.x), float(m.point.y))
                self.yeni_algi_var = True
                return

            # Ölçüm anının dönüşümü: en son TF kullanılırsa robot hareket ederken görüntü gecikmesi kadar
            # (5 Hz algı + kamera gecikmesi) nesne odom'da kayar. Damga yoksa / TF tamponunda yoksa en son TF.
            try:
                if m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
                    raise ValueError('damga yok')
                t = self.tf.lookup_transform(
                    'odom',
                    m.header.frame_id,
                    rclpy.time.Time.from_msg(m.header.stamp),
                    rclpy.duration.Duration(seconds=0.1),
                )
            except Exception:
                t = self.tf.lookup_transform(
                    'odom',
                    m.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1),
                )
            trans = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
            rot = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
            ox, oy, _ = nokta_donustur_3d(trans, rot, (m.point.x, m.point.y, m.point.z))
            self.son_algi_odom = (float(ox), float(oy))
            self.yeni_algi_var = True
        except Exception as e:
            self.get_logger().debug(f'nesne_algi TF dönüşüm hatası: {e}')

    def _iptal_cb(self, _msg: Empty) -> None:
        """/kavrama/iptal sinyali: Nav2 hedefini iptal et, dur, 'HATA iptal'."""
        self.get_logger().info('Kavrama iptal isteği alındı')
        with self._durum_kilit:
            if self.aktif_nav_goal is not None:
                try:
                    self.aktif_nav_goal.cancel_goal_async()
                except Exception:
                    pass
                self.aktif_nav_goal = None
            self._durdur()
            self._kalkan_dur_baslangic_t = None
            self._kalkan_bekleme_bildirildi = False
            self.durum_kodu = 'HATA'
        self.durum('HATA iptal')

    def _hedef_cb(self, m: PointStamped) -> None:
        """/kavrama/hedef geldiğinde durum makinesini PLAN durumuna geçirir."""
        with self._eylem_kilit:
            if self._mesgul_mu():
                uyari = 'HATA önceki görev sürüyor'
                self.get_logger().info(uyari)
                self.p_durum.publish(String(data=uyari))
                return

            if self.harita is None:
                self.durum_kodu = 'HATA'
                self.durum('HATA harita parametresi boş veya yüklenmedi')
                return

            self.ayar.yon = self._varsayilan_yon_rad
            self.algisiz_izinli = True
            nx, ny, nz = float(m.point.x), float(m.point.y), float(m.point.z)
            self.nesne_hedef_map = (nx, ny, nz)
            self.kurtarma_sayisi = 0
            self.aktif_aday_idx = 0
            self.durum_kodu = 'PLAN'

        self._planla()

    # --- Durum Makinesi Geçişleri ---

    def _planla(self) -> None:
        """PLAN: yaklasim_bul ile en iyi 3 adayı bulur."""
        if self.nesne_hedef_map is None or self.harita is None:
            return

        nx, ny, nz = self.nesne_hedef_map
        if nz < self.z_min or nz > self.z_max:
            self.durum_kodu = 'HATA'
            self.durum(f'HATA erisilemez yukseklik z={nz:.2f}')
            return

        try:
            rx, ry, _ = self.tf2d('map', 'base_footprint')
        except Exception:
            rx, ry = 0.0, 0.0

        self.adaylar = []
        self.aktif_aday_idx = 0
        self.durum(f'PLAN nesne ({nx:+.2f}, {ny:+.2f}, z {nz:.2f}), robot ({rx:+.2f}, {ry:+.2f})')

        adaylar = yaklasim_bul(self.harita, (nx, ny), self.ayar, robot=(rx, ry), hepsi=True)
        if not adaylar:
            self.durum_kodu = 'HATA'
            self.durum('HATA geçerli yaklaşma yok (engel / erişim)')
            return

        # En iyi 3 aday
        self.adaylar = adaylar[:3]
        self.aktif_aday_idx = 0

        # RViz /kavrama/plan yayını (en iyi adayın Q ve P'si)
        y0 = self.adaylar[0]
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = [
            poz(y0.oncesi[0], y0.oncesi[1], y0.theta),
            poz(y0.kavrama[0], y0.kavrama[1], y0.theta),
        ]
        self.p_plan.publish(pa)

        self._nav_q_baslat()

    def _nav_q_baslat(self) -> None:
        """NAV_Q: Seçilen adayın Q pozuna Nav2 NavigateToPose ile hassas yaklaşma."""
        if self.aktif_aday_idx >= len(self.adaylar):
            self.durum_kodu = 'HATA'
            self.durum('HATA tüm adaylar denendi')
            return

        y = self.adaylar[self.aktif_aday_idx]
        (px, py, th), (qx, qy, _) = y.kavrama, y.oncesi

        self.t_nav_q_baslangic = self._simdi_s()
        self.nav_q_gecmis = []
        try:
            rx, ry, _ = self.tf2d('map', 'base_footprint', timeout_sec=0.05)
            self._nav_q_kalan_mesafe_m = float(math.hypot(rx - qx, ry - qy))
        except Exception:
            self._nav_q_kalan_mesafe_m = 0.0

        self.durum_kodu = 'NAV_Q'
        self.durum(
            f'NAV_Q aday {self.aktif_aday_idx + 1}/{len(self.adaylar)}: '
            f'Q ({qx:+.2f}, {qy:+.2f}, {math.degrees(th):+.0f}°) -> '
            f'P ({px:+.2f}, {py:+.2f}), açıklık {y.aciklik:.2f}'
        )

        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.durum_kodu = 'HATA'
            self.durum('HATA navigate_to_pose eylem sunucusu yok')
            return

        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose = poz(qx, qy, th)

        bt_hassas = self.get_parameter('bt_hassas').value
        if bt_hassas:
            g.behavior_tree = bt_hassas

        send_future = self.nav.send_goal_async(g)
        send_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.durum(f'NAV_Q hedef reddedildi (aday {self.aktif_aday_idx + 1})')
            self._nav_q_gecis_yap('SONRAKI_ADAY', neden='Nav2 hedef reddi')
            return

        with self._durum_kilit:
            if self.durum_kodu != 'NAV_Q':
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self.aktif_nav_goal = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future) -> None:
        self.aktif_nav_goal = None
        if self.durum_kodu != 'NAV_Q':
            return

        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self._nav_q_gecis_yap('SON_YAKLASMA')
        elif status != 5:  # Başarısızlık (iptal / STATUS_CANCELED hariç, örn. STATUS_ABORTED)
            # Robotun harita pozu Q'ya q_kabul_xy ve q_kabul_yaw içindeyse son yaklaşmaya geç
            yakin_kabul = False
            try:
                rx, ry, ryaw = self.tf2d('map', 'base_footprint')
                y = self.adaylar[self.aktif_aday_idx]
                qx, qy, th = y.oncesi[0], y.oncesi[1], y.theta
                yakin, d, a = q_yakin_mi((rx, ry, ryaw), (qx, qy, th), self.q_kabul_xy, self.q_kabul_yaw)
                if yakin:
                    yakin_kabul = True
                    mesaj = f'NAV_Q yakin_kabul ({d * 100:.1f} cm, {math.degrees(a):.1f}°)'
                    self._nav_q_gecis_yap('SON_YAKLASMA', mesaj=mesaj)
                    return
            except Exception as e:
                self.get_logger().warn(f'NAV_Q yakin_kabul TF/hesaplama hatası: {e}')

            if not yakin_kabul:
                self.durum(f'NAV_Q başarısız durum {status} (aday {self.aktif_aday_idx + 1})')
                self._nav_q_gecis_yap('SONRAKI_ADAY', neden=f'Nav2 basarisiz ({status})')
        else:
            self.durum(f'NAV_Q iptal edildi durum {status} (aday {self.aktif_aday_idx + 1})')
            self._nav_q_gecis_yap('SONRAKI_ADAY', neden=f'Nav2 iptal ({status})')

    def _nav_q_gecis_yap(
        self,
        hedef_durum: str,
        mesaj: str | None = None,
        neden: str = '',
    ) -> bool:
        """NAV_Q durumundan tek ve korumalı çıkış noktası (N13 geçiş koruması).

        Parametreler:
            hedef_durum: 'SON_YAKLASMA' veya 'SONRAKI_ADAY'.
            mesaj: Geçiş öncesinde self.durum(...) ile yayınlanacak durum metni.
            neden: 'SONRAKI_ADAY' için başarısızlık açıklaması.

        Döndürür:
            bool: Geçiş bu çağrı ile yapıldıysa True, durum NAV_Q olmadığı için atlandıysa False.
        """
        with self._durum_kilit:
            yeni_durum, gecis_izni = nav_q_durum_gecisi(self.durum_kodu, hedef_durum)
            if not gecis_izni:
                return False
            self.durum_kodu = yeni_durum  # 'GECIS'

            # Varsa aktif Nav2 hedefini iptal et (iptal sonucunu bekleme)
            if self.aktif_nav_goal is not None:
                try:
                    self.aktif_nav_goal.cancel_goal_async()
                except Exception as e:
                    self.get_logger().debug(f'Nav2 hedef iptal hatası: {e}')
                self.aktif_nav_goal = None

        if mesaj:
            self.durum(mesaj)

        if hedef_durum == 'SON_YAKLASMA':
            self._son_yaklasma_baslat()
        elif hedef_durum == 'SONRAKI_ADAY':
            self._aday_basarisiz_isle(neden)

        return True

    def _aday_basarisiz_isle(self, neden: str) -> None:
        """Nav2 aşamasında aday başarısız olursa sonraki adaya geçer."""
        self.durum_kodu = 'GECIS'
        self.aktif_aday_idx += 1
        if self.aktif_aday_idx < len(self.adaylar):
            self.durum(f'NAV_Q sonraki adaya geçiliyor ({self.aktif_aday_idx + 1}/{len(self.adaylar)}): {neden}')
            self._nav_q_baslat()
        else:
            self.durum_kodu = 'HATA'
            self.durum('HATA tüm adaylar denendi')

    def _son_yaklasma_baslat(self) -> None:
        """SON_YAKLASMA: Kapalı çevrim denetleyiciyi başlatır."""
        y = self.adaylar[self.aktif_aday_idx]

        # TF map->odom: adayın th'sini ve nesne konumunu odom çerçevesine çevir
        try:
            t_mo = self.tf.lookup_transform('odom', 'map', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            tx = t_mo.transform.translation.x
            ty = t_mo.transform.translation.y
            yaw_mo = yaw_q(t_mo.transform.rotation)
        except Exception as e:
            self.durum_kodu = 'HATA'
            self.durum(f'HATA TF odom->map bulunamadı: {e}')
            return

        th_odom = map_yonu_odoma(y.theta, yaw_mo)

        nx, ny, _ = self.nesne_hedef_map  # type: ignore[misc]
        self.nesne_yedek_odom = map_noktasi_odoma((nx, ny), (tx, ty), yaw_mo)

        self.son_denetleyici = SonYaklasma(ayar=self.son_ayar, yaklasma_yaw_odom=th_odom)

        # Ölçüm kontrolü: taze /kavrama/nesne_algi var mı?
        if self.yeni_algi_var and self.son_algi_odom is not None:
            self.algisiz = False
            son_baslangic_mesaji = f'SON_YAKLASMA algı aktif, odom yönü {math.degrees(th_odom):+.1f}°'
        elif not self.algisiz_izinli:
            self.durum_kodu = 'HATA'
            self.durum('HATA nesne algısı yok (algisiz_izinli=False)')
            return
        else:
            self.algisiz = True
            self.tek_olcum_verildi = False
            son_baslangic_mesaji = (
                f'SON_YAKLASMA algisiz: odom yönü {math.degrees(th_odom):+.1f}°, '
                f'yedek map nesne ({self.nesne_yedek_odom[0]:+.2f}, {self.nesne_yedek_odom[1]:+.2f})'
            )

        self.t_son_yaklasma_baslangic = self.get_clock().now()
        self.son_durum_log_zamani = 0.0
        self._kalkan_dur_baslangic_t = None
        self._kalkan_bekleme_bildirildi = False
        self.durum_kodu = 'SON_YAKLASMA'
        self.durum(son_baslangic_mesaji)

    def _hazir_isle(self, bilgi: dict) -> None:
        """HAZIR: /kavrama/nesne_robotta yayını, P hatası ve yaklaşma süresi raporu."""
        now = self.get_clock().now()
        bas_ns = self.t_son_yaklasma_baslangic.nanoseconds if self.t_son_yaklasma_baslangic else now.nanoseconds
        sure = (now.nanoseconds - bas_ns) / 1e9

        nesne_odom = bilgi.get('nesne_odom')
        if nesne_odom is None:
            nesne_odom = self.nesne_yedek_odom

        # N2 nesne_robotta_son
        lx, ly = nesne_robotta_son(self.odom, nesne_odom)  # type: ignore[arg-type]
        nz = self.nesne_hedef_map[2] if self.nesne_hedef_map else 0.0

        out = PointStamped()
        out.header.frame_id = 'base_footprint'
        out.header.stamp = now.to_msg()
        out.point.x = float(lx)
        out.point.y = float(ly)
        out.point.z = float(nz)
        self.p_nesne.publish(out)

        # P hatası tahmini
        hedef = bilgi.get('hedef')
        if hedef is not None and self.odom is not None:
            px, py, th_g = hedef
            e_xy = math.hypot(self.odom[0] - px, self.odom[1] - py)
            e_yaw = abs(wrap(self.odom[2] - th_g))
        else:
            e_xy = 0.0
            e_yaw = 0.0

        self._son_nesne_robotta_xyz = (float(lx), float(ly), float(nz))
        self._son_nesne_robotta_msg = out
        self._son_hata_cm = float(e_xy * 100.0)
        self._son_yon_hatasi_deg = float(math.degrees(e_yaw))
        self._son_sure_s = float(sure)
        self._kalkan_dur_baslangic_t = None
        self._kalkan_bekleme_bildirildi = False
        self.durum_kodu = 'HAZIR'

        self.durum(
            f'HAZIR nesne robotta ({lx:+.3f}, {ly:+.3f}, {nz:.2f}) m; '
            f'P hatası {e_xy * 100:.1f} cm, {math.degrees(e_yaw):.1f}°; sure {sure:.2f} s'
        )

    def _kurtarma_baslat(self, neden: str) -> None:
        """KURTARMA: Son yaklaşma hatasında 0.20 m düz geri çekilme (en çok 2 kurtarma)."""
        self.durum_kodu = 'GECIS'
        if self.kurtarma_sayisi >= 2 or (self.aktif_aday_idx + 1) >= len(self.adaylar):
            self.durum_kodu = 'HATA'
            self.durum('HATA tüm adaylar denendi')
            return

        self.kurtarma_baslangic_odom = self.odom
        self._kalkan_dur_baslangic_t = None
        self._kalkan_bekleme_bildirildi = False
        self.durum_kodu = 'KURTARMA'
        self.durum(
            f'KURTARMA {neden}: odomla düz 0.20 m geri çekiliniyor '
            f'(kurtarma {self.kurtarma_sayisi + 1}/2)'
        )

    # --- 50 Hz Denetim Döngüsü ---

    def _dongu_50hz(self) -> None:
        if self.durum_kodu == 'NAV_Q':
            self._dongu_nav_q()
        elif self.durum_kodu == 'SON_YAKLASMA':
            self._dongu_son_yaklasma()
        elif self.durum_kodu == 'KURTARMA':
            self._dongu_kurtarma()

    def _dongu_nav_q(self) -> None:
        """NAV_Q 50 Hz döngüsü: erken kabul ve zaman aşımı denetimleri."""
        if self.durum_kodu != 'NAV_Q':
            return
        if not self.adaylar or self.aktif_aday_idx >= len(self.adaylar):
            return

        simdi = self._simdi_s()
        gecen_sure = simdi - self.t_nav_q_baslangic

        y = self.adaylar[self.aktif_aday_idx]
        qx, qy, th = y.oncesi[0], y.oncesi[1], y.theta

        try:
            rx, ry, ryaw = self.tf2d('map', 'base_footprint', timeout_sec=0.02)
            tf_alindi = True
        except Exception:
            tf_alindi = False
            rx, ry, ryaw = 0.0, 0.0, 0.0

        if tf_alindi:
            _, d, a = q_yakin_mi((rx, ry, ryaw), (qx, qy, th), self.q_erken_xy, self.q_erken_yaw)
            self._nav_q_kalan_mesafe_m = d

            # 4. Eylem geri bildiriminde NAV_Q sırasında kalan_mesafe_m = robot–Q mesafesi (5 Hz)
            self._eylem_geri_bildirim_yayinla(
                durum='NAV_Q',
                ayrinti=f'kalan {d:.2f} m',
                kalan_mesafe_m=d,
                zorla=False,
            )

            # Geçmişe ekle ve buda
            self.nav_q_gecmis.append((simdi, d, a))
            budama_t = simdi - max(self.q_erken_sure * 2.0, 5.0)
            self.nav_q_gecmis = [k for k in self.nav_q_gecmis if k[0] >= budama_t]

            # 1. Erken kabul: q_erken_xy ve q_erken_yaw içinde q_erken_sure boyunca kalırsa
            if erken_kabul_durumu(
                self.nav_q_gecmis,
                xy=self.q_erken_xy,
                yaw=self.q_erken_yaw,
                sure=self.q_erken_sure,
            ):
                mesaj = f'NAV_Q erken_kabul ({d * 100:.1f} cm, {math.degrees(a):.1f}°)'
                self._nav_q_gecis_yap('SON_YAKLASMA', mesaj=mesaj)
                return

        # 2. Zaman aşımı: nav_q_zaman_asimi içinde bitmezse
        if gecen_sure >= self.nav_q_zaman_asimi:
            yakin = False
            d_k, a_k = 0.0, 0.0
            if tf_alindi:
                yakin, d_k, a_k = q_yakin_mi(
                    (rx, ry, ryaw),
                    (qx, qy, th),
                    self.q_kabul_xy,
                    self.q_kabul_yaw,
                )

            if yakin:
                mesaj = f'NAV_Q yakin_kabul ({d_k * 100:.1f} cm, {math.degrees(a_k):.1f}°)'
                self._nav_q_gecis_yap('SON_YAKLASMA', mesaj=mesaj)
            else:
                self.durum(f'NAV_Q zaman aşımı ({self.nav_q_zaman_asimi:.0f} s)')
                self._nav_q_gecis_yap(
                    'SONRAKI_ADAY',
                    neden=f'zaman aşımı ({self.nav_q_zaman_asimi:.0f} s)',
                )
            return

    def _dongu_son_yaklasma(self) -> None:
        if self.durum_kodu != 'SON_YAKLASMA':
            return
        if self.odom is None or self.son_denetleyici is None:
            return

        now = self.get_clock().now()
        t = now.nanoseconds / 1e9

        # Hareketli engel kalkanı bekleme kontrolü ve zaman aşımı ötelemesi (N24)
        kalkan_bekliyor = bool(self.kalkan_uy and kalkan_dur_mu(self.kalkan_durum))
        if self.kalkan_uy:
            asildi, self._kalkan_dur_baslangic_t, gecen_s = kalkan_bekleme_kontrolu(
                self._kalkan_dur_baslangic_t,
                t,
                self.kalkan_durum,
                self.kalkan_maks_bekleme,
            )
            if asildi:
                self.durum_kodu = 'HATA'
                self._durdur()
                self.durum('HATA:kalkan_bekleme')
                return

            if kalkan_bekliyor and self.son_denetleyici.onceki_t is not None:
                dt_kalkan = max(0.0, t - self.son_denetleyici.onceki_t)
                if dt_kalkan > 0.0:
                    self.son_denetleyici.duraklat(dt_kalkan)
                    if self.t_son_yaklasma_baslangic is not None:
                        self.t_son_yaklasma_baslangic += rclpy.duration.Duration(
                            nanoseconds=int(dt_kalkan * 1e9)
                        )
        else:
            self._kalkan_dur_baslangic_t = None

        # Ölçüm belirleme: /kavrama/nesne_algi geldiyse kullan, yoksa başlangıçta map konumu tek ölçüm
        olcum_robot = None
        if self.yeni_algi_var and self.son_algi_odom is not None:
            self.yeni_algi_var = False
            self.algisiz = False
            olcum_robot = nesne_robotta_son(self.odom, self.son_algi_odom)
        elif self.algisiz and not self.tek_olcum_verildi:
            self.tek_olcum_verildi = True
            if self.nesne_yedek_odom is not None:
                olcum_robot = nesne_robotta_son(self.odom, self.nesne_yedek_odom)

        # Güvenlik mesafesi (scan_noktalari_robotta, nesne_robotta, ±25° koni, 0.08 m hariç, tampon 0.161 m)
        guvenlik_m = None
        if self.tarama_noktalari is not None and len(self.tarama_noktalari) > 0:
            tahmin_odom = self.son_denetleyici.nesne_tahmin.konum
            if tahmin_odom is None:
                tahmin_odom = self.nesne_yedek_odom
            nesne_robotta_tahmin = nesne_robotta_son(self.odom, tahmin_odom) if tahmin_odom else None
            guvenlik_m = guvenlik_mesafesi(
                self.tarama_noktalari,
                nesne_robotta=nesne_robotta_tahmin,
                koni=math.radians(25),
                haric_r=0.08,
                tampon=0.161,
            )

        # Denetleyici adımı
        v, w, adim_durum, bilgi = self.son_denetleyici.adim(
            robot_odom_pose=self.odom,
            t=t,
            olcum_robot_xy=olcum_robot,
            guvenlik_mesafe=guvenlik_m,
        )

        if adim_durum == 'TAMAM':
            self.durum_kodu = 'GECIS'
            self._durdur()
            self._hazir_isle(bilgi)
            return
        elif adim_durum.startswith('HATA'):
            if kalkan_bekliyor:
                # Kalkan DUR durumundayken engelden kaynaklı HATA durumlarında
                # KURTARMA yerine bekleme sürdürülür (insan önünde geçiş bekleniyor).
                v, w = 0.0, 0.0
            else:
                self.durum_kodu = 'GECIS'
                self._durdur()
                self._kurtarma_baslat(adim_durum)
                return

        # Kalkan hız filtreleme (kavramanın yayınladığı her komut kalkan_uygula'dan geçer)
        v_cikis, w_cikis = (float(v), float(w))
        if self.kalkan_uy:
            v_cikis, w_cikis = kalkan_uygula(v_cikis, w_cikis, self.kalkan_durum)

        # Komut yayınla
        cmd = Twist()
        cmd.linear.x = float(v_cikis)
        cmd.angular.z = float(w_cikis)
        self.p_cmd.publish(cmd)

        rho = float(bilgi.get('rho', float('nan')))
        algisiz_bilgi = 'algisiz ' if self.algisiz else ''
        kalkan_bilgi = 'KALKAN_BEKLE ' if kalkan_bekliyor else ''
        ayrinti_metni = f'{kalkan_bilgi}{algisiz_bilgi}v={v_cikis:.2f} w={w_cikis:.2f} rho={rho:.2f} m'

        # Eylem geri bildirimi: SON_YAKLASMA durumunda 5 Hz (kalan_mesafe_m = rho)
        self._eylem_geri_bildirim_yayinla(
            durum='SON_YAKLASMA',
            ayrinti=ayrinti_metni,
            kalan_mesafe_m=rho,
            zorla=False,
        )

        yeni_kalkan_dur_gecisi = kalkan_bekliyor and not self._kalkan_bekleme_bildirildi
        if kalkan_bekliyor:
            self._kalkan_bekleme_bildirildi = True
        else:
            self._kalkan_bekleme_bildirildi = False

        # 1 Hz aralıkla periyodik durum yayını (/kavrama/durum) veya DUR başlangıcında anında yayın
        if yeni_kalkan_dur_gecisi or (t - self.son_durum_log_zamani) > 1.0:
            self.son_durum_log_zamani = t
            self.durum(f'SON_YAKLASMA {ayrinti_metni}')

    def _dongu_kurtarma(self) -> None:
        if self.durum_kodu != 'KURTARMA':
            return
        if self.odom is None or self.kurtarma_baslangic_odom is None:
            return

        now_s = self._simdi_s()
        if self.kalkan_uy:
            asildi, self._kalkan_dur_baslangic_t, _ = kalkan_bekleme_kontrolu(
                self._kalkan_dur_baslangic_t,
                now_s,
                self.kalkan_durum,
                self.kalkan_maks_bekleme,
            )
            if asildi:
                self.durum_kodu = 'HATA'
                self._durdur()
                self.durum('HATA:kalkan_bekleme')
                return
        else:
            self._kalkan_dur_baslangic_t = None

        x, y, _ = self.odom
        x0, y0, _ = self.kurtarma_baslangic_odom
        kat_edilen = math.hypot(x - x0, y - y0)

        # Odomla düz 0.20 m geri; durum geçiş koruması ile aday atlama önlenir
        yeni_durum, yeni_kurtarma, yeni_idx, bitti = kurtarma_durum_gecisi(
            self.durum_kodu, kat_edilen, self.kurtarma_sayisi, self.aktif_aday_idx, 0.20
        )

        if not bitti:
            v_kurtarma = -0.08
            w_kurtarma = 0.0
            if self.kalkan_uy:
                v_kurtarma, w_kurtarma = kalkan_uygula(v_kurtarma, w_kurtarma, self.kalkan_durum)

            cmd = Twist()
            cmd.linear.x = float(v_kurtarma)
            cmd.angular.z = float(w_kurtarma)
            self.p_cmd.publish(cmd)
        else:
            # Durumu aday artırma/NAV_Q başlatmadan ÖNCE geçiş durumuna al
            self.durum_kodu = yeni_durum  # 'GECIS'
            self._durdur()
            self._kalkan_dur_baslangic_t = None
            self.kurtarma_sayisi = yeni_kurtarma
            self.aktif_aday_idx = yeni_idx
            self.durum(
                f'KURTARMA tamamlandı; sonraki aday {self.aktif_aday_idx + 1}/{len(self.adaylar)} NAV_Q başlatılıyor'
            )
            self._nav_q_baslat()


def main(args=None) -> None:
    rclpy.init(args=args)
    n = KavramaDugumu()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
