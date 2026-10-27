#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kavrama_bench.py — AgileX LIMO kavrama yaklaşmasının simülasyonda ölçümü (Görev N6).

Simülasyon (Gazebo Classic), Nav2 ve kavrama düğümü (kavrama_dugumu.py) çalışırken
koşar; onları başlatmaz. Hedef nesne konumlarını sırayla `/kavrama/hedef` üzerinden
yayınlar, yaklaşma sürecini izler ve Gazebo'daki gerçek robot pozunu (`/odom_gercek`)
kullanarak yaklaşma doğruluğunu, yön hatasını ve bildirilen nesne pozisyonu farkını ölçer.

Konular:
  Yayınlanan:
    /kavrama/hedef            geometry_msgs/PointStamped (frame: map, hedef konumu)
  Dinlenen:
    /kavrama/durum            std_msgs/String (PLAN, NAV, HIZALA, YAKLAS / SON_YAKLASMA, HAZIR, HATA)
    /kavrama/nesne_robotta    geometry_msgs/PointStamped (frame: base_footprint, kol tarafına bildirilen)
    /odom_gercek              nav_msgs/Odometry (Gazebo zemin gerçeği poz: dünya = harita varsayımı)

Kabul Ölçütü:
  |Δx| < 2 cm, |Δy| < 2 cm ve |yön hatası| < 3° → 'KAVRANABILIR'

Kullanım:
  python3 kavrama_bench.py --hedefler kavrama_hedefler_lab_oda.yaml --cikti sonuc.yaml --zaman_asimi 180
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty, String
except ImportError:
    # Saf fonksiyonların ROS'suz ortamda da pytest ile test edilebilmesi için
    rclpy = None
    Node = object


# ==============================================================================
# Saf Matematik ve Dönüşüm Fonksiyonları (ROS Bağımsız)
# ==============================================================================

def aci_sar(aci_rad: float) -> float:
    """Açıyı [-pi, pi] aralığına sarar (wrap)."""
    return math.atan2(math.sin(aci_rad), math.cos(aci_rad))


def yaw_q(q) -> float:
    """Dörtlüden (quaternion) 2B yaw (z rotasyonu) açısını hesaplar (radyan)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def robot_cercevesine_donustur(robot_x: float, robot_y: float, robot_yaw: float,
                               nesne_x: float, nesne_y: float) -> tuple:
    """Dünya/harita çerçevesindeki nesne konumunu robot (base_footprint) çerçevesine çevirir.

    Robot x ekseni ileriye, y ekseni sol tarafa doğrudur.
    Döndürme: R_inv * (P_nesne - P_robot)
    Döndürür: (x_r, y_r) [metre]
    """
    dx = nesne_x - robot_x
    dy = nesne_y - robot_y
    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)
    x_r = c * dx + s * dy
    y_r = -s * dx + c * dy
    return float(x_r), float(y_r)


def kavrama_hatasi_hesapla(x_r: float, y_r: float,
                           d_kavrama: float = 0.22, yanal: float = 0.0) -> dict:
    """Nesnenin robot çerçevesindeki gerçek konumu ile hedef kavrama noktası arasındaki hatayı hesaplar.

    Hedef kavrama noktası: x = d_kavrama (0.22 m), y = yanal (0.0 m).
    Döndürür:
      dx_cm: x ekseni hatası (cm)
      dy_cm: y ekseni (yanal) hatası (cm)
      toplam_cm: 2B Öklid mesafesi hatası (cm)
    """
    dx_m = x_r - d_kavrama
    dy_m = y_r - yanal
    toplam_m = math.hypot(dx_m, dy_m)
    return {
        'dx_cm': float(dx_m * 100.0),
        'dy_cm': float(dy_m * 100.0),
        'toplam_cm': float(toplam_m * 100.0),
    }


def yon_hatasi_hesapla(robot_x: float, robot_y: float, robot_yaw: float,
                       nesne_x: float, nesne_y: float) -> float:
    """Robotun yönelimi ile robot->nesne görüş hattı arasındaki açıyı (derece) hesaplar.

    Pozitif açı robotun nesnenin soluna doğru baktığını, negatif ise sağına doğru baktığını belirtir.
    Döndürür: açısal hata (derece)
    """
    dx = nesne_x - robot_x
    dy = nesne_y - robot_y
    if math.hypot(dx, dy) < 1e-6:
        return 0.0
    hedef_yon = math.atan2(dy, dx)
    fark_rad = aci_sar(robot_yaw - hedef_yon)
    return float(math.degrees(fark_rad))


def bildirilen_fark_hesapla(bildirilen_x: float, bildirilen_y: float,
                            gercek_x_r: float, gercek_y_r: float) -> dict:
    """`/kavrama/nesne_robotta` bildirimi ile Gazebo gerçek konumu arasındaki farkı hesaplar (milimetre)."""
    dx_mm = (bildirilen_x - gercek_x_r) * 1000.0
    dy_mm = (bildirilen_y - gercek_y_r) * 1000.0
    toplam_mm = math.hypot(dx_mm, dy_mm)
    return {
        'dx_mm': float(dx_mm),
        'dy_mm': float(dy_mm),
        'toplam_mm': float(toplam_mm),
    }


def kavranabilir_mi(dx_cm: float, dy_cm: float, yon_hatasi_deg: float,
                    tol_mesafe_cm: float = 2.0, tol_yon_deg: float = 3.0) -> bool:
    """Kabul ölçütü bayrağı kontrolü: |Δx| < 2 cm, |Δy| < 2 cm ve |yön| < 3°."""
    return (abs(dx_cm) < tol_mesafe_cm and
            abs(dy_cm) < tol_mesafe_cm and
            abs(yon_hatasi_deg) < tol_yon_deg)


def hedefler_yaml_oku(dosya_yolu: str) -> list:
    """Hedefler YAML dosyasını okur ve doğrulanmış hedef listesini döndürür."""
    if not os.path.isfile(dosya_yolu):
        raise FileNotFoundError(f"Hedef dosyası bulunamadı: {dosya_yolu}")
    with open(dosya_yolu, 'r', encoding='utf-8') as f:
        veri = yaml.safe_load(f)

    if isinstance(veri, dict) and 'hedefler' in veri:
        liste = veri['hedefler']
    elif isinstance(veri, list):
        liste = veri
    else:
        raise ValueError(f"Geçersiz hedef dosyası formatı (liste veya 'hedefler' bekleniyor): {dosya_yolu}")

    hedefler = []
    for i, h in enumerate(liste):
        ad = str(h.get('ad', f'hedef_{i+1}'))
        x = float(h['x'])
        y = float(h['y'])
        z = float(h.get('z', 0.0))
        yon = h.get('yon_derece', None)
        yon_deg = float(yon) if yon is not None else None
        beklenen = h.get('beklenen', None)
        hedefler.append({
            'ad': ad,
            'x': x,
            'y': y,
            'z': z,
            'yon_derece': yon_deg,
            'beklenen': beklenen,
        })
    return hedefler


def istatistik_ozet(sonuclar: list) -> dict:
    """Test sonuçlarından medyan, maks ve başarı istatistiklerini hesaplar."""
    toplam = len(sonuclar)
    if toplam == 0:
        return {
            'toplam_hedef': 0,
            'basarili_sayisi': 0,
            'basari_orani': 0.0,
            'kavranabilir_sayisi': 0,
            'kavranabilir_orani': 0.0,
            'kavrama_hatasi_cm': {'medyan': 0.0, 'maks': 0.0},
            'yon_hatasi_derece': {'medyan': 0.0, 'maks': 0.0},
            'bildirilen_gercek_fark_mm': {'medyan': None, 'maks': None},
            'tum_hedefler_kavranabilir': False,
        }

    basarili_sonuclar = [
        s for s in sonuclar
        if (s.get('sonuc') == 'HAZIR' and s.get('beklenen') != 'HATA')
        or (s.get('sonuc') == 'HATA' and s.get('beklenen') == 'HATA')
    ]
    basarili_sayisi = len(basarili_sonuclar)
    basari_orani = basarili_sayisi / float(toplam)

    # İstatistikler için öncelikli olarak başarıyla hedefe varan (HAZIR) sonuçları kullan
    hazir_sonuclar = [s for s in sonuclar if s.get('sonuc') == 'HAZIR']
    orneklem = hazir_sonuclar if hazir_sonuclar else sonuclar

    # Kavrama mesafe hataları (cm)
    k_hatalari = [
        s['kavrama_hatasi']['toplam_cm']
        for s in orneklem
        if s.get('kavrama_hatasi') is not None
    ]
    k_medyan = float(np.median(k_hatalari)) if k_hatalari else 0.0
    k_maks = float(np.max(k_hatalari)) if k_hatalari else 0.0

    # Yön hataları (mutlak derece)
    y_hatalari = [
        abs(s['yon_hatasi_derece'])
        for s in orneklem
        if s.get('yon_hatasi_derece') is not None
    ]
    y_medyan = float(np.median(y_hatalari)) if y_hatalari else 0.0
    y_maks = float(np.max(y_hatalari)) if y_hatalari else 0.0

    # Bildirilen - gerçek farkları (mm)
    b_farklar = [
        s['bildirilen_fark']['toplam_mm']
        for s in orneklem
        if s.get('bildirilen_fark') is not None
    ]
    b_medyan = float(np.median(b_farklar)) if b_farklar else None
    b_maks = float(np.max(b_farklar)) if b_farklar else None

    kavranabilir_sayisi = sum(
        1 for s in sonuclar
        if s.get('kabul') in ('KAVRANABILIR', 'BEKLENEN_HATA')
    )
    kavranabilir_orani = kavranabilir_sayisi / float(toplam)

    return {
        'toplam_hedef': toplam,
        'basarili_sayisi': basarili_sayisi,
        'basari_orani': round(basari_orani, 4),
        'kavranabilir_sayisi': kavranabilir_sayisi,
        'kavranabilir_orani': round(kavranabilir_orani, 4),
        'kavrama_hatasi_cm': {
            'medyan': round(k_medyan, 2),
            'maks': round(k_maks, 2),
        },
        'yon_hatasi_derece': {
            'medyan': round(y_medyan, 2),
            'maks': round(y_maks, 2),
        },
        'bildirilen_gercek_fark_mm': {
            'medyan': round(b_medyan, 2) if b_medyan is not None else None,
            'maks': round(b_maks, 2) if b_maks is not None else None,
        },
        'tum_hedefler_kavranabilir': (kavranabilir_sayisi == toplam),
    }


# ==============================================================================
# ROS 2 Benchmark Düğümü
# ==============================================================================

if rclpy is not None:

    class KavramaBenchNode(Node):
        """Kavrama yaklaşmasını simülasyonda test eden ROS 2 düğümü."""

        def __init__(self, use_sim_time: bool = True):
            super().__init__(
                'kavrama_bench',
                parameter_overrides=[
                    Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
                ],
            )
            # Yayıncı: /kavrama/hedef (PointStamped, frame map)
            self.pub_hedef = self.create_publisher(PointStamped, '/kavrama/hedef', 10)
            # Yayıncı: /kavrama/iptal (Empty, zaman aşımında görevi durdurmak için)
            self.pub_iptal = self.create_publisher(Empty, '/kavrama/iptal', 10)

            # Abonelikler
            # 1. /kavrama/durum (String)
            self.sub_durum = self.create_subscription(
                String, '/kavrama/durum', self._cb_durum, 50
            )
            # 2. /kavrama/nesne_robotta (PointStamped, frame base_footprint)
            self.sub_nesne = self.create_subscription(
                PointStamped, '/kavrama/nesne_robotta', self._cb_nesne, 10
            )
            # 3. /odom_gercek (Odometry) - Gazebo zemin gerçeği poz
            self.sub_odom = self.create_subscription(
                Odometry, '/odom_gercek', self._cb_odom, qos_profile_sensor_data
            )

            # Durum Takip Değişkenleri
            self.son_gercek_odom = None          # (x, y, z, yaw, stamp_s)
            self.son_bildirilen_nesne = None     # (x, y, z, stamp_s)
            self.durum_gecmisi = []              # [(zaman_s, durum_str)]
            self.aktif_hedef_adi = None
            self.sonuc = None                    # 'HAZIR', 'HATA', 'ZAMAN_ASIMI'
            self.son_yaklasma_zamani = None
            self.bitis_zamani = None
            self.bitis_sim_zamani = None
            self.bitis_wall_zamani = None

        def sim_zamani(self) -> float:
            """Geçerli simülasyon saatini (saniye) döndürür; saat yoksa sistem saati."""
            t_ns = self.get_clock().now().nanoseconds
            if t_ns > 0:
                return t_ns * 1e-9
            return time.time()

        def _cb_odom(self, msg: Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = yaw_q(q)
            t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t_s == 0.0:
                t_s = self.sim_zamani()
            self.son_gercek_odom = (float(p.x), float(p.y), float(p.z), float(yaw), t_s)

        def _cb_nesne(self, msg: PointStamped):
            t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t_s == 0.0:
                t_s = self.sim_zamani()
            self.son_bildirilen_nesne = (
                float(msg.point.x),
                float(msg.point.y),
                float(msg.point.z),
                t_s,
            )

        def _cb_durum(self, msg: String):
            t_s = self.sim_zamani()
            text = msg.data.strip()
            self.durum_gecmisi.append((t_s, text))
            self.get_logger().info(f"[{self.aktif_hedef_adi}] DURUM: {text}")

            # Son yaklaşma başlangıcı tespiti: 'SON_YAKLASMA' veya 'YAKLAS'
            if text.startswith('SON_YAKLASMA'):
                if self.son_yaklasma_zamani is None:
                    self.son_yaklasma_zamani = t_s
            elif text.startswith('YAKLAS') and self.son_yaklasma_zamani is None:
                self.son_yaklasma_zamani = t_s

            # Bitiş durumları: 'HAZIR' veya 'HATA'
            if text.startswith('HAZIR'):
                if self.sonuc is None:
                    self.sonuc = 'HAZIR'
                    self.bitis_zamani = t_s
                    self.bitis_sim_zamani = t_s
                    self.bitis_wall_zamani = time.time()
            elif text.startswith('HATA'):
                if self.sonuc is None:
                    self.sonuc = 'HATA'
                    self.bitis_zamani = t_s
                    self.bitis_sim_zamani = t_s
                    self.bitis_wall_zamani = time.time()

        def odom_bekle(self, timeout_sec: float = 10.0) -> bool:
            """İlk /odom_gercek mesajının gelmesini bekler."""
            t0 = time.time()
            while time.time() - t0 < timeout_sec and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.son_gercek_odom is not None:
                    return True
            return False

        def bekle(self, bekleme_suresi_s: float):
            """Düğüm olay döngüsünü işleterek belirtilen süre kadar bekler."""
            t0 = self.sim_zamani()
            t_wall0 = time.time()
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                # Hem simülasyon hem de gerçek zaman sınırını kontrol et
                dt_sim = self.sim_zamani() - t0
                dt_wall = time.time() - t_wall0
                if dt_sim >= bekleme_suresi_s or dt_wall >= (bekleme_suresi_s * 2.5):
                    break

        def hedef_yurut(self, hedef: dict, zaman_asimi: float = 180.0,
                        d_kavrama: float = 0.22, yanal: float = 0.0) -> dict:
            """Tek bir hedefi yayınlar, bitişi veya zaman aşımını bekler ve ölçümleri kaydeder."""
            ad = hedef['ad']
            self.aktif_hedef_adi = ad
            self.sonuc = None
            self.son_yaklasma_zamani = None
            self.bitis_zamani = None
            self.bitis_sim_zamani = None
            self.bitis_wall_zamani = None
            self.durum_gecmisi = []
            self.son_bildirilen_nesne = None

            # Hedef mesajını hazırla ve yayınla
            pt = PointStamped()
            pt.header.frame_id = 'map'
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = float(hedef['x'])
            pt.point.y = float(hedef['y'])
            pt.point.z = float(hedef['z'])

            baslangic_zamani = self.sim_zamani()
            baslangic_wall = time.time()
            self.get_logger().info(
                f"Hedef başlatılıyor: '{ad}' -> ({pt.point.x:.2f}, {pt.point.y:.2f}, {pt.point.z:.2f})"
            )
            self.pub_hedef.publish(pt)

            # Yanıt bekleme döngüsü
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                if self.sonuc is not None:
                    break

                gecen_sure = self.sim_zamani() - baslangic_zamani
                gecen_wall = time.time() - baslangic_wall

                if gecen_sure > zaman_asimi or gecen_wall > (zaman_asimi * 2.0):
                    self.get_logger().warn(
                        f"Hedef '{ad}' zaman aşımına uğradı ({zaman_asimi:.1f}s). /kavrama/iptal yayınlanıyor..."
                    )
                    self.sonuc = 'ZAMAN_ASIMI'
                    self.bitis_zamani = self.sim_zamani()
                    self.bitis_sim_zamani = self.sim_zamani()
                    self.bitis_wall_zamani = time.time()
                    self.pub_iptal.publish(Empty())

                    # Kavrama düğümünden 'HATA' durumunu en çok 10 s bekle
                    t_iptal_sim = self.sim_zamani()
                    t_iptal_wall = time.time()
                    while rclpy.ok():
                        rclpy.spin_once(self, timeout_sec=0.05)
                        if self.durum_gecmisi and self.durum_gecmisi[-1][1].startswith('HATA'):
                            self.get_logger().info("İptal sonrası 'HATA' durumu alındı.")
                            break
                        dt_isim = self.sim_zamani() - t_iptal_sim
                        dt_iwall = time.time() - t_iptal_wall
                        if dt_isim >= 10.0 or dt_iwall >= 10.0:
                            self.get_logger().warn("İptal sonrası 'HATA' durumu 10 saniye içinde gelmedi.")
                            break
                    break

            # Olası geciken bildirim mesajlarını (/kavrama/nesne_robotta) yakalamak için kısa bekleme
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)

            bitis_zamani = self.bitis_sim_zamani or self.bitis_zamani or self.sim_zamani()
            bitis_wall = self.bitis_wall_zamani or time.time()

            sure_sim_s = max(0.0, bitis_zamani - baslangic_zamani)
            sure_duvar_s = max(0.0, bitis_wall - baslangic_wall)

            # Sim zamanı tutarlı şekilde ilerlemişse sim zamanı, aksi halde duvar saati
            toplam_sure_s = sure_sim_s if sure_sim_s > 0.05 else sure_duvar_s

            son_yaklasma_suresi_s = None
            if self.son_yaklasma_zamani is not None and self.sonuc == 'HAZIR':
                son_yaklasma_suresi_s = max(0.0, bitis_zamani - self.son_yaklasma_zamani)

            # Robotun Gazebo'daki gerçek pozu (/odom_gercek)
            if self.son_gercek_odom is not None:
                rx, ry, rz, ryaw, odom_t = self.son_gercek_odom
            else:
                self.get_logger().error("UYARI: /odom_gercek verisi bulunamadı!")
                rx, ry, rz, ryaw, odom_t = 0.0, 0.0, 0.0, 0.0, 0.0

            # Gerçek robot çerçevesinde nesne konumu (x_r, y_r)
            x_r, y_r = robot_cercevesine_donustur(rx, ry, ryaw, hedef['x'], hedef['y'])

            # Kavrama hedefi (d_kavrama, yanal) ile farkı
            kavrama_hatasi = kavrama_hatasi_hesapla(x_r, y_r, d_kavrama=d_kavrama, yanal=yanal)

            # Robot yönünün robot->nesne doğrultusuna göre hatası (derece)
            yon_hatasi_deg = yon_hatasi_hesapla(rx, ry, ryaw, hedef['x'], hedef['y'])

            # /kavrama/nesne_robotta bildirilen ↔ gerçek fark
            bildirilen_detay = None
            fark_detay = None
            if self.son_bildirilen_nesne is not None:
                bx, by, bz, bt = self.son_bildirilen_nesne
                bildirilen_detay = {'x': bx, 'y': by, 'z': bz}
                fark_detay = bildirilen_fark_hesapla(bx, by, x_r, y_r)

            # Kabul ölçütü: |Δx|, |Δy| < 2 cm ve |yön| < 3° → 'KAVRANABILIR'
            # Olumsuz test: beklenen == 'HATA' olan hedefte HATA sonucunu BAŞARILI say ('BEKLENEN_HATA')
            beklenen = hedef.get('beklenen')
            if beklenen == 'HATA':
                kabul_bayragi = 'BEKLENEN_HATA' if self.sonuc == 'HATA' else 'KAVRANAMAZ'
            else:
                kabul_edildi = (
                    self.sonuc == 'HAZIR' and
                    kavranabilir_mi(
                        kavrama_hatasi['dx_cm'],
                        kavrama_hatasi['dy_cm'],
                        yon_hatasi_deg,
                        tol_mesafe_cm=2.0,
                        tol_yon_deg=3.0,
                    )
                )
                kabul_bayragi = 'KAVRANABILIR' if kabul_edildi else 'KAVRANAMAZ'

            durum_listesi = [
                {'zaman_s': round(t, 2), 'mesaj': m}
                for t, m in self.durum_gecmisi
            ]

            return {
                'ad': ad,
                'hedef_konum': {'x': hedef['x'], 'y': hedef['y'], 'z': hedef['z']},
                'beklenen': beklenen,
                'sonuc': self.sonuc,
                'kabul': kabul_bayragi,
                'toplam_sure_s': round(toplam_sure_s, 2),
                'sure_sim_s': round(sure_sim_s, 2),
                'sure_duvar_s': round(sure_duvar_s, 2),
                'son_yaklasma_suresi_s': round(son_yaklasma_suresi_s, 2) if son_yaklasma_suresi_s is not None else None,
                'robot_gercek_poz': {
                    'x': round(rx, 4),
                    'y': round(ry, 4),
                    'z': round(rz, 4),
                    'yaw_deg': round(math.degrees(ryaw), 2),
                },
                'nesne_gercek_robotta': {
                    'x_r': round(x_r, 4),
                    'y_r': round(y_r, 4),
                },
                'kavrama_hatasi': {
                    'dx_cm': round(kavrama_hatasi['dx_cm'], 2),
                    'dy_cm': round(kavrama_hatasi['dy_cm'], 2),
                    'toplam_cm': round(kavrama_hatasi['toplam_cm'], 2),
                },
                'yon_hatasi_derece': round(yon_hatasi_deg, 2),
                'bildirilen_robotta': bildirilen_detay,
                'bildirilen_fark': {
                    'dx_mm': round(fark_detay['dx_mm'], 1),
                    'dy_mm': round(fark_detay['dy_mm'], 1),
                    'toplam_mm': round(fark_detay['toplam_mm'], 1),
                } if fark_detay is not None else None,
                'durum_gecmisi': durum_listesi,
            }

else:
    class KavramaBenchNode:
        pass


# ==============================================================================
# Raporlama ve Terminal Tablosu
# ==============================================================================

def tablo_yazdir(sonuclar: list, ozet: dict):
    """Benchmark sonuçlarını hizalı bir metin tablosu olarak konsola basar."""
    col_w = (10, 8, 9, 13, 8, 8, 12, 8, 10, 14)
    line_len = sum(col_w) + len(col_w) - 1

    print("\n" + "=" * line_len)
    print(" KAVRAMA YAKLAŞMASI SİMÜLASYON BENCHMARK SONUÇLARI (Görev N6)")
    print("=" * line_len)

    headers = (
        f"{'Hedef':<{col_w[0]}} "
        f"{'Sonuç':<{col_w[1]}} "
        f"{'Süre(s)':<{col_w[2]}} "
        f"{'SonYakl(s)':<{col_w[3]}} "
        f"{'Δx(cm)':<{col_w[4]}} "
        f"{'Δy(cm)':<{col_w[5]}} "
        f"{'Mesafe(cm)':<{col_w[6]}} "
        f"{'Yön(°)':<{col_w[7]}} "
        f"{'Fark(mm)':<{col_w[8]}} "
        f"{'Kabul':<{col_w[9]}}"
    )
    print(headers)
    print("-" * line_len)

    for s in sonuclar:
        ad = str(s['ad'])[:col_w[0]]
        sonuc = str(s['sonuc'])[:col_w[1]]
        sure = f"{s['toplam_sure_s']:.1f}"
        son_yakl = f"{s['son_yaklasma_suresi_s']:.1f}" if s.get('son_yaklasma_suresi_s') is not None else "-"
        kh = s.get('kavrama_hatasi') or {}
        dx = f"{kh.get('dx_cm', 0.0):+.1f}"
        dy = f"{kh.get('dy_cm', 0.0):+.1f}"
        toplam_cm = f"{kh.get('toplam_cm', 0.0):.1f}"
        yon = f"{s.get('yon_hatasi_derece', 0.0):+.1f}"

        bf = s.get('bildirilen_fark')
        fark_mm = f"{bf['toplam_mm']:.1f}" if bf is not None else "-"
        kabul = str(s.get('kabul', '-'))[:col_w[9]]

        row = (
            f"{ad:<{col_w[0]}} "
            f"{sonuc:<{col_w[1]}} "
            f"{sure:<{col_w[2]}} "
            f"{son_yakl:<{col_w[3]}} "
            f"{dx:<{col_w[4]}} "
            f"{dy:<{col_w[5]}} "
            f"{toplam_cm:<{col_w[6]}} "
            f"{yon:<{col_w[7]}} "
            f"{fark_mm:<{col_w[8]}} "
            f"{kabul:<{col_w[9]}}"
        )
        print(row)

    print("=" * line_len)
    print(" ÖZET İSTATİSTİKLER:")
    toplam = ozet['toplam_hedef']
    basarili = ozet['basarili_sayisi']
    kavranabilir = ozet['kavranabilir_sayisi']
    b_orani = ozet['basari_orani'] * 100.0
    k_orani = ozet['kavranabilir_orani'] * 100.0

    print(f"  Başarı Oranı              : {basarili}/{toplam} (%{b_orani:.1f})")
    print(f"  Kavranabilir Oranı        : {kavranabilir}/{toplam} (%{k_orani:.1f})")

    k_med = ozet['kavrama_hatasi_cm']['medyan']
    k_max = ozet['kavrama_hatasi_cm']['maks']
    print(f"  Kavrama Hatası (cm)       : Medyan = {k_med:.2f} cm, Maks = {k_max:.2f} cm (Kriter: < 2.0 cm)")

    y_med = ozet['yon_hatasi_derece']['medyan']
    y_max = ozet['yon_hatasi_derece']['maks']
    print(f"  Yön Hatası (derece)       : Medyan = {y_med:.2f}°, Maks = {y_max:.2f}° (Kriter: < 3.0°)")

    bf_med = ozet['bildirilen_gercek_fark_mm']['medyan']
    bf_max = ozet['bildirilen_gercek_fark_mm']['maks']
    bf_med_str = f"{bf_med:.1f} mm" if bf_med is not None else "Yok"
    bf_max_str = f"{bf_max:.1f} mm" if bf_max is not None else "Yok"
    print(f"  Bildirilen-Gerçek Farkı   : Medyan = {bf_med_str}, Maks = {bf_max_str}")

    genel_durum = "KAVRANABILIR (GEÇTİ)" if ozet['tum_hedefler_kavranabilir'] else "KALDI / KAVRANAMAZ"
    print(f"  Genel Sonuç               : {genel_durum}")
    print("=" * line_len + "\n")


def yaml_kaydet(dosya_yolu: str, sonuclar: list, ozet: dict, parametreler: dict):
    """Benchmark sonuçlarını ve özetini yapısal bir YAML dosyasına kaydeder."""
    abs_yol = os.path.abspath(dosya_yolu)
    os.makedirs(os.path.dirname(abs_yol), exist_ok=True)

    rapor = {
        'tarih': datetime.now().isoformat(),
        'parametreler': parametreler,
        'ozet': ozet,
        'hedefler': sonuclar,
    }

    with open(abs_yol, 'w', encoding='utf-8') as f:
        yaml.safe_dump(rapor, f, allow_unicode=True, sort_keys=False)
    print(f"Benchmark sonuçları YAML olarak kaydedildi: {abs_yol}")


# ==============================================================================
# Ana Yürütücü (Main)
# ==============================================================================

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="AgileX LIMO Kavrama Yaklaşması Simülasyon Benchmark Aracı"
    )
    parser.add_argument(
        '--hedefler',
        type=str,
        required=True,
        help="Hedefler listesini içeren YAML dosyasının yolu (ad, x, y, z, isteğe bağlı yon_derece)",
    )
    parser.add_argument(
        '--cikti',
        type=str,
        default=None,
        help="Ölçüm sonuçlarının kaydedileceği YAML çıktısı yolu",
    )
    parser.add_argument(
        '--zaman_asimi',
        type=float,
        default=180.0,
        help="Her hedef için bekleme zaman aşımı süresi (saniye, varsayılan: 180)",
    )
    parser.add_argument(
        '--d_kavrama',
        type=float,
        default=0.22,
        help="Robot merkezi ile kavrama noktası arasındaki hedef mesafe (metre, varsayılan: 0.22)",
    )
    parser.add_argument(
        '--yanal',
        type=float,
        default=0.0,
        help="Kavrama noktasının robot eksenine yanal ofseti (metre, varsayılan: 0.0)",
    )
    parser.add_argument(
        '--bekleme',
        type=float,
        default=2.0,
        help="Hedefler arasında bekleme süresi (saniye, varsayılan: 2.0)",
    )
    parser.add_argument(
        '--use_sim_time',
        action='store_true',
        default=True,
        help="Simülasyon saatini (/clock) kullan (varsayılan: True)",
    )

    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    # Hedefleri YAML'dan oku
    try:
        hedefler = hedefler_yaml_oku(args.hedefler)
    except Exception as e:
        print(f"HATA: Hedef dosyası okunamadı: {e}", file=sys.stderr)
        return 1

    print(f"{len(hedefler)} hedef yüklendi: {args.hedefler}")

    if rclpy is None:
        print("HATA: rclpy kütüphanesi yüklenemedi. ROS 2 ortamının kurulu olduğundan emin olun.", file=sys.stderr)
        return 1

    rclpy.init(args=None)
    node = KavramaBenchNode(use_sim_time=args.use_sim_time)

    print("Gazebo gerçek robot pozisyonu (/odom_gercek) bekleniyor...")
    if not node.odom_bekle(timeout_sec=15.0):
        print("UYARI: /odom_gercek mesajı 15 saniye içinde alınamadı! Simülasyon açık mı?", file=sys.stderr)

    sonuclar = []

    try:
        for idx, hedef in enumerate(hedefler):
            print(f"\n[{idx+1}/{len(hedefler)}] '{hedef['ad']}' hedefine yaklaşma başlatılıyor...")
            sonuc = node.hedef_yurut(
                hedef=hedef,
                zaman_asimi=args.zaman_asimi,
                d_kavrama=args.d_kavrama,
                yanal=args.yanal,
            )
            sonuclar.append(sonuc)

            # Hedefler arasında robotu başlangıca döndürme yok; sırayla koşar. Her hedef sonrası 2 s bekle.
            if idx < len(hedefler) - 1:
                print(f"Sonraki hedefe geçmeden önce {args.bekleme:.1f} saniye bekleniyor...")
                node.bekle(args.bekleme)

    except KeyboardInterrupt:
        print("\nBenchmark kullanıcı tarafından durduruldu.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not sonuclar:
        print("Hiçbir hedef tamamlanamadı.")
        return 1

    # İstatistiksel özet hesapla
    ozet = istatistik_ozet(sonuclar)

    # Tabloyu konsola bas
    tablo_yazdir(sonuclar, ozet)

    # Çıktı YAML dosyasına kaydet
    if args.cikti:
        parametreler = {
            'hedefler_dosyasi': args.hedefler,
            'zaman_asimi_s': args.zaman_asimi,
            'd_kavrama_m': args.d_kavrama,
            'yanal_m': args.yanal,
            'hedef_arasi_bekleme_s': args.bekleme,
        }
        yaml_kaydet(args.cikti, sonuclar, ozet, parametreler)

    return 0 if ozet.get('tum_hedefler_kavranabilir', False) else 1


if __name__ == '__main__':
    sys.exit(main())
