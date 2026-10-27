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
"""

from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
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


class KavramaDugumu(Node):
    """ICP'siz, nesneye göreli son yaklaşmalı ve kurtarmalı kavrama durum makinesi."""

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

        self.ayar = Ayar(
            d_kavrama=d_kavrama_val,
            yanal=yanal_val,
            d_on=d_on_val,
            pay=p('pay', 0.05).value,
            r_nesne=p('r_nesne', 0.12).value,
            yon=None if math.isnan(yon) else math.radians(yon),
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

        # Durum Değişkenleri
        self.durum_kodu: str = 'BOSTA'  # 'BOSTA', 'PLAN', 'NAV_Q', 'SON_YAKLASMA', 'HAZIR', 'KURTARMA', 'HATA'
        self.odom: tuple[float, float, float] | None = None
        self.tarama_noktalari: np.ndarray | None = None
        self.son_algi_odom: tuple[float, float] | None = None
        self.yeni_algi_var: bool = False
        self.aktif_nav_goal = None

        self.nesne_hedef_map: tuple[float, float, float] | None = None
        self.nesne_yedek_odom: tuple[float, float] | None = None
        self.adaylar: list = []
        self.aktif_aday_idx: int = 0
        self.kurtarma_sayisi: int = 0
        self.kurtarma_baslangic_odom: tuple[float, float, float] | None = None

        self.son_denetleyici: SonYaklasma | None = None
        self.algisiz: bool = False
        self.tek_olcum_verildi: bool = False
        self.t_son_yaklasma_baslangic: rclpy.time.Time | None = None
        self.son_durum_log_zamani: float = 0.0

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

    def durum(self, s: str) -> None:
        """Durum bilgisini konsola ve /kavrama/durum konusuna yayınlar ('DURUM ayrıntı' biçiminde)."""
        self.get_logger().info(s)
        self.p_durum.publish(String(data=s))

    def tf2d(self, hedef: str, kaynak: str) -> tuple[float, float, float]:
        """TF ağacından kaynak->hedef 2B (x, y, yaw) dönüşümünü çeker."""
        t = self.tf.lookup_transform(hedef, kaynak, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)).transform
        return t.translation.x, t.translation.y, yaw_q(t.rotation)

    def _durdur(self) -> None:
        """Robotun tekerlek hızlarını sıfırlar."""
        self.p_cmd.publish(Twist())

    # --- Abonelik Geri Çağrıları ---

    def _odom_cb(self, m: Odometry) -> None:
        self.odom = (
            float(m.pose.pose.position.x),
            float(m.pose.pose.position.y),
            float(yaw_q(m.pose.pose.orientation)),
        )

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
        if self.aktif_nav_goal is not None:
            try:
                self.aktif_nav_goal.cancel_goal_async()
            except Exception:
                pass
            self.aktif_nav_goal = None
        self._durdur()
        self.durum_kodu = 'HATA'
        self.durum('HATA iptal')

    def _hedef_cb(self, m: PointStamped) -> None:
        """/kavrama/hedef geldiğinde durum makinesini PLAN durumuna geçirir."""
        if self.durum_kodu in ('PLAN', 'NAV_Q', 'SON_YAKLASMA', 'KURTARMA'):
            self.durum('HATA önceki görev sürüyor')
            return

        if self.harita is None:
            self.durum('HATA harita parametresi boş veya yüklenmedi')
            return

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
            self._aday_basarisiz_isle('Nav2 hedef reddi')
            return

        self.aktif_nav_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future) -> None:
        self.aktif_nav_goal = None
        if self.durum_kodu != 'NAV_Q':
            return
        self.durum_kodu = 'GECIS'

        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self._son_yaklasma_baslat()
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
                    self.durum(f'NAV_Q yakin_kabul ({d * 100:.1f} cm, {math.degrees(a):.1f}°)')
                    self._son_yaklasma_baslat()
                    return
            except Exception as e:
                self.get_logger().warn(f'NAV_Q yakin_kabul TF/hesaplama hatası: {e}')

            if not yakin_kabul:
                self.durum(f'NAV_Q başarısız durum {status} (aday {self.aktif_aday_idx + 1})')
                self._aday_basarisiz_isle(f'Nav2 basarisiz ({status})')
        else:
            self.durum(f'NAV_Q iptal edildi durum {status} (aday {self.aktif_aday_idx + 1})')
            self._aday_basarisiz_isle(f'Nav2 iptal ({status})')

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
            self.durum(f'SON_YAKLASMA algı aktif, odom yönü {math.degrees(th_odom):+.1f}°')
        else:
            self.algisiz = True
            self.tek_olcum_verildi = False
            self.durum(
                f'SON_YAKLASMA algisiz: odom yönü {math.degrees(th_odom):+.1f}°, '
                f'yedek map nesne ({self.nesne_yedek_odom[0]:+.2f}, {self.nesne_yedek_odom[1]:+.2f})'
            )

        self.t_son_yaklasma_baslangic = self.get_clock().now()
        self.son_durum_log_zamani = 0.0
        self.durum_kodu = 'SON_YAKLASMA'

    def _hazir_isle(self, bilgi: dict) -> None:
        """HAZIR: /kavrama/nesne_robotta yayını, P hatası ve yaklaşma süresi raporu."""
        self.durum_kodu = 'HAZIR'
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
        self.durum_kodu = 'KURTARMA'
        self.durum(
            f'KURTARMA {neden}: odomla düz 0.20 m geri çekiliniyor '
            f'(kurtarma {self.kurtarma_sayisi + 1}/2)'
        )

    # --- 50 Hz Denetim Döngüsü ---

    def _dongu_50hz(self) -> None:
        if self.durum_kodu == 'SON_YAKLASMA':
            self._dongu_son_yaklasma()
        elif self.durum_kodu == 'KURTARMA':
            self._dongu_kurtarma()

    def _dongu_son_yaklasma(self) -> None:
        if self.durum_kodu != 'SON_YAKLASMA':
            return
        if self.odom is None or self.son_denetleyici is None:
            return

        now = self.get_clock().now()
        t = now.nanoseconds / 1e9

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
            self.durum_kodu = 'GECIS'
            self._durdur()
            self._kurtarma_baslat(adim_durum)
            return

        # Komut yayınla
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.p_cmd.publish(cmd)

        # 1 Hz aralıkla periyodik durum yayını
        if (t - self.son_durum_log_zamani) > 1.0:
            self.son_durum_log_zamani = t
            algisiz_bilgi = 'algisiz ' if self.algisiz else ''
            rho = bilgi.get('rho', float('nan'))
            self.durum(f'SON_YAKLASMA {algisiz_bilgi}v={v:.2f} w={w:.2f} rho={rho:.2f} m')

    def _dongu_kurtarma(self) -> None:
        if self.durum_kodu != 'KURTARMA':
            return
        if self.odom is None or self.kurtarma_baslangic_odom is None:
            return

        x, y, _ = self.odom
        x0, y0, _ = self.kurtarma_baslangic_odom
        kat_edilen = math.hypot(x - x0, y - y0)

        # Odomla düz 0.20 m geri; durum geçiş koruması ile aday atlama önlenir
        yeni_durum, yeni_kurtarma, yeni_idx, bitti = kurtarma_durum_gecisi(
            self.durum_kodu, kat_edilen, self.kurtarma_sayisi, self.aktif_aday_idx, 0.20
        )

        if not bitti:
            cmd = Twist()
            cmd.linear.x = -0.08
            cmd.angular.z = 0.0
            self.p_cmd.publish(cmd)
        else:
            # Durumu aday artırma/NAV_Q başlatmadan ÖNCE geçiş durumuna al
            self.durum_kodu = yeni_durum  # 'GECIS'
            self._durdur()
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
