"""Kavrama ve navigasyon görselleştirme düğümü (kavrama_gorsel.py).

RViz MarkerArray yayını ile kavrama adımlarını (nesne, Q ve P pozları,
yaklaşma koridoru, P'de robot ayak izi, kavrama noktası, durum metni ve
son yaklaşma güvenlik konisi) görselleştirir.

Giriş Konuları:
  /kavrama/plan           geometry_msgs/PoseArray    [Q, P] yaklaşma planı (map)
  /kavrama/durum          std_msgs/String            'DURUM ayrıntı' durum metni
  /kavrama/nesne_algi     geometry_msgs/PointStamped Algılanan nesne konumu
  /kavrama/hedef          geometry_msgs/PointStamped Hedef nesne konumu (map)
  /kavrama/nesne_robotta  geometry_msgs/PointStamped Robot çerçevesinde nesne konumu
  /scan                   sensor_msgs/LaserScan      Lidar taraması (güvenlik konisi)

Çıkış Konusu:
  /kavrama/isaretler      visualization_msgs/MarkerArray  5 Hz RViz işaretçi dizisi
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, Quaternion, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# LIMO standart ayak izi ölçüleri (Nav2 ile uyumlu)
AYAK_IZI_DX = 0.161  # ±0.161 m (uzunluk / 2)
AYAK_IZI_DY = 0.110  # ±0.110 m (genişlik / 2)
VARSAYILAN_D_KAVRAMA = 0.22  # robot merkezi -> kavrama noktası (m)
KORIDOR_GENISLIK = 0.22 + 2 * 0.05  # ayak izi genişliği + 2 * pay = 0.32 m


# ============================================================================
# Saf Yardımcı Fonksiyonlar (Geometri, Renk ve Hesaplamalar)
# ============================================================================

def yaw_q(q) -> float:
    """Quaternion nesnesinden 2B yaw açısını (radyan) hesaplar."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def wrap_angle(a: float) -> float:
    """Açıyı (-pi, pi] aralığına sarar."""
    return math.atan2(math.sin(a), math.cos(a))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Yaw açısından (x, y, z, w) quaternion demeti üretir."""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def koridor_koseleri(
    q: tuple[float, float],
    p: tuple[float, float],
    genislik: float = KORIDOR_GENISLIK,
) -> list[tuple[float, float]]:
    """Q ve P pozları arasında genişliği `genislik` olan koridorun 4 köşesini döner.

    Köşe sırası saat yönünün tersi / poligon sırasındadır:
      0: Q sağ, 1: P sağ, 2: P sol, 3: Q sol
    """
    qx, qy = float(q[0]), float(q[1])
    px, py = float(p[0]), float(p[1])
    dx = px - qx
    dy = py - qy
    L = math.hypot(dx, dy)
    if L < 1e-6:
        theta = 0.0
    else:
        theta = math.atan2(dy, dx)

    h = genislik / 2.0
    # İleri yöne dik sağ normal: (sin theta, -cos theta)
    # Sol normal: (-sin theta, cos theta)
    s, c = math.sin(theta), math.cos(theta)
    nx_sag, ny_sag = s, -c
    nx_sol, ny_sol = -s, c

    c0 = (qx + h * nx_sag, qy + h * ny_sag)
    c1 = (px + h * nx_sag, py + h * ny_sag)
    c2 = (px + h * nx_sol, py + h * ny_sol)
    c3 = (qx + h * nx_sol, qy + h * ny_sol)
    return [c0, c1, c2, c3]


def ayak_izi_koseleri(
    pos: tuple[float, float],
    yaw: float,
    dx: float = AYAK_IZI_DX,
    dy: float = AYAK_IZI_DY,
) -> list[tuple[float, float]]:
    """Robotun (x, y, yaw) pozundaki dikdörtgen ayak izi köşelerini (4 nokta) döner.

    Köşe sırası:
      0: Ön-Sol (+dx, +dy)
      1: Arka-Sol (-dx, +dy)
      2: Arka-Sağ (-dx, -dy)
      3: Ön-Sağ (+dx, -dy)
    """
    x0, y0 = float(pos[0]), float(pos[1])
    c, s = math.cos(yaw), math.sin(yaw)
    yerel_koseler = [
        (dx, dy),
        (-dx, dy),
        (-dx, -dy),
        (dx, -dy),
    ]
    harita_koseler = []
    for lx, ly in yerel_koseler:
        hx = x0 + c * lx - s * ly
        hy = y0 + s * lx + c * ly
        harita_koseler.append((hx, hy))
    return harita_koseler


def kavrama_noktasi_hesapla(
    pos: tuple[float, float],
    yaw: float,
    d_kavrama: float = VARSAYILAN_D_KAVRAMA,
    yanal: float = 0.0,
) -> tuple[float, float]:
    """Robotun pozundan (pos_x, pos_y, yaw) robot önündeki kavrama noktasını hesaplar.

    P pozunda robotun önünde d_kavrama mesafesindeki nokta hedef nesneyle çakışmalıdır.
    """
    px, py = float(pos[0]), float(pos[1])
    c, s = math.cos(yaw), math.sin(yaw)
    # Robot ekseni x ileri (+c, +s), sol normal yanal (+sol: -s, +c)
    kx = px + d_kavrama * c - yanal * s
    ky = py + d_kavrama * s + yanal * c
    return (kx, ky)


def renk_secimi(
    oge_turu: str,
    durum_veya_algili: Any = None,
) -> tuple[float, float, float, float]:
    """İşaretçi türü ve durumuna göre (r, g, b, a) renk demeti döner.

    Öğe Türleri:
      'nesne': algi_var ise yeşil, yoksa turuncu
      'Q': mavi
      'P': yeşil
      'koridor': yarı saydam açık mavi / camgöbeği
      'ayak_izi_P': sarımsı yeşil
      'kavrama_noktasi': mor / eflatun
      'durum': HATA ise kırmızı, HAZIR ise yeşil, diğer sarı/beyaz
      'guvenlik_konisi': engel varsa kırmızı, yoksa yarı saydam turuncu/sarı
    """
    if oge_turu == 'nesne':
        if bool(durum_veya_algili):
            # Algı konumu: Yeşil
            return (0.0, 1.0, 0.0, 0.9)
        # Hedef konumu: Turuncu
        return (1.0, 0.55, 0.0, 0.9)

    elif oge_turu == 'Q':
        # Mavi ok
        return (0.1, 0.4, 1.0, 0.9)

    elif oge_turu == 'P':
        # Yeşil ok
        return (0.0, 0.9, 0.2, 0.9)

    elif oge_turu == 'koridor':
        # Yarı saydam açık mavi
        return (0.2, 0.6, 1.0, 0.35)

    elif oge_turu == 'ayak_izi_P':
        # Belirgin sarımsı yeşil
        return (0.2, 0.85, 0.2, 0.85)

    elif oge_turu == 'kavrama_noktasi':
        # Eflatun / Macenta küçük küre
        return (1.0, 0.0, 0.8, 0.95)

    elif oge_turu == 'durum':
        s = str(durum_veya_algili).strip().upper()
        if 'HATA' in s:
            return (1.0, 0.1, 0.1, 1.0)
        elif 'HAZIR' in s:
            return (0.1, 1.0, 0.2, 1.0)
        return (1.0, 0.95, 0.2, 1.0)

    elif oge_turu == 'guvenlik_konisi':
        if bool(durum_veya_algili):
            # Engel algılandı: Kırmızı uyarı
            return (1.0, 0.1, 0.1, 0.75)
        # Güvenli: Yarı saydam kehribar/turuncu
        return (1.0, 0.7, 0.0, 0.45)

    return (1.0, 1.0, 1.0, 1.0)


def guvenlik_konisi_noktalari(
    r: float = 0.5,
    koni_derece: float = 25.0,
    adim_derece: float = 2.5,
) -> list[tuple[float, float]]:
    """Base_footprint çerçevesinde ±koni_derece açısında r yarıçaplı koni poligonu döner.

    Orijinden (0, 0) başlar, yay boyunca noktaları dolaşır ve orijine geri döner.
    """
    noktalar = [(0.0, 0.0)]
    koni_rad = math.radians(koni_derece)
    adim_rad = math.radians(adim_derece)
    n_adim = int(round((2.0 * koni_rad) / adim_rad))
    for i in range(n_adim + 1):
        aci = -koni_rad + i * adim_rad
        if aci > koni_rad:
            aci = koni_rad
        x = r * math.cos(aci)
        y = r * math.sin(aci)
        noktalar.append((float(x), float(y)))
    noktalar.append((0.0, 0.0))
    return noktalar


def guvenlik_konisinde_engel_var_mi(
    scan_ranges,
    angle_min: float,
    angle_increment: float,
    koni_rad: float = math.radians(25.0),
    maks_d: float = 0.50,
    min_d: float = 0.161,
) -> bool:
    """Lidar taramasında ön ±koni içinde [min_d, maks_d] arasında engel olup olmadığını denetler."""
    if scan_ranges is None or len(scan_ranges) == 0:
        return False
    r_arr = np.asarray(scan_ranges, dtype=float)
    angles = angle_min + np.arange(len(r_arr)) * angle_increment
    valid = np.isfinite(r_arr) & (r_arr >= min_d) & (r_arr <= maks_d) & (np.abs(angles) <= koni_rad)
    return bool(np.any(valid))


# ============================================================================
# ROS Marker Nesne Üretim Yardımcıları
# ============================================================================

def marker_nesne(
    pos: tuple[float, float, float],
    algi_var: bool,
    stamp,
    frame_id: str = 'map',
    r: float = 0.03,
    h: float = 0.08,
) -> Marker:
    """'nesne' işaretçisi (silindir, r=0.03 m, z yüksekliğinde; algı varsa yeşil, yoksa turuncu)."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'nesne'
    m.id = 0
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = float(pos[2])
    m.pose.orientation.w = 1.0
    m.scale.x = 2.0 * r
    m.scale.y = 2.0 * r
    m.scale.z = float(h)
    rgba = renk_secimi('nesne', algi_var)
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return m


def marker_ok(
    ns: str,
    pos: tuple[float, float],
    theta: float,
    stamp,
    frame_id: str = 'map',
    uzunluk: float = 0.25,
    genislik: float = 0.04,
) -> Marker:
    """'Q' (mavi) ve 'P' (yeşil) ok işaretçisi."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = 0
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = 0.02
    qx, qy, qz, qw = quaternion_from_yaw(theta)
    m.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    m.scale.x = float(uzunluk)
    m.scale.y = float(genislik)
    m.scale.z = float(genislik)
    rgba = renk_secimi(ns)
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return m


def marker_koridor(
    q: tuple[float, float],
    p: tuple[float, float],
    stamp,
    frame_id: str = 'map',
    genislik: float = KORIDOR_GENISLIK,
) -> Marker:
    """'koridor' işaretçisi (Q->P arası, genişlik 0.32 m, yarı saydam TRIANGLE_LIST)."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'koridor'
    m.id = 0
    m.type = Marker.TRIANGLE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 1.0
    m.scale.y = 1.0
    m.scale.z = 1.0
    rgba = renk_secimi('koridor')
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])

    koseler = koridor_koseleri(q, p, genislik=genislik)
    # c0: Q sağ, c1: P sağ, c2: P sol, c3: Q sol
    z = 0.005
    pts = [Point(x=float(c[0]), y=float(c[1]), z=z) for c in koseler]

    # İki üçgen (üst ve alt yüzler için her iki yönde sarım)
    # Üçgen 1: c0 -> c1 -> c2
    # Üçgen 2: c0 -> c2 -> c3
    m.points = [
        pts[0], pts[1], pts[2],
        pts[0], pts[2], pts[3],
        # Ters sarım (çift taraflı görünüm için)
        pts[0], pts[2], pts[1],
        pts[0], pts[3], pts[2],
    ]
    return m


def marker_ayak_izi(
    p_pos: tuple[float, float],
    theta: float,
    stamp,
    frame_id: str = 'map',
) -> Marker:
    """'ayak_izi_P' işaretçisi (P'de robot ayak izi dikdörtgeni ±0.161 x ±0.11, LINE_STRIP)."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'ayak_izi_P'
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.015  # çizgi kalınlığı
    rgba = renk_secimi('ayak_izi_P')
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])

    koseler = ayak_izi_koseleri(p_pos, theta)
    z = 0.01
    pts = [Point(x=float(c[0]), y=float(c[1]), z=z) for c in koseler]
    # Kapalı poligon
    pts.append(Point(x=float(koseler[0][0]), y=float(koseler[0][1]), z=z))
    m.points = pts
    return m


def marker_kavrama_noktasi(
    p_pos: tuple[float, float],
    theta: float,
    stamp,
    frame_id: str = 'map',
    d_kavrama: float = VARSAYILAN_D_KAVRAMA,
    z: float = 0.05,
) -> Marker:
    """'kavrama_noktasi' işaretçisi (P'de robot önünde d_kavrama'daki nokta, küçük küre)."""
    kx, ky = kavrama_noktasi_hesapla(p_pos, theta, d_kavrama=d_kavrama)
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'kavrama_noktasi'
    m.id = 0
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = float(kx)
    m.pose.position.y = float(ky)
    m.pose.position.z = float(z)
    m.pose.orientation.w = 1.0
    m.scale.x = 0.04
    m.scale.y = 0.04
    m.scale.z = 0.04
    rgba = renk_secimi('kavrama_noktasi')
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return m


def marker_durum(
    metin: str,
    stamp,
    frame_id: str = 'base_footprint',
    z_yukseklik: float = 0.50,
) -> Marker:
    """'durum' işaretçisi (TEXT_VIEW_FACING, robotun 0.5 m üstünde durum metni)."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'durum'
    m.id = 0
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = float(z_yukseklik)
    m.pose.orientation.w = 1.0
    m.scale.z = 0.12  # yazı boyutu
    m.text = metin if metin else 'BOSTA'
    rgba = renk_secimi('durum', metin)
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return m


def marker_guvenlik_konisi(
    stamp,
    gorunur: bool,
    engel_var: bool = False,
    frame_id: str = 'base_footprint',
    r: float = 0.50,
    koni_derece: float = 25.0,
) -> Marker:
    """'guvenlik_konisi' işaretçisi (base_footprint'te ±25° koni, 0.5 m, LINE_STRIP)."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'guvenlik_konisi'
    m.id = 0
    m.type = Marker.LINE_STRIP

    if not gorunur:
        m.action = Marker.DELETE
        return m

    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.015  # çizgi kalınlığı
    rgba = renk_secimi('guvenlik_konisi', engel_var)
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])

    noktalar = guvenlik_konisi_noktalari(r=r, koni_derece=koni_derece)
    z = 0.02
    m.points = [Point(x=float(x), y=float(y), z=z) for x, y in noktalar]
    return m


def marker_sil_hepsi(stamp) -> Marker:
    """Tüm işaretçileri temizleyen DELETEALL işaretçisi döner."""
    m = Marker()
    m.header.stamp = stamp
    m.action = Marker.DELETEALL
    return m


# ============================================================================
# Düğüm Sınıfı: KavramaGorsel
# ============================================================================

class KavramaGorsel(Node):
    """Kavrama yaklaşması durumunu ve geometrisini RViz üzerinde görselleştiren düğüm."""

    def __init__(self) -> None:
        super().__init__('kavrama_gorsel')

        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        # Durum Değişkenleri
        self.plan_q: tuple[float, float, float] | None = None  # (qx, qy, q_yaw)
        self.plan_p: tuple[float, float, float] | None = None  # (px, py, p_yaw)
        self.plan_frame: str = 'map'

        self.durum_metni: str = 'BOSTA'
        self.durum_kodu: str = 'BOSTA'
        self.onceki_durum_kodu: str = 'BOSTA'
        self.plan_temizlendi: bool = False

        self.hedef_nokta: tuple[float, float, float] | None = None  # (x, y, z)
        self.hedef_frame: str = 'map'

        self.algi_nokta: tuple[float, float, float] | None = None   # (x, y, z)
        self.algi_zamani: float = 0.0
        self.algi_gecerlilik_s: float = 3.0

        self.nesne_robotta: tuple[float, float, float] | None = None

        self.son_scan: LaserScan | None = None
        self.scan_engel_var: bool = False

        # Yayıncı
        self.p_isaretler = self.create_publisher(MarkerArray, '/kavrama/isaretler', 10)

        # Abonelikler
        self.create_subscription(PoseArray, '/kavrama/plan', self._plan_cb, 10, callback_group=self.cb)
        self.create_subscription(String, '/kavrama/durum', self._durum_cb, 10, callback_group=self.cb)
        self.create_subscription(PointStamped, '/kavrama/nesne_algi', self._nesne_algi_cb, 10, callback_group=self.cb)
        self.create_subscription(PointStamped, '/kavrama/hedef', self._hedef_cb, 10, callback_group=self.cb)
        self.create_subscription(PointStamped, '/kavrama/nesne_robotta', self._nesne_robotta_cb, 10, callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data, callback_group=self.cb)

        # 5 Hz Görselleştirme Timer'ı (1.0 / 5.0 s)
        self.timer_5hz = self.create_timer(0.20, self._dongu_5hz, callback_group=self.cb)

        self.get_logger().info('Kavrama görselleştirme düğümü hazır (/kavrama/isaretler, 5 Hz)')

    # --- Abonelik Geri Çağrıları ---

    def _plan_cb(self, m: PoseArray) -> None:
        """/kavrama/plan [Q, P] yaklaşma pozlarını alır."""
        if len(m.poses) >= 2:
            q_pose = m.poses[0]
            p_pose = m.poses[1]
            q_yaw = yaw_q(q_pose.orientation)
            p_yaw = yaw_q(p_pose.orientation)
            self.plan_q = (float(q_pose.position.x), float(q_pose.position.y), float(q_yaw))
            self.plan_p = (float(p_pose.position.x), float(p_pose.position.y), float(p_yaw))
            self.plan_frame = m.header.frame_id if m.header.frame_id else 'map'
            self.get_logger().debug(f'Plan alındı: Q={self.plan_q}, P={self.plan_p}')

    def _durum_cb(self, m: String) -> None:
        """/kavrama/durum metnini alır ve PLAN geçişinde eski işaretleri temizler."""
        self.durum_metni = m.data.strip()
        parcalar = self.durum_metni.split()
        yeni_kod = parcalar[0] if parcalar else 'BOSTA'

        # Durum PLAN'a dönünce eski işaretleri sil (DELETEALL)
        if yeni_kod == 'PLAN' and self.durum_kodu != 'PLAN':
            self._isaretleri_temizle()
            self.plan_q = None
            self.plan_p = None
            self.algi_nokta = None

        self.onceki_durum_kodu = self.durum_kodu
        self.durum_kodu = yeni_kod

    def _isaretleri_temizle(self) -> None:
        """RViz'deki tüm mevcut işaretçileri temizler (DELETEALL)."""
        ma = MarkerArray()
        ma.markers.append(marker_sil_hepsi(self.get_clock().now().to_msg()))
        self.p_isaretler.publish(ma)
        self.get_logger().info('Durum PLAN: Eski görselleştirme işaretçileri temizlendi (DELETEALL)')

    def _nesne_algi_cb(self, m: PointStamped) -> None:
        """/kavrama/nesne_algi bilgisini map çerçevesine dönüştürerek saklar."""
        simdi = self.get_clock().now().nanoseconds / 1e9
        try:
            if m.header.frame_id in ('', 'map'):
                self.algi_nokta = (float(m.point.x), float(m.point.y), float(m.point.z))
                self.algi_zamani = simdi
                return

            t = self.tf.lookup_transform(
                'map',
                m.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1),
            )
            # 3B dönüşüm
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            rx, ry, rz, rw = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
            # Euler-Rodrigues dönme
            px, py, pz = m.point.x, m.point.y, m.point.z
            r_vec_x = ry * pz - rz * py
            r_vec_y = rz * px - rx * pz
            r_vec_z = rx * py - ry * px

            cx = rw * px + r_vec_x
            cy = rw * py + r_vec_y
            cz = rw * pz + r_vec_z

            fx = ry * cz - rz * cy
            fy = rz * cx - rx * cz
            fz = rx * cy - ry * cx

            mx = tx + px + 2.0 * fx
            my = ty + py + 2.0 * fy
            mz = tz + pz + 2.0 * fz

            self.algi_nokta = (float(mx), float(my), float(mz))
            self.algi_zamani = simdi
        except Exception:
            # TF başarısızsa doğrudan noktayı kullan
            self.algi_nokta = (float(m.point.x), float(m.point.y), float(m.point.z))
            self.algi_zamani = simdi

    def _hedef_cb(self, m: PointStamped) -> None:
        """/kavrama/hedef (PointStamped map) bilgisini saklar."""
        self.hedef_nokta = (float(m.point.x), float(m.point.y), float(m.point.z))
        self.hedef_frame = m.header.frame_id if m.header.frame_id else 'map'

    def _nesne_robotta_cb(self, m: PointStamped) -> None:
        """/kavrama/nesne_robotta bilgisini saklar."""
        self.nesne_robotta = (float(m.point.x), float(m.point.y), float(m.point.z))

    def _scan_cb(self, m: LaserScan) -> None:
        """Lidar taramasında güvenlik konisi içi engel denetimi yapar."""
        self.son_scan = m
        self.scan_engel_var = guvenlik_konisinde_engel_var_mi(
            m.ranges,
            m.angle_min,
            m.angle_increment,
            koni_rad=math.radians(25.0),
            maks_d=0.50,
            min_d=0.161,
        )

    # --- 5 Hz Döngüsü ---

    def _dongu_5hz(self) -> None:
        """5 Hz frekansta tüm aktif işaretçileri üretir ve MarkerArray yayınlar."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        t_sec = now.nanoseconds / 1e9
        markers: list[Marker] = []

        # 1. 'nesne' (silindir, r=0.03, z yüksekliğinde; algı varsa yeşil, yoksa turuncu)
        algi_taze = (self.algi_nokta is not None) and ((t_sec - self.algi_zamani) <= self.algi_gecerlilik_s)
        nesne_pos = None
        if algi_taze and self.algi_nokta is not None:
            # Algı konumu
            z_val = self.hedef_nokta[2] if self.hedef_nokta is not None else self.algi_nokta[2]
            nesne_pos = (self.algi_nokta[0], self.algi_nokta[1], z_val)
            markers.append(marker_nesne(nesne_pos, algi_var=True, stamp=stamp, frame_id='map'))
        elif self.hedef_nokta is not None:
            # Hedef konumu
            nesne_pos = self.hedef_nokta
            markers.append(marker_nesne(nesne_pos, algi_var=False, stamp=stamp, frame_id=self.hedef_frame))

        # 2. 'Q' ve 'P' (ok, yön theta; Q mavi, P yeşil)
        if self.plan_q is not None and self.plan_p is not None:
            qx, qy, q_th = self.plan_q
            px, py, p_th = self.plan_p
            markers.append(marker_ok('Q', (qx, qy), q_th, stamp=stamp, frame_id=self.plan_frame))
            markers.append(marker_ok('P', (px, py), p_th, stamp=stamp, frame_id=self.plan_frame))

            # 3. 'koridor' (Q->P arası, ayak izi genişliği 0.22 + 2*0.05 şerit, yarı saydam)
            markers.append(marker_koridor((qx, qy), (px, py), stamp=stamp, frame_id=self.plan_frame))

            # 4. 'ayak_izi_P' (P'de robot ayak izi dikdörtgeni ±0.161 x ±0.11)
            markers.append(marker_ayak_izi((px, py), p_th, stamp=stamp, frame_id=self.plan_frame))

            # 5. 'kavrama_noktasi' (P'de robot önünde d_kavrama=0.22'deki nokta, küçük küre)
            z_kav = nesne_pos[2] if nesne_pos is not None else 0.05
            markers.append(marker_kavrama_noktasi((px, py), p_th, stamp=stamp, frame_id=self.plan_frame, z=z_kav))

        # 6. 'durum' (TEXT_VIEW_FACING, robotun 0.5 m üstünde son durum metni; HATA kırmızı, HAZIR yeşil)
        markers.append(marker_durum(self.durum_metni, stamp=stamp, frame_id='base_footprint', z_yukseklik=0.50))

        # 7. 'guvenlik_konisi' (base_footprint'te ±25° koni, 0.5 m, SON_YAKLASMA sırasında görünür)
        son_yaklasma_aktif = (self.durum_kodu == 'SON_YAKLASMA')
        markers.append(
            marker_guvenlik_konisi(
                stamp=stamp,
                gorunur=son_yaklasma_aktif,
                engel_var=self.scan_engel_var,
                frame_id='base_footprint',
                r=0.50,
                koni_derece=25.0,
            )
        )

        ma = MarkerArray()
        ma.markers = markers
        self.p_isaretler.publish(ma)


# ============================================================================
# Main Giriş Noktası
# ============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    n = KavramaGorsel()
    ex = MultiThreadedExecutor(num_threads=2)
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
