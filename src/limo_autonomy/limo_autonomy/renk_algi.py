#!/usr/bin/env python3
"""Kameradan nesne algısı prototipi: Renk ve derinlik füzyonu (renk_algi.py).

LIMO ve Intel RealSense D435 kamerası için renk tabanlı (HSV) nesne tespiti
ve derinlik görüntüsü (16UC1 mm / 32FC1 m) ile 3B konum kestirimi.

Algoritma Akışı:
  1. /camera/color/image_raw konusundan renk görüntüsü (1280x720 rgb8/bgr8) alınır.
  2. HSV renk uzayında kırmızı nesne (elma) için çift aralıklı eşikleme yapılır
     (0..10 ve 170..180 kırmızı sarması).
  3. Morfolojik açma ile gürültüler temizlenir ve en büyük bağlı bileşen seçilir.
     min_alan_px (150 px) altındaki küçük lekeler elenir.
  4. Maskenin ağırlık merkezinden (cx, cy) renk optik çerçevesindeki görüş ışını hesaplanır.
  5. Derinlik görüntüsünde (/camera/depth/image_raw, 848x480) TF veya dış kalibrasyon
     (camera_color_optical_frame -> camera_depth_optical_frame) ile izdüşüm yapılır.
  6. İzdüşüm noktası çevresindeki 7x7 pencereden geçerli derinliklerin medyanı alınır.
  7. Derinlik nesnenin ön yüzeyidir; küre merkezi için nesne yarıçapı kadar (nesne_yaricap=0.035 m)
     ışın boyunca 3B uzayda ilerletilir.
  8. Nesne merkezinin 3B konumu PointStamped olarak /kavrama/nesne_algi konusuna,
     görüş durumu ise /kavrama/nesne_algi_durum konusuna ('GORUYOR'/'GORMUYOR') yayınlanır.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

# İsteğe bağlı cv_bridge desteği; yoksa dahili numpy dönüştürücüler kullanılır.
try:
    from cv_bridge import CvBridge

    CV_BRIDGE_VAR = True
except ImportError:
    CvBridge = None
    CV_BRIDGE_VAR = False


# ==============================================================================
# 1. Saf Fonksiyonlar (ROS bağımsız, birim test edilebilir)
# ==============================================================================

def k_ayristir(k: Sequence[float] | np.ndarray) -> tuple[float, float, float, float]:
    """Kamera iç parametrelerinden (fx, fy, cx, cy) değerlerini ayrıştırır.

    Desteklenen biçimler:
      - 3x3 matris (np.ndarray veya iç içe liste)
      - 9 elemanlı düz liste (sensor_msgs/CameraInfo.k)
      - 4 elemanlı demet/liste (fx, fy, cx, cy)
    """
    if hasattr(k, 'shape') and getattr(k, 'shape') == (3, 3):
        return float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
    k_list = list(k)
    if len(k_list) == 9:
        return float(k_list[0]), float(k_list[4]), float(k_list[2]), float(k_list[5])
    if len(k_list) == 4:
        return float(k_list[0]), float(k_list[1]), float(k_list[2]), float(k_list[3])
    raise ValueError(f'Geçersiz K iç parametre boyutu: {len(k_list)}')


def hsv_maske(
    goruntu: np.ndarray,
    hsv_alt: Sequence[int] = (0, 120, 70),
    hsv_ust: Sequence[int] = (10, 255, 255),
    hsv_alt2: Sequence[int] = (170, 120, 70),
    hsv_ust2: Sequence[int] = (180, 255, 255),
    rgb_mi: bool = False,
    morfoloji_kernel: int = 3,
) -> np.ndarray:
    """HSV renk uzayında kırmızı sarma eşiklemesi ve morfolojik açma uygular.

    Args:
        goruntu: BGR veya RGB renkli görüntü (uint8).
        hsv_alt: 1. kırmızı aralığı alt sınırı [H, S, V].
        hsv_ust: 1. kırmızı aralığı üst sınırı [H, S, V].
        hsv_alt2: 2. kırmızı aralığı (sarma: 170-180) alt sınırı.
        hsv_ust2: 2. kırmızı aralığı üst sınırı.
        rgb_mi: Görüntü RGB formatında ise True, BGR ise False.
        morfoloji_kernel: Morfolojik açma eliptik çekirdek boyutu (px).

    Returns:
        İkili (binary) maske (0 ve 255 değerlerinde uint8).
    """
    if goruntu is None or goruntu.size == 0:
        return np.empty((0, 0), dtype=np.uint8)

    donusum = cv2.COLOR_RGB2HSV if rgb_mi else cv2.COLOR_BGR2HSV
    hsv = cv2.cvtColor(goruntu, donusum)

    alt1 = np.array(hsv_alt, dtype=np.uint8)
    ust1 = np.array(hsv_ust, dtype=np.uint8)
    mask1 = cv2.inRange(hsv, alt1, ust1)

    alt2 = np.array(hsv_alt2, dtype=np.uint8)
    ust2 = np.array(hsv_ust2, dtype=np.uint8)
    mask2 = cv2.inRange(hsv, alt2, ust2)

    mask = cv2.bitwise_or(mask1, mask2)

    if morfoloji_kernel > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (int(morfoloji_kernel), int(morfoloji_kernel))
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return mask


def en_buyuk_bilesen_filtrele(
    maske: np.ndarray,
    min_alan_px: int = 150,
) -> tuple[np.ndarray, int]:
    """Maskedeki bağlı bileşenleri analiz eder ve en büyük bileşeni döndürür.

    min_alan_px değerinden küçük bileşenler elenir (küçük leke reddi).

    Returns:
        (temiz_maske, alan):
            temiz_maske: Yalnızca en büyük bileşeni içeren ikili maske (0 veya 255).
            alan: En büyük bileşenin piksel alanı (yoksa 0).
    """
    if maske is None or maske.size == 0 or cv2.countNonZero(maske) == 0:
        return np.zeros_like(maske) if maske is not None else np.empty((0, 0), dtype=np.uint8), 0

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(maske, connectivity=8)

    # 0 etiketi arka plandır, 1..num_labels-1 ön plan bileşenleridir.
    if num_labels <= 1:
        return np.zeros_like(maske), 0

    en_buyuk_alan = 0
    en_buyuk_idx = -1

    for i in range(1, num_labels):
        alan = int(stats[i, cv2.CC_STAT_AREA])
        if alan >= min_alan_px and alan > en_buyuk_alan:
            en_buyuk_alan = alan
            en_buyuk_idx = i

    if en_buyuk_idx == -1:
        return np.zeros_like(maske), 0

    temiz = np.zeros_like(maske)
    temiz[labels == en_buyuk_idx] = 255
    return temiz, en_buyuk_alan


def maske_bul(
    goruntu: np.ndarray,
    hsv_alt: Sequence[int] = (0, 120, 70),
    hsv_ust: Sequence[int] = (10, 255, 255),
    hsv_alt2: Sequence[int] = (170, 120, 70),
    hsv_ust2: Sequence[int] = (180, 255, 255),
    min_alan_px: int = 150,
    rgb_mi: bool = False,
    morfoloji_kernel: int = 3,
) -> tuple[np.ndarray, int]:
    """Renk görüntüsünden kırmızı nesne maskesini ve alanını bulur."""
    ham_maske = hsv_maske(
        goruntu,
        hsv_alt=hsv_alt,
        hsv_ust=hsv_ust,
        hsv_alt2=hsv_alt2,
        hsv_ust2=hsv_ust2,
        rgb_mi=rgb_mi,
        morfoloji_kernel=morfoloji_kernel,
    )
    return en_buyuk_bilesen_filtrele(ham_maske, min_alan_px=min_alan_px)


def merkez_bul(
    maske: np.ndarray,
    min_alan_px: int = 150,
) -> tuple[float, float] | None:
    """İkili maskeden nesnenin ağırlık merkezini (cx, cy) piksel olarak hesaplar.

    Returns:
        (cx, cy): Piksel koordinatları (u: sütun, v: satır) veya alan yetersizse None.
    """
    if maske is None or maske.size == 0:
        return None

    m = cv2.moments(maske)
    alan = float(m['m00'])
    if alan < float(min_alan_px):
        return None

    cx = float(m['m10'] / alan)
    cy = float(m['m01'] / alan)
    return (cx, cy)


def piksel_isin(
    u: float,
    v: float,
    k: Sequence[float] | np.ndarray,
    birim: bool = True,
) -> tuple[float, float, float]:
    """Piksel koordinatından kamera optik çerçevesindeki 3B görüş ışınını hesaplar.

    Kamera optik ekseninde Z ileri, X sağa, Y aşağı doğrudur.

    Args:
        u: Piksel sütun koordinatı (x).
        v: Piksel satır koordinatı (y).
        k: Kamera iç parametre matrisi K.
        birim: True ise birim vektör (|r| = 1), False ise normalleştirilmiş ışın (xn, yn, 1.0).

    Returns:
        (rx, ry, rz): Optik çerçevedeki 3B ışın yönü.
    """
    fx, fy, cx, cy = k_ayristir(k)
    xn = (float(u) - cx) / fx
    yn = (float(v) - cy) / fy
    zn = 1.0

    if not birim:
        return (xn, yn, zn)

    norm = math.sqrt(xn * xn + yn * yn + zn * zn)
    if norm < 1e-9:
        return (0.0, 0.0, 1.0)
    return (xn / norm, yn / norm, zn / norm)


def isin_derinlik_3b(
    isin: Sequence[float],
    derinlik_m: float,
    nesne_yaricap: float = 0.035,
    oklid_mi: bool = False,
) -> tuple[float, float, float]:
    """Görüş ışını ve derinlik bilgisinden nesne merkezinin 3B koordinatını hesaplar.

    Derinlik görüntüsündeki ölçüm nesnenin kamera tarafındaki ön yüzeyidir.
    Küre nesnenin gerçek merkezini elde etmek için ışın boyunca nesne yarıçapı
    (nesne_yaricap) kadar sahne içerisine ilerlenir.

    Args:
        isin: Kamera optik çerçevesindeki görüş ışını (rx, ry, rz).
        derinlik_m: Ölçülen derinlik mesafesi (metre).
        nesne_yaricap: Nesnenin yarıçapı (m, varsayılan elma için 0.035 m).
        oklid_mi: True ise derinlik optik merkeze Öklid mesafesidir;
                  False ise standart ROS/RGB-D derinliği (optik eksen boyunca Z).

    Returns:
        (X, Y, Z): Nesne merkezinin kamera optik çerçevesindeki konumu (m, m, m).
    """
    rx, ry, rz = float(isin[0]), float(isin[1]), float(isin[2])
    d = float(derinlik_m)
    r = float(nesne_yaricap)

    if d <= 0.0:
        return (0.0, 0.0, 0.0)

    if oklid_mi:
        norm = math.sqrt(rx * rx + ry * ry + rz * rz)
        if norm < 1e-9:
            return (0.0, 0.0, d + r)
        ux, uy, uz = rx / norm, ry / norm, rz / norm
        d_toplam = d + r
        return (ux * d_toplam, uy * d_toplam, uz * d_toplam)

    # Standart ROS REP-118 derinliği: Z = derinlik_m
    if abs(rz) < 1e-9:
        s = 1.0
    else:
        s = d / rz

    px_surf = rx * s
    py_surf = ry * s
    pz_surf = d

    # Işın boyunca birim yön vektörü
    norm_surf = math.sqrt(px_surf * px_surf + py_surf * py_surf + pz_surf * pz_surf)
    if norm_surf < 1e-9:
        return (0.0, 0.0, d + r)

    ux = px_surf / norm_surf
    uy = py_surf / norm_surf
    uz = pz_surf / norm_surf

    return (px_surf + r * ux, py_surf + r * uy, pz_surf + r * uz)


def renk_derinlik_esle(
    u_renk: float,
    v_renk: float,
    derinlik_m: float,
    k_renk: Sequence[float] | np.ndarray,
    k_derinlik: Sequence[float] | np.ndarray,
    t_renk_derinlik: np.ndarray | None = None,
) -> tuple[float, float]:
    """Renk pikselini ve derinliği, dış kalibrasyonla derinlik kamera pikseline eşler.

    Args:
        u_renk: Renk görüntüsündeki sütun pikseli.
        v_renk: Renk görüntüsündeki satır pikseli.
        derinlik_m: Tahmini veya ölçülen derinlik (m).
        k_renk: Renk kamerası iç parametreleri.
        k_derinlik: Derinlik kamerası iç parametreleri.
        t_renk_derinlik: Renk optik çerçevesinden derinlik optik çerçevesine
                         4x4 homojen dönüşüm matrisi (P_d = T * P_c).
                         None verilirse eş konumlu (t=0, R=I) kabul edilir.

    Returns:
        (u_derinlik, v_derinlik): Derinlik görüntüsündeki piksel koordinatı.
    """
    fx_c, fy_c, cx_c, cy_c = k_ayristir(k_renk)
    fx_d, fy_d, cx_d, cy_d = k_ayristir(k_derinlik)

    d = max(1e-3, float(derinlik_m))
    xn_c = (float(u_renk) - cx_c) / fx_c
    yn_c = (float(v_renk) - cy_c) / fy_c

    # Renk optik çerçevesinde 3B nokta
    pc = np.array([xn_c * d, yn_c * d, d], dtype=np.float64)

    if t_renk_derinlik is not None:
        pc_h = np.array([pc[0], pc[1], pc[2], 1.0], dtype=np.float64)
        pd = (t_renk_derinlik @ pc_h)[:3]
    else:
        pd = pc

    zd = float(pd[2])
    if zd <= 1e-6:
        return (0.0, 0.0)

    ud = fx_d * (float(pd[0]) / zd) + cx_d
    vd = fy_d * (float(pd[1]) / zd) + cy_d
    return (float(ud), float(vd))


def derinlik_medyan_al(
    derinlik_goruntu: np.ndarray,
    u: float,
    v: float,
    pencere_boyut: int = 7,
    min_gecerli_m: float = 0.05,
    max_gecerli_m: float = 12.0,
) -> float | None:
    """Derinlik görüntüsünde (u, v) çevresindeki pencereden geçerli derinliklerin medyanını alır.

    Desteklenen derinlik formatları:
      - 16UC1 (uint16 milimetre cinsinden derinlik)
      - 32FC1 (float32 metre cinsinden derinlik)

    Args:
        derinlik_goruntu: 2B derinlik görüntüsü dizisi.
        u: Piksel sütunu (x).
        v: Piksel satırı (y).
        pencere_boyut: Pencere kenar uzunluğu (varsayılan 7x7).
        min_gecerli_m: Asgari geçerli derinlik eşiği (m).
        max_gecerli_m: Azami geçerli derinlik eşiği (m).

    Returns:
        Medyan derinlik değeri (metre cinsinden float) veya geçerli piksel yoksa None.
    """
    if derinlik_goruntu is None or derinlik_goruntu.size == 0:
        return None

    h, w = derinlik_goruntu.shape[:2]
    u0 = int(round(float(u)))
    v0 = int(round(float(v)))

    yari = pencere_boyut // 2
    u_min = max(0, u0 - yari)
    u_max = min(w, u0 + yari + 1)
    v_min = max(0, v0 - yari)
    v_max = min(h, v0 + yari + 1)

    if u_min >= u_max or v_min >= v_max:
        return None

    kirpma = derinlik_goruntu[v_min:v_max, u_min:u_max]

    if kirpma.dtype == np.uint16:
        # uint16 (mm) -> metreye çevir
        mask = (kirpma > 0) & (kirpma < 65535)
        if not np.any(mask):
            return None
        degerler = kirpma[mask].astype(np.float64) / 1000.0
    else:
        # float (m)
        mask = np.isfinite(kirpma) & (kirpma > 0.0)
        if not np.any(mask):
            return None
        degerler = kirpma[mask].astype(np.float64)

    gecerli = degerler[(degerler >= min_gecerli_m) & (degerler <= max_gecerli_m)]
    if len(gecerli) == 0:
        return None

    return float(np.median(gecerli))


# ==============================================================================
# 2. Yardımcı Dönüşümler (Görüntü ve TF)
# ==============================================================================

def goruntu_mesajindan_numpy(msg: Image) -> np.ndarray:
    """sensor_msgs/Image mesajını OpenCV uyumlu numpy dizisine dönüştürür."""
    enc = msg.encoding.lower()
    h, w = msg.height, msg.width

    if enc in ('bgr8', 'rgb8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size == h * w * 3:
            arr = arr.reshape((h, w, 3))
        elif msg.step > 0 and arr.size >= h * msg.step:
            arr = arr.reshape((h, msg.step))[:, :w * 3].reshape((h, w, 3))
        else:
            arr = arr[:h * w * 3].reshape((h, w, 3))
        if enc == 'rgb8':
            # Standart OpenCV BGR formatına çevir
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr
    elif enc == 'mono8':
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size == h * w:
            return arr.reshape((h, w))
        elif msg.step > 0 and arr.size >= h * msg.step:
            return arr.reshape((h, msg.step))[:, :w]
        else:
            return arr[:h * w].reshape((h, w))
    elif enc in ('16uc1', 'mono16'):
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        if arr.size == h * w:
            return arr.reshape((h, w))
        elif msg.step > 0 and arr.size >= h * (msg.step // 2):
            return arr.reshape((h, msg.step // 2))[:, :w]
        else:
            return arr[:h * w].reshape((h, w))
    elif enc == '32fc1':
        arr = np.frombuffer(msg.data, dtype=np.float32)
        if arr.size == h * w:
            return arr.reshape((h, w))
        elif msg.step > 0 and arr.size >= h * (msg.step // 4):
            return arr.reshape((h, msg.step // 4))[:, :w]
        else:
            return arr[:h * w].reshape((h, w))
    else:
        # Bilinmeyen formatlarda bayt verisini uint8 olarak dene
        return np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, -1))


def numpy_dan_goruntu_mesaji(
    arr: np.ndarray,
    encoding: str = 'bgr8',
    stamp=None,
    frame_id: str = '',
) -> Image:
    """OpenCV uyumlu numpy dizisini sensor_msgs/Image mesajına dönüştürür."""
    msg = Image()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0

    if arr.ndim == 2:
        channels = 1
        itemsize = arr.itemsize
    else:
        channels = arr.shape[2]
        itemsize = arr.itemsize

    msg.step = int(msg.width * channels * itemsize)
    msg.data = arr.tobytes()
    return msg


def quaternion_dan_rotasyon_matrisi(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion geometrisinden 3x3 rotasyon matrisi üretir."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


# ==============================================================================
# 3. RenkAlgiDugumu (ROS 2 Düğümü)
# ==============================================================================

class RenkAlgiDugumu(Node):
    """Kameradan renk ve derinlik füzyonu ile nesne algılayan ROS 2 düğümü."""

    # D435 varsayılan fabrika iç parametreleri (CameraInfo henüz gelmemişse yedek)
    # Renk: 1280x720, hfov 1.22693 rad -> fx = 908.97
    K_RENK_VARSAYILAN = np.array([
        [908.97, 0.0, 640.0],
        [0.0, 908.97, 360.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    # Derinlik: 848x480, hfov 1.55737 rad -> fx = 429.73
    K_DERINLIK_VARSAYILAN = np.array([
        [429.73, 0.0, 424.0],
        [0.0, 429.73, 240.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    # D435 fabrika dış kalibrasyonu (camera_color_optical_frame -> camera_depth_optical_frame):
    # camera_link'te renk kamerası derinliğe göre y=+0.0148 m soldadır.
    # Optik çerçevede (+X sağa) derinlik kamerası renk kamerasına göre +0.0148 m sağdadır;
    # dolayısıyla renk optik çerçevesinden derinlik optik çerçevesine dönüşümde tx=-0.0148 m'dir.
    T_RENK_DERINLIK_VARSAYILAN = np.array([
        [1.0, 0.0, 0.0, -0.0148],
        [0.0, 1.0, 0.0, -0.0002],
        [0.0, 0.0, 1.0,  0.0000],
        [0.0, 0.0, 0.0,  1.0000],
    ], dtype=np.float64)

    def __init__(self) -> None:
        super().__init__('renk_algi')

        # Düğüm Parametreleri
        p = self.declare_parameter
        p('hsv_alt', [0, 120, 70])
        p('hsv_ust', [10, 255, 255])
        p('hsv_alt2', [170, 120, 70])
        p('hsv_ust2', [180, 255, 255])
        p('min_alan_px', 150)
        p('nesne_yaricap', 0.035)
        p('hz', 5.0)
        p('hata_ayiklama', False)
        p('pencere_boyut', 7)
        p('cerceve', 'camera_color_optical_frame')
        p('derinlik_cerceve', 'camera_depth_optical_frame')

        p('renk_topic', '/camera/color/image_raw')
        p('renk_info_topic', '/camera/color/camera_info')
        p('derinlik_topic', '/camera/depth/image_raw')
        p('derinlik_info_topic', '/camera/depth/camera_info')
        p('algi_topic', '/kavrama/nesne_algi')
        p('durum_topic', '/kavrama/nesne_algi_durum')
        p('algi_goruntu_topic', '/kavrama/algi_goruntu')

        # Son alınan mesajlar ve durum
        self.son_renk_info: CameraInfo | None = None
        self.son_derinlik_info: CameraInfo | None = None
        self.son_derinlik_msg: Image | None = None
        self.goruyor: bool = False
        self.son_islem_zamani = self.get_clock().now()

        # TF2 Dinleyicisi
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Yayıncılar
        algi_topic = str(self.get_parameter('algi_topic').value)
        durum_topic = str(self.get_parameter('durum_topic').value)
        goruntu_topic = str(self.get_parameter('algi_goruntu_topic').value)

        self.pub_algi = self.create_publisher(PointStamped, algi_topic, 10)
        self.pub_durum = self.create_publisher(String, durum_topic, 10)
        self.pub_goruntu = self.create_publisher(Image, goruntu_topic, 10)

        # Abonelikler
        renk_topic = str(self.get_parameter('renk_topic').value)
        renk_info_topic = str(self.get_parameter('renk_info_topic').value)
        derinlik_topic = str(self.get_parameter('derinlik_topic').value)
        derinlik_info_topic = str(self.get_parameter('derinlik_info_topic').value)

        self.sub_renk_info = self.create_subscription(
            CameraInfo, renk_info_topic, self._renk_info_cb, qos_profile_sensor_data
        )
        self.sub_derinlik_info = self.create_subscription(
            CameraInfo, derinlik_info_topic, self._derinlik_info_cb, qos_profile_sensor_data
        )
        self.sub_derinlik = self.create_subscription(
            Image, derinlik_topic, self._derinlik_cb, qos_profile_sensor_data
        )
        self.sub_renk = self.create_subscription(
            Image, renk_topic, self._renk_cb, qos_profile_sensor_data
        )

        # Durum yayınlama döngüsü (1 Hz, sahte_algi arayüzü ile birebir uyumlu)
        self.timer_durum = self.create_timer(1.0, self._durum_dongusu)

        hz = float(self.get_parameter('hz').value)
        self.get_logger().info(
            f"renk_algi dugumu baslatildi: hz={hz}, min_alan_px={self.get_parameter('min_alan_px').value}, "
            f"cerceve='{self.get_parameter('cerceve').value}'"
        )

    def _renk_info_cb(self, msg: CameraInfo) -> None:
        self.son_renk_info = msg

    def _derinlik_info_cb(self, msg: CameraInfo) -> None:
        self.son_derinlik_info = msg

    def _derinlik_cb(self, msg: Image) -> None:
        self.son_derinlik_msg = msg

    def _durum_dongusu(self) -> None:
        durum_str = 'GORUYOR' if self.goruyor else 'GORMUYOR'
        self.pub_durum.publish(String(data=durum_str))

    def _tf_renk_derinlik_al(self) -> np.ndarray:
        """TF ağacından veya fabrika varsayılanından renk->derinlik optik dönüşümünü çeker."""
        derinlik_cerceve = str(self.get_parameter('derinlik_cerceve').value)
        renk_cerceve = str(self.get_parameter('cerceve').value)
        try:
            t = self.tf_buffer.lookup_transform(
                derinlik_cerceve,
                renk_cerceve,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05),
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            r = quaternion_dan_rotasyon_matrisi(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
            t_mat = np.eye(4, dtype=np.float64)
            t_mat[:3, :3] = r
            t_mat[:3, 3] = [tx, ty, tz]
            return t_mat
        except Exception:
            return self.T_RENK_DERINLIK_VARSAYILAN

    def _renk_cb(self, msg: Image) -> None:
        # Hız sınırı denetimi (hz)
        hz = float(self.get_parameter('hz').value)
        simdi = self.get_clock().now()
        dt = (simdi - self.son_islem_zamani).nanoseconds / 1e9
        asgari_aralik = 1.0 / max(1e-3, hz)

        if 0.0 <= dt < (asgari_aralik - 1e-4):
            return

        self.son_islem_zamani = simdi
        self._algi_yurut(msg)

    def _algi_yurut(self, renk_msg: Image) -> None:
        """Gelen renk görüntüsünü işler, derinlikle eşler ve nesne konumunu yayınlar."""
        try:
            bgr = goruntu_mesajindan_numpy(renk_msg)
        except Exception as e:
            self.get_logger().warn(f'Renk goruntu mesaji cozumlenemedi: {e}')
            self.goruyor = False
            return

        hsv_alt = list(self.get_parameter('hsv_alt').value)
        hsv_ust = list(self.get_parameter('hsv_ust').value)
        hsv_alt2 = list(self.get_parameter('hsv_alt2').value)
        hsv_ust2 = list(self.get_parameter('hsv_ust2').value)
        min_alan_px = int(self.get_parameter('min_alan_px').value)
        nesne_yaricap = float(self.get_parameter('nesne_yaricap').value)
        hata_ayiklama = bool(self.get_parameter('hata_ayiklama').value)
        pencere_boyut = int(self.get_parameter('pencere_boyut').value)
        cerceve = str(self.get_parameter('cerceve').value)

        # 1. Renk maskesi ve en büyük bileşen
        mask, alan = maske_bul(
            bgr,
            hsv_alt=hsv_alt,
            hsv_ust=hsv_ust,
            hsv_alt2=hsv_alt2,
            hsv_ust2=hsv_ust2,
            min_alan_px=min_alan_px,
        )

        merkez = merkez_bul(mask, min_alan_px=min_alan_px)

        if merkez is None:
            self.goruyor = False
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, None, None, renk_msg.header)
            return

        cx, cy = merkez

        # 2. Derinlik görüntüsü denetimi
        if self.son_derinlik_msg is None:
            self.get_logger().debug('Derinlik goruntusu henuz alinmadi')
            self.goruyor = False
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
            return

        try:
            derinlik_img = goruntu_mesajindan_numpy(self.son_derinlik_msg)
        except Exception as e:
            self.get_logger().warn(f'Derinlik goruntusu cozumlenemedi: {e}')
            self.goruyor = False
            return

        # İç parametreleri al (varsayılan yedekli)
        k_renk = (
            self.son_renk_info.k
            if self.son_renk_info is not None and any(self.son_renk_info.k)
            else self.K_RENK_VARSAYILAN
        )
        k_derinlik = (
            self.son_derinlik_info.k
            if self.son_derinlik_info is not None and any(self.son_derinlik_info.k)
            else self.K_DERINLIK_VARSAYILAN
        )

        # TF / dış kalibrasyon dönüşümü
        t_renk_derinlik = self._tf_renk_derinlik_al()

        # 3. İki aşamalı derinlik izdüşümü ve medyan alma
        # İlk aşama: 0.6 m nominal mesafe varsayımıyla ilk piksel kestirimi
        ud_ilk, vd_ilk = renk_derinlik_esle(
            cx, cy, 0.6, k_renk, k_derinlik, t_renk_derinlik=t_renk_derinlik
        )
        d_medyan = derinlik_medyan_al(
            derinlik_img, ud_ilk, vd_ilk, pencere_boyut=pencere_boyut
        )

        # İkinci aşama: Gerçek ölçülen derinlikle hassas piksel eşlemesi ve medyan yenileme
        if d_medyan is not None and d_medyan > 0.05:
            ud_son, vd_son = renk_derinlik_esle(
                cx, cy, d_medyan, k_renk, k_derinlik, t_renk_derinlik=t_renk_derinlik
            )
            d_rafine = derinlik_medyan_al(
                derinlik_img, ud_son, vd_son, pencere_boyut=pencere_boyut
            )
            if d_rafine is not None:
                d_medyan = d_rafine

        if d_medyan is None or d_medyan <= 0.05:
            self.goruyor = False
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
            return

        # 4. Renk ışını ve 3B konum kestirimi (kamera renk optik çerçevesinde)
        isin = piksel_isin(cx, cy, k_renk, birim=True)
        x_3b, y_3b, z_3b = isin_derinlik_3b(
            isin, d_medyan, nesne_yaricap=nesne_yaricap, oklid_mi=False
        )

        # 5. PointStamped yayını
        algi_msg = PointStamped()
        algi_msg.header.stamp = renk_msg.header.stamp
        algi_msg.header.frame_id = cerceve
        algi_msg.point.x = float(x_3b)
        algi_msg.point.y = float(y_3b)
        algi_msg.point.z = float(z_3b)

        self.pub_algi.publish(algi_msg)
        self.goruyor = True

        if hata_ayiklama:
            self._hata_ayiklama_yayinla(
                bgr, mask, (cx, cy), (x_3b, y_3b, z_3b), renk_msg.header
            )

    def _hata_ayiklama_yayinla(
        self,
        bgr: np.ndarray,
        mask: np.ndarray,
        merkez: tuple[float, float] | None,
        konum_3b: tuple[float, float, float] | None,
        header: Header,
    ) -> None:
        """Hata ayıklama görüntüsünü (/kavrama/algi_goruntu) oluşturur ve yayınlar."""
        gorsel = bgr.copy()

        # Maskelenen bölgeyi yarı şeffaf vurgula
        if mask is not None and mask.shape == bgr.shape[:2]:
            yesil_katman = np.zeros_like(bgr)
            yesil_katman[mask > 0] = (0, 160, 255)  # Turuncu/kırmızı vurgu
            gorsel = cv2.addWeighted(gorsel, 0.8, yesil_katman, 0.2, 0)

        if merkez is not None:
            cx, cy = int(round(merkez[0])), int(round(merkez[1]))
            cv2.circle(gorsel, (cx, cy), 8, (0, 255, 0), 2)
            cv2.drawMarker(
                gorsel, (cx, cy), (0, 255, 0),
                markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2
            )

            if konum_3b is not None:
                x, y, z = konum_3b
                etiket = f'Elma: X={x:+.2f}m Y={y:+.2f}m Z={z:.2f}m'
                cv2.putText(
                    gorsel, etiket, (cx + 12, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )
            else:
                cv2.putText(
                    gorsel, 'Derinlik Yok', (cx + 12, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
                )
        else:
            cv2.putText(
                gorsel, 'NESNE BULUNAMADI', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
            )

        msg = numpy_dan_goruntu_mesaji(
            gorsel, encoding='bgr8', stamp=header.stamp, frame_id=header.frame_id
        )
        self.pub_goruntu.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    dugum = RenkAlgiDugumu()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
