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
  6. İki aşamalı derinlik kestirimi ile leke kutusunun iç bölgesinden (kenardan %25 içerideki
     çekirdek alan) geçerli derinlik pikselleri toplanır ve düşük yüzdelik (derinlik_yuzdelik=30)
     seçilir. Bu sayede leke kenarına ve arkasındaki zemine taşan derinlik sıçramaları önlenir.
     Piksel sayısı min_derinlik_piksel (6) altındaysa 7x7 pencere-medyan yedek yoluna dönülür.
  7. Tutarlılık denetimi: Seçilen derinliğin ±derinlik_bant_m (0.05 m) bandındaki geçerli piksel
     oranı <%40 ise karışık arka plan kabul edilir ve ölçüm yayınlanmaz.
  8. Azami menzil denetimi: maks_menzil_m (3.0 m) üzerindeki uzak ölçümler yayınlanmaz.
  9. Yüzey-Merkez Düzeltmesi Analizi:
     Derinlik kamerası nesnenin kameraya bakan ön yüzeyini ölçer. Küre merkezi için nesne yarıçapı
     kadar (nesne_yaricap=0.035 m) ışın boyunca 3B uzayda ilerletme yapılır (isin_derinlik_3b).
     Öteleme fiziksel olarak doğrudur (ön yüzey derinliği + r = merkez). Simde menzille büyüyen sistematik
     ileri sapma ölçüldü (0.8 m'de +2.4 cm, 3 m'de +4 cm); KAYNAĞI DOĞRULANMADI (adaylar: sensör modelinin
     derinlik ölçek hatası, renk-derinlik eşleme, piksel seçimi). Son yaklaşmada (<0.6 m) sapma <1 cm.
 10. Nesne merkezinin 3B konumu PointStamped olarak /kavrama/nesne_algi konusuna,
     görüş durumu ise /kavrama/nesne_algi_durum konusuna ('GORUYOR'/'GORMUYOR'/'KISMI') yayınlanır.
     'KISMI' durumu: Lekenin sınırlayıcı kutusu görüntü kenarına kenar_payi_px kadar
     yakınsa ölçüm yayınlanmaz ve kısmi görünüm durumu yayınlanır.
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
    kutu_ver: bool = False,
) -> tuple[np.ndarray, int] | tuple[np.ndarray, int, tuple[int, int, int, int] | None]:
    """Maskedeki bağlı bileşenleri analiz eder ve en büyük bileşeni döndürür.

    min_alan_px değerinden küçük bileşenler elenir (küçük leke reddi).

    Args:
        maske: İkili maske görüntüsü.
        min_alan_px: Asgari geçerli bileşen alanı.
        kutu_ver: True ise (temiz_maske, alan, kutu), False ise (temiz_maske, alan) döner.

    Returns:
        kutu_ver False ise (temiz_maske, alan),
        kutu_ver True ise (temiz_maske, alan, kutu) burada kutu=(x, y, w, h).
    """
    if maske is None or maske.size == 0 or cv2.countNonZero(maske) == 0:
        bos = np.zeros_like(maske) if maske is not None else np.empty((0, 0), dtype=np.uint8)
        return (bos, 0, None) if kutu_ver else (bos, 0)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(maske, connectivity=8)

    # 0 etiketi arka plandır, 1..num_labels-1 ön plan bileşenleridir.
    if num_labels <= 1:
        bos = np.zeros_like(maske)
        return (bos, 0, None) if kutu_ver else (bos, 0)

    en_buyuk_alan = 0
    en_buyuk_idx = -1

    for i in range(1, num_labels):
        alan = int(stats[i, cv2.CC_STAT_AREA])
        if alan >= min_alan_px and alan > en_buyuk_alan:
            en_buyuk_alan = alan
            en_buyuk_idx = i

    if en_buyuk_idx == -1:
        bos = np.zeros_like(maske)
        return (bos, 0, None) if kutu_ver else (bos, 0)

    temiz = np.zeros_like(maske)
    temiz[labels == en_buyuk_idx] = 255

    if kutu_ver:
        kutu = (
            int(stats[en_buyuk_idx, cv2.CC_STAT_LEFT]),
            int(stats[en_buyuk_idx, cv2.CC_STAT_TOP]),
            int(stats[en_buyuk_idx, cv2.CC_STAT_WIDTH]),
            int(stats[en_buyuk_idx, cv2.CC_STAT_HEIGHT]),
        )
        return temiz, en_buyuk_alan, kutu

    return temiz, en_buyuk_alan


def sinirlayici_kutu_bul(
    maske: np.ndarray,
    min_alan_px: int = 150,
) -> tuple[int, int, int, int] | None:
    """Maskedeki en büyük bağlı bileşenin sınırlayıcı kutusunu (x, y, w, h) döner."""
    sonuc = en_buyuk_bilesen_filtrele(maske, min_alan_px=min_alan_px, kutu_ver=True)
    _, alan, kutu = sonuc  # type: ignore[misc]
    return kutu if alan >= min_alan_px else None


def kenara_yakin_mi(
    kutu: tuple[int, int, int, int],
    goruntu_sekli: tuple[int, int] | Sequence[int] | np.ndarray,
    kenar_payi_px: int = 4,
) -> bool:
    """Sınırlayıcı kutunun (x, y, w, h) görüntü kenarlarına kenar_payi_px kadar yakın olup olmadığını denetler.

    Args:
        kutu: (x, y, w, h) sınırlayıcı kutu piksel koordinatları.
        goruntu_sekli: (yukseklik, genislik) / (genislik, yukseklik) veya görüntü dizisi.
        kenar_payi_px: Kenara yakınlık emniyet payı (piksel).

    Returns:
        Kutu herhangi bir kenara kenar_payi_px kadar yakınsa True, değilse False.
    """
    if kutu is None or len(kutu) < 4:
        return False
    x, y, w, h = int(kutu[0]), int(kutu[1]), int(kutu[2]), int(kutu[3])

    if hasattr(goruntu_sekli, 'shape'):
        H, W = int(goruntu_sekli.shape[0]), int(goruntu_sekli.shape[1])
    elif len(goruntu_sekli) >= 2:
        d0, d1 = int(goruntu_sekli[0]), int(goruntu_sekli[1])
        if d0 > d1:
            W, H = d0, d1
        else:
            H, W = d0, d1
    else:
        raise ValueError(f'Geçersiz görüntü boyutu: {goruntu_sekli}')

    pay = max(0, int(kenar_payi_px))

    if x <= pay:
        return True
    if y <= pay:
        return True
    if (x + w) >= (W - pay):
        return True
    if (y + h) >= (H - pay):
        return True
    return False


def gorunen_yaricap_px_hesapla(
    alan: int | float = 0,
    kutu: tuple[int, int, int, int] | None = None,
) -> float:
    """Lekenin piksel alanından veya sınırlayıcı kutusundan görünen yarıçapı hesaplar.

    Daire için r = sqrt(alan / pi). Alan yoksa kutunun ortalama yarıçapı (w + h) / 4 alınır.
    """
    if alan > 0:
        return math.sqrt(float(alan) / math.pi)
    if kutu is not None and len(kutu) >= 4:
        return float(kutu[2] + kutu[3]) / 4.0
    return 0.0


def fiziksel_yaricap_kestir(
    gorunen_yaricap_px: float,
    derinlik_m: float,
    fx: float,
) -> float:
    """Görünen piksel yarıçapı, derinlik ve odak uzaklığından fiziksel yarıçapı kestirir.

    r_fiziksel = r_px · derinlik / fx
    """
    if fx <= 0.0:
        return 0.0
    return float(gorunen_yaricap_px) * float(derinlik_m) / float(fx)


def boyut_uygun_mu(
    gorunen_yaricap_px: float,
    derinlik_m: float,
    fx: float,
    beklenen_yaricap_m: float = 0.035,
    tolerans_orani: float = 0.60,
) -> bool:
    """Kestirilen fiziksel yarıçapın beklenen yarıçap toleransı (±%60) içinde olup olmadığını denetler.

    Args:
        gorunen_yaricap_px: Görüntüdeki nesne yarıçapı (piksel).
        derinlik_m: Nesne yüzeyine ölçülen derinlik mesafesi (m).
        fx: Renk kamerası yatay odak uzaklığı (piksel).
        beklenen_yaricap_m: Beklenen fiziksel yarıçap (m, varsayılan 0.035 m).
        tolerans_orani: Kabul edilebilir azami bağıl sapma (varsayılan 0.60, yani ±%60).

    Returns:
        Kestirilen boyut beklenen aralıktaysa True, tutarsızsa False.
    """
    if fx <= 0.0 or derinlik_m <= 0.0 or gorunen_yaricap_px <= 0.0:
        return False
    r_kestirilen = gorunen_yaricap_px * derinlik_m / fx
    r_min = beklenen_yaricap_m * (1.0 - tolerans_orani)
    r_max = beklenen_yaricap_m * (1.0 + tolerans_orani)
    return r_min <= r_kestirilen <= r_max


def maske_bul(
    goruntu: np.ndarray,
    hsv_alt: Sequence[int] = (0, 120, 70),
    hsv_ust: Sequence[int] = (10, 255, 255),
    hsv_alt2: Sequence[int] = (170, 120, 70),
    hsv_ust2: Sequence[int] = (180, 255, 255),
    min_alan_px: int = 150,
    rgb_mi: bool = False,
    morfoloji_kernel: int = 3,
    kutu_ver: bool = False,
) -> tuple[np.ndarray, int] | tuple[np.ndarray, int, tuple[int, int, int, int] | None]:
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
    return en_buyuk_bilesen_filtrele(ham_maske, min_alan_px=min_alan_px, kutu_ver=kutu_ver)


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


def derinlik_gecerli_pikselleri_ayikla(
    derinlik_verisi: np.ndarray | Sequence[float],
    min_gecerli_m: float = 0.05,
    max_gecerli_m: float = 12.0,
    maske: np.ndarray | None = None,
) -> np.ndarray:
    """Derinlik görüntüsü kırpmasından veya dizisinden geçerli derinlik değerlerini (m) ayıklar.

    Desteklenen formatlar:
      - 2B veya 1B uint16 (mm) -> metreye çevrilir
      - 2B veya 1B float32/float64 (m)
      - 1B float listesi/dizisi

    Args:
        derinlik_verisi: Derinlik kırpması (2B) veya 1B değer dizisi.
        min_gecerli_m: Asgari geçerli derinlik eşiği (m).
        max_gecerli_m: Azami geçerli derinlik eşiği (m).
        maske: İsteğe bağlı 2B ikili maske (yalnızca maske > 0 pikselleri alınır).

    Returns:
        1B float64 metre cinsinden geçerli değerler dizisi.
    """
    if derinlik_verisi is None:
        return np.empty(0, dtype=np.float64)

    arr = np.asarray(derinlik_verisi)
    if arr.size == 0:
        return np.empty(0, dtype=np.float64)

    if maske is not None and maske.shape == arr.shape:
        arr = arr[maske > 0]
        if arr.size == 0:
            return np.empty(0, dtype=np.float64)

    if arr.dtype == np.uint16:
        gecerli_mask = (arr > 0) & (arr < 65535)
        if not np.any(gecerli_mask):
            return np.empty(0, dtype=np.float64)
        degerler = arr[gecerli_mask].astype(np.float64) / 1000.0
    else:
        arr_f = arr.astype(np.float64)
        gecerli_mask = np.isfinite(arr_f) & (arr_f > 0.0)
        if not np.any(gecerli_mask):
            return np.empty(0, dtype=np.float64)
        degerler = arr_f[gecerli_mask]

    filtre = (degerler >= float(min_gecerli_m)) & (degerler <= float(max_gecerli_m))
    return degerler[filtre]


def leke_kutusu_ic_sinirlari(
    u_merkez: float,
    v_merkez: float,
    genislik: float,
    yukseklik: float,
    kenar_payi_orani: float = 0.25,
    goruntu_genislik: int | None = None,
    goruntu_yukseklik: int | None = None,
) -> tuple[int, int, int, int]:
    """Leke merkez ve boyutundan kenardan kenar_payi_orani kadar içerideki ROI sınırlarını hesaplar.

    Kenardan %25 içeri girildiğinde (kenar_payi_orani=0.25), kutunun orta %50'lik çekirdek
    bölgesi seçilir. Bu, nesne siluet kenarındaki zemin/arka plan sızıntılarını önler.

    Args:
        u_merkez: Leke merkezinin sütun koordinatı (x).
        v_merkez: Leke merkezinin satır koordinatı (y).
        genislik: Leke sınırlayıcı kutusunun genişliği (px).
        yukseklik: Leke sınırlayıcı kutusunun yüksekliği (px).
        kenar_payi_orani: Dış kenarlardan içeri girilecek oran (varsayılan 0.25, yani %25).
        goruntu_genislik: Görüntü genişlik sınırı (isteğe bağlı kırpma).
        goruntu_yukseklik: Görüntü yükseklik sınırı (isteğe bağlı kırpma).

    Returns:
        (u_min, u_max, v_min, v_max): Dilimleme sınırları.
    """
    pay = max(0.0, min(0.45, float(kenar_payi_orani)))
    w_ic = max(1.0, float(genislik) * (1.0 - 2.0 * pay))
    h_ic = max(1.0, float(yukseklik) * (1.0 - 2.0 * pay))

    u_min = int(round(float(u_merkez) - w_ic / 2.0))
    u_max = int(round(float(u_merkez) + w_ic / 2.0))
    v_min = int(round(float(v_merkez) - h_ic / 2.0))
    v_max = int(round(float(v_merkez) + h_ic / 2.0))

    if goruntu_genislik is not None:
        u_min = max(0, min(int(goruntu_genislik), u_min))
        u_max = max(0, min(int(goruntu_genislik), u_max))
    if goruntu_yukseklik is not None:
        v_min = max(0, min(int(goruntu_yukseklik), v_min))
        v_max = max(0, min(int(goruntu_yukseklik), v_max))

    return (u_min, u_max, v_min, v_max)


def derinlik_yuzdelik_al(
    derinlikler: Sequence[float] | np.ndarray,
    yuzdelik: float = 30.0,
) -> float | None:
    """Geçerli derinlik değerleri dizisinden düşük yüzdeliği (vars. 30. yüzdelik) seçer.

    Medyan yerine düşük yüzdelik kullanılması, nesnenin ön yüzeyini temsil eden en yakın
    sağlam değerlerin seçilmesini sağlar ve arka plan/zemin sıçramalarını önler.

    Args:
        derinlikler: 1B geçerli derinlik değerleri (metre).
        yuzdelik: Seçilecek yüzdelik (0-100 arası, varsayılan 30).

    Returns:
        Seçilen derinlik (m) veya dizi boşsa None.
    """
    if derinlikler is None:
        return None
    arr = np.asarray(derinlikler, dtype=np.float64)
    if arr.size == 0:
        return None
    arr = arr[np.isfinite(arr) & (arr > 0.0)]
    if arr.size == 0:
        return None
    q = max(0.0, min(100.0, float(yuzdelik)))
    return float(np.percentile(arr, q))


def derinlik_tutarlilik_orani(
    derinlikler: Sequence[float] | np.ndarray,
    secilen_derinlik: float,
    derinlik_bant_m: float = 0.05,
) -> float:
    """Seçilen derinliğin ±derinlik_bant_m civarındaki piksel oranını hesaplar."""
    if derinlikler is None or secilen_derinlik is None:
        return 0.0
    arr = np.asarray(derinlikler, dtype=np.float64)
    if arr.size == 0:
        return 0.0
    arr = arr[np.isfinite(arr) & (arr > 0.0)]
    if arr.size == 0:
        return 0.0
    fark = np.abs(arr - float(secilen_derinlik))
    sayi = np.count_nonzero(fark <= float(derinlik_bant_m))
    return float(sayi / arr.size)


def derinlik_tutarlilik_uygun_mu(
    derinlikler: Sequence[float] | np.ndarray,
    secilen_derinlik: float,
    derinlik_bant_m: float = 0.05,
    min_oran: float = 0.40,
) -> bool:
    """Seçilen derinliğin ±derinlik_bant_m civarındaki piksel oranının >= min_oran olup olmadığını denetler."""
    oran = derinlik_tutarlilik_orani(derinlikler, secilen_derinlik, derinlik_bant_m=derinlik_bant_m)
    return bool(oran >= float(min_oran))


def menzil_uygun_mu(
    derinlik_m: float | None,
    maks_menzil_m: float = 3.0,
    min_menzil_m: float = 0.05,
) -> bool:
    """Ölçülen derinliğin geçerli menzil [min_menzil_m, maks_menzil_m] içinde olup olmadığını denetler."""
    if derinlik_m is None or not math.isfinite(derinlik_m):
        return False
    d = float(derinlik_m)
    return float(min_menzil_m) <= d <= float(maks_menzil_m)


def derinlik_saglam_hesapla(
    derinlikler: Sequence[float] | np.ndarray,
    yedek_derinlik: float | None = None,
    derinlik_yuzdelik: float = 30.0,
    min_derinlik_piksel: int = 6,
    derinlik_bant_m: float = 0.05,
    min_tutarlilik_orani: float = 0.40,
    maks_menzil_m: float = 3.0,
    durum_ver: bool = False,
) -> float | tuple[float | None, str] | None:
    """Leke piksellerinden sağlam derinlik kestirimi, tutarlılık ve menzil denetimini yürütür.

    İşlem Adımları:
      1. Piksel sayısı < min_derinlik_piksel ise yedek yola (pencere medyanı) döner.
      2. Yeterli piksel varsa derinlik_yuzdelik (vars. 30) ile düşük yüzdeliği seçer.
      3. Tutarlılık denetimi: Seçilen derinlik etrafındaki ±derinlik_bant_m içindeki piksel
         oranı < min_tutarlilik_orani (%40) ise karışık arka plan reddi uygulanır (None).
      4. Azami menzil denetimi: d > maks_menzil_m ise ölçüm reddedilir (None).

    Args:
        derinlikler: Leke maskesi/iç bölgesinden toplanan derinlik değerleri.
        yedek_derinlik: Yetersiz piksel durumunda kullanılacak yedek derinlik (pencere medyanı).
        derinlik_yuzdelik: Düşük yüzdelik parametresi (varsayılan 30).
        min_derinlik_piksel: Asgari leke içi derinlik piksel sayısı (varsayılan 6).
        derinlik_bant_m: Tutarlılık bandı toleransı (metre, varsayılan 0.05 m).
        min_tutarlilik_orani: Asgari bant içi piksel oranı (varsayılan 0.40, yani %40).
        maks_menzil_m: Azami yayınlama menzili (metre, varsayılan 3.0 m).
        durum_ver: True ise (derinlik, durum_kodu), False ise derinlik döner.

    Returns:
        durum_ver False ise float derinlik veya None,
        durum_ver True ise (derinlik, durum_kodu) demeti döner.
        Durum kodları: 'GECERLI', 'YEDEK_YOL', 'AZ_PIKSEL', 'TUTARSIZ_ARKA_PLAN',
                       'MAKS_MENZIL_ASILDI', 'GECERSIZ_PIKSEL'.
    """
    arr = np.empty(0, dtype=np.float64)
    if derinlikler is not None:
        arr_raw = np.asarray(derinlikler, dtype=np.float64)
        if arr_raw.size > 0:
            arr = arr_raw[np.isfinite(arr_raw) & (arr_raw > 0.0)]

    # 1. Asgari piksel sayısı denetimi ve yedek yol
    if arr.size < int(min_derinlik_piksel):
        if yedek_derinlik is not None and math.isfinite(yedek_derinlik) and yedek_derinlik > 0.0:
            d_yedek = float(yedek_derinlik)
            if not menzil_uygun_mu(d_yedek, maks_menzil_m=maks_menzil_m):
                return (None, 'MAKS_MENZIL_ASILDI') if durum_ver else None
            return (d_yedek, 'YEDEK_YOL') if durum_ver else d_yedek
        return (None, 'AZ_PIKSEL') if durum_ver else None

    # 2. Düşük yüzdelik seçimi
    d_secilen = derinlik_yuzdelik_al(arr, yuzdelik=derinlik_yuzdelik)
    if d_secilen is None:
        return (None, 'GECERSIZ_PIKSEL') if durum_ver else None

    # 3. Tutarlılık denetimi (karışık arka plan reddi)
    if not derinlik_tutarlilik_uygun_mu(
        arr, d_secilen, derinlik_bant_m=derinlik_bant_m, min_oran=min_tutarlilik_orani
    ):
        return (None, 'TUTARSIZ_ARKA_PLAN') if durum_ver else None

    # 4. Azami menzil denetimi
    if not menzil_uygun_mu(d_secilen, maks_menzil_m=maks_menzil_m):
        return (None, 'MAKS_MENZIL_ASILDI') if durum_ver else None

    return (d_secilen, 'GECERLI') if durum_ver else d_secilen


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

        # N17: Kenar payı ve boyut denetimi parametreleri
        p('kenar_payi_px', 4)
        p('beklenen_yaricap_m', 0.035)
        p('boyut_denetimi', True)

        # N20: Derinlik leke maskesi, yüzdelik, tutarlılık ve azami menzil parametreleri
        p('derinlik_yuzdelik', 30)
        p('min_derinlik_piksel', 6)
        p('derinlik_bant_m', 0.05)
        p('maks_menzil_m', 3.0)

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
        self.durum: str = 'GORMUYOR'
        self.son_islem_zamani = self.get_clock().now()

        # Teşhis ve hata ayıklama sayaçları (N20)
        self.sayac_yedek_derinlik: int = 0
        self.sayac_tutarsiz_derinlik: int = 0
        self.sayac_asiri_menzil: int = 0

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
        self.pub_durum.publish(String(data=self.durum))

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
            self.durum = 'GORMUYOR'
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
        kenar_payi_px = int(self.get_parameter('kenar_payi_px').value)
        beklenen_yaricap_m = float(self.get_parameter('beklenen_yaricap_m').value)
        boyut_denetimi = bool(self.get_parameter('boyut_denetimi').value)
        derinlik_yuzdelik = float(self.get_parameter('derinlik_yuzdelik').value)
        min_derinlik_piksel = int(self.get_parameter('min_derinlik_piksel').value)
        derinlik_bant_m = float(self.get_parameter('derinlik_bant_m').value)
        maks_menzil_m = float(self.get_parameter('maks_menzil_m').value)

        # 1. Renk maskesi, en büyük bileşen ve sınırlayıcı kutu
        mask, alan, kutu = maske_bul(
            bgr,
            hsv_alt=hsv_alt,
            hsv_ust=hsv_ust,
            hsv_alt2=hsv_alt2,
            hsv_ust2=hsv_ust2,
            min_alan_px=min_alan_px,
            kutu_ver=True,
        )

        merkez = merkez_bul(mask, min_alan_px=min_alan_px)

        if merkez is None or kutu is None:
            self.goruyor = False
            self.durum = 'GORMUYOR'
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, None, None, renk_msg.header)
            return

        cx, cy = merkez

        # Kenar denetimi: leke sınırlayıcı kutusu görüntü kenarına kenar_payi_px kadar yakınsa
        # kısmi görünüm sayılır; ölçüm yayınlanmaz ve durum 'KISMI' bildirilir.
        if kenara_yakin_mi(kutu, bgr.shape[:2], kenar_payi_px=kenar_payi_px):
            self.goruyor = False
            self.durum = 'KISMI'
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
            return

        # 2. Derinlik görüntüsü denetimi
        if self.son_derinlik_msg is None:
            self.get_logger().debug('Derinlik goruntusu henuz alinmadi')
            self.goruyor = False
            self.durum = 'GORMUYOR'
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
            return

        try:
            derinlik_img = goruntu_mesajindan_numpy(self.son_derinlik_msg)
        except Exception as e:
            self.get_logger().warn(f'Derinlik goruntusu cozumlenemedi: {e}')
            self.goruyor = False
            self.durum = 'GORMUYOR'
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

        # 3. İki aşamalı derinlik izdüşümü ve leke maskesi/iç bölgesinden sağlam derinlik kestirimi
        fx_c, fy_c, _, _ = k_ayristir(k_renk)
        fx_d, fy_d, _, _ = k_ayristir(k_derinlik)
        w_d = float(kutu[2]) * (fx_d / fx_c)
        h_d = float(kutu[3]) * (fy_d / fy_c)
        H_d, W_d = derinlik_img.shape[:2]

        # İlk aşama: 0.6 m nominal mesafe varsayımıyla ilk piksel kestirimi
        ud_ilk, vd_ilk = renk_derinlik_esle(
            cx, cy, 0.6, k_renk, k_derinlik, t_renk_derinlik=t_renk_derinlik
        )
        u_min1, u_max1, v_min1, v_max1 = leke_kutusu_ic_sinirlari(
            ud_ilk, vd_ilk, w_d, h_d, kenar_payi_orani=0.25,
            goruntu_genislik=W_d, goruntu_yukseklik=H_d,
        )
        pikseller_ilk = derinlik_gecerli_pikselleri_ayikla(derinlik_img[v_min1:v_max1, u_min1:u_max1])
        d_yedek_ilk = derinlik_medyan_al(
            derinlik_img, ud_ilk, vd_ilk, pencere_boyut=pencere_boyut
        )

        if pikseller_ilk.size >= min_derinlik_piksel:
            d_tahmin = derinlik_yuzdelik_al(pikseller_ilk, yuzdelik=derinlik_yuzdelik)
        else:
            d_tahmin = d_yedek_ilk

        # İkinci aşama: Gerçek ölçülen derinlikle hassas piksel eşlemesi ve derinlik yenileme
        pikseller_son = pikseller_ilk
        d_yedek_son = d_yedek_ilk

        if d_tahmin is not None and d_tahmin > 0.05:
            ud_son, vd_son = renk_derinlik_esle(
                cx, cy, d_tahmin, k_renk, k_derinlik, t_renk_derinlik=t_renk_derinlik
            )
            u_min2, u_max2, v_min2, v_max2 = leke_kutusu_ic_sinirlari(
                ud_son, vd_son, w_d, h_d, kenar_payi_orani=0.25,
                goruntu_genislik=W_d, goruntu_yukseklik=H_d,
            )
            pikseller_rafine = derinlik_gecerli_pikselleri_ayikla(derinlik_img[v_min2:v_max2, u_min2:u_max2])
            d_yedek_rafine = derinlik_medyan_al(
                derinlik_img, ud_son, vd_son, pencere_boyut=pencere_boyut
            )
            if pikseller_rafine.size >= min_derinlik_piksel or d_yedek_rafine is not None:
                pikseller_son = pikseller_rafine
                d_yedek_son = d_yedek_rafine

        # Sağlam derinlik kestirimi (yüzdelik, piksel sayısı, tutarlılık ve azami menzil denetimi)
        d_sonuc = derinlik_saglam_hesapla(
            pikseller_son,
            yedek_derinlik=d_yedek_son,
            derinlik_yuzdelik=derinlik_yuzdelik,
            min_derinlik_piksel=min_derinlik_piksel,
            derinlik_bant_m=derinlik_bant_m,
            min_tutarlilik_orani=0.40,
            maks_menzil_m=maks_menzil_m,
            durum_ver=True,
        )
        d_secilen, durum_neden = d_sonuc  # type: ignore[misc]

        if durum_neden == 'YEDEK_YOL':
            self.sayac_yedek_derinlik += 1
            self.get_logger().debug(
                f'Leke derinlik pikseli yetersiz ({pikseller_son.size} < {min_derinlik_piksel}), '
                f'yedek pencere medyanina donuldu: {d_secilen:.3f}m'
            )
        elif durum_neden == 'TUTARSIZ_ARKA_PLAN':
            self.sayac_tutarsiz_derinlik += 1
            self.get_logger().debug(
                f'Derinlik tutarsiz (karisik arka plan reddi): '
                f'piksel_sayisi={pikseller_son.size}, bant=±{derinlik_bant_m:.2f}m'
            )
            self.goruyor = False
            self.durum = 'GORMUYOR'
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
            return
        elif durum_neden == 'MAKS_MENZIL_ASILDI':
            self.sayac_asiri_menzil += 1
            self.get_logger().debug(
                f'Derinlik azami menzili asti (maks={maks_menzil_m:.2f}m)'
            )
            self.goruyor = False
            self.durum = 'GORMUYOR'
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
            return

        if d_secilen is None or d_secilen <= 0.05:
            self.goruyor = False
            self.durum = 'GORMUYOR'
            if hata_ayiklama:
                self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
            return

        # Boyut denetimi: derinlik ile leke boyutunun tutarlılığı
        # (görünen yarıçap_px · derinlik / fx ile kestirilen fiziksel yarıçap, beklenen ±%60 dışında ise reddet)
        if boyut_denetimi:
            fx, _, _, _ = k_ayristir(k_renk)
            r_px = gorunen_yaricap_px_hesapla(alan, kutu)
            if not boyut_uygun_mu(
                r_px, d_secilen, fx, beklenen_yaricap_m=beklenen_yaricap_m, tolerans_orani=0.60
            ):
                self.get_logger().debug(
                    f'Boyut tutarsiz: r_px={r_px:.1f}, d={d_secilen:.3f}m, '
                    f'r_kestirilen={(r_px * d_secilen / fx):.4f}m, beklenen={beklenen_yaricap_m:.4f}m'
                )
                self.goruyor = False
                self.durum = 'GORMUYOR'
                if hata_ayiklama:
                    self._hata_ayiklama_yayinla(bgr, mask, (cx, cy), None, renk_msg.header)
                return

        # 4. Renk ışını ve 3B konum kestirimi (kamera renk optik çerçevesinde)
        isin = piksel_isin(cx, cy, k_renk, birim=True)
        x_3b, y_3b, z_3b = isin_derinlik_3b(
            isin, d_secilen, nesne_yaricap=nesne_yaricap, oklid_mi=False
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
        self.durum = 'GORUYOR'

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
            elif self.durum == 'KISMI':
                cv2.putText(
                    gorsel, 'KISMI GORUNUM (KENAR REDDI)', (cx + 12, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2
                )
            else:
                cv2.putText(
                    gorsel, 'Derinlik Yok', (cx + 12, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
                )
        elif self.durum == 'KISMI':
            cv2.putText(
                gorsel, 'KISMI GORUNUM (KENAR REDDI)', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2
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
