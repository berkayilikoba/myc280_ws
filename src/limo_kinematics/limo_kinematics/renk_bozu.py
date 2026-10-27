"""Gerçek D435 renk kamerası (RGB/BGR) bozulma modeli — saf fonksiyonlar (ROS'suz, numpy + cv2).

Gazebo'daki kusursuz renk görüntüsünü gerçek Intel RealSense D435 renk görüntüsüne istatistiksel
olarak benzetmek için dört ardışık etki uygular:
  1. hareket_bulanikligi : Yaw dönüşünden kaynaklanan yatay çizgi bulanıklığı (L = |w_z| * fx * t_poz)
  2. vinyet_uygula       : Merkezden köşeye radyal parlaklık düşüşü (1 + k1*r^2 + k2*r^4, köşede r=1)
  3. beyaz_denge_uygula  : Kanal başına BGR kazanç çarpanı
  4. gurultu_uygula      : Kanal başına parlaklığa bağlı Poisson-Gauss gürültüsü (sigma^2(I) = a*I + b)
"""

from functools import lru_cache
import math

import cv2
import numpy as np


# Sim D435 renk kamerası (1280x720) için, gerçek kayıtlardan (saha/oda/renk_karakter.py, 3 sahne, 1500 kare, 27.10.2026):
#   gürültü: 640x360'ta σ² = b (a ≈ 0; Poisson kısmı ölçülemeyecek kadar küçük), b = (1.36, 1.26, 1.44) BGR;
#            2x küçültme 4 pikseli ortalar -> 1280x720'de varyans x4.
#   t_poz:   Laplacian keskinliği 0.2 rad/s dönüşte ~%45'e düşüyor -> efektif pozlama 60 ms (iç mekân oto pozlama).
#   vinyet:  sahne içeriğinden ayrılamadı (üst duvar parlak, zemin koyu; ölçüm köşeyi %29 parlak buldu) -> kapalı.
#   beyaz denge: sim dokuları zaten gerçek karelerden -> kapalı (1.0).
VARSAYILAN_PARAM = {
    "k1": 0.0,
    "k2": 0.0,
    "a": [0.0, 0.0, 0.0],
    "b": [5.43, 5.02, 5.77],
    "t_poz": 0.060,
    "fx": 909.0,
    "derinlik_ortalama": 2.0,
    "kazanc_bgr": [1.0, 1.0, 1.0],
}


def _kanal_vektoru(deger, kanal_sayisi=3):
    """Skaler, liste/dizi veya sözlük biçimindeki kanal parametresini yayınlanabilir float32 diziye çevirir."""
    if isinstance(deger, dict):
        for anahtarlar in (("b", "g", "r"), ("B", "G", "R")):
            if all(k in deger for k in anahtarlar):
                deger = [deger[k] for k in anahtarlar]
                break
        else:
            deger = list(deger.values())
    arr = np.asarray(deger, dtype=np.float32)
    if arr.ndim == 0 or arr.size == 1:
        return np.float32(arr.reshape(-1)[0])
    if kanal_sayisi == 1:
        return np.float32(arr.mean())
    return arr.reshape((1, 1, kanal_sayisi))


@lru_cache(maxsize=16)
def _vinyet_maskesi(yukseklik: int, genislik: int, k1: float, k2: float) -> np.ndarray:
    """(H, W) boyutunda radyal vinyet çarpan maskesi üretir (merkezde r=0, köşelerde r=1)."""
    cx = (genislik - 1) * 0.5
    cy = (yukseklik - 1) * 0.5
    payda = max(cx * cx + cy * cy, 1e-12)
    u = np.arange(genislik, dtype=np.float32) - cx
    v = np.arange(yukseklik, dtype=np.float32) - cy
    r2 = (v[:, None] * v[:, None] + u[None, :] * u[None, :]) / payda
    maske = 1.0 + float(k1) * r2 + float(k2) * (r2 * r2)
    return np.clip(maske, 0.0, 2.0).astype(np.float32)


def vinyet_uygula(img: np.ndarray, k1: float, k2: float) -> np.ndarray:
    """Görüntüye I(r)/I(0) = 1 + k1*r^2 + k2*r^4 radyal vinyet uygular (köşede r=1).

    Args:
        img: (H, W, 3) veya (H, W) boyutunda BGR/gri görüntü.
        k1: İkinci dereceden radyal düşüş katsayısı (genelde negatif).
        k2: Dördüncü dereceden radyal düzeltme katsayısı.

    Returns:
        uint8 tipinde, aynı boyutta vinyet uygulanmış görüntü.
    """
    if img.size == 0:
        return img.astype(np.uint8, copy=True)
    h, w = img.shape[:2]
    maske = _vinyet_maskesi(int(h), int(w), float(k1), float(k2))
    if img.ndim == 3:
        maske = maske[:, :, None]
    out = img.astype(np.float32) * maske
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


def gurultu_uygula(img: np.ndarray, a, b, rng=None) -> np.ndarray:
    """Kanal başına Poisson-Gauss gürültüsü uygular: sigma^2(I) = a*I + b.

    Args:
        img: (H, W, 3) veya (H, W) boyutunda görüntü.
        a: Fotona bağlı (Poisson) kazanç katsayısı — skaler veya 3 elemanlı [a_b, a_g, a_r].
        b: Okuma/termal (Gauss) varyans sabiti — skaler veya 3 elemanlı [b_b, b_g, b_r].
        rng: np.random.Generator veya RandomState (None ise yeni oluşturulur).

    Returns:
        uint8 tipinde gürültülü görüntü.
    """
    if img.size == 0:
        return img.astype(np.uint8, copy=True)
    if rng is None:
        rng = np.random.default_rng()
    kanallar = 1 if img.ndim == 2 else img.shape[2]
    a_v = _kanal_vektoru(a, kanallar)
    b_v = _kanal_vektoru(b, kanallar)
    if not np.any(a_v) and kanallar <= 4:
        # Yalnız Gauss kısmı: cv2.randn ~10 kat hızlı (1280x720: 48 ms -> ~5 ms); tohum rng'den türetilir
        cv2.setRNGSeed(int(rng.integers(0, 2**31 - 1)) if hasattr(rng, 'integers') else int(rng.randint(0, 2**31 - 1)))
        gurultu = np.empty(img.shape, np.int16)
        sd = [float(math.sqrt(max(x, 0.0))) for x in b_v.ravel()]
        cv2.randn(gurultu, (0.0,) * kanallar, tuple(sd))
        return cv2.add(img.astype(np.int16), gurultu).clip(0, 255).astype(np.uint8)
    img_f = img.astype(np.float32)
    varyans = np.maximum(a_v * img_f + b_v, 0.0)
    sigma = np.sqrt(varyans)
    gurultu = rng.normal(0.0, 1.0, size=img.shape).astype(np.float32) * sigma
    return np.clip(np.round(img_f + gurultu), 0, 255).astype(np.uint8)


def yatay_bulaniklik_cekirdegi(uzunluk: float) -> np.ndarray:
    """Verilen sürekli piksel uzunluğu L >= 1 için simetrik 1B yatay kutu (çizgi) kerneli üretir."""
    yari = 0.5 * float(uzunluk)
    yari_adim = int(math.ceil(yari - 0.5))
    if yari_adim <= 0:
        return np.ones((1, 1), dtype=np.float32)
    indisler = np.arange(-yari_adim, yari_adim + 1, dtype=np.float32)
    sol = np.maximum(indisler - 0.5, -yari)
    sag = np.minimum(indisler + 0.5, yari)
    agirlik = np.maximum(0.0, sag - sol)
    toplam = float(agirlik.sum())
    if toplam <= 1e-12:
        return np.ones((1, 1), dtype=np.float32)
    return (agirlik / toplam).reshape(1, -1).astype(np.float32)


def hareket_bulanikligi(
    img: np.ndarray,
    w_z: float,
    v_x: float = 0.0,
    t_poz: float = 0.010,
    fx: float = 454.5,
    derinlik_ortalama: float = 2.0,
) -> np.ndarray:
    """Yaw dönüşünden kaynaklanan yatay hareket bulanıklığını uygular (L = |w_z| * fx * t_poz).

    İleri hareket (v_x) etkisi ihmal edilebilir düzeydedir. Bulanıklık uzunluğu L < 1.0 piksel
    olduğunda görüntüye dokunulmaz.

    Args:
        img: (H, W, 3) veya (H, W) görüntü.
        w_z: Gövde açısal hızı (yaw, rad/s).
        v_x: Gövde ileri doğrusal hızı (m/s, arayüz uyumu için; ihmal edilir).
        t_poz: Efektif pozlama süresi (s).
        fx: Görüntü genişliğine karşılık gelen yatay odak uzaklığı (px; 640 genişlikte 454.5).
        derinlik_ortalama: Ortalama sahne derinliği (m, varsayılan 2.0).

    Returns:
        uint8 tipinde, aynı boyutta görüntü.
    """
    _ = (v_x, derinlik_ortalama)
    if img.size == 0:
        return img.astype(np.uint8, copy=True)
    uzunluk = abs(float(w_z)) * float(fx) * float(t_poz)
    if uzunluk < 1.0:
        if img.dtype == np.uint8:
            return img.copy()
        return np.clip(np.round(img), 0, 255).astype(np.uint8)
    max_uzunluk = float(max(img.shape[1], 1))
    uzunluk = min(uzunluk, max_uzunluk)
    cekirdek = yatay_bulaniklik_cekirdegi(uzunluk)
    if img.dtype == np.uint8:
        return cv2.filter2D(img, -1, cekirdek, borderType=cv2.BORDER_REFLECT_101)
    bulanik = cv2.filter2D(img.astype(np.float32), -1, cekirdek, borderType=cv2.BORDER_REFLECT_101)
    return np.clip(np.round(bulanik), 0, 255).astype(np.uint8)


def beyaz_denge_uygula(img: np.ndarray, kazanc_bgr) -> np.ndarray:
    """Görüntü kanallarına BGR beyaz denge kazançlarını uygular.

    Args:
        img: (H, W, 3) BGR görüntü (veya 2B gri görüntü).
        kazanc_bgr: 3 elemanlı [k_b, k_g, k_r] kazanç dizisi/listesi.

    Returns:
        uint8 tipinde beyaz dengesi uygulanmış görüntü.
    """
    if img.size == 0:
        return img.astype(np.uint8, copy=True)
    kanallar = 1 if img.ndim == 2 else img.shape[2]
    kazanc = _kanal_vektoru(kazanc_bgr, kanallar)
    out = img.astype(np.float32) * kazanc
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


def renk_boz(
    img: np.ndarray,
    param: dict = None,
    w_z: float = 0.0,
    v_x: float = 0.0,
    rng=None,
) -> np.ndarray:
    """Simülasyon renk görüntüsüne sırayla bulanıklık -> vinyet -> beyaz denge -> gürültü uygular.

    Args:
        img: Giriş BGR görüntüsü (H, W, 3).
        param: renk_param.yaml ile uyumlu parametre sözlüğü.
        w_z: Gövde yaw açısal hızı (rad/s).
        v_x: Gövde ileri doğrusal hızı (m/s).
        rng: Rastgele sayı üreteci (np.random.Generator).

    Returns:
        Bozulma modeli uygulanmış uint8 BGR görüntü.
    """
    p = dict(VARSAYILAN_PARAM)
    if param:
        p.update(param)

    t_poz = float(p.get("t_poz", VARSAYILAN_PARAM["t_poz"]))
    fx = float(p.get("fx", VARSAYILAN_PARAM["fx"]))
    derinlik_ort = float(p.get("derinlik_ortalama", VARSAYILAN_PARAM["derinlik_ortalama"]))
    k1 = float(p.get("k1", p.get("vinyet_k1", VARSAYILAN_PARAM["k1"])))
    k2 = float(p.get("k2", p.get("vinyet_k2", VARSAYILAN_PARAM["k2"])))
    kazanc_bgr = p.get("kazanc_bgr", VARSAYILAN_PARAM["kazanc_bgr"])
    a = p.get("a", p.get("gurultu_a", VARSAYILAN_PARAM["a"]))
    b = p.get("b", p.get("gurultu_b", VARSAYILAN_PARAM["b"]))

    out = hareket_bulanikligi(img, w_z=w_z, v_x=v_x, t_poz=t_poz, fx=fx, derinlik_ortalama=derinlik_ort)
    if k1 or k2:  # kapalı adımları atla (1280x720'de her biri ~9 ms)
        out = vinyet_uygula(out, k1=k1, k2=k2)
    if np.any(np.asarray(kazanc_bgr, dtype=np.float32) != 1.0):
        out = beyaz_denge_uygula(out, kazanc_bgr=kazanc_bgr)
    out = gurultu_uygula(out, a=a, b=b, rng=rng)
    return out


PARAMETRE_ACIKLAMALARI = """
Parametre Anlamları (renk_param.yaml & renk_boz sözlüğü):
- a / gurultu_a       : Kanal başına [B, G, R] Poisson (parlaklığa orantılı) gürültü eğimi; sigma^2(I) = a*I + b.
- b / gurultu_b       : Kanal başına [B, G, R] Gauss (sabit okuma) gürültü varyansı (0..255 seviye^2).
- k1 / vinyet_k1      : Radyal vinyet 2. derece katsayısı; I(r)/I(0) = 1 + k1*r^2 + k2*r^4 (köşede r=1).
- k2 / vinyet_k2      : Radyal vinyet 4. derece katsayısı.
- t_poz               : Hareket bulanıklığını belirleyen efektif pozlama süresi (saniye); L = |w_z| * fx * t_poz.
- fx                  : Yatay odak uzaklığı (piksel; 1280x720'de 909.0, 2x küçültülmüş 640x360'ta 454.5).
- derinlik_ortalama   : Ortalama sahne derinliği (metre, varsayılan 2.0).
- kazanc_bgr          : Beyaz denge için kanal başına [B, G, R] çarpanları (ortalaması 1.0'a normalize).
- kanal_ortalama_bgr  : Gerçek veride ölçülen [B, G, R] kanal ortalamaları (0..255).
- hsv_s_medyan        : HSV doygunluk (S) kanalının medyan değeri (0..255).
- parlaklik_p1/p50/p99: Gri parlaklık dağılımının %1, %50 (medyan) ve %99 yüzdelik değerleri.
"""
