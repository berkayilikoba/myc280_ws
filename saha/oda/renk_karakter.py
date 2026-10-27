#!/usr/bin/env python3
"""Gerçek D435 sahne kayıtlarından renk kamerası bozulma modeli parametrelerini ölçer.

Kullanım:
    python3 saha/oda/renk_karakter.py <cikti_dizini> <sahne>... [--her 3]

Ölçülenler:
  1. Gürültü-parlaklık eğrisi: 8x8 düşük dokulu yamalarda yüksek geçiren (yama - Gauss(sigma=1.5))
     std'si -> kanal başına Poisson-Gauss modeli sigma^2(I) = a*I + b.
  2. Vinyet: kare medyanına normalize parlaklık ortalaması -> radyal profil I(r)/I(0) = 1 + k1*r^2 + k2*r^4
     (köşede r=1).
  3. Hareket bulanıklığı: kare başına Laplacian varyansı vs gövde açısal hızı |w| ve doğrusal hız v;
     keskin karelere yatay bulanıklık kerneli (L = |w|*fx*t_poz, 640 genişlikte fx=454.5, 1280'de 909.0)
     uygulayarak ızgara araması ile efektif pozlama süresi t_poz tahmini.
  4. Renk istatistikleri: BGR kanal ortalamaları/oranları (beyaz denge), HSV S medyanı,
     parlaklık yüzdelikleri (p1, p50, p99).
Not: Görüntüler 640x360 BGR olup orijinal 1280x720 D435 görüntüsünden 2x küçültülmüştür.
"""

import argparse
import math
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


FX_1280 = 909.0
FX_640 = 454.5


def yatay_bulaniklik_cekirdegi(uzunluk: float) -> np.ndarray:
    """Sürekli piksel uzunluğu L için simetrik 1B yatay çizgi kerneli üretir."""
    if uzunluk <= 1.0:
        return np.ones((1, 1), dtype=np.float32)
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


def _quat_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _zaman_oku(zaman_yolu: Path):
    """zaman.txt ('i t t') dosyasından (indis_listesi, zaman_saniye_dizisi) döndürür."""
    indisler = []
    zamanlar = []
    if not zaman_yolu.is_file():
        return indisler, np.zeros(0, dtype=np.float64)
    with open(zaman_yolu, "r", encoding="utf-8") as f:
        for satir in f:
            s = satir.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split()
            if len(p) >= 2:
                indisler.append(int(float(p[0])))
                zamanlar.append(float(p[1]))
            elif len(p) == 1:
                indisler.append(len(indisler))
                zamanlar.append(float(p[0]))
    t = np.asarray(zamanlar, dtype=np.float64)
    if t.size > 1:
        dt_med = float(np.median(np.abs(np.diff(t))))
        if dt_med > 1e6:  # nanosaniye ise saniyeye çevir
            t = t * 1e-9
    return indisler, t


def _poses_oku(poses_yolu: Path):
    """poses.txt ('x y z qx qy qz qw') dosyasından (N, 7) float64 dizi döndürür."""
    satirlar = []
    if not poses_yolu.is_file():
        return np.zeros((0, 7), dtype=np.float64)
    with open(poses_yolu, "r", encoding="utf-8") as f:
        for satir in f:
            s = satir.strip()
            if not s or s.startswith("#"):
                continue
            p = [float(x) for x in s.split()]
            if len(p) >= 7:
                satirlar.append(p[-7:])
    if not satirlar:
        return np.zeros((0, 7), dtype=np.float64)
    return np.asarray(satirlar, dtype=np.float64)


def sahne_hizlarini_hesapla(sahne_dizini: Path):
    """poses.txt ve zaman.txt'den kare başına (v, w_z) hızlarını hesaplar."""
    indisler, t = _zaman_oku(sahne_dizini / "zaman.txt")
    poses = _poses_oku(sahne_dizini / "poses.txt")
    n = min(len(t), len(poses))
    if n < 2:
        return {}, []

    t = t[:n]
    poses = poses[:n]
    indisler = indisler[:n]

    x, y, z = poses[:, 0], poses[:, 1], poses[:, 2]
    yaw = np.array([_quat_yaw(q[0], q[1], q[2], q[3]) for q in poses[:, 3:7]], dtype=np.float64)

    v_dizi = np.zeros(n, dtype=np.float64)
    w_dizi = np.zeros(n, dtype=np.float64)

    for i in range(n):
        i0 = max(0, i - 1)
        i1 = min(n - 1, i + 1)
        dt = float(t[i1] - t[i0])
        if dt <= 1e-5 or dt > 5.0:
            # Komşu farka geri dön
            if i > 0 and 1e-5 < (t[i] - t[i - 1]) <= 5.0:
                i0, i1 = i - 1, i
                dt = float(t[i1] - t[i0])
            elif i + 1 < n and 1e-5 < (t[i + 1] - t[i]) <= 5.0:
                i0, i1 = i, i + 1
                dt = float(t[i1] - t[i0])
            else:
                continue
        ds = math.hypot(x[i1] - x[i0], y[i1] - y[i0])
        dyaw = math.atan2(math.sin(yaw[i1] - yaw[i0]), math.cos(yaw[i1] - yaw[i0]))
        v = ds / dt
        w = dyaw / dt
        # Olası SLAM sıçramalarını sınırla
        if abs(v) <= 5.0 and abs(w) <= 10.0:
            v_dizi[i] = v
            w_dizi[i] = w

    hiz_indis = {int(indisler[i]): (float(v_dizi[i]), float(w_dizi[i])) for i in range(n)}
    hiz_satir = [(float(v_dizi[i]), float(w_dizi[i])) for i in range(n)]
    return hiz_indis, hiz_satir


def dusuk_dokulu_yamalari_topla(img: np.ndarray, yama_boyutu: int = 8):
    """8x8 düşük gradyanlı yamalardan (ortalama_BGR, yuksek_geciren_std_BGR) döndürür."""
    h, w = img.shape[:2]
    h8 = (h // yama_boyutu) * yama_boyutu
    w8 = (w // yama_boyutu) * yama_boyutu
    if h8 < yama_boyutu or w8 < yama_boyutu:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

    img_f = img[:h8, :w8].astype(np.float32)
    bulanik = cv2.GaussianBlur(img_f, (0, 0), sigmaX=1.5, sigmaY=1.5)
    yuksek = img_f - bulanik

    gri_b = cv2.cvtColor(bulanik, cv2.COLOR_BGR2GRAY)
    gx = cv2.Sobel(gri_b, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gri_b, cv2.CV_32F, 0, 1, ksize=3)
    grad = np.hypot(gx, gy)

    ny, nx = h8 // yama_boyutu, w8 // yama_boyutu
    grad_b = grad.reshape(ny, yama_boyutu, nx, yama_boyutu).mean(axis=(1, 3))
    gri_blok = gri_b.reshape(ny, yama_boyutu, nx, yama_boyutu)
    gri_ort = gri_blok.mean(axis=(1, 3))
    gri_aralik = gri_blok.max(axis=(1, 3)) - gri_blok.min(axis=(1, 3))

    gecerli = (gri_ort >= 5.0) & (gri_ort <= 245.0)
    if not np.any(gecerli):
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

    grad_esik = float(np.percentile(grad_b[gecerli], 15.0))
    secim = gecerli & (grad_b <= max(grad_esik, 1.5)) & (gri_aralik <= 10.0)
    if int(np.sum(secim)) < 8:
        secim = gecerli & (grad_b <= float(np.percentile(grad_b[gecerli], 25.0)))

    img_blok = img_f.reshape(ny, yama_boyutu, nx, yama_boyutu, 3)
    yuk_blok = yuksek.reshape(ny, yama_boyutu, nx, yama_boyutu, 3)

    yama_ort = img_blok.mean(axis=(1, 3))[secim]
    yama_std = yuk_blok.std(axis=(1, 3))[secim]
    return yama_ort.astype(np.float32), yama_std.astype(np.float32)


def poisson_gauss_uydur(yama_ort: np.ndarray, yama_std: np.ndarray):
    """Kanal başına sigma^2(I) = a*I + b doğrusal modelini uydurur."""
    a_liste, b_liste = [], []
    kutu_verileri = []
    var_tum = yama_std ** 2

    for c in range(3):
        ic = yama_ort[:, c]
        vc = var_tum[:, c]
        if ic.size < 10:
            a_liste.append(0.02)
            b_liste.append(1.0)
            kutu_verileri.append((np.array([0.0, 255.0]), np.array([1.0, 6.1])))
            continue

        # Parlaklık dilimlerinde medyan varyans alarak doku aykırı değerlerini bastır
        kenarlar = np.linspace(5.0, 245.0, 25)
        k_i, k_v, k_w = [], [], []
        for j in range(len(kenarlar) - 1):
            m = (ic >= kenarlar[j]) & (ic < kenarlar[j + 1])
            say = int(np.sum(m))
            if say >= 5:
                k_i.append(float(np.mean(ic[m])))
                k_v.append(float(np.median(vc[m])))
                k_w.append(math.sqrt(say))

        if len(k_i) >= 3:
            xi = np.asarray(k_i, dtype=np.float64)
            yi = np.asarray(k_v, dtype=np.float64)
            wi = np.asarray(k_w, dtype=np.float64)
            a_c, b_c = np.polyfit(xi, yi, 1, w=wi)
        else:
            xi = ic.astype(np.float64)
            yi = np.clip(vc, 0.0, np.percentile(vc, 90.0)).astype(np.float64)
            a_c, b_c = np.polyfit(xi, yi, 1)

        if a_c < 0.0:
            a_c = 0.0
            b_c = float(np.median(yi))
        if b_c < 0.0:
            b_c = 0.0
            a_c = max(0.0, float(np.sum(xi * yi) / max(float(np.sum(xi * xi)), 1e-9)))

        a_liste.append(float(a_c))
        b_liste.append(float(b_c))
        kutu_verileri.append((xi, yi))

    return a_liste, b_liste, kutu_verileri


def vinyet_uydur(ort_norm_goruntu: np.ndarray):
    """I(r)/I(0) = 1 + k1*r^2 + k2*r^4 modelini radyal profile uydurur (köşede r=1)."""
    h, w = ort_norm_goruntu.shape[:2]
    cx, cy = (w - 1) * 0.5, (h - 1) * 0.5
    r_kose = max(math.hypot(cx, cy), 1e-6)
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    r = np.hypot(u - cx, v - cy) / r_kose

    kenarlar = np.linspace(0.0, 1.0, 51)
    r_merkez = []
    i_profil = []
    for j in range(len(kenarlar) - 1):
        m = (r >= kenarlar[j]) & (r < kenarlar[j + 1])
        if np.any(m):
            r_merkez.append(0.5 * (kenarlar[j] + kenarlar[j + 1]))
            i_profil.append(float(np.mean(ort_norm_goruntu[m])))

    r_merkez = np.asarray(r_merkez, dtype=np.float64)
    i_profil = np.asarray(i_profil, dtype=np.float64)

    merkez_maske = r <= 0.08
    i0 = float(np.mean(ort_norm_goruntu[merkez_maske])) if np.any(merkez_maske) else float(i_profil[0])
    i0 = max(i0, 1e-6)
    oran = i_profil / i0

    r2 = r_merkez ** 2
    r4 = r2 ** 2
    x_mat = np.column_stack([r2, r4])
    hedef = oran - 1.0
    k1, k2 = np.linalg.lstsq(x_mat, hedef, rcond=None)[0]
    return float(k1), float(k2), r_merkez, oran


def hareket_bulanikligi_t_poz_bul(keskin_kareler, w_dizi: np.ndarray, lap_dizi: np.ndarray, fx: float):
    """Keskin karelere yatay bulanıklık kerneli uygulayıp Laplacian oranını eşleştirerek t_poz bulur."""
    if len(lap_dizi) == 0:
        return 0.010, np.linspace(0.0, 2.0, 20), np.ones(20), 100.0

    sabit_maske = w_dizi < 0.05
    if int(np.sum(sabit_maske)) < 3:
        esik = float(np.percentile(w_dizi, 20.0))
        sabit_maske = w_dizi <= max(esik, 1e-4)
    lap_sabit = float(np.median(lap_dizi[sabit_maske])) if np.any(sabit_maske) else float(np.max(lap_dizi))
    lap_sabit = max(lap_sabit, 1e-6)
    oran_gercek = lap_dizi / lap_sabit

    if not keskin_kareler:
        return 0.010, np.linspace(0.0, 2.0, 20), np.ones(20), lap_sabit

    # Keskin kareler üzerinde L uzunluğuna göre Laplacian düşüş eğrisini (R_sim(L)) önceden hesapla
    l_izgara = np.linspace(0.0, 25.0, 101)
    r_sim_toplam = np.zeros_like(l_izgara, dtype=np.float64)
    gecerli_keskin = 0

    for gri in keskin_kareler:
        lap0 = float(cv2.Laplacian(gri, cv2.CV_64F).var())
        if lap0 <= 1e-6:
            continue
        gecerli_keskin += 1
        oranlar = [1.0]
        for l_deger in l_izgara[1:]:
            if l_deger < 1.0:
                oranlar.append(1.0)
            else:
                cek = yatay_bulaniklik_cekirdegi(float(l_deger))
                bulanik = cv2.filter2D(gri, -1, cek, borderType=cv2.BORDER_REFLECT_101)
                oranlar.append(float(cv2.Laplacian(bulanik, cv2.CV_64F).var()) / lap0)
        r_sim_toplam += np.asarray(oranlar, dtype=np.float64)

    if gecerli_keskin == 0:
        return 0.010, np.linspace(0.0, 2.0, 20), np.ones(20), lap_sabit
    r_sim_l = r_sim_toplam / gecerli_keskin

    # Açısal hız dilimlerinde medyan Laplacian oranı üzerinden ızgara araması
    hareket_maske = w_dizi >= 0.05
    if int(np.sum(hareket_maske)) >= 5:
        w_har = w_dizi[hareket_maske]
        o_har = oran_gercek[hareket_maske]
    else:
        w_har = w_dizi
        o_har = oran_gercek

    t_adaylari = np.linspace(0.001, 0.150, 150)
    en_iyi_t = 0.010
    en_kucuk_hata = float("inf")

    for t_aday in t_adaylari:
        l_tahmin = w_har * float(fx) * float(t_aday)
        oran_tahmin = np.interp(l_tahmin, l_izgara, r_sim_l)
        # Aykırı sahne dokularına dayanıklı Huber/L1 benzeri kayıp
        fark = oran_tahmin - o_har
        hata = float(np.mean(np.where(np.abs(fark) < 0.25, fark ** 2, 0.5 * np.abs(fark) - 0.0625)))
        if hata < en_kucuk_hata:
            en_kucuk_hata = hata
            en_iyi_t = float(t_aday)

    w_egrisi = np.linspace(0.0, max(float(np.max(w_dizi)), 0.5), 80)
    oran_egrisi = np.interp(w_egrisi * float(fx) * en_iyi_t, l_izgara, r_sim_l)
    return en_iyi_t, w_egrisi, oran_egrisi, lap_sabit


def histogram_yuzdelik(hist: np.ndarray, yuzdelikler):
    """256 kutulu tam sayı histogramından istenen yüzdelikleri hesaplar."""
    kumu = np.cumsum(hist, dtype=np.float64)
    toplam = float(kumu[-1]) if kumu.size > 0 else 0.0
    if toplam <= 0.0:
        return [0.0 for _ in yuzdelikler]
    sonuc = []
    for p in yuzdelikler:
        hedef = (float(p) / 100.0) * toplam
        idx = int(np.searchsorted(kumu, hedef))
        sonuc.append(float(min(max(idx, 0), 255)))
    return sonuc


def _temiz_tip(obj):
    """YAML serileştirmesi için numpy tiplerini standart Python tiplerine dönüştürür."""
    if isinstance(obj, dict):
        return {str(k): _temiz_tip(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, np.ndarray)):
        return [_temiz_tip(x) for x in obj]
    if isinstance(obj, (np.floating, float)):
        return round(float(obj), 6)
    if isinstance(obj, (np.integer, int)):
        return int(obj)
    return obj


def grafikleri_kaydet(
    cikti_dizini: Path,
    a_bgr,
    b_bgr,
    kutu_verileri,
    k1: float,
    k2: float,
    r_merkez: np.ndarray,
    vinyet_oran: np.ndarray,
    w_dizi: np.ndarray,
    v_dizi: np.ndarray,
    lap_dizi: np.ndarray,
    lap_sabit: float,
    t_poz: float,
    w_egrisi: np.ndarray,
    oran_egrisi: np.ndarray,
):
    """gurultu.png, vinyet.png ve bulaniklik.png grafiklerini kaydeder."""
    # 1. Gürültü grafiği
    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=120)
    renkler = [("B (Mavi)", "tab:blue"), ("G (Yeşil)", "tab:green"), ("R (Kırmızı)", "tab:red")]
    x_cizgi = np.linspace(0.0, 255.0, 100)
    for c, (ad, renk) in enumerate(renkler):
        xi, yi = kutu_verileri[c]
        ax.scatter(xi, yi, color=renk, alpha=0.65, s=24, label=f"{ad} ölçüm")
        ax.plot(
            x_cizgi,
            a_bgr[c] * x_cizgi + b_bgr[c],
            color=renk,
            linewidth=2.0,
            label=f"{ad}: $\\sigma^2={a_bgr[c]:.4f}I + {b_bgr[c]:.3f}$",
        )
    ax.set_title("D435 Renk Kamerası Gürültü-Parlaklık Eğrisi (640x360, 8x8 Yama)")
    ax.set_xlabel("Yama Ortalama Parlaklığı I [0..255]")
    ax.set_ylabel("Yüksek Geçiren Yama Varyansı $\\sigma^2(I)$")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(cikti_dizini / "gurultu.png")
    plt.close(fig)

    # 2. Vinyet grafiği
    fig, ax = plt.subplots(figsize=(6.5, 4.2), dpi=120)
    r_cizgi = np.linspace(0.0, 1.0, 100)
    vinyet_model = 1.0 + k1 * (r_cizgi ** 2) + k2 * (r_cizgi ** 4)
    ax.scatter(r_merkez, vinyet_oran, color="tab:purple", s=22, alpha=0.75, label="Ölçülen radyal profil")
    ax.plot(
        r_cizgi,
        vinyet_model,
        color="black",
        linewidth=2.0,
        label=f"Model: $1 + ({k1:.4f})r^2 + ({k2:.4f})r^4$",
    )
    ax.set_title("D435 Radyal Vinyet Profili (Merkez r=0, Köşe r=1)")
    ax.set_xlabel("Normalize Yarıçap r")
    ax.set_ylabel("Normalize Parlaklık $I(r)/I(0)$")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(cikti_dizini / "vinyet.png")
    plt.close(fig)

    # 3. Hareket bulanıklığı grafiği
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.5, 4.2), dpi=120)
    oran_gercek = lap_dizi / max(lap_sabit, 1e-6)
    ax1.scatter(w_dizi, oran_gercek, color="tab:orange", alpha=0.45, s=16, label="Kareler (Lap / Lap_sabit)")
    ax1.plot(
        w_egrisi,
        oran_egrisi,
        color="tab:red",
        linewidth=2.2,
        label=f"Bulanıklık modeli ($t_{{poz}}={t_poz*1000:.1f}$ ms)",
    )
    ax1.set_title("Laplacian Oranı vs Açısal Hız $|\\omega|$")
    ax1.set_xlabel("Gövde Açısal Hızı $|\\omega|$ [rad/s]")
    ax1.set_ylabel("Normalize Laplacian Varyansı")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)

    ax2.scatter(v_dizi, lap_dizi, color="teal", alpha=0.45, s=16, label="Kareler")
    ax2.axhline(lap_sabit, color="gray", linestyle="--", linewidth=1.2, label=f"Sabit medyan ({lap_sabit:.1f})")
    ax2.set_title("Laplacian Varyansı vs Doğrusal Hız $|v|$")
    ax2.set_xlabel("Gövde Doğrusal Hızı $|v|$ [m/s]")
    ax2.set_ylabel("Laplacian Varyansı")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(cikti_dizini / "bulaniklik.png")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Gerçek D435 sahne dizinlerinden renk kamerası bozulma parametrelerini ölçer."
    )
    parser.add_argument("cikti_dizini", help="Çıktı dizini (<cikti>/renk_param.yaml ve grafikler)")
    parser.add_argument("sahneler", nargs="+", help="Sahne dizinleri (rgb/, poses.txt, zaman.txt içeren)")
    parser.add_argument("--her", type=int, default=3, help="Her N karede bir örnekle (varsayılan: 3)")
    args = parser.parse_args(argv)

    adim = max(1, int(args.her))
    cikti_dizini = Path(args.cikti_dizini).expanduser().resolve()
    cikti_dizini.mkdir(parents=True, exist_ok=True)

    yama_ort_listesi = []
    yama_std_listesi = []
    vinyet_toplam = None
    vinyet_sayac = 0

    w_listesi = []
    v_listesi = []
    lap_listesi = []
    keskin_adaylar = []  # (|w|, gri_goruntu)

    kanal_toplam = np.zeros(3, dtype=np.float64)
    piksel_sayisi = 0
    hist_s = np.zeros(256, dtype=np.int64)
    hist_gri = np.zeros(256, dtype=np.int64)

    cozunurluk_wh = [640, 360]
    islenen_kare = 0
    kullanilan_sahneler = []

    for sahne_str in args.sahneler:
        sahne_yolu = Path(sahne_str).expanduser().resolve()
        rgb_dizini = sahne_yolu / "rgb"
        if not rgb_dizini.is_dir():
            print(f"[UYARI] rgb dizini bulunamadı, atlanıyor: {rgb_dizini}", file=sys.stderr)
            continue

        png_dosyalar = sorted(rgb_dizini.glob("*.png"))
        if not png_dosyalar:
            print(f"[UYARI] PNG görüntüsü bulunamadı, atlanıyor: {rgb_dizini}", file=sys.stderr)
            continue

        kullanilan_sahneler.append(sahne_yolu.name)
        hiz_indis, hiz_satir = sahne_hizlarini_hesapla(sahne_yolu)

        for satir_no in range(0, len(png_dosyalar), adim):
            png_yolu = png_dosyalar[satir_no]
            img = cv2.imread(str(png_yolu), cv2.IMREAD_COLOR)
            if img is None or img.size == 0:
                continue

            h, w = img.shape[:2]
            cozunurluk_wh = [int(w), int(h)]
            islenen_kare += 1

            # 1. Düşük dokulu yamalardan gürültü örneklemi
            y_ort, y_std = dusuk_dokulu_yamalari_topla(img, yama_boyutu=8)
            if y_ort.shape[0] > 0:
                yama_ort_listesi.append(y_ort)
                yama_std_listesi.append(y_std)

            # 2. Vinyet için medyan-normalize gri parlaklık biriktir
            gri = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gri_f = gri.astype(np.float32)
            med = float(np.median(gri_f))
            if med > 1.0:
                norm_gri = gri_f / med
                if vinyet_toplam is None or vinyet_toplam.shape != norm_gri.shape:
                    vinyet_toplam = np.zeros_like(norm_gri, dtype=np.float64)
                    vinyet_sayac = 0
                vinyet_toplam += norm_gri
                vinyet_sayac += 1

            # 3. Hareket bulanıklığı için Laplacian varyansı ve gövde hızı
            lap_var = float(cv2.Laplacian(gri, cv2.CV_64F).var())
            v_kare, w_kare = 0.0, 0.0
            if len(png_dosyalar) == len(hiz_satir) and satir_no < len(hiz_satir):
                v_kare, w_kare = hiz_satir[satir_no]
            else:
                try:
                    dosya_indis = int(png_yolu.stem)
                except ValueError:
                    dosya_indis = -1
                if dosya_indis in hiz_indis:
                    v_kare, w_kare = hiz_indis[dosya_indis]
                elif satir_no < len(hiz_satir):
                    v_kare, w_kare = hiz_satir[satir_no]

            w_abs = abs(float(w_kare))
            v_abs = abs(float(v_kare))
            w_listesi.append(w_abs)
            v_listesi.append(v_abs)
            lap_listesi.append(lap_var)

            if w_abs < 0.05 and v_abs < 0.05 and len(keskin_adaylar) < 25:
                keskin_adaylar.append((w_abs, gri.copy()))
            elif len(keskin_adaylar) < 10:
                keskin_adaylar.append((w_abs, gri.copy()))

            # 4. Renk ve parlaklık istatistikleri
            kanal_toplam += img.reshape(-1, 3).sum(axis=0, dtype=np.float64)
            piksel_sayisi += int(h * w)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            hist_s += np.bincount(hsv[:, :, 1].ravel(), minlength=256)
            hist_gri += np.bincount(gri.ravel(), minlength=256)

    if islenen_kare == 0:
        raise RuntimeError("Hiçbir sahnede geçerli RGB görüntüsü okunamadı.")

    # 1. Poisson-Gauss gürültü uydurma
    if yama_ort_listesi:
        tum_yama_ort = np.concatenate(yama_ort_listesi, axis=0)
        tum_yama_std = np.concatenate(yama_std_listesi, axis=0)
    else:
        tum_yama_ort = np.zeros((0, 3), dtype=np.float32)
        tum_yama_std = np.zeros((0, 3), dtype=np.float32)
    a_bgr, b_bgr, kutu_verileri = poisson_gauss_uydur(tum_yama_ort, tum_yama_std)

    # 2. Vinyet uydurma
    if vinyet_toplam is not None and vinyet_sayac > 0:
        ort_norm = (vinyet_toplam / float(vinyet_sayac)).astype(np.float32)
    else:
        ort_norm = np.ones((cozunurluk_wh[1], cozunurluk_wh[0]), dtype=np.float32)
    k1, k2, r_merkez, vinyet_oran = vinyet_uydur(ort_norm)

    # 3. Hareket bulanıklığı (efektif pozlama süresi t_poz)
    fx_aktif = FX_640 * (float(cozunurluk_wh[0]) / 640.0)
    keskin_adaylar.sort(key=lambda x: x[0])
    keskin_kareler = [g for _, g in keskin_adaylar[:15]]
    w_dizi = np.asarray(w_listesi, dtype=np.float64)
    v_dizi = np.asarray(v_listesi, dtype=np.float64)
    lap_dizi = np.asarray(lap_listesi, dtype=np.float64)
    t_poz, w_egrisi, oran_egrisi, lap_sabit = hareket_bulanikligi_t_poz_bul(
        keskin_kareler, w_dizi, lap_dizi, fx=fx_aktif
    )

    # 4. Renk istatistikleri
    kanal_ort = (kanal_toplam / max(piksel_sayisi, 1)).tolist()
    genel_ort = max(float(np.mean(kanal_ort)), 1e-6)
    kazanc_bgr = [float(c / genel_ort) for c in kanal_ort]
    yesil_ort = max(float(kanal_ort[1]), 1e-6)
    kanal_orani_bgr = [float(c / yesil_ort) for c in kanal_ort]

    s_p10, s_medyan, s_p90 = histogram_yuzdelik(hist_s, [10.0, 50.0, 90.0])
    p1, p50, p99 = histogram_yuzdelik(hist_gri, [1.0, 50.0, 99.0])

    cozunurluk_notu = (
        f"Görüntüler {cozunurluk_wh[0]}x{cozunurluk_wh[1]} BGR olup orijinal 1280x720 D435 "
        f"görüntüsünden 2x küçültülmüştür (fx: {FX_1280} -> {fx_aktif:.1f})."
    )

    param = _temiz_tip(
        {
            "cozunurluk": cozunurluk_wh,
            "orijinal_cozunurluk": [1280, 720],
            "cozunurluk_notu": cozunurluk_notu,
            "islenen_kare_sayisi": islenen_kare,
            "her_n_kare": adim,
            "sahneler": kullanilan_sahneler,
            # Gürültü parametreleri (kanal başına BGR: sigma^2(I) = a*I + b)
            "a": a_bgr,
            "b": b_bgr,
            "gurultu_a": a_bgr,
            "gurultu_b": b_bgr,
            # Vinyet parametreleri (I(r)/I(0) = 1 + k1*r^2 + k2*r^4, köşede r=1)
            "k1": k1,
            "k2": k2,
            "vinyet_k1": k1,
            "vinyet_k2": k2,
            # Hareket bulanıklığı parametreleri (L = |w_z| * fx * t_poz)
            "t_poz": t_poz,
            "fx": fx_aktif,
            "fx_1280": FX_1280,
            "derinlik_ortalama": 2.0,
            "laplacian_sabit_medyan": lap_sabit,
            # Renk ve parlaklık istatistikleri
            "kazanc_bgr": kazanc_bgr,
            "kanal_ortalama_bgr": kanal_ort,
            "kanal_orani_bgr": kanal_orani_bgr,
            "hsv_s_medyan": s_medyan,
            "hsv_s_p10": s_p10,
            "hsv_s_p90": s_p90,
            "parlaklik_p1": p1,
            "parlaklik_p50": p50,
            "parlaklik_p99": p99,
            "parlaklik_yuzdelik": {"p1": p1, "p50": p50, "p99": p99},
        }
    )

    yaml_yolu = cikti_dizini / "renk_param.yaml"
    with open(yaml_yolu, "w", encoding="utf-8") as f:
        f.write("# D435 Renk Kamerası Bozulma Modeli Parametreleri\n")
        f.write(f"# Not: {cozunurluk_notu}\n")
        yaml.safe_dump(param, f, sort_keys=False, allow_unicode=True)

    grafikleri_kaydet(
        cikti_dizini=cikti_dizini,
        a_bgr=a_bgr,
        b_bgr=b_bgr,
        kutu_verileri=kutu_verileri,
        k1=k1,
        k2=k2,
        r_merkez=r_merkez,
        vinyet_oran=vinyet_oran,
        w_dizi=w_dizi,
        v_dizi=v_dizi,
        lap_dizi=lap_dizi,
        lap_sabit=lap_sabit,
        t_poz=t_poz,
        w_egrisi=w_egrisi,
        oran_egrisi=oran_egrisi,
    )

    rapor = (
        "=== D435 Renk Kamerası Karakterizasyon Raporu ===\n"
        f"Not: {cozunurluk_notu}\n"
        f"Sahneler ({len(kullanilan_sahneler)}): {', '.join(kullanilan_sahneler)} (toplam {islenen_kare} kare, --her {adim})\n"
        f"1. Gürültü (Poisson-Gauss sigma^2 = a*I + b):\n"
        f"   a (BGR): [{a_bgr[0]:.5f}, {a_bgr[1]:.5f}, {a_bgr[2]:.5f}]\n"
        f"   b (BGR): [{b_bgr[0]:.4f}, {b_bgr[1]:.4f}, {b_bgr[2]:.4f}]\n"
        f"2. Vinyet (I(r)/I(0) = 1 + k1*r^2 + k2*r^4, köşe r=1):\n"
        f"   k1 = {k1:.5f}, k2 = {k2:.5f} (köşe çarpanı: {1.0 + k1 + k2:.4f})\n"
        f"3. Hareket Bulanıklığı (L = |w|*fx*t_poz, fx={fx_aktif:.1f} px):\n"
        f"   t_poz = {t_poz:.5f} s ({t_poz*1000:.2f} ms), sabit Laplacian medyanı = {lap_sabit:.1f}\n"
        f"4. Renk İstatistikleri:\n"
        f"   Kanal ortalamaları BGR : [{kanal_ort[0]:.2f}, {kanal_ort[1]:.2f}, {kanal_ort[2]:.2f}]\n"
        f"   Beyaz denge kazanç BGR : [{kazanc_bgr[0]:.4f}, {kazanc_bgr[1]:.4f}, {kazanc_bgr[2]:.4f}]\n"
        f"   HSV S medyanı          : {s_medyan:.1f} (p10={s_p10:.1f}, p90={s_p90:.1f})\n"
        f"   Parlaklık yüzdelikleri : p1={p1:.1f}, p50={p50:.1f}, p99={p99:.1f}\n"
        f"Çıktılar: {yaml_yolu}, gurultu.png, vinyet.png, bulaniklik.png"
    )
    print(rapor)


if __name__ == "__main__":
    main()
