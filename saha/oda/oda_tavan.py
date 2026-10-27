#!/usr/bin/env python3
"""D435 derinlik görüntülerinden tavan yüksekliğini ve alt sınırını ölçer."""

import argparse
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


KUTU_BOYU_M = 0.02
MAKS_NOKTA_KARE = 20000
SIGMA_KATSAYI = 0.00176


def poz_matrisi(satir: np.ndarray) -> np.ndarray:
    """[x, y, z, qx, qy, qz, qw] satırını 4x4 homojen matrise çevirir."""
    x, y, z, qx, qy, qz, qw = satir
    norm = float(np.linalg.norm([qx, qy, qz, qw]))
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    T[0, 1] = 2.0 * (qx * qy - qz * qw)
    T[0, 2] = 2.0 * (qx * qz + qy * qw)
    T[1, 0] = 2.0 * (qx * qy + qz * qw)
    T[1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    T[1, 2] = 2.0 * (qy * qz - qx * qw)
    T[2, 0] = 2.0 * (qx * qz - qy * qw)
    T[2, 1] = 2.0 * (qy * qz + qx * qw)
    T[2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    T[:3, 3] = [x, y, z]
    return T


def duzeltme_matrisi(dx: float, dy: float, dyaw_deg: float) -> np.ndarray:
    """Gövde çerçevesinde (dx, dy, dyaw) kalibrasyon düzeltme matrisi."""
    rad = np.deg2rad(dyaw_deg)
    c, s = float(np.cos(rad)), float(np.sin(rad))
    return np.array(
        [
            [c, -s, 0.0, dx],
            [s, c, 0.0, dy],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def derinlik_agirligi(d: np.ndarray) -> np.ndarray:
    """D435 derinlik gürültüsü σ = 0.00176·d² için 1/σ² ağırlığı."""
    d_guvenli = np.maximum(d.astype(np.float64), 0.2)
    sigma = SIGMA_KATSAYI * (d_guvenli * d_guvenli)
    return 1.0 / (sigma * sigma)


def kare_noktalarini_cikar(
    depth: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    T_wc_duz: np.ndarray,
    zmin: float,
    zmax_m: float,
    rng: np.random.Generator,
    maks_nokta: int = MAKS_NOKTA_KARE,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Tek kareden üst yarı (v < cy), d <= zmax_m ve dünya z > zmin noktalarını ve yataylık maskesini döndürür."""
    h, w = depth.shape[:2]
    v_ust_sinir = min(h, int(np.ceil(cy)))
    if v_ust_sinir <= 4:
        return None

    # 5x5 komşuluk hesabı için sınırda 2 piksel pay bırak
    v_dilim = min(h, v_ust_sinir + 2)
    d_dilim = depth[:v_dilim, :].astype(np.float32, copy=False)

    gecerli = np.isfinite(d_dilim) & (d_dilim > 0.0) & (d_dilim <= zmax_m)
    if not np.any(gecerli[:v_ust_sinir, :]):
        return None

    u_grid, v_grid = np.meshgrid(
        np.arange(w, dtype=np.float32),
        np.arange(v_dilim, dtype=np.float32),
    )
    z_c = np.where(gecerli, d_dilim, 0.0)
    x_c = (u_grid - np.float32(cx)) * z_c / np.float32(fx)
    y_c = (v_grid - np.float32(cy)) * z_c / np.float32(fy)
    p_c = np.stack([x_c, y_c, z_c], axis=-1)

    r_wc = T_wc_duz[:3, :3].astype(np.float32)
    t_wc = T_wc_duz[:3, 3].astype(np.float32)
    p_w = p_c @ r_wc.T + t_wc

    # 5x5 komşulukta geçerlilik ve derinlik sürekliliği (oklüzyon kenarlarını ele)
    k5 = np.ones((5, 5), dtype=np.uint8)
    gecerli_5x5 = (
        cv2.erode(
            gecerli.astype(np.uint8),
            k5,
            borderType=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        > 0
    )
    d_max = cv2.dilate(d_dilim, k5)
    d_min = cv2.erode(d_dilim, k5)
    surekli_5x5 = gecerli_5x5 & ((d_max - d_min) <= 0.25 * d_dilim)

    # 5x5 komşulukta yerel 3B teğet vektörlerin çapraz çarpımı ile yüzey normali
    dp_du = cv2.Sobel(p_w, cv2.CV_32F, 1, 0, ksize=5)
    dp_dv = cv2.Sobel(p_w, cv2.CV_32F, 0, 1, ksize=5)
    normal_w = np.cross(dp_du, dp_dv)
    normal_boy = np.linalg.norm(normal_w, axis=-1)
    nz_abs = np.zeros_like(normal_boy, dtype=np.float32)
    gecerli_norm = normal_boy > 1e-12
    nz_abs[gecerli_norm] = np.abs(normal_w[..., 2][gecerli_norm]) / normal_boy[gecerli_norm]
    yatay_2d = surekli_5x5 & (nz_abs > 0.9)

    # Yalnız üst yarı (v < cy), geçerli derinlik ve dünya z > zmin
    ust_maske = (v_grid < np.float32(cy)) & gecerli & (p_w[..., 2] > np.float32(zmin))
    if not np.any(ust_maske):
        return None

    xyz = p_w[ust_maske]
    d_sec = d_dilim[ust_maske]
    yatay_sec = yatay_2d[ust_maske]

    n_nokta = len(xyz)
    if n_nokta > maks_nokta:
        ind = rng.choice(n_nokta, size=maks_nokta, replace=False)
        xyz = xyz[ind]
        d_sec = d_sec[ind]
        yatay_sec = yatay_sec[ind]

    return xyz, d_sec, yatay_sec


def tepe_bul(
    z_yatay: np.ndarray,
    d_yatay: np.ndarray,
    kenarlar: np.ndarray,
    min_toplam_yatay: int = 50,
    min_oran: float = 0.05,
    agirlikli_secim: bool = True,
) -> Tuple[Optional[float], np.ndarray, np.ndarray]:
    """2 cm z histogramında en yüksek belirgin tepeyi (en az %5 pay) bulur."""
    n_kutu = len(kenarlar) - 1
    if n_kutu <= 0 or len(z_yatay) == 0:
        return None, np.zeros(max(n_kutu, 0), dtype=np.int64), np.zeros(max(n_kutu, 0), dtype=np.float64)

    agirlik = derinlik_agirligi(d_yatay)
    sayi_hist, _ = np.histogram(z_yatay, bins=kenarlar)
    agirlik_hist, _ = np.histogram(z_yatay, bins=kenarlar, weights=agirlik)

    n_yatay = len(z_yatay)
    if n_yatay < min_toplam_yatay:
        return None, sayi_hist, agirlik_hist

    merkezler = 0.5 * (kenarlar[:-1] + kenarlar[1:])
    esik_sayi = max(int(np.ceil(min_oran * n_yatay)), 20)

    # Tek kutuda >= %5 veya 3 komşu kutuda (±2 cm) >= %5 olan yerel tepeleri incele
    sayi_3 = np.convolve(sayi_hist, np.array([1, 1, 1], dtype=np.int64), mode="same")
    agirlik_3 = np.convolve(agirlik_hist, np.array([1.0, 1.0, 1.0], dtype=np.float64), mode="same")

    aday_maske = sayi_hist >= esik_sayi
    if not np.any(aday_maske):
        # Gürültü nedeniyle tepe iki komşu 2 cm kutuya bölünmüşse 3-kutu toplamını kontrol et
        dolu_kutular = sayi_hist[sayi_hist > 0]
        ort_kutu = float(np.mean(dolu_kutular)) if len(dolu_kutular) > 0 else 0.0
        aday_maske = (sayi_3 >= esik_sayi) & (sayi_hist >= max(10, int(0.025 * n_yatay))) & (
            sayi_hist >= 2.0 * ort_kutu
        )

    if not np.any(aday_maske):
        return None, sayi_hist, agirlik_hist

    aday_indisler = np.where(aday_maske)[0]
    if agirlikli_secim:
        en_iyi_k = int(aday_indisler[np.argmax(agirlik_3[aday_indisler])])
    else:
        en_iyi_k = int(aday_indisler[np.argmax(sayi_3[aday_indisler])])

    tepe_merkez = float(merkezler[en_iyi_k])
    bant_maske = np.abs(z_yatay - tepe_merkez) <= (1.5 * KUTU_BOYU_M)
    if np.any(bant_maske):
        w_bant = agirlik[bant_maske] if agirlikli_secim else np.ones(np.sum(bant_maske), dtype=np.float64)
        tavan_z = float(np.sum(z_yatay[bant_maske] * w_bant) / np.sum(w_bant))
    else:
        tavan_z = tepe_merkez

    return round(tavan_z, 4), sayi_hist, agirlik_hist


def alt_sinir_hesapla(z_tum: np.ndarray, d_tum: np.ndarray, yatay: np.ndarray) -> Optional[float]:
    """Duvarlarda (yatay olmayan noktalarda) görülen en yüksek güvenilir z değerini (alt sınır) hesaplar."""
    if len(z_tum) == 0:
        return None
    duvar_maske = ~yatay
    if np.any(duvar_maske):
        z_duvar = z_tum[duvar_maske]
        d_duvar = d_tum[duvar_maske]
        yakin_duvar = z_duvar[d_duvar < 6.0]
        kaynak = yakin_duvar if len(yakin_duvar) >= 50 else z_duvar
        return round(float(np.percentile(kaynak, 99.0)), 4)
    return round(float(np.percentile(z_tum, 99.0)), 4)


def grafik_histogram_kaydet(
    cikti_yolu: Path,
    z_tum: np.ndarray,
    z_yatay: np.ndarray,
    d_yatay: np.ndarray,
    kenarlar: np.ndarray,
    sayi_hist: np.ndarray,
    agirlik_hist: np.ndarray,
    tavan_z: Optional[float],
    tavan_z_yakin: Optional[float],
    z99: Optional[float],
    alt_sinir: Optional[float],
) -> None:
    """Tüm noktalar ve yatay noktalar histogramını tahmin çizgileriyle kaydeder."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    merkezler = 0.5 * (kenarlar[:-1] + kenarlar[1:])

    if len(z_tum) > 0:
        tum_hist, _ = np.histogram(z_tum, bins=kenarlar)
        ax1.bar(
            merkezler,
            tum_hist,
            width=KUTU_BOYU_M * 0.9,
            color="#b0bec5",
            alpha=0.7,
            label=f"Tüm noktalar (n={len(z_tum)})",
        )
    if len(z_yatay) > 0:
        ax1.bar(
            merkezler,
            sayi_hist,
            width=KUTU_BOYU_M * 0.9,
            color="#1e88e5",
            alpha=0.85,
            label=f"Yatay noktalar |n_z|>0.9 (n={len(z_yatay)})",
        )

    if tavan_z is not None:
        ax1.axvline(tavan_z, color="#d32f2f", linestyle="--", linewidth=2.0, label=f"tavan_z = {tavan_z:.3f} m")
    if tavan_z_yakin is not None:
        ax1.axvline(
            tavan_z_yakin,
            color="#2e7d32",
            linestyle="-.",
            linewidth=1.8,
            label=f"tavan_z_yakin (d<6m) = {tavan_z_yakin:.3f} m",
        )
    if z99 is not None:
        ax1.axvline(z99, color="#6a1b9a", linestyle=":", linewidth=1.6, label=f"z99 = {z99:.3f} m")
    if alt_sinir is not None and tavan_z is None:
        ax1.axvline(
            alt_sinir,
            color="#ef6c00",
            linestyle="--",
            linewidth=2.0,
            label=f"Duvar üst alt_sinir = {alt_sinir:.3f} m",
        )

    baslik = (
        f"Tavan Yüksekliği Histogramı — Tahmin: {tavan_z:.3f} m"
        if tavan_z is not None
        else f"Tavan Tepkisi Yok (tavan_z: null) — Duvar Alt Sınırı: {alt_sinir if alt_sinir is not None else 'yok'} m"
    )
    ax1.set_title(baslik)
    ax1.set_ylabel("Nokta Sayısı (2 cm kutu)")
    ax1.grid(True, linestyle=":", alpha=0.5)
    ax1.legend(loc="upper right")

    # Alt grafik: 1/σ² ağırlıklı yatay histogram ve d<6m yatay histogram
    if len(z_yatay) > 0 and np.sum(agirlik_hist) > 0:
        agirlik_norm = agirlik_hist / np.max(agirlik_hist)
        ax2.bar(
            merkezler,
            agirlik_norm,
            width=KUTU_BOYU_M * 0.9,
            color="#fb8c00",
            alpha=0.75,
            label="1/σ² ağırlıklı yatay histogram (normalize)",
        )
        yakin_maske = d_yatay < 6.0
        if np.any(yakin_maske):
            yakin_hist, _ = np.histogram(z_yatay[yakin_maske], bins=kenarlar)
            if np.max(yakin_hist) > 0:
                ax2.plot(
                    merkezler,
                    yakin_hist / np.max(yakin_hist),
                    color="#2e7d32",
                    linewidth=1.8,
                    label=f"Yakın yatay noktalar d<6m (n={int(np.sum(yakin_maske))})",
                )

    if tavan_z is not None:
        ax2.axvline(tavan_z, color="#d32f2f", linestyle="--", linewidth=2.0, label=f"tavan_z = {tavan_z:.3f} m")
    if tavan_z_yakin is not None:
        ax2.axvline(
            tavan_z_yakin,
            color="#2e7d32",
            linestyle="-.",
            linewidth=1.8,
            label=f"tavan_z_yakin = {tavan_z_yakin:.3f} m",
        )

    ax2.set_xlabel("Dünya Z Yüksekliği (m)")
    ax2.set_ylabel("Normalize Ağırlık / Yoğunluk")
    ax2.grid(True, linestyle=":", alpha=0.5)
    ax2.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(str(cikti_yolu), dpi=150)
    plt.close(fig)


def grafik_ustten_kaydet(
    cikti_yolu: Path,
    xyz_tum: np.ndarray,
    yatay_tum: np.ndarray,
    tavan_z: Optional[float],
    alt_sinir: Optional[float],
) -> None:
    """Tavan bandındaki (tavan_z ± 0.1 m) yatay noktaların x-y dağılımını kaydeder."""
    fig, ax = plt.subplots(figsize=(8, 8))

    if len(xyz_tum) > 0:
        # Oda sınırlarını görmek için tüm z > zmin noktalarını arka planda seyrelterek çiz
        n_arka = min(len(xyz_tum), 30000)
        adim = max(1, len(xyz_tum) // n_arka)
        arka = xyz_tum[::adim]
        ax.scatter(
            arka[:, 0],
            arka[:, 1],
            s=1.0,
            c="#cfd8dc",
            alpha=0.4,
            label="z > zmin tüm noktalar (duvar/üst yapı)",
        )

    xyz_yatay = xyz_tum[yatay_tum] if len(xyz_tum) > 0 else np.empty((0, 3))
    if tavan_z is not None and len(xyz_yatay) > 0:
        bant_maske = np.abs(xyz_yatay[:, 2] - tavan_z) <= 0.10
        bant_xyz = xyz_yatay[bant_maske]
        if len(bant_xyz) > 0:
            sc = ax.scatter(
                bant_xyz[:, 0],
                bant_xyz[:, 1],
                c=bant_xyz[:, 2],
                cmap="viridis",
                s=4.0,
                alpha=0.85,
                label=f"Tavan bandı ({tavan_z:.2f} ± 0.10 m, n={len(bant_xyz)})",
            )
            plt.colorbar(sc, ax=ax, label="Z Yüksekliği (m)")
        ax.set_title(f"Tavan Bandı (z = {tavan_z:.3f} ± 0.10 m) Yatay Noktaların X-Y Dağılımı")
    else:
        if len(xyz_yatay) > 0:
            sc = ax.scatter(
                xyz_yatay[:, 0],
                xyz_yatay[:, 1],
                c=xyz_yatay[:, 2],
                cmap="plasma",
                s=4.0,
                alpha=0.8,
                label=f"Yatay noktalar (n={len(xyz_yatay)})",
            )
            plt.colorbar(sc, ax=ax, label="Z Yüksekliği (m)")
        ax.set_title(
            f"Tavan Tepkisi Yok (tavan_z: null, alt_sinir: {alt_sinir if alt_sinir is not None else 'yok'} m) — X-Y Dağılımı"
        )

    ax.set_xlabel("Harita X (m)")
    ax.set_ylabel("Harita Y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(str(cikti_yolu), dpi=150)
    plt.close(fig)


def argumanlari_coz() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Gerçek D435 derinlik görüntülerinden tavan yüksekliğini ölçer."
    )
    p.add_argument("cikti_dizini", type=Path, help="Çıktı dosyalarının yazılacağı dizin")
    p.add_argument("sahne", type=Path, nargs="+", help="Bir veya birden çok sahne dizini")
    p.add_argument("--her", type=int, default=5, help="Her N. kareyi işle (varsayılan: 5)")
    p.add_argument("--zmin", type=float, default=1.8, help="Alt dünya z eşiği (m, varsayılan: 1.8)")
    p.add_argument(
        "--zmax_m",
        "--zmax",
        dest="zmax_m",
        type=float,
        default=9.0,
        help="Maksimum kamera derinliği (m, varsayılan: 9.0)",
    )
    p.add_argument("--dx", type=float, default=0.09, help="Gövde x öteleme düzeltmesi (m)")
    p.add_argument("--dy", type=float, default=0.01, help="Gövde y öteleme düzeltmesi (m)")
    p.add_argument("--dyaw", type=float, default=-1.0, help="Gövde yaw düzeltmesi (derece)")
    return p.parse_args()


def main() -> None:
    args = argumanlari_coz()
    cikti_dizin = args.cikti_dizini.expanduser().resolve()
    cikti_dizin.mkdir(parents=True, exist_ok=True)

    dT = duzeltme_matrisi(args.dx, args.dy, args.dyaw)
    rng = np.random.default_rng(42)

    tum_xyz_listesi: List[np.ndarray] = []
    tum_d_listesi: List[np.ndarray] = []
    tum_yatay_listesi: List[np.ndarray] = []
    sahne_basina: Dict[str, Any] = {}

    for sahne_yolu in args.sahne:
        sahne = sahne_yolu.expanduser().resolve()
        cfg_yolu = sahne / "vlmaps_config.yaml"
        if cfg_yolu.exists():
            with cfg_yolu.open("r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            K = cfg["cam_calib_mat"]
            fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
        else:
            fx, fy, cx, cy = 454.4875, 454.2144, 324.6746, 189.4620

        poses = np.atleast_2d(np.loadtxt(sahne / "poses.txt", dtype=np.float64))
        poses_kam = np.atleast_2d(np.loadtxt(sahne / "poses_kamera.txt", dtype=np.float64))
        n_kare = min(len(poses), len(poses_kam))

        s_xyz_list: List[np.ndarray] = []
        s_d_list: List[np.ndarray] = []
        s_yatay_list: List[np.ndarray] = []
        kullanilan_kare = 0

        for i in range(0, n_kare, max(1, args.her)):
            depth_yolu = sahne / "depth" / f"{i:06d}.npy"
            if not depth_yolu.exists():
                continue

            depth = np.load(str(depth_yolu))
            T_wb = poz_matrisi(poses[i])
            T_wc = poz_matrisi(poses_kam[i])
            T_wc_duz = T_wb @ dT @ np.linalg.inv(T_wb) @ T_wc

            sonuc = kare_noktalarini_cikar(
                depth=depth,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                T_wc_duz=T_wc_duz,
                zmin=args.zmin,
                zmax_m=args.zmax_m,
                rng=rng,
                maks_nokta=MAKS_NOKTA_KARE,
            )
            kullanilan_kare += 1
            if sonuc is not None:
                xyz, d_sec, yatay_sec = sonuc
                s_xyz_list.append(xyz)
                s_d_list.append(d_sec)
                s_yatay_list.append(yatay_sec)

        if len(s_xyz_list) > 0:
            s_xyz = np.concatenate(s_xyz_list, axis=0)
            s_d = np.concatenate(s_d_list, axis=0)
            s_yatay = np.concatenate(s_yatay_list, axis=0)
            tum_xyz_listesi.append(s_xyz)
            tum_d_listesi.append(s_d)
            tum_yatay_listesi.append(s_yatay)

            s_z = s_xyz[:, 2]
            s_z_ust = max(
                float(np.ceil(float(np.max(s_z)) / KUTU_BOYU_M) * KUTU_BOYU_M),
                args.zmin + 0.10,
            )
            s_kenarlar = np.arange(args.zmin, s_z_ust + KUTU_BOYU_M * 0.5, KUTU_BOYU_M)
            s_tavan_z, _, _ = tepe_bul(
                s_z[s_yatay], s_d[s_yatay], s_kenarlar, min_toplam_yatay=30, agirlikli_secim=True
            )
            s_yakin_maske = s_yatay & (s_d < 6.0)
            s_tavan_yakin, _, _ = tepe_bul(
                s_z[s_yakin_maske],
                s_d[s_yakin_maske],
                s_kenarlar,
                min_toplam_yatay=20,
                agirlikli_secim=True,
            )
            s_z99 = round(float(np.percentile(s_z, 99.0)), 4)
            s_alt_sinir = alt_sinir_hesapla(s_z, s_d, s_yatay)
            sahne_basina[sahne.name] = {
                "kullanilan_kare": int(kullanilan_kare),
                "toplam_nokta": int(len(s_z)),
                "yatay_nokta": int(np.sum(s_yatay)),
                "tavan_z": s_tavan_z,
                "tavan_z_yakin": s_tavan_yakin,
                "z99": s_z99,
                "alt_sinir": s_alt_sinir,
            }
        else:
            sahne_basina[sahne.name] = {
                "kullanilan_kare": int(kullanilan_kare),
                "toplam_nokta": 0,
                "yatay_nokta": 0,
                "tavan_z": None,
                "tavan_z_yakin": None,
                "z99": None,
                "alt_sinir": None,
            }

        print(
            f"[TAVAN] {sahne.name}: kare={kullanilan_kare}, "
            f"toplam={sahne_basina[sahne.name]['toplam_nokta']}, "
            f"yatay={sahne_basina[sahne.name]['yatay_nokta']}, "
            f"tavan_z={sahne_basina[sahne.name]['tavan_z']}"
        )

    if len(tum_xyz_listesi) > 0:
        xyz_tum = np.concatenate(tum_xyz_listesi, axis=0)
        d_tum = np.concatenate(tum_d_listesi, axis=0)
        yatay_tum = np.concatenate(tum_yatay_listesi, axis=0)
    else:
        xyz_tum = np.empty((0, 3), dtype=np.float32)
        d_tum = np.empty((0,), dtype=np.float32)
        yatay_tum = np.empty((0,), dtype=bool)

    toplam_nokta = int(len(xyz_tum))
    yatay_nokta = int(np.sum(yatay_tum))
    z_tum = xyz_tum[:, 2] if toplam_nokta > 0 else np.empty((0,), dtype=np.float32)
    z_yatay = z_tum[yatay_tum]
    d_yatay = d_tum[yatay_tum]

    z_ust = (
        max(float(np.ceil(float(np.max(z_tum)) / KUTU_BOYU_M) * KUTU_BOYU_M), args.zmin + 0.20)
        if toplam_nokta > 0
        else args.zmin + 1.0
    )
    kenarlar = np.arange(args.zmin, z_ust + KUTU_BOYU_M * 0.5, KUTU_BOYU_M)
    merkezler = 0.5 * (kenarlar[:-1] + kenarlar[1:])

    tavan_z, sayi_hist, agirlik_hist = tepe_bul(
        z_yatay, d_yatay, kenarlar, min_toplam_yatay=50, agirlikli_secim=True
    )
    yakin_maske = d_yatay < 6.0
    tavan_z_yakin, _, _ = tepe_bul(
        z_yatay[yakin_maske],
        d_yatay[yakin_maske],
        kenarlar,
        min_toplam_yatay=30,
        agirlikli_secim=True,
    )

    z99 = round(float(np.percentile(z_tum, 99.0)), 4) if toplam_nokta > 0 else None
    alt_sinir = alt_sinir_hesapla(z_tum, d_tum, yatay_tum)

    histogram_listesi = [
        [round(float(merkezler[k]), 4), int(sayi_hist[k]), round(float(agirlik_hist[k]), 4)]
        for k in range(len(merkezler))
    ]

    if tavan_z is None:
        durum_mesaji = (
            f"Tavan tepkisi yok (kamera tavanı görmüyor); "
            f"duvarlarda görülen en yüksek z (alt sınır): {alt_sinir} m"
        )
        print(f"[TAVAN] UYARI: {durum_mesaji}")
    else:
        durum_mesaji = (
            f"Tavan yüksekliği tespit edildi: tavan_z={tavan_z} m "
            f"(yakın d<6m: {tavan_z_yakin} m, z99: {z99} m)"
        )
        print(f"[TAVAN] SONUÇ: {durum_mesaji}")

    rapor: Dict[str, Any] = {
        "tavan_z": tavan_z,
        "tavan_z_yakin": tavan_z_yakin,
        "z99": z99,
        "alt_sinir": alt_sinir,
        "yatay_nokta": yatay_nokta,
        "toplam_nokta": toplam_nokta,
        "durum": durum_mesaji,
        "parametreler": {
            "her": int(args.her),
            "zmin": float(args.zmin),
            "zmax_m": float(args.zmax_m),
            "dx": float(args.dx),
            "dy": float(args.dy),
            "dyaw": float(args.dyaw),
        },
        "sahne_basina": sahne_basina,
        "histogram": histogram_listesi,
    }

    yaml_yolu = cikti_dizin / "tavan.yaml"
    with yaml_yolu.open("w", encoding="utf-8") as f:
        yaml.safe_dump(rapor, f, sort_keys=False, allow_unicode=True)

    grafik_histogram_kaydet(
        cikti_yolu=cikti_dizin / "tavan_hist.png",
        z_tum=z_tum,
        z_yatay=z_yatay,
        d_yatay=d_yatay,
        kenarlar=kenarlar,
        sayi_hist=sayi_hist,
        agirlik_hist=agirlik_hist,
        tavan_z=tavan_z,
        tavan_z_yakin=tavan_z_yakin,
        z99=z99,
        alt_sinir=alt_sinir,
    )
    grafik_ustten_kaydet(
        cikti_yolu=cikti_dizin / "tavan_ustten.png",
        xyz_tum=xyz_tum,
        yatay_tum=yatay_tum,
        tavan_z=tavan_z,
        alt_sinir=alt_sinir,
    )


if __name__ == "__main__":
    main()
