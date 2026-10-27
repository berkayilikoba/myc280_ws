#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kamera görüntülerinden duvar ve zemin dokuları ile Gazebo Classic malzemeleri üretir."""

import argparse
import math
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml


def poz_matrisi(satir: np.ndarray) -> np.ndarray:
    """[x, y, z, qx, qy, qz, qw] satırını 4x4 homojen dönüşüm matrisine çevirir."""
    x, y, z, qx, qy, qz, qw = satir[:7]
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
    """Gövde çerçevesinde (dx, dy, dyaw) kalibrasyon düzeltme matrisi üretir."""
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


def ic_normal_bul(p1: np.ndarray, p2: np.ndarray, oda_siniri: np.ndarray) -> np.ndarray:
    """Duvarın oda_siniri çokgeninin içine bakan birim 2B normalini hesaplar."""
    vec = p2 - p1
    boy = float(np.linalg.norm(vec))
    u = vec / boy if boy > 1e-9 else np.array([1.0, 0.0], dtype=np.float64)
    n_a = np.array([-u[1], u[0]], dtype=np.float64)
    n_b = -n_a
    orta = 0.5 * (p1 + p2)

    if len(oda_siniri) >= 3:
        poly = oda_siniri.astype(np.float32).reshape(-1, 1, 2)
        adimlar = (0.15, 0.35, 0.60)
        skor_a = sum(
            cv2.pointPolygonTest(poly, (float(orta[0] + d * n_a[0]), float(orta[1] + d * n_a[1])), True)
            for d in adimlar
        )
        skor_b = sum(
            cv2.pointPolygonTest(poly, (float(orta[0] + d * n_b[0]), float(orta[1] + d * n_b[1])), True)
            for d in adimlar
        )
        if abs(skor_a - skor_b) > 1e-6:
            return n_a if skor_a > skor_b else n_b
        merkez = oda_siniri.mean(axis=0)
        return n_a if float(np.dot(merkez - orta, n_a)) >= 0.0 else n_b
    return n_a


def duvar_duzlemi_olustur(
    indis: int, duvar: dict, oda_siniri: np.ndarray, tavan: float, coz: float
) -> dict:
    """Duvar için 3B teksel ızgara bileşenlerini ve biriktirme tamponlarını hazırlar."""
    p1 = np.asarray(duvar["p1"], dtype=np.float64)[:2]
    p2 = np.asarray(duvar["p2"], dtype=np.float64)[:2]
    vec = p2 - p1
    vec_boy = float(np.linalg.norm(vec))
    uzunluk = float(duvar.get("uzunluk", vec_boy))
    kalinlik = float(duvar.get("kalinlik", 0.10))
    u = vec / vec_boy if vec_boy > 1e-9 else np.array([1.0, 0.0], dtype=np.float64)

    n_ic = ic_normal_bul(p1, p2, oda_siniri)
    p1_yuz = p1 + 0.5 * kalinlik * n_ic

    W = max(1, int(math.ceil(uzunluk / coz)))
    H = max(1, int(math.ceil(tavan / coz)))

    s_cols = np.minimum((np.arange(W, dtype=np.float64) + 0.5) * coz, uzunluk)
    t_rows = np.clip(tavan - (np.arange(H, dtype=np.float64) + 0.5) * coz, 0.0, tavan)

    A = np.zeros((W, 3), dtype=np.float64)
    A[:, 0] = p1_yuz[0] + s_cols * u[0]
    A[:, 1] = p1_yuz[1] + s_cols * u[1]

    B = np.zeros((H, 3), dtype=np.float64)
    B[:, 2] = t_rows

    normal = np.array([n_ic[0], n_ic[1], 0.0], dtype=np.float64)
    return {
        "tip": "duvar",
        "i": indis,
        "p1": [round(float(p1[0]), 4), round(float(p1[1]), 4)],
        "p2": [round(float(p2[0]), 4), round(float(p2[1]), 4)],
        "yuz_normali": [round(float(n_ic[0]), 4), round(float(n_ic[1]), 4)],
        "A": A,
        "B": B,
        "p0": A[0] + B[0],
        "normal": normal,
        "renk_top": np.zeros((H, W, 3), dtype=np.float32),
        "agirlik_top": np.zeros((H, W), dtype=np.float32),
        "en_iyi_agirlik": np.zeros((H, W, 3), dtype=np.float32),
        "en_iyi_renk": np.zeros((H, W, 3, 3), dtype=np.float32),
        "aday_piksel": 0,
        "reddedilen_piksel": 0,
        "kazanclar": [],
    }


def mobilya_alti_maskesi_olustur(
    kutu: list[float],
    zemin_coz: float,
    H: int,
    W: int,
    mobilya_listesi: list[dict],
    duvar_listesi: list[dict],
) -> np.ndarray:
    """Mobilya çokgenleri ve duvar parçalarının 0.05 m çevresini içeren mobilya_alti maskesini (H, W bool) üretir."""
    xmin, ymin, xmax, ymax = kutu
    mask = np.zeros((H, W), dtype=np.uint8)

    for m in mobilya_listesi:
        cokgen = m.get("cokgen", [])
        if len(cokgen) >= 3:
            pts = np.array(
                [
                    [
                        int(round((pt[0] - xmin) / zemin_coz)),
                        int(round((ymax - pt[1]) / zemin_coz)),
                    ]
                    for pt in cokgen
                ],
                dtype=np.int32,
            )
            cv2.fillPoly(mask, [pts], 255)
        elif len(cokgen) == 2:
            p1 = (
                int(round((cokgen[0][0] - xmin) / zemin_coz)),
                int(round((ymax - cokgen[0][1]) / zemin_coz)),
            )
            p2 = (
                int(round((cokgen[1][0] - xmin) / zemin_coz)),
                int(round((ymax - cokgen[1][1]) / zemin_coz)),
            )
            cv2.line(mask, p1, p2, 255, thickness=1)

    for d in duvar_listesi:
        p1 = d.get("p1")
        p2 = d.get("p2")
        if p1 is None or p2 is None:
            continue
        kalinlik = float(d.get("kalinlik", 0.10))
        vec = np.array([p2[0] - p1[0], p2[1] - p1[1]], dtype=np.float64)
        v_boy = float(np.linalg.norm(vec))
        if v_boy > 1e-6:
            n = np.array([-vec[1], vec[0]]) / v_boy
            offset = 0.5 * kalinlik * n
            c1 = np.array(p1) + offset
            c2 = np.array(p1) - offset
            c3 = np.array(p2) - offset
            c4 = np.array(p2) + offset
            poly = np.array(
                [
                    [
                        int(round((pt[0] - xmin) / zemin_coz)),
                        int(round((ymax - pt[1]) / zemin_coz)),
                    ]
                    for pt in (c1, c2, c3, c4)
                ],
                dtype=np.int32,
            )
            cv2.fillPoly(mask, [poly], 255)
        p1_px = (
            int(round((p1[0] - xmin) / zemin_coz)),
            int(round((ymax - p1[1]) / zemin_coz)),
        )
        p2_px = (
            int(round((p2[0] - xmin) / zemin_coz)),
            int(round((ymax - p2[1]) / zemin_coz)),
        )
        t_px = max(1, int(round(kalinlik / zemin_coz)))
        cv2.line(mask, p1_px, p2_px, 255, thickness=t_px)

    genisletme_px = int(round(0.05 / zemin_coz))
    if genisletme_px > 0:
        ksize = 2 * genisletme_px + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        mask = cv2.dilate(mask, kernel)

    return mask > 0


def zemin_duzlemi_olustur(
    kutu: list[float],
    zemin_coz: float,
    mobilya_listesi: list[dict] | None = None,
    duvar_listesi: list[dict] | None = None,
) -> dict:
    """Zemin (z=0) için 3B teksel ızgara bileşenlerini ve biriktirme tamponlarını hazırlar."""
    xmin, ymin, xmax, ymax = kutu
    W = max(1, int(math.ceil(max(xmax - xmin, zemin_coz) / zemin_coz)))
    H = max(1, int(math.ceil(max(ymax - ymin, zemin_coz) / zemin_coz)))

    x_cols = np.minimum(xmin + (np.arange(W, dtype=np.float64) + 0.5) * zemin_coz, xmax)
    y_rows = np.clip(ymax - (np.arange(H, dtype=np.float64) + 0.5) * zemin_coz, ymin, ymax)

    A = np.zeros((W, 3), dtype=np.float64)
    A[:, 0] = x_cols

    B = np.zeros((H, 3), dtype=np.float64)
    B[:, 1] = y_rows

    mobilya_alti = mobilya_alti_maskesi_olustur(
        kutu, zemin_coz, H, W, mobilya_listesi or [], duvar_listesi or []
    )

    return {
        "tip": "zemin",
        "kutu": [round(float(v), 4) for v in kutu],
        "A": A,
        "B": B,
        "p0": A[0] + B[0],
        "normal": np.array([0.0, 0.0, 1.0], dtype=np.float64),
        "renk_top": np.zeros((H, W, 3), dtype=np.float32),
        "agirlik_top": np.zeros((H, W), dtype=np.float32),
        "en_iyi_agirlik": np.zeros((H, W, 3), dtype=np.float32),
        "en_iyi_renk": np.zeros((H, W, 3, 3), dtype=np.float32),
        "aday_piksel": 0,
        "reddedilen_piksel": 0,
        "kazanclar": [],
        "mobilya_alti": mobilya_alti,
    }


def duzlem_tampon_sifirla(duzlem: dict) -> None:
    """İkinci geçiş öncesi düzlemin biriktirme tamponlarını ve sayaçlarını sıfırlar."""
    duzlem["renk_top"].fill(0.0)
    duzlem["agirlik_top"].fill(0.0)
    if "en_iyi_agirlik" in duzlem:
        duzlem["en_iyi_agirlik"].fill(0.0)
    if "en_iyi_renk" in duzlem:
        duzlem["en_iyi_renk"].fill(0.0)
    duzlem["aday_piksel"] = 0
    duzlem["reddedilen_piksel"] = 0
    duzlem["kazanclar"] = []


def ham_doku_hesapla(
    duzlem: dict, birlestir: str = "enIyi"
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Biriktirilen tamponlardan doldurulmamış ham BGR dokuyu (float32), görülen maskesini ve ağırlığı döndürür."""
    if (
        birlestir == "enIyi"
        and "en_iyi_agirlik" in duzlem
        and "en_iyi_renk" in duzlem
        and np.any(duzlem["en_iyi_agirlik"] > 0.0)
    ):
        w_k = duzlem["en_iyi_agirlik"]
        c_k = duzlem["en_iyi_renk"]
        agirlik = np.sum(w_k, axis=2)
        renk_top = np.sum(c_k * w_k[..., None], axis=2)
    else:
        agirlik = duzlem["agirlik_top"]
        renk_top = duzlem["renk_top"]

    gorulen = agirlik > 0.0
    H, W = agirlik.shape
    doku_f = np.zeros((H, W, 3), dtype=np.float32)
    if np.any(gorulen):
        doku_f[gorulen] = renk_top[gorulen] / agirlik[gorulen, None]
    return doku_f, gorulen, agirlik


def kare_keskinlik_varyansi(bgr: np.ndarray) -> float:
    """Kare için gri seviye Laplacian varyansını hesaplar."""
    gri = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    return float(cv2.Laplacian(gri, cv2.CV_64F).var())


def kare_izdusur(
    duzlem: dict,
    bgr: np.ndarray,
    depth: np.ndarray,
    R_cw: np.ndarray,
    t_cw: np.ndarray,
    cam_pos: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    zemin_gecersiz_menzil: float = 1.2,
    keskinlik_carpani: float = 1.0,
    referans_doku: np.ndarray | None = None,
    referans_gorulen: np.ndarray | None = None,
    depth_pencere_gorunumu: np.ndarray | None = None,
) -> None:
    """Düzlem teksellerini kameraya vektörel izdüşürür, örtüşme süzgeciyle ağırlıklı biriktirir."""
    d_duzlem = float(np.dot(cam_pos - duzlem["p0"], duzlem["normal"]))
    if d_duzlem <= 1e-3:
        return

    img_h, img_w = depth.shape[:2]
    A_c = duzlem["A"] @ R_cw.T + t_cw
    B_c = duzlem["B"] @ R_cw.T

    # Kamera görüş konisi (frustum) ile ayrılabilir 1B satır/sütun ön elemesi
    L_A = np.column_stack(
        [
            A_c[:, 2],
            fx * A_c[:, 0] + (cx + 1.0) * A_c[:, 2],
            (img_w - cx) * A_c[:, 2] - fx * A_c[:, 0],
            fy * A_c[:, 1] + (cy + 1.0) * A_c[:, 2],
            (img_h - cy) * A_c[:, 2] - fy * A_c[:, 1],
        ]
    )
    L_B = np.column_stack(
        [
            B_c[:, 2],
            fx * B_c[:, 0] + (cx + 1.0) * B_c[:, 2],
            (img_w - cx) * B_c[:, 2] - fx * B_c[:, 0],
            fy * B_c[:, 1] + (cy + 1.0) * B_c[:, 2],
            (img_h - cy) * B_c[:, 2] - fy * B_c[:, 1],
        ]
    )

    c_mask = np.all(L_A + L_B.max(axis=0) > 0.0, axis=1)
    if not np.any(c_mask):
        return
    c_ind = np.flatnonzero(c_mask)

    r_mask = np.all(L_B + L_A[c_ind].max(axis=0) > 0.0, axis=1)
    if not np.any(r_mask):
        return
    r_ind = np.flatnonzero(r_mask)

    A_sub = A_c[c_ind]
    B_sub = B_c[r_ind]

    z_c = B_sub[:, None, 2] + A_sub[None, :, 2]
    x_c = B_sub[:, None, 0] + A_sub[None, :, 0]
    y_c = B_sub[:, None, 1] + A_sub[None, :, 1]

    on_maske = z_c > 1e-3
    if not np.any(on_maske):
        return

    inv_z = np.where(on_maske, 1.0 / np.maximum(z_c, 1e-3), 0.0)
    u = np.rint(fx * x_c * inv_z + cx).astype(np.int32)
    v = np.rint(fy * y_c * inv_z + cy).astype(np.int32)

    cerceve_ici = on_maske & (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    if not np.any(cerceve_ici):
        return

    rr, cc = np.nonzero(cerceve_ici)

    if "mobilya_alti" in duzlem and duzlem["mobilya_alti"] is not None:
        gecerli = ~duzlem["mobilya_alti"][r_ind[rr], c_ind[cc]]
        if not np.any(gecerli):
            return
        rr = rr[gecerli]
        cc = cc[gecerli]

    u_g = u[rr, cc]
    v_g = v[rr, cc]
    z_g = z_c[rr, cc]

    n_aday = int(len(z_g))
    duzlem["aday_piksel"] = int(duzlem.get("aday_piksel", 0)) + n_aday

    # Örtüşme kontrolü:
    # - Geçerli derinlik: |z_g - d_olc| <= tol (zemin 0.04 m, duvar 0.08 m)
    #   + Zemin için 3B nokta yükseklik testi: z_3b <= 0.03 m
    # - Geçersiz derinlik: duvarda reddet; zeminde yalnız z_g <= zemin_gecersiz_menzil
    #   VE 7x7 komşuluktaki geçerli derinliklerin medyanı |z_g - d_med| <= tol
    zemin_mi = duzlem.get("tip") == "zemin"
    tol_derinlik = 0.04 if zemin_mi else 0.08
    d_olc = depth[v_g, u_g]
    gecerli_d = d_olc > 0.0
    gorunur = np.zeros(n_aday, dtype=bool)

    if np.any(gecerli_d):
        idx_gec = np.flatnonzero(gecerli_d)
        z_gec = z_g[idx_gec]
        d_gec = d_olc[idx_gec]
        kabul_gec = np.abs(z_gec - d_gec) <= tol_derinlik
        if zemin_mi and np.any(kabul_gec):
            idx_kabul = idx_gec[kabul_gec]
            u_k = u_g[idx_kabul].astype(np.float64)
            v_k = v_g[idx_kabul].astype(np.float64)
            d_k = d_olc[idx_kabul].astype(np.float64)
            x_m = (u_k - cx) * (d_k / fx)
            y_m = (v_k - cy) * (d_k / fy)
            z_3b = R_cw[0, 2] * x_m + R_cw[1, 2] * y_m + R_cw[2, 2] * d_k + cam_pos[2]
            kabul_gec[kabul_gec] = z_3b <= 0.03
        gorunur[idx_gec] = kabul_gec

    if zemin_mi and zemin_gecersiz_menzil > 0.0:
        aday_gecersiz = (~gecerli_d) & (z_g <= zemin_gecersiz_menzil)
        if np.any(aday_gecersiz):
            if depth_pencere_gorunumu is None:
                depth_pad = np.pad(depth, 3, mode="constant", constant_values=0.0)
                depth_pencere_gorunumu = np.lib.stride_tricks.sliding_window_view(depth_pad, (7, 7))
            idx_inv = np.flatnonzero(aday_gecersiz)
            v_inv = v_g[idx_inv]
            u_inv = u_g[idx_inv]
            z_inv = z_g[idx_inv]
            gecersiz_kabul = np.zeros(len(idx_inv), dtype=bool)
            for bas in range(0, len(idx_inv), 8192):
                bit = min(bas + 8192, len(idx_inv))
                yamalar = depth_pencere_gorunumu[v_inv[bas:bit], u_inv[bas:bit]].reshape(-1, 49)
                gec_mask = yamalar > 0.0
                gec_say = np.count_nonzero(gec_mask, axis=1)
                var = gec_say > 0
                if not np.any(var):
                    continue
                yamalar_var = np.where(gec_mask[var], yamalar[var], np.inf)
                sirali = np.sort(yamalar_var, axis=1)
                n_v = gec_say[var]
                satir_ind = np.arange(len(n_v))
                med = 0.5 * (sirali[satir_ind, (n_v - 1) // 2] + sirali[satir_ind, n_v // 2])
                dilim_kabul = np.zeros(bit - bas, dtype=bool)
                dilim_kabul[var] = np.abs(z_inv[bas:bit][var] - med) <= tol_derinlik
                gecersiz_kabul[bas:bit] = dilim_kabul
            gorunur[idx_inv] = gecersiz_kabul

    n_kabul = int(np.count_nonzero(gorunur))
    duzlem["reddedilen_piksel"] = int(duzlem.get("reddedilen_piksel", 0)) + (n_aday - n_kabul)
    if n_kabul == 0:
        return

    rr = rr[gorunur]
    cc = cc[gorunur]
    u_g = u_g[gorunur]
    v_g = v_g[gorunur]
    z_g = z_g[gorunur]
    x_g = x_c[rr, cc]
    y_g = y_c[rr, cc]

    mesafe = np.sqrt(x_g * x_g + y_g * y_g + z_g * z_g)
    cos_aci = np.clip(d_duzlem / np.maximum(mesafe, 1e-3), 0.0, 1.0)
    mesafe_kare = np.maximum(mesafe * mesafe, 1e-6)
    agirlik = ((cos_aci / mesafe_kare) * float(keskinlik_carpani)).astype(np.float32)

    poz = agirlik > 0.0
    if not np.any(poz):
        return

    r_hedef = r_ind[rr[poz]]
    c_hedef = c_ind[cc[poz]]
    w_son = agirlik[poz]
    renk = bgr[v_g[poz], u_g[poz]].astype(np.float32)

    # İkinci geçişte kare pozlama kazancı: medyan(doku / kare pikseli), 0.6..1.6'ya kırp
    if referans_doku is not None and referans_gorulen is not None:
        ortusen = referans_gorulen[r_hedef, c_hedef]
        if np.any(ortusen):
            ref_renk = referans_doku[r_hedef[ortusen], c_hedef[ortusen]]
            kare_renk = renk[ortusen]
            gecerli_kanal = (kare_renk >= 5.0) & (ref_renk >= 5.0)
            if np.any(gecerli_kanal):
                oranlar = ref_renk[gecerli_kanal] / kare_renk[gecerli_kanal]
                kazanc = float(np.clip(np.median(oranlar), 0.6, 1.6))
            else:
                kazanc = 1.0
            duzlem.setdefault("kazanclar", []).append(kazanc)
            renk = np.clip(renk * kazanc, 0.0, 255.0)

    duzlem["renk_top"][r_hedef, c_hedef] += renk * w_son[:, None]
    duzlem["agirlik_top"][r_hedef, c_hedef] += w_son

    if "en_iyi_agirlik" not in duzlem or "en_iyi_renk" not in duzlem:
        H, W = duzlem["agirlik_top"].shape
        duzlem["en_iyi_agirlik"] = np.zeros((H, W, 3), dtype=np.float32)
        duzlem["en_iyi_renk"] = np.zeros((H, W, 3, 3), dtype=np.float32)

    mevcut_w = duzlem["en_iyi_agirlik"][r_hedef, c_hedef]
    en_kucuk_ind = np.argmin(mevcut_w, axis=1)
    en_kucuk_w = mevcut_w[np.arange(len(w_son)), en_kucuk_ind]
    degistir = w_son > en_kucuk_w
    if np.any(degistir):
        r_deg = r_hedef[degistir]
        c_deg = c_hedef[degistir]
        k_deg = en_kucuk_ind[degistir]
        duzlem["en_iyi_agirlik"][r_deg, c_deg, k_deg] = w_son[degistir]
        duzlem["en_iyi_renk"][r_deg, c_deg, k_deg] = renk[degistir]


def _zemin_medyan_blur_doldur(
    doku_f: np.ndarray,
    gorulen: np.ndarray,
    medyan_renk: np.ndarray,
) -> np.ndarray:
    sonuc = doku_f.copy()
    sonuc[~gorulen] = medyan_renk
    if np.any(gorulen) and np.any(~gorulen):
        yumusak = cv2.GaussianBlur(sonuc, (15, 15), 0)
        sonuc[~gorulen] = yumusak[~gorulen]
    return sonuc


def zemin_yama_ile_doldur(
    doku_f: np.ndarray,
    gorulen: np.ndarray,
    agirlik: np.ndarray,
    medyan_renk: np.ndarray,
    yama_boyut: int = 96,
) -> np.ndarray:
    """Görülen zeminden kaliteli yamalar seçip boşlukları örtüşmeli ve renk ölçekli döşer."""
    H, W = gorulen.shape
    if not np.any(~gorulen):
        return doku_f.copy()
    if not np.any(gorulen):
        return _zemin_medyan_blur_doldur(doku_f, gorulen, medyan_renk)

    P = min(int(yama_boyut), H, W)
    if P < 8:
        print("[UYARI] Zemin yama boyutu çok küçük, medyan+blur yöntemine geçiliyor.")
        return _zemin_medyan_blur_doldur(doku_f, gorulen, medyan_renk)

    alan = float(P * P)

    # 1. Tamamen geçerli görülen bölgeler (maske 1)
    gor_int = cv2.integral(gorulen.astype(np.float32))
    gor_toplam = (
        gor_int[P:, P:] - gor_int[:-P, P:] - gor_int[P:, :-P] + gor_int[:-P, :-P]
    )
    tamamen_gecerli = gor_toplam >= (alan - 1e-3)
    if not np.any(tamamen_gecerli):
        print("[UYARI] Zemin yama doldurma için tamamen geçerli bölge bulunamadı, medyan+blur yöntemine geçiliyor.")
        return _zemin_medyan_blur_doldur(doku_f, gorulen, medyan_renk)

    # 2. Yerel ortalama renk ve medyana yakınlık (Lab delta_E < 12)
    b_int = cv2.integral(doku_f[:, :, 0].astype(np.float32))
    g_int = cv2.integral(doku_f[:, :, 1].astype(np.float32))
    r_int = cv2.integral(doku_f[:, :, 2].astype(np.float32))
    ort_b = (b_int[P:, P:] - b_int[:-P, P:] - b_int[P:, :-P] + b_int[:-P, :-P]) / alan
    ort_g = (g_int[P:, P:] - g_int[:-P, P:] - g_int[P:, :-P] + g_int[:-P, :-P]) / alan
    ort_r = (r_int[P:, P:] - r_int[:-P, P:] - r_int[P:, :-P] + r_int[:-P, :-P]) / alan
    ort_bgr = np.stack([ort_b, ort_g, ort_r], axis=-1)

    ort_bgr_norm = np.clip(ort_bgr / 255.0, 0.0, 1.0).astype(np.float32)
    ort_lab = cv2.cvtColor(ort_bgr_norm, cv2.COLOR_BGR2Lab)

    med_bgr_norm = np.clip(medyan_renk.reshape(1, 1, 3) / 255.0, 0.0, 1.0).astype(np.float32)
    med_lab = cv2.cvtColor(med_bgr_norm, cv2.COLOR_BGR2Lab).reshape(3)

    delta_E = np.sqrt(np.sum((ort_lab - med_lab) ** 2, axis=-1))
    renk_uygun = delta_E < 12.0

    # 3. Güçlü kenar içermeyen bölgeler (Canny kenar pikseli oranı < %2)
    gri = cv2.cvtColor(np.clip(doku_f, 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)
    gri_blur = cv2.GaussianBlur(gri, (3, 3), 0)
    kenarlar = cv2.Canny(gri_blur, 50, 150)
    ic_maske = cv2.erode(gorulen.astype(np.uint8), np.ones((3, 3), dtype=np.uint8)) > 0
    kenarlar[~ic_maske] = 0

    kenar_int = cv2.integral((kenarlar > 0).astype(np.float32))
    kenar_toplam = (
        kenar_int[P:, P:] - kenar_int[:-P, P:] - kenar_int[P:, :-P] + kenar_int[:-P, :-P]
    )
    kenar_oran = kenar_toplam / alan
    kenar_uygun = kenar_oran < 0.02

    # 4. Parlaklık standart sapması düşük bölgeler
    gri_f = gri.astype(np.float32)
    gri_int = cv2.integral(gri_f)
    gri2_int = cv2.integral(gri_f * gri_f)
    ort_gri = (gri_int[P:, P:] - gri_int[:-P, P:] - gri_int[P:, :-P] + gri_int[:-P, :-P]) / alan
    ort_gri2 = (gri2_int[P:, P:] - gri2_int[:-P, P:] - gri2_int[P:, :-P] + gri2_int[:-P, :-P]) / alan
    var_gri = np.maximum(0.0, ort_gri2 - ort_gri * ort_gri)
    std_gri = np.sqrt(var_gri)

    on_adaylar = tamamen_gecerli & renk_uygun & kenar_uygun
    if not np.any(on_adaylar):
        print("[UYARI] Zemin yama doldurma için kriterlere uyan aday bulunamadı, medyan+blur yöntemine geçiliyor.")
        return _zemin_medyan_blur_doldur(doku_f, gorulen, medyan_renk)

    stds = std_gri[on_adaylar]
    std_esik = max(float(np.percentile(stds, 50)), 12.0)
    aday_maske = on_adaylar & (std_gri <= std_esik)
    if not np.any(aday_maske):
        aday_maske = on_adaylar

    # Ağırlığı yüksek olan adayları önceliklendir
    ag_int = cv2.integral(agirlik.astype(np.float32))
    ag_ort_p = (
        ag_int[P:, P:] - ag_int[:-P, P:] - ag_int[P:, :-P] + ag_int[:-P, :-P]
    ) / alan
    skor = np.where(aday_maske, ag_ort_p, -1.0)

    valid_indices = np.flatnonzero(skor > 0)
    if len(valid_indices) == 0:
        print("[UYARI] Zemin yama doldurma için pozitif ağırlıklı aday bulunamadı, medyan+blur yöntemine geçiliyor.")
        return _zemin_medyan_blur_doldur(doku_f, gorulen, medyan_renk)

    sorted_indices = valid_indices[np.argsort(-skor.ravel()[valid_indices])]

    # Birbirinden en az bir yama boyutu uzak en iyi 4–8 aday seç
    secilenler: list[tuple[int, int]] = []
    for min_dist_ratio in (1.0, 0.75, 0.5):
        min_dist_sq = (min_dist_ratio * P) ** 2
        for idx in sorted_indices:
            r, c = divmod(int(idx), skor.shape[1])
            if any((r - sr) ** 2 + (c - sc) ** 2 < min_dist_sq for sr, sc in secilenler):
                continue
            secilenler.append((r, c))
            if len(secilenler) >= 8:
                break
        if len(secilenler) >= 4:
            break

    if len(secilenler) == 0:
        print("[UYARI] Zemin yama adayı seçilemedi, medyan+blur yöntemine geçiliyor.")
        return _zemin_medyan_blur_doldur(doku_f, gorulen, medyan_renk)

    yama_listesi = [doku_f[r : r + P, c : c + P].copy() for r, c in secilenler]

    # Hücreler arasında %25 örtüşmeyle doğrusal karıştırma (feathering)
    O = max(1, int(round(P * 0.25)))
    S = max(1, P - O)

    w_1d = np.ones(P, dtype=np.float32)
    ramp = (np.arange(O, dtype=np.float32) + 0.5) / float(O)
    w_1d[:O] = ramp
    w_1d[-O:] = ramp[::-1]
    w_2d = (w_1d[:, None] * w_1d[None, :])[..., None]

    r_starts = []
    r = 0
    while r + P < H:
        r_starts.append(r)
        r += S
    r_starts.append(max(0, H - P))
    r_starts = sorted(set(r_starts))

    c_starts = []
    c = 0
    while c + P < W:
        c_starts.append(c)
        c += S
    c_starts.append(max(0, W - P))
    c_starts = sorted(set(c_starts))

    rng = np.random.RandomState(0)
    dolu_renk = np.zeros((H, W, 3), dtype=np.float32)
    dolu_agirlik = np.zeros((H, W), dtype=np.float32)

    for r in r_starts:
        for c in c_starts:
            idx = int(rng.randint(0, len(yama_listesi)))
            patch = yama_listesi[idx].copy()
            if rng.randint(0, 2):
                patch = np.fliplr(patch)
            if rng.randint(0, 2):
                patch = np.flipud(patch)
            dolu_renk[r : r + P, c : c + P] += patch * w_2d
            dolu_agirlik[r : r + P, c : c + P] += w_2d[..., 0]

    dosenmis = dolu_renk / np.maximum(dolu_agirlik[..., None], 1e-6)

    # Döşenen dolgunun rengini en yakın görülen bölgenin yerel ortalama rengine göre ölçekle
    gor_f = gorulen.astype(np.float32)
    sigma_yerel = 40.0
    agirlik_yerel = cv2.GaussianBlur(gor_f, (0, 0), sigmaX=sigma_yerel, sigmaY=sigma_yerel)
    renk_yerel = cv2.GaussianBlur(
        doku_f * gor_f[..., None], (0, 0), sigmaX=sigma_yerel, sigmaY=sigma_yerel
    )
    yerel_gor = np.where(
        agirlik_yerel[..., None] > 1e-4,
        renk_yerel / np.maximum(agirlik_yerel[..., None], 1e-4),
        medyan_renk,
    )
    alfa_y = np.clip(agirlik_yerel / 0.05, 0.0, 1.0)[..., None]
    yerel_gor = alfa_y * yerel_gor + (1.0 - alfa_y) * medyan_renk

    dosenmis_ort = np.mean(dosenmis, axis=(0, 1))
    oran = yerel_gor / np.maximum(dosenmis_ort, 1.0)
    oran = np.clip(oran, 0.5, 2.0)
    dolgu_olcekli = np.clip(dosenmis * oran, 0.0, 255.0)

    # Dolgu ile görülen bölge sınırında 10 teksel yumuşak kenar
    dist = cv2.distanceTransform((~gorulen).astype(np.uint8), cv2.DIST_L2, 5)
    t = np.clip(dist / 10.0, 0.0, 1.0)
    agirlik_kenar = cv2.GaussianBlur(gor_f, (21, 21), 0)
    renk_kenar = cv2.GaussianBlur(doku_f * gor_f[..., None], (21, 21), 0)
    kenar_renk = np.where(
        agirlik_kenar[..., None] > 1e-4,
        renk_kenar / np.maximum(agirlik_kenar[..., None], 1e-4),
        yerel_gor,
    )
    alfa_kenar = (1.0 - t)[..., None]
    dolgu_son = alfa_kenar * kenar_renk + (1.0 - alfa_kenar) * dolgu_olcekli

    sonuc = doku_f.copy()
    sonuc[~gorulen] = dolgu_son[~gorulen]
    return sonuc


def doku_tamamla(
    duzlem: dict,
    gauss_yumusat: bool = True,
    birlestir: str = "enIyi",
    zemin_doldur: str = "yama",
    yama_boyut: int = 96,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Biriktirilen renklerden son BGR dokuyu, görülen maskesini ve görülen oranı üretir."""
    doku_f, gorulen, agirlik = ham_doku_hesapla(duzlem, birlestir=birlestir)
    maske = np.where(gorulen, 255, 0).astype(np.uint8)
    gorulen_oran = float(np.mean(gorulen)) if gorulen.size > 0 else 0.0

    if np.any(gorulen):
        medyan_renk = np.median(doku_f[gorulen], axis=0)
    else:
        medyan_renk = np.array([200.0, 200.0, 200.0], dtype=np.float32)

    if duzlem.get("tip") == "zemin" and zemin_doldur == "yama" and np.any(gorulen) and np.any(~gorulen):
        doku_f = zemin_yama_ile_doldur(
            doku_f, gorulen, agirlik, medyan_renk, yama_boyut=yama_boyut
        )
    else:
        doku_f[~gorulen] = medyan_renk
        if gauss_yumusat and np.any(gorulen) and np.any(~gorulen):
            yumusak = cv2.GaussianBlur(doku_f, (15, 15), 0)
            doku_f[~gorulen] = yumusak[~gorulen]

    doku_bgr = np.clip(np.rint(doku_f), 0, 255).astype(np.uint8)
    return doku_bgr, maske, round(gorulen_oran, 4)


def yuzey_red_orani(duzlem: dict) -> float:
    """Yüzey için örtüşme/yükseklik süzgecinde reddedilen piksel oranını döndürür."""
    aday = int(duzlem.get("aday_piksel", 0))
    red = int(duzlem.get("reddedilen_piksel", 0))
    return round(float(red / aday), 4) if aday > 0 else 0.0


def yuzey_kazanc_istatistikleri(duzlem: dict) -> dict:
    """Yüzey için kare pozlama kazancı istatistiklerini (min/medyan/max) döndürür."""
    kazanclar = duzlem.get("kazanclar", [])
    if len(kazanclar) > 0:
        arr = np.asarray(kazanclar, dtype=np.float64)
        return {
            "min": round(float(np.min(arr)), 4),
            "medyan": round(float(np.median(arr)), 4),
            "max": round(float(np.max(arr)), 4),
        }
    return {"min": 1.0, "medyan": 1.0, "max": 1.0}


def ogre_malzeme_metni(malzeme_adi: str, doku_dosya: str) -> str:
    """Tek bir doku için Gazebo Classic / OGRE malzeme tanımı döndürür."""
    return (
        f"material {malzeme_adi}\n{{\n  technique\n  {{\n    pass\n    {{\n"
        f"      ambient 0.4 0.4 0.4 1.0\n      diffuse 1 1 1 1\n"
        f"      texture_unit\n      {{\n        texture {doku_dosya}\n"
        f"        filtering trilinear\n      }}\n    }}\n  }}\n}}\n"
    )


def esnek_bool(deger: str | bool) -> bool:
    """CLI bayrakları için esnek boolean çözümleyici."""
    if isinstance(deger, bool):
        return deger
    metin = str(deger).strip().lower()
    if metin in ("1", "true", "evet", "acik", "açık", "yes", "on"):
        return True
    if metin in ("0", "false", "hayir", "hayır", "kapali", "kapalı", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Geçersiz mantıksal değer: {deger}")


def argumanlari_coz() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Kamera görüntülerinden duvar ve zemin dokuları üretir")
    p.add_argument("cikti_dizini", type=Path, help="Çıktı dosyalarının yazılacağı dizin")
    p.add_argument("--yapi", required=True, type=Path, help="Oda yapı YAML dosyası (oda_yapi.yaml)")
    p.add_argument("sahne", type=Path, nargs="+", help="Bir veya birden çok sahne dizini")
    p.add_argument("--dx", type=float, default=0.09, help="Gövde x öteleme düzeltmesi (m)")
    p.add_argument("--dy", type=float, default=0.01, help="Gövde y öteleme düzeltmesi (m)")
    p.add_argument("--dyaw", type=float, default=-1.0, help="Gövde yaw düzeltmesi (derece)")
    p.add_argument("--tavan", type=float, default=2.7, help="Tavan yüksekliği (m)")
    p.add_argument("--cozunurluk", type=float, default=0.01, help="Duvar doku çözünürlüğü (m/px)")
    p.add_argument("--zemin_coz", type=float, default=0.01, help="Zemin doku çözünürlüğü (m/px)")
    p.add_argument("--her", type=int, default=3, help="Her N. kareyi işle")
    p.add_argument("--max_w", type=float, default=0.25, help="Maksimum gövde açısal hızı (rad/s)")
    p.add_argument(
        "--zemin_gecersiz_menzil",
        type=float,
        default=1.2,
        help="Zeminde geçersiz derinlik için maksimum menzil (m)",
    )
    p.add_argument(
        "--birlestir",
        choices=["ortalama", "enIyi"],
        default="enIyi",
        help="Teksel birleştirme yöntemi: ortalama veya enIyi (en yüksek ağırlıklı 3 gözlem)",
    )
    p.add_argument(
        "--pozlama_dengele",
        nargs="?",
        const=True,
        default=True,
        type=esnek_bool,
        help="İki geçişli kare pozlama dengeleme (varsayılan: açık)",
    )
    p.add_argument(
        "--no-pozlama_dengele",
        "--pozlama_dengele_kapat",
        dest="pozlama_dengele",
        action="store_false",
        help="Pozlama dengelemeyi kapatır",
    )
    p.add_argument(
        "--zemin_doldur",
        choices=["medyan", "yama"],
        default="yama",
        help="Görülmeyen zemin bölgelerini doldurma yöntemi: medyan veya yama",
    )
    p.add_argument(
        "--yama_boyut",
        type=int,
        default=96,
        help="Zemin yama doldurma için yama boyutu (teksel, varsayılan: 96)",
    )
    return p.parse_args()


def main() -> None:
    args = argumanlari_coz()
    cikti = args.cikti_dizini.expanduser().resolve()
    doku_dizin = cikti / "doku"
    scripts_dizin = cikti / "materials" / "scripts"
    textures_dizin = cikti / "materials" / "textures"
    for d in (doku_dizin, scripts_dizin, textures_dizin):
        d.mkdir(parents=True, exist_ok=True)

    with args.yapi.expanduser().resolve().open("r", encoding="utf-8") as f:
        yapi = yaml.safe_load(f) or {}

    oda_siniri_ham = yapi.get("oda_siniri", [])
    duvarlar_ham = yapi.get("duvarlar", [])
    mobilya_ham = yapi.get("mobilya", [])
    oda_siniri = (
        np.asarray(oda_siniri_ham, dtype=np.float64)[:, :2]
        if len(oda_siniri_ham) > 0
        else np.zeros((0, 2), dtype=np.float64)
    )

    if len(oda_siniri) > 0:
        xmin, ymin = float(oda_siniri[:, 0].min()), float(oda_siniri[:, 1].min())
        xmax, ymax = float(oda_siniri[:, 0].max()), float(oda_siniri[:, 1].max())
    elif len(duvarlar_ham) > 0:
        tum_pt = np.vstack([[d["p1"][:2], d["p2"][:2]] for d in duvarlar_ham]).astype(np.float64)
        xmin, ymin = float(tum_pt[:, 0].min()), float(tum_pt[:, 1].min())
        xmax, ymax = float(tum_pt[:, 0].max()), float(tum_pt[:, 1].max())
    else:
        xmin, ymin, xmax, ymax = -5.0, -5.0, 5.0, 5.0

    duvar_duzlemleri = [
        duvar_duzlemi_olustur(i, d, oda_siniri, args.tavan, args.cozunurluk)
        for i, d in enumerate(duvarlar_ham)
    ]
    zemin_duzlemi = zemin_duzlemi_olustur(
        [xmin, ymin, xmax, ymax],
        args.zemin_coz,
        mobilya_listesi=mobilya_ham,
        duvar_listesi=duvarlar_ham,
    )
    tum_duzlemler = duvar_duzlemleri + [zemin_duzlemi]

    dT = duzeltme_matrisi(args.dx, args.dy, args.dyaw)
    kare_listesi: list[dict] = []

    for sahne_yolu in args.sahne:
        sahne = sahne_yolu.expanduser().resolve()
        with (sahne / "vlmaps_config.yaml").open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        K = np.asarray(cfg["cam_calib_mat"], dtype=np.float64).reshape(3, 3)
        fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])

        poses = np.atleast_2d(np.loadtxt(sahne / "poses.txt", dtype=np.float64))
        poses_kam = np.atleast_2d(np.loadtxt(sahne / "poses_kamera.txt", dtype=np.float64))
        zaman_yolu = sahne / "zaman.txt"
        if zaman_yolu.exists() and len(poses) >= 2:
            zaman = np.atleast_2d(np.loadtxt(zaman_yolu, dtype=np.float64))
            t_sutun = zaman[:, 1] if zaman.shape[1] > 1 else zaman[:, 0]
            qx, qy, qz, qw = poses[:, 3], poses[:, 4], poses[:, 5], poses[:, 6]
            yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            w_hiz = np.gradient(np.unwrap(yaw), t_sutun)
            n_kare = min(len(poses), len(poses_kam), len(zaman))
        else:
            w_hiz = np.zeros(len(poses), dtype=np.float64)
            n_kare = min(len(poses), len(poses_kam))

        for i in range(0, n_kare, max(1, args.her)):
            if abs(w_hiz[i]) > args.max_w:
                continue
            rgb_yolu = sahne / "rgb" / f"{i:06d}.png"
            depth_yolu = sahne / "depth" / f"{i:06d}.npy"
            if not rgb_yolu.exists() or not depth_yolu.exists():
                continue

            gri = cv2.imread(str(rgb_yolu), cv2.IMREAD_GRAYSCALE)
            if gri is None:
                continue
            lap_var = kare_keskinlik_varyansi(gri)

            T_wb = poz_matrisi(poses[i])
            T_wc = poz_matrisi(poses_kam[i])
            T_wc_duz = T_wb @ dT @ np.linalg.inv(T_wb) @ T_wc
            T_cw = np.linalg.inv(T_wc_duz)

            kare_listesi.append(
                {
                    "rgb_yolu": rgb_yolu,
                    "depth_yolu": depth_yolu,
                    "R_cw": T_cw[:3, :3],
                    "t_cw": T_cw[:3, 3],
                    "cam_pos": T_wc_duz[:3, 3],
                    "fx": fx,
                    "fy": fy,
                    "cx": cx,
                    "cy": cy,
                    "lap_var": lap_var,
                    "keskinlik": 1.0,
                }
            )

    if len(kare_listesi) > 0:
        medyan_var = float(np.median([k["lap_var"] for k in kare_listesi]))
        if medyan_var > 1e-9:
            for k in kare_listesi:
                k["keskinlik"] = float(np.clip(k["lap_var"] / medyan_var, 0.2, 1.5))

    def gecis_calistir(referans_kullan: bool) -> None:
        for k in kare_listesi:
            bgr = cv2.imread(str(k["rgb_yolu"]), cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            depth = np.load(str(k["depth_yolu"])).astype(np.float32)
            depth[(depth < 0.0) | ~np.isfinite(depth)] = 0.0
            if depth.shape[:2] != bgr.shape[:2]:
                depth = cv2.resize(
                    depth, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST
                )
            depth_pad = np.pad(depth, 3, mode="constant", constant_values=0.0)
            depth_win = np.lib.stride_tricks.sliding_window_view(depth_pad, (7, 7))

            for duzlem in tum_duzlemler:
                ref_doku = duzlem.get("ref_doku") if referans_kullan else None
                ref_gor = duzlem.get("ref_gorulen") if referans_kullan else None
                kare_izdusur(
                    duzlem,
                    bgr,
                    depth,
                    k["R_cw"],
                    k["t_cw"],
                    k["cam_pos"],
                    k["fx"],
                    k["fy"],
                    k["cx"],
                    k["cy"],
                    zemin_gecersiz_menzil=args.zemin_gecersiz_menzil,
                    keskinlik_carpani=k["keskinlik"],
                    referans_doku=ref_doku,
                    referans_gorulen=ref_gor,
                    depth_pencere_gorunumu=depth_win,
                )

    # 1. Geçiş
    gecis_calistir(referans_kullan=False)

    # 2. Geçiş: Pozlama dengeleme (1. geçişte enIyi doku referans alınır)
    if args.pozlama_dengele and len(kare_listesi) > 0:
        for duzlem in tum_duzlemler:
            ref_doku, ref_gorulen, _ = ham_doku_hesapla(duzlem, birlestir="enIyi")
            duzlem["ref_doku"] = ref_doku
            duzlem["ref_gorulen"] = ref_gorulen
            duzlem_tampon_sifirla(duzlem)
        gecis_calistir(referans_kullan=True)

    islenen_kare = len(kare_listesi)

    # Duvar dokularını, maskelerini ve malzeme betiğini kaydet
    malzeme_bloklari = []
    duvar_ozetleri = []

    for duz in duvar_duzlemleri:
        i = duz["i"]
        doku_bgr, maske, oran = doku_tamamla(
            duz,
            gauss_yumusat=True,
            birlestir=args.birlestir,
            zemin_doldur=args.zemin_doldur,
            yama_boyut=args.yama_boyut,
        )
        doku_adi = f"duvar_{i}.png"
        maske_adi = f"duvar_{i}_gorulen.png"
        malzeme_adi = f"Oda/duvar_{i}"

        doku_yolu = doku_dizin / doku_adi
        cv2.imwrite(str(doku_yolu), doku_bgr)
        cv2.imwrite(str(doku_dizin / maske_adi), maske)
        shutil.copy2(doku_yolu, textures_dizin / doku_adi)

        malzeme_bloklari.append(ogre_malzeme_metni(malzeme_adi, doku_adi))
        duvar_ozetleri.append(
            {
                "i": int(i),
                "p1": duz["p1"],
                "p2": duz["p2"],
                "yuz_normali": duz["yuz_normali"],
                "doku": f"doku/{doku_adi}",
                "malzeme": malzeme_adi,
                "gorulen_oran": oran,
                "kazanc_istatistikleri": yuzey_kazanc_istatistikleri(duz),
                "reddedilen_piksel_orani": yuzey_red_orani(duz),
            }
        )

    # Zemin dokusunu ve maskesini kaydet
    zemin_bgr, zemin_maske, zemin_oran = doku_tamamla(
        zemin_duzlemi,
        gauss_yumusat=False,
        birlestir=args.birlestir,
        zemin_doldur=args.zemin_doldur,
        yama_boyut=args.yama_boyut,
    )
    zemin_doku_yolu = doku_dizin / "zemin.png"
    cv2.imwrite(str(zemin_doku_yolu), zemin_bgr)
    cv2.imwrite(str(doku_dizin / "zemin_gorulen.png"), zemin_maske)
    shutil.copy2(zemin_doku_yolu, textures_dizin / "zemin.png")
    malzeme_bloklari.append(ogre_malzeme_metni("Oda/zemin", "zemin.png"))

    (scripts_dizin / "oda.material").write_text("\n".join(malzeme_bloklari), encoding="utf-8")

    maskeli_oran = (
        float(np.mean(zemin_duzlemi["mobilya_alti"]))
        if "mobilya_alti" in zemin_duzlemi and zemin_duzlemi["mobilya_alti"] is not None
        else 0.0
    )

    doku_yaml = {
        "duvarlar": duvar_ozetleri,
        "zemin": {
            "kutu": zemin_duzlemi["kutu"],
            "doku": "doku/zemin.png",
            "malzeme": "Oda/zemin",
            "gorulen_oran": zemin_oran,
            "maskeli_oran": round(maskeli_oran, 4),
            "kazanc_istatistikleri": yuzey_kazanc_istatistikleri(zemin_duzlemi),
            "reddedilen_piksel_orani": yuzey_red_orani(zemin_duzlemi),
        },
        "tavan": round(float(args.tavan), 4),
        "cozunurluk": round(float(args.cozunurluk), 4),
    }
    with (cikti / "doku.yaml").open("w", encoding="utf-8") as f:
        yaml.dump(doku_yaml, f, sort_keys=False, allow_unicode=True)

    print(
        f"[DOKU] Tamamlandı: {islenen_kare} kare, {len(duvar_ozetleri)} duvar, "
        f"zemin görülen oran={zemin_oran:.4f}, maskeli oran={maskeli_oran:.4f}, "
        f"zemin red oranı={yuzey_red_orani(zemin_duzlemi):.4f} -> {cikti}"
    )


if __name__ == "__main__":
    main()
