#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
oda_dunya.py - Duvar, zemin, tavan ve mobilya verilerinden Gazebo Classic SDF dünyası üretici.

Kaynak dosyalar:
  - Yapı YAML (duvarlar, zemin sınırı, mobilya bilgileri)
  - Mesh dizini (oda_mesh.obj, oda_mesh.mtl, oda_mesh.json)
Not: Tavan yüksekliği varsayımdır (varsayılan: 2.7 m).
"""

import argparse
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml


def nokta_al(p):
    """Noktayı [x, y] biçimine dönüştürür."""
    if isinstance(p, dict):
        return float(p.get("x", 0.0)), float(p.get("y", 0.0))
    return float(p[0]), float(p[1])


def sinirlar_coz(sinirlar):
    """Mesh sınırlayıcı kutusunu (min_x, min_y, max_x, max_y) olarak ayrıştırır."""
    if not sinirlar:
        return None
    if isinstance(sinirlar, dict):
        if "min" in sinirlar and "max" in sinirlar:
            return (
                float(sinirlar["min"][0]),
                float(sinirlar["min"][1]),
                float(sinirlar["max"][0]),
                float(sinirlar["max"][1]),
            )
        if "min_x" in sinirlar:
            return (
                float(sinirlar["min_x"]),
                float(sinirlar["min_y"]),
                float(sinirlar["max_x"]),
                float(sinirlar["max_y"]),
            )
    elif isinstance(sinirlar, (list, tuple)):
        if len(sinirlar) == 2 and isinstance(sinirlar[0], (list, tuple)):
            return (
                float(sinirlar[0][0]),
                float(sinirlar[0][1]),
                float(sinirlar[1][0]),
                float(sinirlar[1][1]),
            )
        elif len(sinirlar) == 4:
            return (
                float(sinirlar[0]),
                float(sinirlar[1]),
                float(sinirlar[2]),
                float(sinirlar[3]),
            )
        elif len(sinirlar) == 6:
            return (
                float(sinirlar[0]),
                float(sinirlar[1]),
                float(sinirlar[3]),
                float(sinirlar[4]),
            )
    return None


def nokta_sinirda_mi(pt, bbox):
    """Noktanın sınırlayıcı kutu içinde olup olmadığını kontrol eder."""
    if bbox is None:
        return False
    min_x, min_y, max_x, max_y = bbox
    return (min_x <= pt[0] <= max_x) and (min_y <= pt[1] <= max_y)


def nokta_cokgende_mi(pt, cokgen):
    """
    Noktanın (x, y) 2B çokgenin İÇİNDE olup olmadığını test eder.
    Kenarlar ve dışı False döner.
    """
    if not cokgen or len(cokgen) < 3:
        return False
    x, y = float(pt[0]), float(pt[1])
    n = len(cokgen)

    # Kenar üzerinde olma kontrolü (kenar üzerindeyse içeride sayılmaz)
    tol = 1e-6
    for i in range(n):
        p1 = cokgen[i]
        p2 = cokgen[(i + 1) % n]
        x1, y1 = float(p1[0]), float(p1[1])
        x2, y2 = float(p2[0]), float(p2[1])

        cp = (y - y1) * (x2 - x1) - (x - x1) * (y2 - y1)
        if abs(cp) <= tol:
            if (min(x1, x2) - tol <= x <= max(x1, x2) + tol) and (
                min(y1, y2) - tol <= y <= max(y1, y2) + tol
            ):
                return False

    # Ray casting algoritması
    iceride = False
    p1x, p1y = float(cokgen[0][0]), float(cokgen[0][1])
    for i in range(1, n + 1):
        p2 = cokgen[i % n]
        p2x, p2y = float(p2[0]), float(p2[1])
        if min(p1y, p2y) < y <= max(p1y, p2y):
            if p1y != p2y:
                x_kesim = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                if x < x_kesim:
                    iceride = not iceride
        p1x, p1y = p2x, p2y

    return iceride


def en_kucuk_dikdortgen(cokgen):
    """
    Çokgenin kenar yönlerinde döndürerek en küçük alanlı dönük dikdörtgenini hesaplar.
    Dönüş: (merkez_x, merkez_y, uzunluk_x, genislik_y, yaw_rad)
    """
    pts = np.asarray(cokgen, dtype=float)[:, :2]
    n = len(pts)
    if n == 0:
        return 0.0, 0.0, 0.5, 0.5, 0.0
    if n == 1:
        return float(pts[0, 0]), float(pts[0, 1]), 0.5, 0.5, 0.0
    if n == 2:
        dx = pts[1, 0] - pts[0, 0]
        dy = pts[1, 1] - pts[0, 1]
        uzunluk = max(float(math.hypot(dx, dy)), 0.05)
        yaw = float(math.atan2(dy, dx))
        cx = float((pts[0, 0] + pts[1, 0]) / 2.0)
        cy = float((pts[0, 1] + pts[1, 1]) / 2.0)
        return cx, cy, uzunluk, 0.1, yaw

    en_kucuk_alan = float("inf")
    en_iyi = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])), 0.5, 0.5, 0.0)

    for i in range(n):
        p1 = pts[i]
        p2 = pts[(i + 1) % n]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        kenar = math.hypot(dx, dy)
        if kenar < 1e-6:
            continue
        theta = math.atan2(dy, dx)
        c = math.cos(-theta)
        s = math.sin(-theta)
        R = np.array([[c, -s], [s, c]])
        pts_rot = pts @ R.T
        min_xy = np.min(pts_rot, axis=0)
        max_xy = np.max(pts_rot, axis=0)
        w = max_xy[0] - min_xy[0]
        h = max_xy[1] - min_xy[1]
        alan = w * h
        if alan < en_kucuk_alan:
            en_kucuk_alan = alan
            rot_cx = (min_xy[0] + max_xy[0]) / 2.0
            rot_cy = (min_xy[1] + max_xy[1]) / 2.0
            c_inv = math.cos(theta)
            s_inv = math.sin(theta)
            cx = rot_cx * c_inv - rot_cy * s_inv
            cy = rot_cx * s_inv + rot_cy * c_inv
            en_iyi = (float(cx), float(cy), max(float(w), 0.05), max(float(h), 0.05), float(theta))

    return en_iyi


def ucgenler_duvara_yapisik(ucgenler, duvarlar, pay=0.08):
    """
    Vektörel sürüm: N adet üçgenin her birinin duvara yapışık olup olmadığını test eder.

    Bir üçgenin tüm köşeleri herhangi bir duvar parçasına (p1, p2, kalinlik)
    yatay düzlemde (x,y) dik uzaklığı <= kalinlik/2 + pay olan ve
    parçanın uzunluğu boyunca (izdüşüm t in [-0.05 m, L+0.05 m]) kalan
    üçgenler duvara yapışık kabul edilir.

    Parametreler:
        ucgenler: (N, 3, 3) veya (N, 3, 2) boyutunda üçgen köşeleri dizisi.
        duvarlar: Yapı YAML 'duvarlar' listesi (p1, p2, kalinlik sözlükleri).
        pay: Duvar kalınlığına eklenecek dik uzaklık toleransı (m, varsayılan 0.08).

    Dönüş:
        (N,) boyutunda bool NumPy dizisi (True = duvara yapışık).
    """
    pts = np.asarray(ucgenler, dtype=float)
    if pts.size == 0 or not duvarlar:
        return np.zeros(len(pts), dtype=bool)

    if pts.ndim == 2:
        pts = pts[np.newaxis, ...]

    pts_xy = pts[..., :2]  # (N, 3, 2)
    N = pts_xy.shape[0]
    genel_maske = np.zeros(N, dtype=bool)

    for d in duvarlar:
        if isinstance(d, dict):
            p1 = np.array(nokta_al(d.get("p1", [0.0, 0.0])), dtype=float)
            p2 = np.array(nokta_al(d.get("p2", [0.0, 0.0])), dtype=float)
            kalinlik = float(d.get("kalinlik", 0.15))
        elif isinstance(d, (list, tuple)):
            p1 = np.array(nokta_al(d[0]), dtype=float)
            p2 = np.array(nokta_al(d[1]), dtype=float)
            kalinlik = float(d[2]) if len(d) > 2 else 0.15
        else:
            continue

        v = p2 - p1
        L = float(np.hypot(v[0], v[1]))
        if L < 1e-9:
            continue

        esik_d = kalinlik / 2.0 + float(pay)
        w = pts_xy - p1  # (N, 3, 2)

        # t izdüşümü: (w_x * dx + w_y * dy) / L
        t = (w[..., 0] * v[0] + w[..., 1] * v[1]) / L  # (N, 3)

        # dik uzaklık: |-(w_x * dy) + (w_y * dx)| / L
        d_perp = np.abs(-w[..., 0] * v[1] + w[..., 1] * v[0]) / L  # (N, 3)

        koseler_uygun = (t >= -0.05) & (t <= L + 0.05) & (d_perp <= esik_d)
        ucgen_uygun = np.all(koseler_uygun, axis=-1)  # (N,)

        genel_maske |= ucgen_uygun

    return genel_maske


def ucgen_duvara_yapisik(ucgen_koseleri, duvarlar, pay=0.08):
    """
    Tek bir üçgenin tüm köşelerinin herhangi bir duvar parçasına yapışık olup olmadığını test eder.

    Parametreler:
        ucgen_koseleri: (3, 3) veya (3, 2) boyutunda üçgen köşeleri.
        duvarlar: Yapı YAML 'duvarlar' listesi.
        pay: Dik uzaklık toleransı (m, varsayılan 0.08).

    Dönüş:
        bool: Duvara yapışık ise True, aksi halde False.
    """
    pts = np.asarray(ucgen_koseleri, dtype=float)
    if pts.ndim == 3:
        return ucgenler_duvara_yapisik(pts, duvarlar, pay)
    sonuc = ucgenler_duvara_yapisik(pts, duvarlar, pay)
    return bool(sonuc[0])


ucgen_duvara_yapisik_vektorel = ucgenler_duvara_yapisik


def mesh_duvar_yuzeylerini_kes(giris_obj_yolu, cikis_obj_yolu, duvarlar, pay=0.08):
    """
    Görsel mesh OBJ dosyasındaki duvar yüzeylerine ait üçgenleri süzerek
    cikis_obj_yolu olarak kaydeder.

    Tüm köşeleri herhangi bir duvar parçasına dik uzaklığı <= kalinlik/2 + pay
    olan ve izdüşümü [-0.05, L + 0.05] aralığında kalan üçgenler atılır.
    v/vn/vt satırları aynen korunur; sadece f satırları süzülür.
    Atılan üçgen sayısı ve oranını terminale yazar.
    """
    giris_abs = os.path.abspath(giris_obj_yolu)
    cikis_abs = os.path.abspath(cikis_obj_yolu)

    if not os.path.isfile(giris_abs):
        sys.stderr.write(f"Uyarı: Giriş mesh dosyası bulunamadı: '{giris_abs}'\n")
        return

    with open(giris_abs, "r", encoding="utf-8") as f:
        satirlar = f.readlines()

    koseler = []
    yuz_satir_indisleri = []
    yuz_kose_indisleri = []

    for idx, satir in enumerate(satirlar):
        satir_strip = satir.strip()
        parcalar = satir_strip.split()
        if len(parcalar) >= 4 and parcalar[0] == "v":
            koseler.append([float(parcalar[1]), float(parcalar[2]), float(parcalar[3])])
        elif len(parcalar) >= 4 and parcalar[0] == "f":
            yuz_satir_indisleri.append(idx)
            v_ids = []
            for p in parcalar[1:4]:
                v_str = p.split("/")[0]
                v_idx = int(v_str)
                if v_idx > 0:
                    v_ids.append(v_idx - 1)
                else:
                    v_ids.append(len(koseler) + v_idx)
            yuz_kose_indisleri.append(v_ids)

    koseler_arr = np.asarray(koseler, dtype=float)
    toplam_ucgen = len(yuz_kose_indisleri)

    if toplam_ucgen == 0 or len(koseler_arr) == 0:
        print("Mesh duvar kesme: 0/0 (%0.0) üçgen atıldı.")
        cikis_dizin = os.path.dirname(cikis_abs)
        if cikis_dizin:
            os.makedirs(cikis_dizin, exist_ok=True)
        with open(cikis_abs, "w", encoding="utf-8") as f_out:
            f_out.writelines(satirlar)
        return

    yuzler_arr = np.asarray(yuz_kose_indisleri, dtype=int)
    ucgenler_3b = koseler_arr[yuzler_arr]  # (toplam_ucgen, 3, 3)

    maske = ucgenler_duvara_yapisik(ucgenler_3b, duvarlar, pay=pay)
    atilan_sayisi = int(np.sum(maske))
    oran = (atilan_sayisi / toplam_ucgen * 100.0) if toplam_ucgen > 0 else 0.0

    print(
        f"Mesh duvar kesme: {atilan_sayisi}/{toplam_ucgen} (%{oran:.1f}) üçgen atıldı "
        f"(kalan: {toplam_ucgen - atilan_sayisi})."
    )

    atlanacak_satirlar = {
        yuz_satir_indisleri[i] for i in range(toplam_ucgen) if maske[i]
    }

    cikis_dizin = os.path.dirname(cikis_abs)
    if cikis_dizin:
        os.makedirs(cikis_dizin, exist_ok=True)

    with open(cikis_abs, "w", encoding="utf-8") as f_out:
        for idx, satir in enumerate(satirlar):
            if idx in atlanacak_satirlar:
                continue
            f_out.write(satir)


def mesh_sadelestir(giris_yol, cikis_yol, hedef_ucgen=20000):
    """
    Open3D ile simplify_quadric_decimation uygulayarak düz OBJ (malzemesiz) yazar.
    """
    try:
        import open3d as o3d
    except ImportError as e:
        sys.stderr.write(f"Hata: open3d modülü bulunamadı ({e}). Lütfen open3d yükleyin.\n")
        sys.exit(1)

    giris_abs = os.path.abspath(giris_yol)
    cikis_abs = os.path.abspath(cikis_yol)

    if not os.path.isfile(giris_abs):
        sys.stderr.write(f"Hata: Giriş mesh dosyası bulunamadı: '{giris_abs}'\n")
        sys.exit(1)

    mesh = o3d.io.read_triangle_mesh(giris_abs)
    if not mesh.has_triangles():
        sys.stderr.write(f"Hata: Mesh üçgen içermiyor veya okunamadı: '{giris_abs}'\n")
        sys.exit(1)

    hedef_sayi = max(1, int(hedef_ucgen))
    sade_mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=hedef_sayi)
    sade_mesh.remove_degenerate_triangles()
    sade_mesh.remove_duplicated_triangles()
    sade_mesh.remove_duplicated_vertices()
    sade_mesh.remove_non_manifold_edges()

    cikis_dizin = os.path.dirname(cikis_abs)
    if cikis_dizin:
        os.makedirs(cikis_dizin, exist_ok=True)

    vertices = np.asarray(sade_mesh.vertices)
    triangles = np.asarray(sade_mesh.triangles)

    with open(cikis_abs, "w", encoding="utf-8") as f:
        f.write(f"# Duz OBJ (ucgen: {len(triangles)}, tepe: {len(vertices)})\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for tri in triangles:
            f.write(f"f {tri[0] + 1} {tri[1] + 1} {tri[2] + 1}\n")

    print(
        f"Mesh başarıyla sadeleştirildi: {giris_abs} -> {cikis_abs} "
        f"({len(mesh.triangles)} -> {len(triangles)} üçgen)"
    )


def ogre_isiksiz_malzeme_metni(malzeme_adi, doku_dosya):
    """Tek bir doku için ışıksız (lighting off) OGRE malzeme tanımı döndürür."""
    return (
        f"material {malzeme_adi}\n{{\n  technique\n  {{\n    pass\n    {{\n"
        f"      lighting off\n"
        f"      texture_unit\n      {{\n        texture {doku_dosya}\n"
        f"        filtering trilinear\n      }}\n    }}\n  }}\n}}\n"
    )


def mtl_isiksiz_olustur(giris_mtl_yolu, cikis_mtl_yolu):
    """
    MTL dosyasının ışıksız (unlit) eşdeğerini yazar:
    Her malzemede Ka = Kd = orijinal Kd ve Ks = 0 0 0 yapılır.
    Dönüş: dönüştürülen malzeme sayısı.
    """
    if not os.path.isfile(giris_mtl_yolu):
        return 0

    with open(giris_mtl_yolu, "r", encoding="utf-8") as f:
        satirlar = f.readlines()

    kd_haritasi = {}
    aktif_mtl = None
    for satir in satirlar:
        parcalar = satir.strip().split()
        if len(parcalar) >= 2 and parcalar[0] == "newmtl":
            aktif_mtl = parcalar[1]
        elif aktif_mtl is not None and len(parcalar) >= 4 and parcalar[0] == "Kd":
            kd_haritasi[aktif_mtl] = " ".join(parcalar[1:4])

    cikis_satirlar = []
    aktif_mtl = None
    ka_yazildi = False

    for satir in satirlar:
        parcalar = satir.strip().split()
        if len(parcalar) >= 2 and parcalar[0] == "newmtl":
            aktif_mtl = parcalar[1]
            ka_yazildi = False
            cikis_satirlar.append(satir)
        elif aktif_mtl is not None and len(parcalar) >= 4 and parcalar[0] == "Ka":
            kd_deger = kd_haritasi.get(aktif_mtl, " ".join(parcalar[1:4]))
            cikis_satirlar.append(f"Ka {kd_deger}\n")
            ka_yazildi = True
        elif aktif_mtl is not None and len(parcalar) >= 4 and parcalar[0] == "Kd":
            kd_deger = kd_haritasi.get(aktif_mtl, " ".join(parcalar[1:4]))
            if not ka_yazildi:
                cikis_satirlar.append(f"Ka {kd_deger}\n")
                ka_yazildi = True
            cikis_satirlar.append(f"Kd {kd_deger}\n")
        elif aktif_mtl is not None and len(parcalar) >= 4 and parcalar[0] == "Ks":
            cikis_satirlar.append("Ks 0.0000 0.0000 0.0000\n")
        else:
            cikis_satirlar.append(satir)

    cikis_dizin = os.path.dirname(os.path.abspath(cikis_mtl_yolu))
    if cikis_dizin:
        os.makedirs(cikis_dizin, exist_ok=True)
    with open(cikis_mtl_yolu, "w", encoding="utf-8") as f:
        f.writelines(cikis_satirlar)

    return len(kd_haritasi)


def obj_mtllib_degistir(obj_yolu, yeni_mtl_adi):
    """OBJ dosyasındaki mtllib satırını yeni_mtl_adi ile değiştirir."""
    if not os.path.isfile(obj_yolu):
        return
    with open(obj_yolu, "r", encoding="utf-8") as f:
        satirlar = f.readlines()

    degisti = False
    yeni_satirlar = []
    for satir in satirlar:
        if satir.lstrip().startswith("mtllib "):
            yeni_satirlar.append(f"mtllib {yeni_mtl_adi}\n")
            degisti = True
        else:
            yeni_satirlar.append(satir)

    if not degisti:
        yeni_satirlar.insert(0, f"mtllib {yeni_mtl_adi}\n")

    with open(obj_yolu, "w", encoding="utf-8") as f:
        f.writelines(yeni_satirlar)


def doku_png_bul(doku_dizin_abs, aday_bagil_yollar):
    """Doku dizini altında aday bağıl yollardan var olan ilk PNG dosyasını döndürür."""
    if not doku_dizin_abs:
        return None
    for bagil in aday_bagil_yollar:
        if not bagil:
            continue
        tam_yol = os.path.join(doku_dizin_abs, bagil)
        if os.path.isfile(tam_yol):
            return tam_yol
    return None


def _renk_uclu_coz(deger):
    """[r, g, b] listesini veya sözlüğünü 0..1 aralığında (r, g, b) üçlüsüne çevirir."""
    if isinstance(deger, dict):
        for anahtar in ("renk", "rgb", "color"):
            if anahtar in deger:
                return _renk_uclu_coz(deger[anahtar])
        if all(k in deger for k in ("r", "g", "b")):
            vals = [float(deger["r"]), float(deger["g"]), float(deger["b"])]
            olcek = 255.0 if max(vals) > 1.0 else 1.0
            return (vals[0] / olcek, vals[1] / olcek, vals[2] / olcek)
        return None
    if isinstance(deger, (list, tuple)) and len(deger) >= 3:
        try:
            vals = [float(deger[0]), float(deger[1]), float(deger[2])]
            olcek = 255.0 if max(vals) > 1.0 else 1.0
            return (
                min(max(vals[0] / olcek, 0.0), 1.0),
                min(max(vals[1] / olcek, 0.0), 1.0),
                min(max(vals[2] / olcek, 0.0), 1.0),
            )
        except (ValueError, TypeError):
            return None
    return None


def tavan_rengi_belirle(doku_veri, doku_dizin_abs, varsayilan_renk):
    """
    Tavan rengini (r, g, b) 0..1 olarak belirler:
    1. doku.yaml içinde tavan rengi varsa onu kullanır.
    2. Yoksa duvar doku PNG'lerinin üst şeridinden (z ~ tavan) medyan rengi hesaplar.
    3. Hiçbiri yoksa varsayilan_renk döndürür.
    """
    if isinstance(doku_veri, dict):
        for anahtar in ("tavan", "tavan_rengi"):
            if anahtar in doku_veri:
                cozulen = _renk_uclu_coz(doku_veri[anahtar])
                if cozulen is not None:
                    return cozulen

    if doku_dizin_abs and isinstance(doku_veri, dict):
        try:
            import cv2

            ust_ornekler = []
            doku_duvarlar = doku_veri.get("duvarlar") or []
            duvar_iter = (
                doku_duvarlar
                if isinstance(doku_duvarlar, list)
                else list(doku_duvarlar.values())
            )
            for idx, doku_d in enumerate(duvar_iter):
                bagil = (
                    doku_d.get("doku", f"doku/duvar_{idx}.png")
                    if isinstance(doku_d, dict)
                    else f"doku/duvar_{idx}.png"
                )
                png_adi = os.path.basename(bagil) or f"duvar_{idx}.png"
                png_yolu = doku_png_bul(
                    doku_dizin_abs,
                    [os.path.join("materials", "textures", png_adi), bagil, os.path.join("doku", png_adi)],
                )
                if png_yolu:
                    bgr = cv2.imread(png_yolu, cv2.IMREAD_COLOR)
                    if bgr is not None and bgr.shape[0] > 0 and bgr.shape[1] > 0:
                        ust_h = max(1, bgr.shape[0] // 10)
                        ust_dilim = bgr[:ust_h, :, :].reshape(-1, 3).astype(float) / 255.0
                        ust_ornekler.append(np.median(ust_dilim, axis=0))
            if ust_ornekler:
                med_bgr = np.median(np.vstack(ust_ornekler), axis=0)
                if 0.05 <= float(np.mean(med_bgr)) <= 0.98:
                    return (float(med_bgr[2]), float(med_bgr[1]), float(med_bgr[0]))
        except Exception:
            pass

    return varsayilan_renk


def dunya_uret(
    yapi_yol,
    mesh_dizin,
    cikti_yol,
    tavan=2.7,
    mobilya_yuk=0.6,
    isik_aralik=2.0,
    doku_dizin=None,
    carpisma_mesh=None,
    mesh_duvar_kes=0.08,
    isikli=False,
):
    """Girdi yapı ve mesh verilerini okuyarak Gazebo SDF dünyası üretir."""
    # 1. Yapı dosyasını oku
    with open(yapi_yol, "r", encoding="utf-8") as f:
        yapi = yaml.safe_load(f) or {}

    oda_siniri = [nokta_al(p) for p in yapi.get("oda_siniri", [])]
    duvarlar = yapi.get("duvarlar", [])
    mobilyalar = yapi.get("mobilya", [])

    # 2. Doku YAML dosyasını oku (isteğe bağlı)
    doku_veri = None
    doku_dizin_abs = None
    if doku_dizin:
        doku_dizin_abs = os.path.abspath(doku_dizin)
        doku_yaml_yolu = os.path.join(doku_dizin_abs, "doku.yaml")
        if os.path.isfile(doku_yaml_yolu):
            try:
                with open(doku_yaml_yolu, "r", encoding="utf-8") as f:
                    doku_veri = yaml.safe_load(f) or {}
            except Exception as e:
                sys.stderr.write(f"Uyarı: Doku YAML okunamadı ({e}), düz renk kullanılıyor.\n")
                doku_veri = None
        else:
            sys.stderr.write(
                f"Uyarı: Doku dosyası '{doku_yaml_yolu}' bulunamadı, düz renk kullanılıyor.\n"
            )
            doku_veri = None

    # 3. Mesh dosyalarını kontrol et ve meta veriyi oku
    mesh_dizin_abs = os.path.abspath(mesh_dizin)
    os.makedirs(mesh_dizin_abs, exist_ok=True)
    obj_yolu = os.path.join(mesh_dizin_abs, "oda_mesh.obj")
    json_yolu = os.path.join(mesh_dizin_abs, "oda_mesh.json")

    mesh_var = os.path.isfile(obj_yolu)
    if not mesh_var:
        sys.stderr.write(f"Uyarı: Mesh dosyası '{obj_yolu}' bulunamadı, mobilya_mesh atlanıyor.\n")

    mesh_meta = {}
    if os.path.isfile(json_yolu):
        try:
            with open(json_yolu, "r", encoding="utf-8") as f:
                mesh_meta = json.load(f) or {}
        except Exception as e:
            sys.stderr.write(f"Uyarı: Mesh JSON okunamadı ({e}).\n")

    mesh_sinirlar = sinirlar_coz(mesh_meta.get("sinirlar"))

    zemin_renk = mesh_meta.get("zemin_rengi_tahmini", [0.75, 0.75, 0.75])
    zr, zg, zb = float(zemin_renk[0]), float(zemin_renk[1]), float(zemin_renk[2])

    duvar_renk = mesh_meta.get("duvar_rengi_tahmini", [0.85, 0.85, 0.85])
    dr, dg, db = float(duvar_renk[0]), float(duvar_renk[1]), float(duvar_renk[2])

    # 4. Zemin ve tavan sınırlarını hesapla
    if oda_siniri:
        sinir_pts = np.array(oda_siniri)
        min_x, min_y = np.min(sinir_pts, axis=0)
        max_x, max_y = np.max(sinir_pts, axis=0)
    elif duvarlar:
        tum_noktalar = []
        for d in duvarlar:
            tum_noktalar.append(nokta_al(d.get("p1", [0, 0])))
            tum_noktalar.append(nokta_al(d.get("p2", [0, 0])))
        sinir_pts = np.array(tum_noktalar)
        min_x, min_y = np.min(sinir_pts, axis=0)
        max_x, max_y = np.max(sinir_pts, axis=0)
    else:
        min_x, min_y, max_x, max_y = -5.0, -5.0, 5.0, 5.0

    # 0.5 m pay ekle
    zemin_boy_x = (max_x - min_x) + 1.0
    zemin_boy_y = (max_y - min_y) + 1.0
    zemin_merkez_x = (min_x + max_x) / 2.0
    zemin_merkez_y = (min_y + max_y) / 2.0

    # 5. XML / SDF bloklarını oluştur
    modeller_xml = []
    isiksiz_malzeme_bloklari = []
    duvar_obj_sayisi = 0
    isiksiz_mtl_malzeme_sayisi = 0

    # Zemin modeli
    zemin_doku_visual_xml = ""
    if doku_veri is not None and "zemin" in doku_veri:
        doku_zemin = doku_veri.get("zemin") or {}
        zemin_kutu = doku_zemin.get("kutu") if isinstance(doku_zemin, dict) else None
        zemin_malzeme = (
            doku_zemin.get("malzeme", "Oda/zemin") if isinstance(doku_zemin, dict) else "Oda/zemin"
        )
        zemin_doku_bagil = (
            doku_zemin.get("doku", "doku/zemin.png")
            if isinstance(doku_zemin, dict)
            else "doku/zemin.png"
        )
        zemin_png_adi = os.path.basename(zemin_doku_bagil) or "zemin.png"

        doku_zemin_bx = zemin_boy_x
        doku_zemin_by = zemin_boy_y
        doku_zemin_dx = 0.0
        doku_zemin_dy = 0.0

        if isinstance(zemin_kutu, (list, tuple)) and len(zemin_kutu) == 4:
            # oda_doku.py: kutu = [xmin, ymin, xmax, ymax] (harita çerçevesi); doku link merkezine göre konumlanır
            xmin, ymin, xmax, ymax = (float(v) for v in zemin_kutu)
            doku_zemin_bx, doku_zemin_by = xmax - xmin, ymax - ymin
            doku_zemin_dx = (xmin + xmax) / 2.0 - zemin_merkez_x
            doku_zemin_dy = (ymin + ymax) / 2.0 - zemin_merkez_y
        elif isinstance(zemin_kutu, (list, tuple)):
            if len(zemin_kutu) >= 2:
                doku_zemin_bx = float(zemin_kutu[0])
                doku_zemin_by = float(zemin_kutu[1])
        elif isinstance(zemin_kutu, dict):
            doku_zemin_bx = float(zemin_kutu.get("boy_x", zemin_kutu.get("x", zemin_boy_x)))
            doku_zemin_by = float(zemin_kutu.get("boy_y", zemin_kutu.get("y", zemin_boy_y)))
            doku_zemin_dx = float(zemin_kutu.get("dx", 0.0))
            doku_zemin_dy = float(zemin_kutu.get("dy", 0.0))

        zemin_png = doku_png_bul(
            doku_dizin_abs,
            [
                os.path.join("materials", "textures", zemin_png_adi),
                zemin_doku_bagil,
                os.path.join("doku", zemin_png_adi),
            ],
        )
        zemin_aktif_malzeme = "Oda/isiksiz/zemin" if not isikli else zemin_malzeme
        if not isikli:
            isiksiz_malzeme_bloklari.append(
                ogre_isiksiz_malzeme_metni("Oda/isiksiz/zemin", zemin_png_adi)
            )

        if isinstance(zemin_kutu, (list, tuple)) and len(zemin_kutu) == 4:
            # Kutu üst yüzünün Gazebo UV eşlemesi belirsiz (simde doku dönük/ölçeksiz çıktı, 27.10 denemesi):
            # zemin, köşe başına açık UV'li OBJ dörtgeni. Doku satır 0 = ymax (oda_doku.py) = OBJ v=1.
            if zemin_png:
                hedef_zemin_png = os.path.join(mesh_dizin_abs, zemin_png_adi)
                if os.path.abspath(zemin_png) != os.path.abspath(hedef_zemin_png):
                    shutil.copy(zemin_png, hedef_zemin_png)
            with open(os.path.join(mesh_dizin_abs, "zemin_doku.mtl"), "w", encoding="utf-8") as f:
                f.write(f"newmtl zemin\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd {zemin_png_adi}\n")
            with open(os.path.join(mesh_dizin_abs, "zemin_doku.obj"), "w", encoding="utf-8") as f:
                f.write("mtllib zemin_doku.mtl\no zemin\n")
                for x, y in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)):
                    f.write(f"v {x:.4f} {y:.4f} 0.0\n")
                f.write(
                    "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\nusemtl zemin\n"
                    "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n"
                )
            zemin_doku_visual_xml = f"""
        <!-- Doku: dünya koordinatlı OBJ dörtgeni; link (merkez, z=-0.01) telafisiyle dünyada z=0.001 -->
        <visual name="visual_doku">
          <pose>{-zemin_merkez_x:.4f} {-zemin_merkez_y:.4f} 0.0110 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>file://{mesh_dizin_abs}/zemin_doku.obj</uri>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>file://{doku_dizin_abs}/materials/scripts</uri>
              <uri>file://{doku_dizin_abs}/materials/textures</uri>
              <name>{zemin_aktif_malzeme}</name>
            </script>
          </material>
        </visual>"""
        else:
            zemin_doku_visual_xml = f"""
        <!-- Doku katmanı: link z=-0.01 olduğundan +0.011 -> dünyada z=0.001 (zemin kutusunun üstü 0.0) -->
        <visual name="visual_doku">
          <pose>{doku_zemin_dx:.4f} {doku_zemin_dy:.4f} 0.0110 0 0 0</pose>
          <geometry>
            <box>
              <size>{doku_zemin_bx:.4f} {doku_zemin_by:.4f} 0.0020</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://{doku_dizin_abs}/materials/scripts</uri>
              <uri>file://{doku_dizin_abs}/materials/textures</uri>
              <name>{zemin_aktif_malzeme}</name>
            </script>
          </material>
        </visual>"""

    zemin_amb = (zr, zg, zb) if not isikli else (0.6 * zr, 0.6 * zg, 0.6 * zb)
    zemin_xml = f"""    <!-- Zemin Modeli -->
    <model name="zemin">
      <static>true</static>
      <link name="zemin_link">
        <pose>{zemin_merkez_x:.4f} {zemin_merkez_y:.4f} -0.01 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{zemin_boy_x:.4f} {zemin_boy_y:.4f} 0.02</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{zemin_boy_x:.4f} {zemin_boy_y:.4f} 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>{zemin_amb[0]:.4f} {zemin_amb[1]:.4f} {zemin_amb[2]:.4f} 1.0</ambient>
            <diffuse>{zr:.4f} {zg:.4f} {zb:.4f} 1.0</diffuse>
          </material>
        </visual>{zemin_doku_visual_xml}
      </link>
    </model>"""
    modeller_xml.append(zemin_xml)

    # Tavan modeli
    varsayilan_tavan_renk = (
        (dr, dg, db)
        if "duvar_rengi_tahmini" in mesh_meta
        else ((0.62, 0.62, 0.62) if not isikli else (0.92, 0.92, 0.92))
    )
    tr, tg, tb = tavan_rengi_belirle(doku_veri, doku_dizin_abs, varsayilan_tavan_renk)
    tavan_amb = (tr, tg, tb) if not isikli else (0.6 * tr, 0.6 * tg, 0.6 * tb)
    tavan_z = tavan + 0.01
    tavan_xml = f"""    <!-- Tavan Modeli -->
    <model name="tavan">
      <static>true</static>
      <link name="tavan_link">
        <pose>{zemin_merkez_x:.4f} {zemin_merkez_y:.4f} {tavan_z:.4f} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{zemin_boy_x:.4f} {zemin_boy_y:.4f} 0.02</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{zemin_boy_x:.4f} {zemin_boy_y:.4f} 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>{tavan_amb[0]:.4f} {tavan_amb[1]:.4f} {tavan_amb[2]:.4f} 1.0</ambient>
            <diffuse>{tr:.4f} {tg:.4f} {tb:.4f} 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
    modeller_xml.append(tavan_xml)

    # Duvarlar modeli
    doku_duvarlar = doku_veri.get("duvarlar") if (doku_veri and isinstance(doku_veri, dict)) else None

    duvar_linkleri = []
    for i, d in enumerate(duvarlar):
        p1 = nokta_al(d.get("p1", [0, 0]))
        p2 = nokta_al(d.get("p2", [0, 0]))
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        mesafe = math.hypot(dx, dy)
        kalinlik = float(d.get("kalinlik", 0.15))
        uzunluk = mesafe + kalinlik
        genislik = kalinlik
        yukseklik = tavan
        mx = (p1[0] + p2[0]) / 2.0
        my = (p1[1] + p2[1]) / 2.0
        mz = tavan / 2.0
        yaw = math.atan2(dy, dx)

        doku_d = None
        if doku_duvarlar is not None:
            if isinstance(doku_duvarlar, list) and i < len(doku_duvarlar):
                doku_d = doku_duvarlar[i]
            elif isinstance(doku_duvarlar, dict):
                doku_d = (
                    doku_duvarlar.get(i)
                    or doku_duvarlar.get(str(i))
                    or doku_duvarlar.get(f"duvar_{i}")
                )

        if doku_d is not None:
            malzeme = (
                doku_d.get("malzeme", f"Oda/duvar_{i}")
                if isinstance(doku_d, dict)
                else f"Oda/duvar_{i}"
            )
            doku_bagil = (
                doku_d.get("doku", f"doku/duvar_{i}.png")
                if isinstance(doku_d, dict)
                else f"doku/duvar_{i}.png"
            )
            duvar_png_adi = os.path.basename(doku_bagil) or f"duvar_{i}.png"

            p1_d = np.array(
                nokta_al(doku_d.get("p1", p1) if isinstance(doku_d, dict) else p1),
                dtype=float,
            )
            p2_d = np.array(
                nokta_al(doku_d.get("p2", p2) if isinstance(doku_d, dict) else p2),
                dtype=float,
            )
            vec = p2_d - p1_d
            vec_boy = float(np.hypot(vec[0], vec[1]))
            doku_uzunluk = float(d.get("uzunluk", vec_boy))
            u = vec / vec_boy if vec_boy > 1e-9 else np.array([1.0, 0.0], dtype=float)

            yuz_normali = doku_d.get("yuz_normali") if isinstance(doku_d, dict) else None
            if yuz_normali is not None and isinstance(yuz_normali, (list, tuple, dict)):
                nx, ny = nokta_al(yuz_normali)
                n_boy = math.hypot(nx, ny)
                if n_boy > 1e-9:
                    n_ic = np.array([nx / n_boy, ny / n_boy], dtype=float)
                else:
                    n_ic = np.array([-u[1], u[0]], dtype=float)
            else:
                n_a = np.array([-u[1], u[0]], dtype=float)
                orta = 0.5 * (p1_d + p2_d)
                dot = (zemin_merkez_x - orta[0]) * n_a[0] + (zemin_merkez_y - orta[1]) * n_a[1]
                n_ic = n_a if dot >= 0.0 else -n_a

            p1_yuz = p1_d + 0.5 * kalinlik * n_ic
            p2_yuz = p1_yuz + doku_uzunluk * u
            p1_arka = p1_yuz - kalinlik * n_ic
            p2_arka = p2_yuz - kalinlik * n_ic

            # Ön yüz (1..4) ve arka yüz (5..8) köşeleri
            v1 = (p1_yuz[0], p1_yuz[1], 0.0)
            v2 = (p2_yuz[0], p2_yuz[1], 0.0)
            v3 = (p2_yuz[0], p2_yuz[1], float(tavan))
            v4 = (p1_yuz[0], p1_yuz[1], float(tavan))
            v5 = (p1_arka[0], p1_arka[1], 0.0)
            v6 = (p2_arka[0], p2_arka[1], 0.0)
            v7 = (p2_arka[0], p2_arka[1], float(tavan))
            v8 = (p1_arka[0], p1_arka[1], float(tavan))

            e1 = np.array([v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]], dtype=float)
            e2 = np.array([v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]], dtype=float)
            cross = np.cross(e1, e2)
            n_ic_3b = np.array([n_ic[0], n_ic[1], 0.0], dtype=float)
            if float(np.dot(cross, n_ic_3b)) >= 0.0:
                on_yuz_f = "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n"
                arka_yuz_f = "f 5/2/2 7/4/2 6/1/2\nf 5/2/2 8/3/2 7/4/2\n"
            else:
                on_yuz_f = "f 1/1/1 3/3/1 2/2/1\nf 1/1/1 4/4/1 3/3/1\n"
                arka_yuz_f = "f 5/2/2 6/1/2 7/4/2\nf 5/2/2 7/4/2 8/3/2\n"

            duvar_png_kaynak = doku_png_bul(
                doku_dizin_abs,
                [
                    os.path.join("materials", "textures", duvar_png_adi),
                    doku_bagil,
                    os.path.join("doku", duvar_png_adi),
                ],
            )
            if duvar_png_kaynak:
                hedef_png = os.path.join(mesh_dizin_abs, duvar_png_adi)
                if os.path.abspath(duvar_png_kaynak) != os.path.abspath(hedef_png):
                    shutil.copy(duvar_png_kaynak, hedef_png)

            mtl_dosya = f"duvar_doku_{i}.mtl"
            obj_dosya = f"duvar_doku_{i}.obj"
            with open(os.path.join(mesh_dizin_abs, mtl_dosya), "w", encoding="utf-8") as f:
                f.write(
                    f"newmtl duvar_{i}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd {duvar_png_adi}\n"
                )
            with open(os.path.join(mesh_dizin_abs, obj_dosya), "w", encoding="utf-8") as f:
                f.write(f"mtllib {mtl_dosya}\no duvar_{i}\n")
                for vx, vy, vz in (v1, v2, v3, v4, v5, v6, v7, v8):
                    f.write(f"v {vx:.4f} {vy:.4f} {vz:.4f}\n")
                f.write("vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n")
                f.write(f"vn {n_ic[0]:.4f} {n_ic[1]:.4f} 0.0000\n")
                f.write(f"vn {-n_ic[0]:.4f} {-n_ic[1]:.4f} 0.0000\n")
                f.write(f"usemtl duvar_{i}\n")
                f.write(on_yuz_f)
                f.write(arka_yuz_f)

            duvar_obj_sayisi += 1
            duvar_aktif_malzeme = f"Oda/isiksiz/duvar_{i}" if not isikli else malzeme
            if not isikli:
                isiksiz_malzeme_bloklari.append(
                    ogre_isiksiz_malzeme_metni(f"Oda/isiksiz/duvar_{i}", duvar_png_adi)
                )

            # Link pozunu (mx, my, mz, 0, 0, yaw) telafi ederek dünya koordinatlı OBJ'yi çiz
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            vis_x = -(mx * cos_y + my * sin_y)
            vis_y = mx * sin_y - my * cos_y
            vis_z = -mz
            vis_yaw = -yaw

            duvar_visual_xml = f"""        <visual name="visual_doku">
          <pose>{vis_x:.4f} {vis_y:.4f} {vis_z:.4f} 0 0 {vis_yaw:.4f}</pose>
          <geometry>
            <mesh>
              <uri>file://{mesh_dizin_abs}/{obj_dosya}</uri>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>file://{doku_dizin_abs}/materials/scripts</uri>
              <uri>file://{doku_dizin_abs}/materials/textures</uri>
              <name>{duvar_aktif_malzeme}</name>
            </script>
          </material>
        </visual>"""
        else:
            duvar_amb = (dr, dg, db) if not isikli else (0.6 * dr, 0.6 * dg, 0.6 * db)
            duvar_visual_xml = f"""        <visual name="visual">
          <geometry>
            <box>
              <size>{uzunluk:.4f} {genislik:.4f} {yukseklik:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>{duvar_amb[0]:.4f} {duvar_amb[1]:.4f} {duvar_amb[2]:.4f} 1.0</ambient>
            <diffuse>{dr:.4f} {dg:.4f} {db:.4f} 1.0</diffuse>
          </material>
        </visual>"""

        link_xml = f"""      <link name="duvar_{i}">
        <pose>{mx:.4f} {my:.4f} {mz:.4f} 0 0 {yaw:.4f}</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{uzunluk:.4f} {genislik:.4f} {yukseklik:.4f}</size>
            </box>
          </geometry>
        </collision>
{duvar_visual_xml}
      </link>"""
        duvar_linkleri.append(link_xml)

    if duvar_linkleri:
        duvarlar_ic = "\n".join(duvar_linkleri)
        duvarlar_xml = f"""    <!-- Duvarlar Modeli -->
    <model name="duvarlar">
      <static>true</static>
{duvarlar_ic}
    </model>"""
        modeller_xml.append(duvarlar_xml)

    # Işıksız OGRE malzeme betiğini yaz (oda_isiksiz.material)
    if doku_dizin_abs and not isikli and isiksiz_malzeme_bloklari:
        scripts_dizin = os.path.join(doku_dizin_abs, "materials", "scripts")
        os.makedirs(scripts_dizin, exist_ok=True)
        isiksiz_mat_yolu = os.path.join(scripts_dizin, "oda_isiksiz.material")
        with open(isiksiz_mat_yolu, "w", encoding="utf-8") as f:
            f.write("\n".join(isiksiz_malzeme_bloklari))

    # Mobilya mesh modeli
    if mesh_var:
        gorsel_obj_yolu = obj_yolu
        kesik_obj_yolu = os.path.join(mesh_dizin_abs, "oda_mesh_kesik.obj")
        if mesh_duvar_kes > 0:
            mesh_duvar_yuzeylerini_kes(
                giris_obj_yolu=obj_yolu,
                cikis_obj_yolu=kesik_obj_yolu,
                duvarlar=duvarlar,
                pay=mesh_duvar_kes,
            )
            gorsel_obj_yolu = kesik_obj_yolu
        elif not isikli:
            shutil.copy(obj_yolu, kesik_obj_yolu)
            gorsel_obj_yolu = kesik_obj_yolu

        if not isikli:
            giris_mtl_yolu = os.path.join(mesh_dizin_abs, "oda_mesh.mtl")
            isiksiz_mtl_yolu = os.path.join(mesh_dizin_abs, "oda_mesh_isiksiz.mtl")
            if os.path.isfile(giris_mtl_yolu):
                isiksiz_mtl_malzeme_sayisi = mtl_isiksiz_olustur(
                    giris_mtl_yolu, isiksiz_mtl_yolu
                )
                obj_mtllib_degistir(gorsel_obj_yolu, "oda_mesh_isiksiz.mtl")

        carpisma_obj_yolu = obj_yolu
        if carpisma_mesh:
            carpisma_mesh_abs = os.path.abspath(carpisma_mesh)
            if os.path.isfile(carpisma_mesh_abs):
                carpisma_obj_yolu = carpisma_mesh_abs
            else:
                sys.stderr.write(
                    f"Uyarı: Çarpışma mesh dosyası '{carpisma_mesh_abs}' bulunamadı, "
                    f"varsayılan mesh '{obj_yolu}' kullanılıyor.\n"
                )

        mesh_xml = f"""    <!-- Mobilya Mesh Modeli -->
    <model name="mobilya_mesh">
      <static>true</static>
      <link name="mobilya_mesh_link">
        <pose>0 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://{carpisma_obj_yolu}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://{gorsel_obj_yolu}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>"""
        modeller_xml.append(mesh_xml)

    # Mesh kapsamı dışındaki mobilyalar için yedek kutular / silindirler
    mobilya_kutu_linkleri = []
    for m in mobilyalar:
        mid = m.get("id", len(mobilya_kutu_linkleri))
        cokgen = [nokta_al(p) for p in m.get("cokgen", [])]
        if "merkez" in m and m["merkez"] is not None:
            merkez = nokta_al(m["merkez"])
        elif cokgen:
            pts_m = np.array(cokgen)
            merkez = (float(np.mean(pts_m[:, 0])), float(np.mean(pts_m[:, 1])))
        else:
            merkez = (0.0, 0.0)

        # Eğer mesh varsa ve merkez mesh sınırları içindeyse yedek kutu üretme
        if mesh_var and nokta_sinirda_mi(merkez, mesh_sinirlar):
            continue

        ince = bool(m.get("ince", False))
        if ince:
            # İnce mobilya: silindir (yarıçap 0.02, yükseklik 0.7)
            silindir_z = 0.7 / 2.0
            link_xml = f"""      <link name="mobilya_{mid}">
        <pose>{merkez[0]:.4f} {merkez[1]:.4f} {silindir_z:.4f} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>0.7</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>0.7</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.35 0.3 1.0</ambient>
            <diffuse>0.7 0.6 0.5 1.0</diffuse>
          </material>
        </visual>
      </link>"""
        else:
            # Kutusal mobilya: çokgenin en küçük dönük dikdörtgeni
            if cokgen:
                bx, by, bw, bh, byaw = en_kucuk_dikdortgen(cokgen)
            else:
                bx, by = merkez[0], merkez[1]
                bw, bh, byaw = 0.6, 0.6, 0.0

            kutu_z = mobilya_yuk / 2.0
            link_xml = f"""      <link name="mobilya_{mid}">
        <pose>{bx:.4f} {by:.4f} {kutu_z:.4f} 0 0 {byaw:.4f}</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{bw:.4f} {bh:.4f} {mobilya_yuk:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{bw:.4f} {bh:.4f} {mobilya_yuk:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.35 0.3 1.0</ambient>
            <diffuse>0.7 0.6 0.5 1.0</diffuse>
          </material>
        </visual>
      </link>"""
        mobilya_kutu_linkleri.append(link_xml)

    if mobilya_kutu_linkleri:
        kutu_ic = "\n".join(mobilya_kutu_linkleri)
        mobilya_kutu_xml = f"""    <!-- Mesh Dışı Mobilya Yedek Modeli -->
    <model name="mobilya_kutu">
      <static>true</static>
{kutu_ic}
    </model>"""
        modeller_xml.append(mobilya_kutu_xml)

    tum_modeller_metni = "\n".join(modeller_xml)

    # 6. İç mekân ışıklarını oluştur
    isik_diffuse_metni = "0.0 0.0 0.0 1" if not isikli else "0.2 0.2 0.2 1"
    isik_bloklari = []
    if isik_aralik > 0:
        isik_z = tavan - 0.15
        hedef_cokgen = oda_siniri if oda_siniri else [
            [min_x, min_y],
            [max_x, min_y],
            [max_x, max_y],
            [min_x, max_y],
        ]

        xs = np.arange(min_x, max_x + 1e-6, isik_aralik)
        ys = np.arange(min_y, max_y + 1e-6, isik_aralik)

        isik_noktalari = []
        for x in xs:
            for y in ys:
                if nokta_cokgende_mi((x, y), hedef_cokgen):
                    isik_noktalari.append((x, y))

        if len(isik_noktalari) == 0:
            cx = (min_x + max_x) / 2.0
            cy = (min_y + max_y) / 2.0
            if nokta_cokgende_mi((cx, cy), hedef_cokgen):
                isik_noktalari.append((cx, cy))

        for idx, (lx, ly) in enumerate(isik_noktalari):
            light_xml = f"""    <light type="point" name="ic_isik_{idx}">
      <pose>{lx:.4f} {ly:.4f} {isik_z:.4f} 0 0 0</pose>
      <diffuse>{isik_diffuse_metni}</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>8</range>
        <constant>0.4</constant>
        <linear>0.08</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>"""
            isik_bloklari.append(light_xml)

    isik_sayisi = len(isik_bloklari)
    if isik_sayisi > 0:
        isiklar_xml = f"    <!-- Ic mekan isiklari: {isik_sayisi} adet nokta isik -->\n" + "\n".join(isik_bloklari)
    else:
        isiklar_xml = "    <!-- Ic mekan isiklari: 0 adet (isik eklenmedi) -->"

    sahne_ambient_metni = "1 1 1 1" if not isikli else "0.25 0.25 0.25 1"

    # 7. Tam SDF dünya içeriğini derle
    sdf_icerik = f"""<?xml version="1.0" ?>
<!--
  Gazebo Classic SDF Dunyasi (lab_oda_3b)
  Kaynak yapi: {os.path.abspath(yapi_yol)}
  Kaynak mesh dizini: {mesh_dizin_abs}
  Not: Tavan yuksekligi ({tavan:.2f} m) varsayimdir.
-->
<sdf version="1.6">
  <world name="lab_oda_3b">
    <!-- oda_dogrula.py robotu ışınlamak için /gazebo/set_entity_state kullanır -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros><update_rate>1.0</update_rate></plugin>
    <include>
      <uri>model://sun</uri>
    </include>
    <scene>
      <ambient>{sahne_ambient_metni}</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
{isiklar_xml}
{tum_modeller_metni}
  </world>
</sdf>
"""

    # 8. XML geçerliliğini doğrula
    try:
        ET.fromstring(sdf_icerik)
    except ET.ParseError as e:
        sys.stderr.write(f"Hata: Üretilen SDF XML formatı geçersiz: {e}\n")
        raise

    # 9. Çıktı dosyasını yaz
    cikti_abs = os.path.abspath(cikti_yol)
    cikis_dizin = os.path.dirname(cikti_abs)
    if cikis_dizin:
        os.makedirs(cikis_dizin, exist_ok=True)

    with open(cikti_abs, "w", encoding="utf-8") as f:
        f.write(sdf_icerik)

    isiksiz_doku_sayisi = len(isiksiz_malzeme_bloklari)
    toplam_isiksiz_malzeme = isiksiz_doku_sayisi + isiksiz_mtl_malzeme_sayisi
    print(
        f"Dünya üretildi: yazılan duvar OBJ sayısı={duvar_obj_sayisi}, "
        f"ışıksız malzeme sayısı={toplam_isiksiz_malzeme} "
        f"(doku={isiksiz_doku_sayisi}, mesh_mtl={isiksiz_mtl_malzeme_sayisi})"
    )


def ana():
    """CLI giriş noktası."""
    parser = argparse.ArgumentParser(
        description="Duvar/zemin/tavan/mobilya mesh'inden Gazebo Classic SDF dünyası üretici"
    )
    parser.add_argument("--yapi", default=None, help="Oda yapı YAML dosya yolu")
    parser.add_argument("--mesh_dizin", default=None, help="Mesh dizini (oda_mesh.obj, oda_mesh.json)")
    parser.add_argument("--cikti", default=None, help="Çıktı Gazebo dünya dosyası yolu (.world)")
    parser.add_argument("--tavan", type=float, default=2.7, help="Tavan yüksekliği (m), varsayılan 2.7")
    parser.add_argument(
        "--mobilya_yuk",
        type=float,
        default=0.6,
        help="Mesh'te görünmeyen mobilya için kutu yüksekliği (m), varsayılan 0.6",
    )
    parser.add_argument(
        "--isik_aralik",
        type=float,
        default=2.0,
        help="İç mekan tavan ışıkları ızgara aralığı (m). 0 verilirse ışık eklenmez, varsayılan 2.0",
    )
    parser.add_argument(
        "--doku_dizin",
        type=str,
        default=None,
        help="Doku dizini (doku.yaml, materials/scripts, materials/textures)",
    )
    parser.add_argument(
        "--carpisma_mesh",
        type=str,
        default=None,
        help="Mobilya mesh modeli collision için sadeleştirilmiş OBJ dosya yolu",
    )
    parser.add_argument(
        "--mesh_duvar_kes",
        type=float,
        default=0.08,
        help="Duvar kutularıyla çakışan mesh yüzeylerini kesme payı (m). 0 = kapalı, varsayılan 0.08",
    )
    parser.add_argument(
        "--isikli",
        action="store_true",
        help="Eski ışıklı sahne ve malzeme ayarlarını kullan (varsayılan: ışıksız/unlit)",
    )
    parser.add_argument(
        "--sadelestir",
        nargs=2,
        metavar=("GIRIS", "CIKIS"),
        help="Giriş OBJ dosyasını sadeleştirip çıkış OBJ olarak kaydeder",
    )
    parser.add_argument(
        "--ucgen",
        type=int,
        default=20000,
        help="Sadeleştirme için hedef üçgen sayısı, varsayılan 20000",
    )

    args = parser.parse_args()

    if args.sadelestir:
        giris_obj, cikis_obj = args.sadelestir
        mesh_sadelestir(giris_yol=giris_obj, cikis_yol=cikis_obj, hedef_ucgen=args.ucgen)
        return

    if not args.yapi or not args.mesh_dizin or not args.cikti:
        parser.error("Dünya üretimi için --yapi, --mesh_dizin ve --cikti parametreleri zorunludur.")

    dunya_uret(
        yapi_yol=args.yapi,
        mesh_dizin=args.mesh_dizin,
        cikti_yol=args.cikti,
        tavan=args.tavan,
        mobilya_yuk=args.mobilya_yuk,
        isik_aralik=args.isik_aralik,
        doku_dizin=args.doku_dizin,
        carpisma_mesh=args.carpisma_mesh,
        mesh_duvar_kes=args.mesh_duvar_kes,
        isikli=args.isikli,
    )


if __name__ == "__main__":
    ana()
