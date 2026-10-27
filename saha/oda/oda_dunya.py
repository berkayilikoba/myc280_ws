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

    # Zemin modeli
    zemin_doku_visual_xml = ""
    if doku_veri is not None and "zemin" in doku_veri:
        doku_zemin = doku_veri.get("zemin") or {}
        zemin_kutu = doku_zemin.get("kutu") if isinstance(doku_zemin, dict) else None
        zemin_malzeme = (
            doku_zemin.get("malzeme", "Oda/zemin") if isinstance(doku_zemin, dict) else "Oda/zemin"
        )

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

        zemin_png = os.path.join(doku_dizin_abs, "materials", "textures", "zemin.png")
        if isinstance(zemin_kutu, (list, tuple)) and len(zemin_kutu) == 4 and os.path.isfile(zemin_png):
            # Kutu üst yüzünün Gazebo UV eşlemesi belirsiz (simde doku dönük/ölçeksiz çıktı, 27.10 denemesi):
            # zemin, köşe başına açık UV'li OBJ dörtgeni. Doku satır 0 = ymax (oda_doku.py) = OBJ v=1.
            import shutil
            shutil.copy(zemin_png, os.path.join(mesh_dizin_abs, "zemin.png"))
            with open(os.path.join(mesh_dizin_abs, "zemin_doku.mtl"), "w") as f:
                f.write("newmtl zemin\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd zemin.png\n")
            with open(os.path.join(mesh_dizin_abs, "zemin_doku.obj"), "w") as f:
                f.write("mtllib zemin_doku.mtl\no zemin\n")
                for x, y in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)):
                    f.write(f"v {x:.4f} {y:.4f} 0.0\n")
                f.write("vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\nusemtl zemin\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
            zemin_doku_visual_xml = f"""
        <!-- Doku: dünya koordinatlı OBJ dörtgeni; link (merkez, z=-0.01) telafisiyle dünyada z=0.001 -->
        <visual name="visual_doku">
          <pose>{-zemin_merkez_x:.4f} {-zemin_merkez_y:.4f} 0.0110 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>file://{mesh_dizin_abs}/zemin_doku.obj</uri>
            </mesh>
          </geometry>
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
              <name>{zemin_malzeme}</name>
            </script>
          </material>
        </visual>"""

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
            <ambient>{0.6*zr:.4f} {0.6*zg:.4f} {0.6*zb:.4f} 1.0</ambient>
            <diffuse>{zr:.4f} {zg:.4f} {zb:.4f} 1.0</diffuse>
          </material>
        </visual>{zemin_doku_visual_xml}
      </link>
    </model>"""
    modeller_xml.append(zemin_xml)

    # Tavan modeli
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
            <ambient>0.552 0.552 0.552 1.0</ambient>
            <diffuse>0.92 0.92 0.92 1.0</diffuse>
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

        # Doku görseli kontrolü
        duvar_doku_visual_xml = ""
        if doku_duvarlar is not None:
            doku_d = None
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
                yuz_normali = doku_d.get("yuz_normali") if isinstance(doku_d, dict) else None

                # Yüz normaline göre yerel y eksenindeki işaret (+1 veya -1)
                # Duvar yerel y ekseni vektörü: (-sin(yaw), cos(yaw))
                sign = 1.0
                if yuz_normali is not None:
                    if isinstance(yuz_normali, (list, tuple, dict)):
                        nx, ny = nokta_al(yuz_normali)
                        dot = nx * (-math.sin(yaw)) + ny * math.cos(yaw)
                        sign = 1.0 if dot >= 0 else -1.0
                    else:
                        try:
                            sign = 1.0 if float(yuz_normali) >= 0 else -1.0
                        except (ValueError, TypeError):
                            sign = 1.0
                else:
                    # Normal verilmediyse oda merkezine bakan tarafı seç
                    dot = (zemin_merkez_x - mx) * (-math.sin(yaw)) + (zemin_merkez_y - my) * math.cos(yaw)
                    sign = 1.0 if dot >= 0 else -1.0

                offset_y = sign * (kalinlik / 2.0 + 0.001)
                duvar_doku_visual_xml = f"""
        <!-- Doku Visual (kalinlik/2 + 0.001) -->
        <visual name="visual_doku">
          <pose>0 {offset_y:.4f} 0 0 0 0</pose>
          <geometry>
            <box>
              <size>{uzunluk:.4f} 0.0020 {yukseklik:.4f}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://{doku_dizin_abs}/materials/scripts</uri>
              <uri>file://{doku_dizin_abs}/materials/textures</uri>
              <name>{malzeme}</name>
            </script>
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
        <visual name="visual">
          <geometry>
            <box>
              <size>{uzunluk:.4f} {genislik:.4f} {yukseklik:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>{0.6*dr:.4f} {0.6*dg:.4f} {0.6*db:.4f} 1.0</ambient>
            <diffuse>{dr:.4f} {dg:.4f} {db:.4f} 1.0</diffuse>
          </material>
        </visual>{duvar_doku_visual_xml}
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

    # Mobilya mesh modeli
    if mesh_var:
        gorsel_obj_yolu = obj_yolu
        if mesh_duvar_kes > 0:
            kesik_obj_yolu = os.path.join(mesh_dizin_abs, "oda_mesh_kesik.obj")
            mesh_duvar_yuzeylerini_kes(
                giris_obj_yolu=obj_yolu,
                cikis_obj_yolu=kesik_obj_yolu,
                duvarlar=duvarlar,
                pay=mesh_duvar_kes,
            )
            gorsel_obj_yolu = kesik_obj_yolu

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
      <diffuse>0.2 0.2 0.2 1</diffuse>
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
      <ambient>0.25 0.25 0.25 1</ambient>
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
    )


if __name__ == "__main__":
    ana()
