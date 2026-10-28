#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Görev B: oda_yapi.py
2B doluluk haritasından (map_server YAML + PGM) oda sınırını, duvarları
ve mobilya ayak izlerini çıkarır.

Kullanım:
    python3 oda_yapi.py <harita.yaml> <cikti_oneki> [--eps 0.05 --duvar_pay 0.10 --min_duvar 0.4 --kalinlik 0.10 --min_alan 0.0012 --min_doluluk 0.6 --eksen_tol 10.0 --capraz_doluluk 0.9 --snap_tol 10.0 --cift_mesafe 0.20]
"""

import argparse
import math
import os
import sys
import cv2
import numpy as np
import yaml


def pixel_to_world(u, v, ox, oy, r, H):
    """Piksel (u=sütun, v=satır) koordinatını harita dünya (x, y) koordinatına çevirir."""
    x = ox + (u + 0.5) * r
    y = oy + (H - v - 0.5) * r
    return [float(x), float(y)]


def world_to_pixel(x, y, ox, oy, r, H):
    """Harita dünya (x, y) koordinatını piksel (u, v) koordinatına çevirir."""
    u = (x - ox) / r - 0.5
    v = H - 0.5 - (y - oy) / r
    return float(u), float(v)


def world_to_img4x(x, y, ox, oy, r, H, W):
    """Dünya koordinatını 4x büyütülmüş görüntü piksel koordinatına çevirir."""
    u4 = int(round(((x - ox) / r) * 4.0))
    v4 = int(round((H - (y - oy) / r) * 4.0))
    u4 = max(0, min(W * 4 - 1, u4))
    v4 = max(0, min(H * 4 - 1, v4))
    return u4, v4


def dist_to_segment(points, p1, p2):
    """
    Noktalar dizisinin (N, 2) bir doğru parçasına (p1 -> p2) olan en kısa uzaklığını hesaplar.
    """
    ab = p2 - p1
    ab_len_sq = float(np.dot(ab, ab))
    if ab_len_sq < 1e-10:
        return np.linalg.norm(points - p1, axis=1)
    ap = points - p1
    t = np.clip(np.dot(ap, ab) / ab_len_sq, 0.0, 1.0)
    proj = p1 + t[:, np.newaxis] * ab
    return np.linalg.norm(points - proj, axis=1)


def can_merge(seg1, seg2, angle_thresh_deg=5.0, dist_thresh=0.05, gap_thresh=0.3):
    """
    İki doğru parçasının aynı doğrultuda (açı farkı < 5°),
    doğru uzaklığı < 0.05 m ve üst üste / ardışık (boşluk < 0.3 m) olup olmadığını denetler.
    """
    p1 = np.array(seg1[0], dtype=np.float64)
    p2 = np.array(seg1[1], dtype=np.float64)
    q1 = np.array(seg2[0], dtype=np.float64)
    q2 = np.array(seg2[1], dtype=np.float64)

    v1 = p2 - p1
    v2 = q2 - q1
    len1 = np.linalg.norm(v1)
    len2 = np.linalg.norm(v2)
    if len1 < 1e-6 or len2 < 1e-6:
        return False

    u1 = v1 / len1
    u2 = v2 / len2

    # 1. Açı farkı denetimi (< 5°)
    theta1 = np.arctan2(u1[1], u1[0]) % np.pi
    theta2 = np.arctan2(u2[1], u2[0]) % np.pi
    d_theta = abs(theta1 - theta2)
    if d_theta > np.pi / 2.0:
        d_theta = np.pi - d_theta
    if d_theta >= np.radians(angle_thresh_deg):
        return False

    # Vektörlerin yönünü hizala
    if np.dot(u1, u2) < 0:
        u2 = -u2
        q1, q2 = q2, q1

    # 2. Doğru uzaklığı (< 0.05 m)
    # Orta noktaların karşı doğrunun dik eksenine olan mesafesi
    m1 = (p1 + p2) / 2.0
    m2 = (q1 + q2) / 2.0
    n1 = np.array([-u1[1], u1[0]])
    n2 = np.array([-u2[1], u2[0]])

    dist_m2_to_l1 = abs(np.dot(m2 - p1, n1))
    dist_m1_to_l2 = abs(np.dot(m1 - q1, n2))
    if max(dist_m2_to_l1, dist_m1_to_l2) >= dist_thresh:
        return False

    # 3. Doğrultu boyunca üst üste / ardışık boşluk denetimi (< 0.3 m)
    u_avg = u1 + u2
    u_avg = u_avg / np.linalg.norm(u_avg)

    t1_a = np.dot(p1, u_avg)
    t1_b = np.dot(p2, u_avg)
    a1, b1 = min(t1_a, t1_b), max(t1_a, t1_b)

    t2_a = np.dot(q1, u_avg)
    t2_b = np.dot(q2, u_avg)
    a2, b2 = min(t2_a, t2_b), max(t2_a, t2_b)

    if a1 <= b2 and a2 <= b1:
        gap = 0.0
    else:
        gap = max(a1, a2) - min(b1, b2)

    if gap >= gap_thresh:
        return False

    return True


def merge_two_lines(seg1, seg2):
    """
    İki doğru parçasını uzunluk ağırlıklı doğrultu ve merkez kullanarak birleştirir.
    """
    p1 = np.array(seg1[0], dtype=np.float64)
    p2 = np.array(seg1[1], dtype=np.float64)
    q1 = np.array(seg2[0], dtype=np.float64)
    q2 = np.array(seg2[1], dtype=np.float64)

    len1 = np.linalg.norm(p2 - p1)
    len2 = np.linalg.norm(q2 - q1)
    if len1 < 1e-6:
        return [q1.tolist(), q2.tolist()]
    if len2 < 1e-6:
        return [p1.tolist(), p2.tolist()]

    u1 = (p2 - p1) / len1
    u2 = (q2 - q1) / len2
    if np.dot(u1, u2) < 0:
        u2 = -u2

    u_avg = len1 * u1 + len2 * u2
    u_avg = u_avg / np.linalg.norm(u_avg)

    center = (len1 * (p1 + p2) / 2.0 + len2 * (q1 + q2) / 2.0) / (len1 + len2)

    pts = [p1, p2, q1, q2]
    projs = [np.dot(p - center, u_avg) for p in pts]
    min_p = min(projs)
    max_p = max(projs)

    new_p1 = center + min_p * u_avg
    new_p2 = center + max_p * u_avg

    return [new_p1.tolist(), new_p2.tolist()]


def segments_intersect(p1, p2, q1, q2, end_margin=0.1, min_angle_deg=20.0):
    """
    İki doğru parçasının birbirini kesip kesmediğini (uç noktalardan > 0.1 m içeride)
    ve açı farkının > 20° olup olmadığını denetler.
    """
    p1 = np.array(p1, dtype=np.float64)
    p2 = np.array(p2, dtype=np.float64)
    q1 = np.array(q1, dtype=np.float64)
    q2 = np.array(q2, dtype=np.float64)

    d1 = p2 - p1
    d2 = q2 - q1
    len1 = float(np.linalg.norm(d1))
    len2 = float(np.linalg.norm(d2))

    if len1 <= 2.0 * end_margin or len2 <= 2.0 * end_margin:
        return False

    u1 = d1 / len1
    u2 = d2 / len2

    # Açı farkı denetimi (> 20°)
    cos_angle = abs(float(np.dot(u1, u2)))
    cos_angle = min(1.0, max(0.0, cos_angle))
    angle_rad = math.acos(cos_angle)
    if math.degrees(angle_rad) <= min_angle_deg:
        return False

    # Kesişim noktası hesabı: p1 + t*d1 = q1 + u*d2
    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-9:
        return False

    r = q1 - p1
    t = (r[0] * d2[1] - r[1] * d2[0]) / det
    u = (r[0] * d1[1] - r[1] * d1[0]) / det

    # Uç noktalardan > end_margin (0.1 m) içeride mi?
    dist_p1 = t * len1
    dist_p2 = (1.0 - t) * len1
    dist_q1 = u * len2
    dist_q2 = (1.0 - u) * len2

    if (
        dist_p1 > end_margin
        and dist_p2 > end_margin
        and dist_q1 > end_margin
        and dist_q2 > end_margin
    ):
        return True

    return False


def hesapla_doluluk(p1, p2, dolu_mask, ox, oy, resolution, H, W, radius_cells=1.5):
    """
    Parça boyunca her 1 hücrede bir örnek nokta alır.
    Noktanın 1.5 hücre yarıçapında dolu hücre varsa dolu sayar.
    oran = dolu sayılan / örnek sayısı
    """
    p1 = np.array(p1, dtype=np.float64)
    p2 = np.array(p2, dtype=np.float64)
    uzunluk = float(np.linalg.norm(p2 - p1))
    if uzunluk < 1e-6:
        return 0.0

    num_samples = max(2, int(round(uzunluk / resolution)) + 1)
    ts = np.linspace(0.0, 1.0, num_samples)
    dolu_sayilan = 0
    r2 = radius_cells * radius_cells + 1e-7

    for t in ts:
        pt = p1 + t * (p2 - p1)
        u, v = world_to_pixel(pt[0], pt[1], ox, oy, resolution, H)

        u_min = max(0, int(math.floor(u - radius_cells)))
        u_max = min(W - 1, int(math.ceil(u + radius_cells)))
        v_min = max(0, int(math.floor(v - radius_cells)))
        v_max = min(H - 1, int(math.ceil(v + radius_cells)))

        if u_min > u_max or v_min > v_max:
            continue

        sub = dolu_mask[v_min : v_max + 1, u_min : u_max + 1]
        if not np.any(sub):
            continue

        sub_v, sub_u = np.where(sub)
        actual_u = u_min + sub_u
        actual_v = v_min + sub_v
        dists_sq = (actual_u - u) ** 2 + (actual_v - v) ** 2
        if np.any(dists_sq <= r2):
            dolu_sayilan += 1

    return float(dolu_sayilan) / float(num_samples)


def get_wall_axis(d, theta1, theta2, tol_deg=10.0):
    """
    Duvarın hangi baskın eksene (1 veya 2) ait olduğunu belirler.
    d içinde 'axis_id' varsa onu döner, yoksa açı farkına göre belirler.
    """
    if "axis_id" in d and d["axis_id"] in (1, 2):
        return d["axis_id"]

    p1 = d["p1"]
    p2 = d["p2"]
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    if math.hypot(dx, dy) < 1e-6:
        return None

    ang_deg = math.degrees(math.atan2(dy, dx)) % 180.0
    if ang_deg < 0.0:
        ang_deg += 180.0
    if ang_deg >= 180.0:
        ang_deg -= 180.0

    fark1 = abs(ang_deg - theta1) % 180.0
    if fark1 > 90.0:
        fark1 = 180.0 - fark1

    fark2 = abs(ang_deg - theta2) % 180.0
    if fark2 > 90.0:
        fark2 = 180.0 - fark2

    if fark1 <= tol_deg and fark1 <= fark2:
        return 1
    elif fark2 <= tol_deg and fark2 < fark1:
        return 2
    return None


def main():
    parser = argparse.ArgumentParser(
        description="2B doluluk haritasından duvarlar ve mobilya ayak izlerini çıkarır."
    )
    parser.add_argument("harita_yaml", help="Girdi harita YAML dosyası yolu")
    parser.add_argument("cikti_oneki", help="Çıktı dosyaları ön eki (<cikti>.yaml ve <cikti>.png)")
    parser.add_argument(
        "--eps",
        type=float,
        default=0.05,
        help="Oda sınırı için approxPolyDP epsilon değeri (m) [0.05]",
    )
    parser.add_argument(
        "--duvar_pay",
        type=float,
        default=0.10,
        help="Oda sınırından duvar adayı arama mesafesi (m) [0.10]",
    )
    parser.add_argument(
        "--min_duvar",
        type=float,
        default=0.4,
        help="Minimum duvar parça uzunluğu (m) [0.4]",
    )
    parser.add_argument(
        "--kalinlik",
        type=float,
        default=0.10,
        help="Duvar kalınlığı (m) [0.10]",
    )
    parser.add_argument(
        "--min_alan",
        type=float,
        default=0.0012,
        help="Minimum mobilya alanı (m²) [0.0012]",
    )
    parser.add_argument(
        "--min_doluluk",
        type=float,
        default=0.6,
        help="Minimum duvar doluluk oranı [0.6]",
    )
    parser.add_argument(
        "--eksen_tol",
        type=float,
        default=10.0,
        help="Baskın eksen açı toleransı (derece) [10.0]",
    )
    parser.add_argument(
        "--capraz_doluluk",
        type=float,
        default=0.9,
        help="Eksen dışı çapraz parçalar için minimum doluluk eşiği [0.9]",
    )
    parser.add_argument(
        "--snap_tol",
        type=float,
        default=10.0,
        help="Baskın eksene hizalama toleransı (derece) [10.0, 0 ise kapalı]",
    )
    parser.add_argument(
        "--cift_mesafe",
        type=float,
        default=0.20,
        help="İç içe paralel çift duvarları eleme dik mesafe eşiği (m) [0.20, 0 ise kapalı]",
    )

    args = parser.parse_args()

    # 1. Girdi Haritayı Oku
    yaml_path = os.path.abspath(args.harita_yaml)
    if not os.path.isfile(yaml_path):
        print(f"Hata: YAML dosyası bulunamadı: {yaml_path}", file=sys.stderr)
        sys.exit(1)

    with open(yaml_path, "r", encoding="utf-8") as f:
        map_meta = yaml.safe_load(f)

    img_rel = map_meta["image"]
    img_path = (
        img_rel
        if os.path.isabs(img_rel)
        else os.path.join(os.path.dirname(yaml_path), img_rel)
    )

    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Hata: Harita görüntüsü açılamadı: {img_path}", file=sys.stderr)
        sys.exit(1)

    H, W = img.shape
    resolution = float(map_meta["resolution"])
    origin = map_meta["origin"]
    ox, oy = float(origin[0]), float(origin[1])
    negate = int(map_meta.get("negate", 0))
    occupied_thresh = float(map_meta.get("occupied_thresh", 0.65))

    # Dolu ve serbest alan maskeleri
    p = (
        (255.0 - img.astype(np.float32)) / 255.0
        if negate == 0
        else img.astype(np.float32) / 255.0
    )
    dolu_mask = p > occupied_thresh
    serbest_mask = (img > 250).astype(np.uint8) if negate == 0 else (img < 5).astype(np.uint8)

    dolu_v, dolu_u = np.where(dolu_mask)
    if len(dolu_u) > 0:
        dolu_pts_x = ox + (dolu_u + 0.5) * resolution
        dolu_pts_y = oy + (H - dolu_v - 0.5) * resolution
        dolu_pts_world = np.column_stack([dolu_pts_x, dolu_pts_y])
    else:
        dolu_pts_world = np.empty((0, 2), dtype=np.float64)

    # 2. ODA SINIRI
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        serbest_mask, connectivity=8
    )
    if num_labels > 1:
        # En büyük serbest bağlı bileşen (arkaplan label 0 hariç)
        max_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        serbest_en_buyuk = (labels == max_label).astype(np.uint8) * 255
    else:
        serbest_en_buyuk = np.zeros_like(serbest_mask)

    # Küçük boşlukları kapat (morph close 5 hücre)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    serbest_closed = cv2.morphologyEx(serbest_en_buyuk, cv2.MORPH_CLOSE, kernel_close)

    # Dış kontur ve çokgen yaklaşıklama
    contours_info = cv2.findContours(
        serbest_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

    oda_siniri = []
    approx = None
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        eps_px = args.eps / resolution
        approx = cv2.approxPolyDP(max_contour, eps_px, closed=True)
        for pt in approx:
            u_pt, v_pt = pt[0][0], pt[0][1]
            oda_siniri.append(pixel_to_world(u_pt, v_pt, ox, oy, resolution, H))

    # 3. DUVARLAR
    duvar_adayi_mask = np.zeros((H, W), dtype=np.uint8)
    if approx is not None and len(approx) > 0:
        poly_edge_img = np.zeros((H, W), dtype=np.uint8)
        cv2.polylines(poly_edge_img, [approx], isClosed=True, color=255, thickness=1)
        dist_px = cv2.distanceTransform(255 - poly_edge_img, cv2.DIST_L2, 5)
        dist_m = dist_px * resolution
        duvar_adayi_mask = (
            (dolu_mask > 0) & (dist_m <= args.duvar_pay)
        ).astype(np.uint8) * 255

    min_line_len_px = max(1, int(round(args.min_duvar / resolution)))
    max_line_gap_px = max(1, int(round(0.15 / resolution)))

    lines = cv2.HoughLinesP(
        duvar_adayi_mask,
        rho=1,
        theta=np.pi / 180.0,
        threshold=15,
        minLineLength=min_line_len_px,
        maxLineGap=max_line_gap_px,
    )

    raw_segments = []
    if lines is not None:
        for line in lines:
            u1, v1, u2, v2 = line[0]
            p1 = pixel_to_world(u1, v1, ox, oy, resolution, H)
            p2 = pixel_to_world(u2, v2, ox, oy, resolution, H)
            if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) >= 1e-4:
                raw_segments.append([p1, p2])

    # Doğru parçalarını birleştirme
    segments = list(raw_segments)
    merged_any = True
    while merged_any:
        merged_any = False
        n = len(segments)
        for i in range(n):
            for j in range(i + 1, n):
                if can_merge(
                    segments[i],
                    segments[j],
                    angle_thresh_deg=5.0,
                    dist_thresh=0.05,
                    gap_thresh=0.3,
                ):
                    new_seg = merge_two_lines(segments[i], segments[j])
                    segments[i] = new_seg
                    segments.pop(j)
                    merged_any = True
                    break
            if merged_any:
                break

    aday_duvarlar = []
    for seg in segments:
        p1, p2 = seg[0], seg[1]
        uzunluk = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if uzunluk >= args.min_duvar:
            doluluk = hesapla_doluluk(
                p1, p2, dolu_mask, ox, oy, resolution, H, W, radius_cells=1.5
            )
            aday_duvarlar.append(
                {
                    "p1": [round(float(p1[0]), 4), round(float(p1[1]), 4)],
                    "p2": [round(float(p2[0]), 4), round(float(p2[1]), 4)],
                    "uzunluk": round(float(uzunluk), 4),
                    "kalinlik": round(float(args.kalinlik), 4),
                    "doluluk": round(float(doluluk), 3),
                }
            )

    atilan_parca_sayisi = 0
    eksen_disi_atilan = 0

    # 1. Doluluk filtreleme: doluluk < min_doluluk olanları ele
    gecerli_duvarlar = []
    for d in aday_duvarlar:
        if d["doluluk"] < args.min_doluluk:
            atilan_parca_sayisi += 1
        else:
            gecerli_duvarlar.append(d)

    # 2. Odanın baskın iki ekseni ve eksen dışı çapraz parçaları ele
    if len(gecerli_duvarlar) > 0:
        hist = np.zeros(90, dtype=np.float64)
        for d in gecerli_duvarlar:
            p1, p2 = d["p1"], d["p2"]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            ang_deg = math.degrees(math.atan2(dy, dx)) % 180.0
            if ang_deg < 0.0:
                ang_deg += 180.0
            if ang_deg >= 180.0:
                ang_deg -= 180.0
            bin_idx = min(89, max(0, int(ang_deg // 2.0)))
            hist[bin_idx] += d["uzunluk"]

        max_idx = int(np.argmax(hist))
        theta1 = float((max_idx + 0.5) * 2.0)
        theta2 = float(theta1 + 90.0)

        eksen_duvarlari = []
        for d in gecerli_duvarlar:
            p1, p2 = d["p1"], d["p2"]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            ang_deg = math.degrees(math.atan2(dy, dx)) % 180.0
            if ang_deg < 0.0:
                ang_deg += 180.0
            if ang_deg >= 180.0:
                ang_deg -= 180.0

            fark1 = abs(ang_deg - theta1) % 180.0
            if fark1 > 90.0:
                fark1 = 180.0 - fark1

            fark2 = abs(ang_deg - theta2) % 180.0
            if fark2 > 90.0:
                fark2 = 180.0 - fark2

            eksen_ici = (fark1 <= args.eksen_tol) or (fark2 <= args.eksen_tol)

            if (not eksen_ici) and (d["doluluk"] < args.capraz_doluluk):
                eksen_disi_atilan += 1
                atilan_parca_sayisi += 1
            else:
                eksen_duvarlari.append(d)

        gecerli_duvarlar = eksen_duvarlari
    else:
        theta1 = 0.0
        theta2 = 90.0

    # 2.5 Duvarları baskın eksene hizala (snap) ve birleştir
    hizalanan_sayisi = 0
    birlesen_sayisi = 0

    if args.snap_tol > 0 and len(gecerli_duvarlar) > 0:
        for d in gecerli_duvarlar:
            p1 = np.array(d["p1"], dtype=np.float64)
            p2 = np.array(d["p2"], dtype=np.float64)
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            ang_deg = math.degrees(math.atan2(dy, dx)) % 180.0
            if ang_deg < 0.0:
                ang_deg += 180.0
            if ang_deg >= 180.0:
                ang_deg -= 180.0

            fark1 = abs(ang_deg - theta1) % 180.0
            if fark1 > 90.0:
                fark1 = 180.0 - fark1

            fark2 = abs(ang_deg - theta2) % 180.0
            if fark2 > 90.0:
                fark2 = 180.0 - fark2

            if fark1 <= fark2:
                target_theta = theta1
                min_fark = fark1
                axis_id = 1
            else:
                target_theta = theta2
                min_fark = fark2
                axis_id = 2

            if min_fark <= args.snap_tol:
                alpha = math.radians(target_theta)
                u = np.array([math.cos(alpha), math.sin(alpha)], dtype=np.float64)
                n = np.array([-math.sin(alpha), math.cos(alpha)], dtype=np.float64)

                s1 = float(np.dot(p1, u))
                s2 = float(np.dot(p2, u))

                # Hat konumu: Duvarın yakınındaki (dist_to_segment <= kalinlik) dolu hücrelerin
                # eksene dik koordinatının medyanı ile hat konumunu belirle
                if len(dolu_pts_world) > 0:
                    dists = dist_to_segment(dolu_pts_world, p1, p2)
                    nearby = dolu_pts_world[dists <= args.kalinlik]
                    if len(nearby) > 0:
                        c = float(np.median(np.dot(nearby, n)))
                    else:
                        mid = (p1 + p2) / 2.0
                        c = float(np.dot(mid, n))
                else:
                    mid = (p1 + p2) / 2.0
                    c = float(np.dot(mid, n))

                new_p1 = s1 * u + c * n
                new_p2 = s2 * u + c * n

                d["p1"] = [float(new_p1[0]), float(new_p1[1])]
                d["p2"] = [float(new_p2[0]), float(new_p2[1])]
                d["axis_id"] = axis_id
                hizalanan_sayisi += 1

        # Aynı eksendeki parçaları birleştir (dik uzaklık <= 0.06 m, boşluk <= 0.3 m)
        merged_any = True
        while merged_any:
            merged_any = False
            n_duv = len(gecerli_duvarlar)
            for i in range(n_duv):
                for j in range(i + 1, n_duv):
                    d_i = gecerli_duvarlar[i]
                    d_j = gecerli_duvarlar[j]

                    axis_i = d_i.get("axis_id")
                    axis_j = d_j.get("axis_id")
                    if axis_i is None or axis_j is None or axis_i != axis_j:
                        continue

                    seg_i = [d_i["p1"], d_i["p2"]]
                    seg_j = [d_j["p1"], d_j["p2"]]
                    if can_merge(
                        seg_i,
                        seg_j,
                        angle_thresh_deg=1.0,
                        dist_thresh=0.0601,
                        gap_thresh=0.3001,
                    ):
                        new_seg = merge_two_lines(seg_i, seg_j)
                        d_i["p1"] = new_seg[0]
                        d_i["p2"] = new_seg[1]
                        gecerli_duvarlar.pop(j)
                        birlesen_sayisi += 1
                        merged_any = True
                        break
                if merged_any:
                    break

        # Uzunluk ve doluluk alanlarını yeni uçlarla yeniden hesapla
        for d in gecerli_duvarlar:
            p1_pt = d["p1"]
            p2_pt = d["p2"]
            uzunluk = math.hypot(p2_pt[0] - p1_pt[0], p2_pt[1] - p1_pt[1])
            doluluk = hesapla_doluluk(
                p1_pt, p2_pt, dolu_mask, ox, oy, resolution, H, W, radius_cells=1.5
            )
            d["p1"] = [round(float(p1_pt[0]), 4), round(float(p1_pt[1]), 4)]
            d["p2"] = [round(float(p2_pt[0]), 4), round(float(p2_pt[1]), 4)]
            d["uzunluk"] = round(float(uzunluk), 4)
            d["kalinlik"] = round(float(args.kalinlik), 4)
            d["doluluk"] = round(float(doluluk), 3)

    # 2.6 İç içe paralel çift duvarları ele (aynı eksen, dik uzaklık <= cift_mesafe, örtüşme >= %80)
    cift_duvar_atilan = 0
    if args.cift_mesafe > 0 and len(gecerli_duvarlar) > 0:
        axis_tol = args.snap_tol if args.snap_tol > 0 else args.eksen_tol
        cift_tekrar = True
        while cift_tekrar:
            cift_tekrar = False
            n_duv = len(gecerli_duvarlar)
            for i in range(n_duv):
                for j in range(i + 1, n_duv):
                    d_i = gecerli_duvarlar[i]
                    d_j = gecerli_duvarlar[j]

                    axis_i = get_wall_axis(d_i, theta1, theta2, axis_tol)
                    axis_j = get_wall_axis(d_j, theta1, theta2, axis_tol)

                    if axis_i is None or axis_j is None or axis_i != axis_j:
                        continue

                    # Hangisi uzun hangisi kısa?
                    if d_i["uzunluk"] > d_j["uzunluk"]:
                        u_idx, k_idx = i, j
                    elif d_j["uzunluk"] > d_i["uzunluk"]:
                        u_idx, k_idx = j, i
                    else:
                        if d_i["doluluk"] < d_j["doluluk"]:
                            u_idx, k_idx = j, i
                        else:
                            u_idx, k_idx = i, j

                    d_uzun = gecerli_duvarlar[u_idx]
                    d_kisa = gecerli_duvarlar[k_idx]

                    target_theta = theta1 if axis_i == 1 else theta2
                    alpha = math.radians(target_theta)
                    u_vec = np.array([math.cos(alpha), math.sin(alpha)], dtype=np.float64)
                    n_vec = np.array([-math.sin(alpha), math.cos(alpha)], dtype=np.float64)

                    p1_u = np.array(d_uzun["p1"], dtype=np.float64)
                    p2_u = np.array(d_uzun["p2"], dtype=np.float64)
                    p1_k = np.array(d_kisa["p1"], dtype=np.float64)
                    p2_k = np.array(d_kisa["p2"], dtype=np.float64)

                    # Dik uzaklık
                    c_u = float(np.dot((p1_u + p2_u) / 2.0, n_vec))
                    c_k = float(np.dot((p1_k + p2_k) / 2.0, n_vec))
                    dik_uzaklik = abs(c_u - c_k)

                    if dik_uzaklik > args.cift_mesafe + 1e-6:
                        continue

                    # Eksen yönündeki izdüşüm aralıkları
                    t_u1 = float(np.dot(p1_u, u_vec))
                    t_u2 = float(np.dot(p2_u, u_vec))
                    a_u, b_u = min(t_u1, t_u2), max(t_u1, t_u2)

                    t_k1 = float(np.dot(p1_k, u_vec))
                    t_k2 = float(np.dot(p2_k, u_vec))
                    a_k, b_k = min(t_k1, t_k2), max(t_k1, t_k2)

                    len_k_proj = b_k - a_k
                    if len_k_proj < 1e-6:
                        overlap_oran = 1.0 if (a_u <= a_k <= b_u) else 0.0
                    else:
                        overlap = max(0.0, min(b_u, b_k) - max(a_u, a_k))
                        overlap_oran = overlap / len_k_proj

                    if overlap_oran >= 0.80 - 1e-9:
                        gecerli_duvarlar.pop(k_idx)
                        cift_duvar_atilan += 1
                        atilan_parca_sayisi += 1
                        cift_tekrar = True
                        break

                if cift_tekrar:
                    break

    for d in gecerli_duvarlar:
        d.pop("axis_id", None)

    # 3. Kesişen parçaları ele: iki parça kesişiyorsa (> 0.1 m içeride) ve açı farkı > 20° ise,
    # düşük doluluk oranlı olanı at
    while True:
        worst_idx = None
        worst_doluluk = float("inf")
        worst_len = float("inf")
        n_gecerli = len(gecerli_duvarlar)

        for i in range(n_gecerli):
            for j in range(i + 1, n_gecerli):
                seg_i = gecerli_duvarlar[i]
                seg_j = gecerli_duvarlar[j]
                if segments_intersect(
                    seg_i["p1"],
                    seg_i["p2"],
                    seg_j["p1"],
                    seg_j["p2"],
                    end_margin=0.1,
                    min_angle_deg=20.0,
                ):
                    if seg_i["doluluk"] < seg_j["doluluk"]:
                        drop_cand = i
                    elif seg_j["doluluk"] < seg_i["doluluk"]:
                        drop_cand = j
                    else:
                        drop_cand = i if seg_i["uzunluk"] < seg_j["uzunluk"] else j

                    cand = gecerli_duvarlar[drop_cand]
                    if cand["doluluk"] < worst_doluluk or (
                        cand["doluluk"] == worst_doluluk and cand["uzunluk"] < worst_len
                    ):
                        worst_doluluk = cand["doluluk"]
                        worst_len = cand["uzunluk"]
                        worst_idx = drop_cand

        if worst_idx is None:
            break

        gecerli_duvarlar.pop(worst_idx)
        atilan_parca_sayisi += 1

    duvarlar = gecerli_duvarlar
    toplam_duvar_uzunlugu = sum(w["uzunluk"] for w in duvarlar)

    # 4. MOBİLYA
    dolu_v, dolu_u = np.where(dolu_mask)
    mobilya_listesi = []

    if len(dolu_u) > 0:
        dolu_pts_x = ox + (dolu_u + 0.5) * resolution
        dolu_pts_y = oy + (H - dolu_v - 0.5) * resolution
        dolu_pts_world = np.column_stack([dolu_pts_x, dolu_pts_y])

        if len(duvarlar) > 0:
            min_dist_to_wall = np.full(len(dolu_pts_world), np.inf)
            for w in duvarlar:
                p1_w = np.array(w["p1"], dtype=np.float64)
                p2_w = np.array(w["p2"], dtype=np.float64)
                dists = dist_to_segment(dolu_pts_world, p1_w, p2_w)
                min_dist_to_wall = np.minimum(min_dist_to_wall, dists)
            mobilya_indices = np.where(min_dist_to_wall > 0.06)[0]
        else:
            mobilya_indices = np.arange(len(dolu_pts_world))

        mobilya_img = np.zeros((H, W), dtype=np.uint8)
        mobilya_img[dolu_v[mobilya_indices], dolu_u[mobilya_indices]] = 255

        # 1 hücre dilate ve bağlı bileşenler
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mobilya_dilated = cv2.dilate(mobilya_img, kernel_dilate, iterations=1)

        num_mob, labels_mob, _, _ = cv2.connectedComponentsWithStats(
            mobilya_dilated, connectivity=8
        )

        mobilya_id = 1
        for k in range(1, num_mob):
            mask_k = (labels_mob == k) & (mobilya_img > 0)
            hucre_sayisi = int(np.count_nonzero(mask_k))
            alan_m2 = hucre_sayisi * (resolution ** 2)

            if alan_m2 < args.min_alan:
                continue

            vk, uk = np.where(mask_k)
            xk = ox + (uk + 0.5) * resolution
            yk = oy + (H - vk - 0.5) * resolution
            pts_comp = np.column_stack([xk, yk])

            merkez = [round(float(np.mean(xk)), 4), round(float(np.mean(yk)), 4)]

            # Dışbükey zarf (convex hull)
            hull = cv2.convexHull(pts_comp.astype(np.float32))
            cokgen = [
                [round(float(pt[0][0]), 4), round(float(pt[0][1]), 4)]
                for pt in hull
            ]

            # İnce bayrağı (en küçük dikdörtgen kısa kenarı < 0.06 m)
            rect = cv2.minAreaRect(pts_comp.astype(np.float32))
            kisa_kenar = min(rect[1][0], rect[1][1])
            ince = bool(kisa_kenar < 0.06)

            mobilya_listesi.append(
                {
                    "id": mobilya_id,
                    "merkez": merkez,
                    "alan_m2": round(float(alan_m2), 6),
                    "cokgen": cokgen,
                    "ince": ince,
                }
            )
            mobilya_id += 1

    # 5. ÇIKTI YAML
    out_base = args.cikti_oneki
    if out_base.endswith(".yaml"):
        out_base = out_base[:-5]
    elif out_base.endswith(".png"):
        out_base = out_base[:-4]

    yaml_out_path = f"{out_base}.yaml"
    png_out_path = f"{out_base}.png"

    cikis_verisi = {
        "kaynak": yaml_path,
        "cozunurluk": float(resolution),
        "eksenler": [round(float(theta1), 2), round(float(theta2), 2)],
        "oda_siniri": oda_siniri,
        "duvarlar": duvarlar,
        "mobilya": mobilya_listesi,
    }

    os.makedirs(os.path.dirname(os.path.abspath(yaml_out_path)), exist_ok=True)
    with open(yaml_out_path, "w", encoding="utf-8") as f:
        yaml.dump(cikis_verisi, f, sort_keys=False, allow_unicode=True)

    # 6. HATA AYIKLAMA PNG (4x büyütülmüş, harita gri, duvarlar kırmızı 2px, mobilya yeşil, oda sınırı mavi)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img_4x = cv2.resize(img_bgr, (W * 4, H * 4), interpolation=cv2.INTER_NEAREST)

    # Oda sınırı mavi (BGR: 255, 0, 0)
    if len(oda_siniri) >= 3:
        sinir_4x = np.array(
            [world_to_img4x(pt[0], pt[1], ox, oy, resolution, H, W) for pt in oda_siniri],
            dtype=np.int32,
        ).reshape((-1, 1, 2))
        cv2.polylines(img_4x, [sinir_4x], isClosed=True, color=(255, 0, 0), thickness=2)

    # Mobilya yeşil çokgen (BGR: 0, 255, 0)
    for m in mobilya_listesi:
        cokgen_pts = m["cokgen"]
        if len(cokgen_pts) >= 3:
            pts_4x = np.array(
                [world_to_img4x(pt[0], pt[1], ox, oy, resolution, H, W) for pt in cokgen_pts],
                dtype=np.int32,
            ).reshape((-1, 1, 2))
            cv2.polylines(img_4x, [pts_4x], isClosed=True, color=(0, 255, 0), thickness=2)
        elif len(cokgen_pts) == 2:
            p1_4x = world_to_img4x(cokgen_pts[0][0], cokgen_pts[0][1], ox, oy, resolution, H, W)
            p2_4x = world_to_img4x(cokgen_pts[1][0], cokgen_pts[1][1], ox, oy, resolution, H, W)
            cv2.line(img_4x, p1_4x, p2_4x, color=(0, 255, 0), thickness=2)
        elif len(cokgen_pts) == 1:
            p_4x = world_to_img4x(cokgen_pts[0][0], cokgen_pts[0][1], ox, oy, resolution, H, W)
            cv2.circle(img_4x, p_4x, radius=2, color=(0, 255, 0), thickness=-1)

    # Duvarlar kırmızı çizgi (BGR: 0, 0, 255), 2 px
    for w in duvarlar:
        p1_4x = world_to_img4x(w["p1"][0], w["p1"][1], ox, oy, resolution, H, W)
        p2_4x = world_to_img4x(w["p2"][0], w["p2"][1], ox, oy, resolution, H, W)
        cv2.line(img_4x, p1_4x, p2_4x, color=(0, 0, 255), thickness=2)

    cv2.imwrite(png_out_path, img_4x)

    # Özet bilgi
    print(f"İşlem tamamlandı.")
    print(f"  Oda Sınırı Nokta Sayısı : {len(oda_siniri)}")
    print(f"  Duvar Sayısı             : {len(duvarlar)} (Toplam uzunluk: {toplam_duvar_uzunlugu:.2f} m)")
    print(f"  Atılan Parça Sayısı      : {atilan_parca_sayisi}")
    print(f"  Eksen Dışı Atılan Sayısı : {eksen_disi_atilan}")
    print(f"  Eksen dışı atılan        : {eksen_disi_atilan}")
    print(f"  eksene hizalanan: {hizalanan_sayisi}, birleşen: {birlesen_sayisi}")
    print(f"  çift duvar atılan: {cift_duvar_atilan}")
    print(f"  Baskın Eksenler (derece) : [{theta1:.2f}, {theta2:.2f}]")
    print(f"  Mobilya Sayısı           : {len(mobilya_listesi)}")
    print(f"  Çıktı YAML               : {yaml_out_path}")
    print(f"  Hata Ayıklama PNG        : {png_out_path}")


if __name__ == "__main__":
    main()
