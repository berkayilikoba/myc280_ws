#!/usr/bin/env python3
"""Kamera–lidar dış kalibrasyonu, ChArUco tahtası ile (olcum_protokolu.md 1.3). Lidar TF'i (laser_duz) referans kabul
edilir; sonuç base_link -> camera_link (kamera_kalibrasyon*.yaml biçiminde).

Yöntem (düzlem–çizgi, Zhang & Pless 2004): her tahta konumunda renk görüntüsünden tahtanın 6B pozu (OpenCV ChArUco +
solvePnP) -> kamera çerçevesinde düzlem (n, d). Aynı anda lidar taraması tahtayı bir çizgide keser; o noktalar
p_l (laser_duz) için n·(T_co_l p_l) = d olmalı. Tüm konumlarda bu artıklar en aza indirilir (Huber).
Kayıt: kayit_baslat.sh, robot SABİT; tahta 6-8 farklı konuma (mesafe 0.8-2 m, sağ/sol, farklı eğimler: dik, öne/arkaya
±20-30°, yana döndürülmüş) her biri >= 2 s hareketsiz tutulur. Lidar düzlemi (yerden ~17 cm) tahtayı kesmeli: tahtanın
alt kenarı yerden ~10 cm'de olsun; eller/ayak lidar düzleminde tahtanın yanında olmasın.
Gözlenebilirlik: yalnız dik tahtalarla z, roll, pitch belirsiz kalır -> eğim çeşitliliği şart; yoksa --serbest x y yaw
(z, roll, pitch zemin_egimi.py'den, kalibrasyon dosyasındaki gibi kalır).

Kullanım (Humble):
  charuco_kalib.py --uret tahta.png                  # A3'e %100 ölçekle basılacak tahta (300 dpi); kare boyunu ölçüp --kare ver
  charuco_kalib.py <kayit_dizini> [--kare 0.055 --isaret 0.040] [--serbest x y z roll pitch yaw] [--kalib ...yaml]
Çıktı: <kayit>/charuco_kalib.yaml (yeni base_link_to_camera_link, eski ile fark, konum başına artıklar)
"""

import argparse
import json
import math
import os

import cv2
import numpy as np
import yaml

BURADA = os.path.dirname(os.path.abspath(__file__))
SOZLUK = cv2.aruco.DICT_5X5_100
LIDAR_VARSAYILAN = (0.137, 0.015, 0.02, -0.1686)        # ortam.sh lidar_tf_yayinla (kayıtta tf_static yoksa)
PARAM = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']


def tahta(a):
    return cv2.aruco.CharucoBoard((a.sx, a.sy), a.kare, a.isaret, cv2.aruco.getPredefinedDictionary(SOZLUK))


def uret(a):
    dpi = 300
    px = lambda m: int(round(m / 0.0254 * dpi))
    img = tahta(a).generateImage((px(a.sx * a.kare), px(a.sy * a.kare)), marginSize=0, borderBits=1)
    W, H = px(0.420), px(0.297)                          # A3 yatay
    sayfa = np.full((H, W), 255, np.uint8)
    y0, x0 = (H - img.shape[0]) // 2, (W - img.shape[1]) // 2
    sayfa[y0:y0 + img.shape[0], x0:x0 + img.shape[1]] = img
    cv2.putText(sayfa, f'ChArUco {a.sx}x{a.sy} kare {a.kare * 1000:.0f} mm isaret {a.isaret * 1000:.0f} mm DICT_5X5_100 '
                f'- %100 olcekle bas, kareyi olc', (x0, H - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
    cv2.imwrite(a.kayit, sayfa)
    print(f'{a.kayit}: {W}x{H} px (A3, {dpi} dpi), tahta {a.sx * a.kare * 100:.1f} x {a.sy * a.kare * 100:.1f} cm')


def gruplar(tespit, min_s, tol_m=0.01, tol_aci=1.0):
    """Ardışık tespitlerden hareketsiz tahta konumları: merkez < tol_m ve normal < tol_aci içinde, >= min_s."""
    out, cur = [], []
    for d in tespit:
        if cur and (np.linalg.norm(d['c'] - cur[0]['c']) > tol_m or
                    math.degrees(math.acos(min(1.0, abs(float(d['n'] @ cur[0]['n']))))) > tol_aci or
                    d['t'] - cur[-1]['t'] > 0.5):
            if cur[-1]['t'] - cur[0]['t'] >= min_s:
                out.append(cur)
            cur = []
        cur.append(d)
    if cur and cur[-1]['t'] - cur[0]['t'] >= min_s:
        out.append(cur)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kayit', help='kayıt dizini (ya da --uret ile çıktı PNG)')
    ap.add_argument('--uret', action='store_true')
    ap.add_argument('--sx', type=int, default=7)
    ap.add_argument('--sy', type=int, default=5)
    ap.add_argument('--kare', type=float, default=0.055, help='kare kenarı (m), basılıdan ölçülmüş')
    ap.add_argument('--isaret', type=float, default=0.040, help='ArUco işaret kenarı (m)')
    ap.add_argument('--her', type=int, default=3, help='her N. renk karesi')
    ap.add_argument('--min_s', type=float, default=1.5)
    ap.add_argument('--serbest', nargs='+', default=PARAM, choices=PARAM)
    ap.add_argument('--kalib', default=os.path.join(BURADA, '..', 'config', 'kamera_kalibrasyon_d435.yaml'))
    a = ap.parse_args()
    if a.uret:
        return uret(a)

    from rclpy.serialization import deserialize_message
    from scipy.optimize import least_squares
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from tf2_msgs.msg import TFMessage
    from bag2vlmaps import T, damga, goruntu, kamera_dis, okuyucu, q2R, rpy2R, zincir

    sb, kb = os.path.join(a.kayit, 'sensor'), os.path.join(a.kayit, 'kamera')
    taramalar, tfs, ci = [], {}, None
    for konu, veri, _ in okuyucu(sb, ['/scan_sabit', '/tf_static', '/camera/color/camera_info']):
        if konu == '/scan_sabit':
            m = deserialize_message(veri, LaserScan)
            r = np.array(m.ranges)
            aci = m.angle_min + np.arange(len(r)) * m.angle_increment
            k = np.isfinite(r) & (r > 0.1) & (r < 4.0)
            taramalar.append((damga(m.header), np.c_[r[k] * np.cos(aci[k]), r[k] * np.sin(aci[k]), np.zeros(k.sum())]))
        elif konu == '/tf_static':
            for tt in deserialize_message(veri, TFMessage).transforms:
                tr, q = tt.transform.translation, tt.transform.rotation
                tfs[(tt.header.frame_id, tt.child_frame_id)] = T(q2R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        elif ci is None:
            ci = deserialize_message(veri, CameraInfo)
    K, Dist = np.array(ci.k).reshape(3, 3), np.array(ci.d, float)
    T_b_l = tfs.get(('base_link', 'laser_duz'))
    if T_b_l is None:
        x, y, z, yaw = LIDAR_VARSAYILAN
        T_b_l = T(rpy2R(0, 0, yaw), [x, y, z])
        print('tf_static\'te base_link -> laser_duz yok: ortam.sh değerleri kullanıldı')
    T_cl_co = zincir(tfs, 'camera_link', 'camera_color_frame', 'camera_color_optical_frame')
    T_b_cl0 = kamera_dis(tfs, a.kalib)
    R0 = T_b_cl0[:3, :3]
    p0 = np.array([*T_b_cl0[:3, 3], math.atan2(R0[2, 1], R0[2, 2]), -math.asin(R0[2, 0]), math.atan2(R0[1, 0], R0[0, 0])])

    det = cv2.aruco.CharucoDetector(tahta(a))
    brd = tahta(a)
    tespit, say = [], 0
    for _, veri, _ in okuyucu(kb, ['/camera/color/image_raw']):
        say += 1
        if say % a.her:
            continue
        m = deserialize_message(veri, Image)
        gri = cv2.cvtColor(goruntu(m), cv2.COLOR_BGR2GRAY)
        kc, ki, _, _ = det.detectBoard(gri)
        if ki is None or len(ki) < 8:
            continue
        nesne, resim = brd.matchImagePoints(kc, ki)
        ok, rv, tv = cv2.solvePnP(nesne, resim, K, Dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        hata = float(np.sqrt(np.mean((cv2.projectPoints(nesne, rv, tv, K, Dist)[0] - resim) ** 2)))
        R = cv2.Rodrigues(rv)[0]
        n = R[:, 2]                                       # tahta normali (kamera çerçevesi)
        c = R @ np.array([a.sx * a.kare / 2, a.sy * a.kare / 2, 0]) + tv.ravel()
        tespit.append({'t': damga(m.header), 'R': R, 'tv': tv.ravel(), 'n': n, 'c': c, 'hata': hata, 'kose': len(ki)})
    print(f'{say} renk karesi, {len(tespit)} tahta tespiti')
    G = gruplar(tespit, a.min_s)
    print(f'{len(G)} hareketsiz tahta konumu')
    if len(G) < 3:
        raise SystemExit('en az 3 farklı konum gerekli')

    konumlar = []
    for g in G:
        t1, t2 = g[0]['t'], g[-1]['t']
        b = min(g, key=lambda d: d['hata'])
        n, R, tv = np.median([d['n'] for d in g], 0), b['R'], np.median([d['tv'] for d in g], 0)
        n /= np.linalg.norm(n)
        d = float(n @ tv)
        P = np.vstack([p for ts, p in taramalar if t1 <= ts <= t2] or [np.zeros((0, 3))])
        konumlar.append({'t': (t1, t2), 'n': n, 'd': d, 'R': R, 'tv': tv, 'P': P, 'hata_px': b['hata'],
                         'mesafe': float(np.linalg.norm(b['c'])), 'egim': math.degrees(math.acos(min(1, abs(n[1]))))})

    def T_co_l(p):
        T_b_co = T(rpy2R(*p[3:]), p[:3]) @ T_cl_co
        return np.linalg.inv(T_b_co) @ T_b_l

    def sec(p, pay=0.01):
        """her konumda tahta üstündeki lidar noktaları (tahta dikdörtgeni içinde, düzleme < 5 cm)"""
        M = T_co_l(p)
        for k in konumlar:
            Q = (M[:3, :3] @ k['P'].T).T + M[:3, 3]
            yerel = (Q - k['tv']) @ k['R']                 # tahta çerçevesi (x, y düzlemde, z normal)
            ic = ((yerel[:, 0] > pay) & (yerel[:, 0] < a.sx * a.kare - pay) & (yerel[:, 1] > pay) &
                  (yerel[:, 1] < a.sy * a.kare - pay) & (np.abs(yerel[:, 2]) < 0.05))
            k['sec'] = k['P'][ic]

    serbest = [PARAM.index(s) for s in a.serbest]

    def artik(q, p_tam):
        p = p_tam.copy()
        p[serbest] = q
        M = T_co_l(p)
        r = []
        for k in konumlar:
            if len(k['sec']) < 5:
                continue
            Q = (M[:3, :3] @ k['sec'].T).T + M[:3, 3]
            r.append((Q @ k['n'] - k['d']) / math.sqrt(len(k['sec'])))   # her konum eşit ağırlıkta
        return np.concatenate(r)

    p = p0.copy()
    for tur in range(3):                                   # nokta seçimi <-> uydurma
        sec(p, pay=0.02 if tur == 0 else 0.01)
        gecerli = sum(len(k['sec']) >= 5 for k in konumlar)
        if gecerli < 3:
            raise SystemExit(f'lidar noktası olan konum {gecerli} < 3 (lidar düzlemi tahtayı kesiyor mu? başlangıç TF çok mu kötü?)')
        s = least_squares(artik, p[serbest], args=(p,), loss='huber', f_scale=0.005)
        p[serbest] = s.x
    sv = np.linalg.svd(s.jac, compute_uv=False)
    kosul = float(sv[0] / max(sv[-1], 1e-12))

    def konum_rms(pp):
        M = T_co_l(pp)
        out = []
        for k in konumlar:
            if len(k['sec']) < 5:
                out.append(None)
                continue
            Q = (M[:3, :3] @ k['sec'].T).T + M[:3, 3]
            out.append(float(np.sqrt(np.mean((Q @ k['n'] - k['d']) ** 2))))
        return out
    once, sonra = konum_rms(p0), konum_rms(p)
    print(f'{"konum":>5} {"mesafe_m":>8} {"egim°":>6} {"px":>5} {"lidar_n":>7} {"önce_mm":>8} {"sonra_mm":>8}')
    tablo = []
    for i, (k, r0, r1) in enumerate(zip(konumlar, once, sonra)):
        print(f'{i:5d} {k["mesafe"]:8.2f} {k["egim"]:6.1f} {k["hata_px"]:5.2f} {len(k["sec"]):7d} '
              f'{(r0 or 0) * 1000:8.1f} {(r1 or 0) * 1000:8.1f}')
        tablo.append({'mesafe_m': round(k['mesafe'], 3), 'dikeyden_egim_derece': round(k['egim'], 1),
                      'yeniden_izdusum_px': round(k['hata_px'], 2), 'lidar_nokta': len(k['sec']),
                      'artik_once_mm': None if r0 is None else round(r0 * 1000, 1),
                      'artik_sonra_mm': None if r1 is None else round(r1 * 1000, 1)})
    fark = p - p0
    print(f'serbest: {a.serbest}, Jacobian koşul sayısı {kosul:.0f} (> 1e3: bazı serbestlikler gözlenemiyor, eğim çeşitliliği artır)')
    print('base_link -> camera_link   eski -> yeni (fark)')
    for i, ad in enumerate(PARAM):
        if i < 3:
            print(f'  {ad:>5}: {p0[i]:+.4f} -> {p[i]:+.4f} m   ({fark[i] * 100:+.1f} cm)')
        else:
            print(f'  {ad:>5}: {math.degrees(p0[i]):+.2f} -> {math.degrees(p[i]):+.2f}°   ({math.degrees(fark[i]):+.2f}°)')
    sonuc = {'base_link_to_camera_link': {'parent': 'base_link', 'child': 'camera_link',
                                          'translation_m': [round(v, 4) for v in p[:3]],
                                          'rotation_rpy_rad': [round(v, 5) for v in p[3:]]},
             'eski': {'translation_m': [round(v, 4) for v in p0[:3]], 'rotation_rpy_rad': [round(v, 5) for v in p0[3:]]},
             'fark_cm_derece': {ad: round(fark[i] * 100 if i < 3 else math.degrees(fark[i]), 2) for i, ad in enumerate(PARAM)},
             'serbest': a.serbest, 'kosul_sayisi': round(kosul, 1),
             'tahta': {'kare_m': a.kare, 'isaret_m': a.isaret, 'boyut': [a.sx, a.sy], 'sozluk': 'DICT_5X5_100'},
             'lidar_referans': 'base_link -> laser_duz (tf_static ya da ortam.sh)', 'konumlar': tablo}
    with open(os.path.join(a.kayit, 'charuco_kalib.yaml'), 'w') as f:
        yaml.safe_dump(json.loads(json.dumps(sonuc, default=float)), f, allow_unicode=True, sort_keys=False)


if __name__ == '__main__':
    main()
