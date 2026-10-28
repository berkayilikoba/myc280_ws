#!/usr/bin/env python3
"""myCobot 280'in LIMO üzerindeki erişim alanı (URDF'ten ileri kinematik, örnekleme): nesne yüksekliğine göre robotun
önünde kolun ulaşabildiği yatay mesafe aralığı -> kavrama için yaklaşma mesafesi (yaklasim.py --erisim).

Robot çerçevesi base_footprint (zemin, z=0; x ileri). Eklem açıları limitler içinde düzgün örneklenir; flanş konumu ve
yaklaşma ekseni (flanş z) kaydedilir. Kavrama türleri: 'ust' (flanş ekseni aşağı, düşeyle < --aci_tol), 'yan' (flanş
ekseni yatay, yataydan < --aci_tol ve ileri bakıyor). Her yükseklik bandında ve robot önündeki ±--sektor° içinde
erişilen yatay uzaklıkların %5-%95 aralığı ve önerilen kavrama mesafesi (medyan) verilir.
Not: URDF'teki kol tabanı (base_link -> joint1, -0.01 0 0) ve tutucu boyu doğrulanmadı; --tutucu ile flanştan
parmak ucuna uzaklık eklenir.
Kullanım: kol_erisim.py [--urdf limo_mycobot.urdf] [--ornek 200000] [--tutucu 0.10] [--cikti ../config/kol_erisim.yaml]
"""

import argparse
import math
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def eksen_R(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def zincir(urdf, kok, uc):
    J = {}
    for j in ET.fromstring(urdf).findall('joint'):
        J[j.find('child').get('link')] = j
    yol, l = [], uc
    while l != kok:
        j = J[l]
        yol.append(j)
        l = j.find('parent').get('link')
    return yol[::-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', help='genişletilmiş URDF; yoksa xacro limo_mycobot.xacro kol_kilit:=false')
    ap.add_argument('--ornek', type=int, default=200000)
    ap.add_argument('--tutucu', type=float, default=0.10, help='flanştan kavrama noktasına (m), ölçülmeli')
    ap.add_argument('--aci_tol', type=float, default=25.0)
    ap.add_argument('--sektor', type=float, default=15.0)
    ap.add_argument('--on_kenar', type=float, default=0.17, help='gövde önü + pay (m); LIMO ayak izi x 0.161')
    ap.add_argument('--cikti')
    a = ap.parse_args()
    if a.urdf:
        urdf = open(a.urdf).read()
    else:
        yol = subprocess.run(['ros2', 'pkg', 'prefix', 'limo_description'], capture_output=True, text=True).stdout.strip()
        urdf = subprocess.run(['xacro', f'{yol}/share/limo_description/urdf/limo_mycobot.xacro', 'kol_kilit:=false'],
                              capture_output=True, text=True, check=True).stdout
    ch = zincir(urdf, 'base_footprint', 'joint6_flange')
    rng = np.random.default_rng(0)
    N = a.ornek
    T = np.tile(np.eye(4), (N, 1, 1))
    for j in ch:
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]
        r = [float(v) for v in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()]
        S = np.eye(4)
        S[:3, :3], S[:3, 3] = rpy2R(*r), xyz
        T = T @ S
        if j.get('type') in ('revolute', 'continuous'):
            lim = j.find('limit')
            lo, hi = (float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else (-math.pi, math.pi)
            ax = [float(v) for v in (j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0').split()]
            q = rng.uniform(lo, hi, N)
            Rq = np.stack([eksen_R(ax, t) for t in q])
            M = np.tile(np.eye(4), (N, 1, 1))
            M[:, :3, :3] = Rq
            T = T @ M
    z_ax = T[:, :3, 2]                                       # flanş yaklaşma ekseni
    p = T[:, :3, 3] + a.tutucu * z_ax                        # kavrama noktası
    kok = T[0, :3, 3] * 0                                     # base_footprint orijini
    yatay = np.hypot(p[:, 0], p[:, 1])
    yon = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    ust = np.degrees(np.arccos(np.clip(-z_ax[:, 2], -1, 1))) < a.aci_tol
    yan = (np.abs(np.degrees(np.arcsin(np.clip(z_ax[:, 2], -1, 1)))) < a.aci_tol) & \
        (np.sum(z_ax[:, :2] * p[:, :2], 1) > 0.5 * yatay * np.hypot(z_ax[:, 0], z_ax[:, 1]))
    on = (np.abs(yon) < a.sektor) & (p[:, 0] > a.on_kenar)          # gövdenin önünde (içi çarpışma)
    sonuc = {'tutucu_m': a.tutucu, 'sektor_derece': a.sektor, 'aci_tol_derece': a.aci_tol, 'on_kenar_m': a.on_kenar,
             'kol_tabani_base_footprint': [round(float(v), 4) for v in T[0, :3, 3] * 0 + kok], 'bantlar': []}
    print(f'{"yükseklik":>12} {"tür":>4} {"örnek":>6} {"yatay %5":>9} {"%95":>6} {"medyan":>7}')
    for z0, z1 in ((0.0, 0.05), (0.05, 0.15), (0.15, 0.30), (0.30, 0.45), (0.45, 0.60), (0.60, 0.80)):
        for ad, m in (('ust', ust), ('yan', yan)):
            k = on & m & (p[:, 2] >= z0) & (p[:, 2] < z1)
            if k.sum() < 30:
                print(f'{z0:.2f}-{z1:.2f} m {ad:>4} {int(k.sum()):6d}  erişilemez')
                sonuc['bantlar'].append({'z_m': [z0, z1], 'tur': ad, 'ornek': int(k.sum()), 'erisilir': False})
                continue
            q5, q50, q95 = np.percentile(yatay[k], [5, 50, 95])
            print(f'{z0:.2f}-{z1:.2f} m {ad:>4} {int(k.sum()):6d} {q5:9.3f} {q95:6.3f} {q50:7.3f}')
            sonuc['bantlar'].append({'z_m': [z0, z1], 'tur': ad, 'ornek': int(k.sum()), 'erisilir': True,
                                     'yatay_m_p5_p50_p95': [round(float(q5), 3), round(float(q50), 3), round(float(q95), 3)]})
    if a.cikti:
        open(a.cikti, 'w').write('# kol_erisim.py çıktısı: myCobot 280 kavrama noktası erişimi, base_footprint çerçevesi\n'
                                 + yaml.safe_dump(sonuc, allow_unicode=True, sort_keys=False))


if __name__ == '__main__':
    main()
