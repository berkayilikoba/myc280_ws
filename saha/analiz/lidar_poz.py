#!/usr/bin/env python3
"""Lidar ile düzeltilmiş gövde pozu: her taramayı haritaya ICP ile oturtur, odom kaymasını giderir (VLMaps pozu).

Odom (/odom_duz) yerinde dönüşlerde yön kaydırıyor (13-14.10 kapsama turları: 4x360° sonra ~6°) ve hızlı sürüşte
tekerlek kayması görmüyor (0.30 m/s: 6 cm). Burada:
  1) Harita (map_server yaml + pgm) dolu hücreleri -> 2B nokta kümesi (KD-ağacı).
  2) Her /scan_sabit taraması (laser_duz; base_link'e göre --lidar x y yaw) için başlangıç tahmini =
     önceki düzeltilmiş poz ∘ odom artışı; noktadan noktaya 2B ICP (yakınlık eşiği 0.3 -> 0.08 m).
  3) Düzeltme C(t) = T_icp(t) · T_odom(t)^-1, taramalar arasında doğrusal ara değer; çıktı her odom örneğinde
     T(t) = C(t) · T_odom(t).
Dünya = harita çerçevesi; robot başlangıç işaretinde ve odom sıfırken haritanın orijinine oturur (poz_ver.sh ile aynı
varsayım). Başlangıç farklıysa --bas x y yaw_derece.

Çıktı CSV (t, x, y, yaw, uyum): bag2vlmaps.py --poz_dosyasi ile kullanılır. Uyum = eşleşen nokta oranı (<0.08 m).
Kullanım (Humble): lidar_poz.py <sensor_bag> <harita.yaml> <cikti.csv> [--lidar 0.137 0.015 -9.66]
"""

import argparse
import math
import os

import cv2
import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

from bag2vlmaps import damga, okuyucu


def harita_noktalari(yol):
    h = yaml.safe_load(open(yol))
    img = cv2.imread(os.path.join(os.path.dirname(yol), h['image']), cv2.IMREAD_UNCHANGED)
    p = (255 - img.astype(float)) / 255.0 if not h.get('negate', 0) else img.astype(float) / 255.0
    v, u = np.nonzero(p > h['occupied_thresh'])
    r, (ox, oy, _) = h['resolution'], h['origin']
    return np.c_[ox + (u + 0.5) * r, oy + (img.shape[0] - v - 0.5) * r]   # pgm satır 0 = üst (y büyük)


def T2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def icp(P, agac, M, T0, esikler=(0.3, 0.2, 0.12, 0.08), it=8):
    T = T0.copy()
    for e in esikler:
        for _ in range(it):
            Q = P @ T[:2, :2].T + T[:2, 2]
            d, j = agac.query(Q, distance_upper_bound=e)
            k = np.isfinite(d)
            if k.sum() < 20:
                return T, 0.0
            A, B = Q[k], M[j[k]]
            ca, cb = A.mean(0), B.mean(0)
            U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
            R = (U @ Vt).T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1
                R = (U @ Vt).T
            dT = np.eye(3)
            dT[:2, :2], dT[:2, 2] = R, cb - R @ ca
            T = dT @ T
            if abs(math.atan2(R[1, 0], R[0, 0])) < 1e-4 and np.linalg.norm(dT[:2, 2]) < 1e-4:
                break
    Q = P @ T[:2, :2].T + T[:2, 2]
    d, _ = agac.query(Q, distance_upper_bound=0.08)
    return T, float(np.isfinite(d).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sensor_bag')
    ap.add_argument('harita')
    ap.add_argument('cikti')
    ap.add_argument('--lidar', type=float, nargs=3, default=[0.137, 0.015, -9.66], help='base_link -> lidar x y yaw(°)')
    ap.add_argument('--bas', type=float, nargs=3, default=[0, 0, 0], help='odom sıfırının haritadaki pozu x y yaw(°)')
    ap.add_argument('--konu', default='/scan_sabit')
    a = ap.parse_args()
    M = harita_noktalari(a.harita)
    agac = cKDTree(M)
    od, sc = [], []
    for konu, v, _ in okuyucu(a.sensor_bag, ['/odom_duz', a.konu]):
        if konu == '/odom_duz':
            m = deserialize_message(v, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            od.append((damga(m.header), p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
        else:
            m = deserialize_message(v, LaserScan)
            r = np.array(m.ranges)
            ang = m.angle_min + np.arange(len(r)) * m.angle_increment
            k = np.isfinite(r) & (r > 0.25) & (r < 8)
            sc.append((damga(m.header), np.c_[r[k] * np.cos(ang[k]), r[k] * np.sin(ang[k])]))
    O = np.array(sorted(od))
    O[:, 3] = np.unwrap(O[:, 3])
    TL = T2(a.lidar[0], a.lidar[1], math.radians(a.lidar[2]))
    Tbas = T2(a.bas[0], a.bas[1], math.radians(a.bas[2]))

    def odom(t):
        return T2(*(np.interp(t, O[:, 0], O[:, i]) for i in (1, 2, 3)))

    sat, C = [], Tbas.copy()
    for t, pts in sc:
        if t < O[0, 0] or t > O[-1, 0] or len(pts) < 50:
            continue
        To = odom(t)
        T0 = C @ To                                    # önceki düzeltme ile tahmin
        P = pts @ TL[:2, :2].T + TL[:2, 2]             # base_link'te tarama
        T, uyum = icp(P, agac, M, T0)
        if uyum > 0.5:
            C = T @ np.linalg.inv(To)
        sat.append((t, *C[:2, 2], math.atan2(C[1, 0], C[0, 0]), uyum))
    S = np.array(sat)
    S[:, 3] = np.unwrap(S[:, 3])
    out = []
    for t, x, y, yw in O:
        Ci = T2(*(np.interp(t, S[:, 0], S[:, i]) for i in (1, 2, 3)))
        T = Ci @ T2(x, y, yw)
        out.append((t, T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0]), np.interp(t, S[:, 0], S[:, 4])))
    out = np.array(out)
    np.savetxt(a.cikti, out, delimiter=',', fmt='%.6f', header='t,x,y,yaw,uyum')
    d0, dn = S[0], S[-1]
    print(f'{len(S)} tarama, uyum medyan {np.median(S[:, 4]):.2f} (min {S[:, 4].min():.2f}, <0.5: {(S[:, 4] < 0.5).sum()})')
    print(f'düzeltme başta: {d0[1] * 100:+.1f} {d0[2] * 100:+.1f} cm {math.degrees(d0[3]):+.1f}°; '
          f'sonda: {dn[1] * 100:+.1f} {dn[2] * 100:+.1f} cm {math.degrees(dn[3]):+.1f}° (odom kayması)')
    e = out[-1] - out[0]
    print(f'düzeltilmiş poz başlangıca dönüş: {math.hypot(e[1], e[2]) * 100:.1f} cm, {math.degrees(e[3]):+.1f}°')


if __name__ == '__main__':
    main()
