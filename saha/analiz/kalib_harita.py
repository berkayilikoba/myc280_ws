#!/usr/bin/env python3
"""Kamera dış kalibrasyonu düzeltmesi (base_link -> kamera: dx, dy, dyaw) haritaya göre, kayıttan (robota dokunmadan).

Girdi: bag2vlmaps.py sahnesi, lidar_poz.py pozuyla (harita çerçevesi) üretilmiş: poses.txt (gövde), poses_kamera.txt
(renk optik), depth/*.npy (renge hizalı, m). Kamera->gövde dönüşümü sahneden: T_bc = T_wb^-1 · T_wc (sabit).
Her N. karede yerden 10-50 cm'deki derinlik noktaları; düzeltme ΔT (gövde çerçevesinde x, y ileri/sol, yaw) için
noktaların haritadaki en yakın duvara uzaklığının (≤ 0.25 m kırpılmış) ortalaması ızgarada en aza indirilir.
Lidar dış kalibrasyonu ve harita lidardan geldiği için sonuç lidara göreli kamera kalibrasyonudur.

Kullanım (Humble): kalib_harita.py <sahne> <harita.yaml> [--her 10]
"""

import argparse
import math
import os

import numpy as np
from scipy.spatial import cKDTree

from lidar_poz import harita_noktalari


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T(s):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = q2R(*s[3:7]), s[:3]
    return M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sahne')
    ap.add_argument('harita')
    ap.add_argument('--her', type=int, default=10)
    a = ap.parse_args()
    import yaml
    K = np.array(yaml.safe_load(open(os.path.join(a.sahne, 'vlmaps_config.yaml')))['cam_calib_mat']).reshape(3, 3)
    PB, PC = np.loadtxt(os.path.join(a.sahne, 'poses.txt')), np.loadtxt(os.path.join(a.sahne, 'poses_kamera.txt'))
    agac = cKDTree(harita_noktalari(a.harita))
    T_bc = np.linalg.inv(T(PB[0])) @ T(PC[0])
    gov, Twb = [], []                                   # gövde çerçevesinde noktalar + gövde pozu (2B)
    for i in range(0, len(PB), a.her):
        d = np.load(os.path.join(a.sahne, 'depth', f'{i:06d}.npy'))
        v, u = np.nonzero((d > 0.2) & (d < 3.0))
        if len(v) > 3000:
            k = np.random.default_rng(i).choice(len(v), 3000, replace=False)
            v, u = v[k], u[k]
        z = d[v, u]
        Pc = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)])
        Pb = (T_bc @ Pc)[:3]
        Wb = T(PB[i])
        h = (Wb @ np.vstack([Pb, np.ones(Pb.shape[1])]))[2]
        m = (h > 0.10) & (h < 0.50)                     # duvar/mobilya bandı (zemin ve tavan değil)
        gov.append(Pb[:2, m].T)
        Twb.append((Wb[0, 3], Wb[1, 3], math.atan2(Wb[1, 0], Wb[0, 0])))
    print(f'{len(gov)} kare, {sum(len(g) for g in gov)} nokta')

    def maliyet(dx, dy, dyaw):
        c, s = math.cos(dyaw), math.sin(dyaw)
        top, say, yakin = 0.0, 0, 0
        for g, (x, y, t) in zip(gov, Twb):
            q = np.c_[c * g[:, 0] - s * g[:, 1] + dx, s * g[:, 0] + c * g[:, 1] + dy]
            ct, st = math.cos(t), math.sin(t)
            w = np.c_[ct * q[:, 0] - st * q[:, 1] + x, st * q[:, 0] + ct * q[:, 1] + y]
            dd, _ = agac.query(w)
            top += np.minimum(dd, 0.25).sum()
            say += len(dd)
            yakin += (dd < 0.05).sum()
        return top / say, yakin / say

    m0 = maliyet(0, 0, 0)
    en = (m0[0], 0.0, 0.0, 0.0)
    for dyaw in np.radians(np.arange(-3, 3.01, 0.5)):
        for dx in np.arange(-0.10, 0.101, 0.01):
            for dy in np.arange(-0.04, 0.041, 0.01):
                c = maliyet(dx, dy, dyaw)[0]
                if c < en[0]:
                    en = (c, dx, dy, dyaw)
    m1 = maliyet(*en[1:])
    print(f'düzeltmesiz: ortalama uzaklık {m0[0] * 100:.2f} cm, <5 cm %{100 * m0[1]:.1f}')
    print(f'en iyi düzeltme: dx {en[1] * 100:+.0f} cm, dy {en[2] * 100:+.0f} cm, dyaw {math.degrees(en[3]):+.1f}° -> '
          f'ortalama {m1[0] * 100:.2f} cm, <5 cm %{100 * m1[1]:.1f}')
    # dx duyarlılığı (dy, dyaw en iyide)
    print('dx taraması: ' + ', '.join(f'{dx * 100:+.0f}:{maliyet(dx, en[2], en[3])[0] * 100:.2f}'
                                       for dx in np.arange(-0.10, 0.101, 0.02)))


if __name__ == '__main__':
    main()
