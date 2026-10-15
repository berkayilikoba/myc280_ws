#!/usr/bin/env python3
"""Dönüş hızına göre derinlik-renk hizası (donus_testi.py kaydının bag2vlmaps sahnesi).

Her kare: gövde açısal hızı (/odom_duz, renk damgasında), renk-derinlik damga farkı (zaman.txt) ve hiza ölçütü
(vlmaps_dogrula.py 1. kontrolü: derinlik süreksizliğinin en yakın renk kenarına uzaklığı, px). Açısal hız
aralıklarına göre medyanlar. Hareket telafisi doğruysa hiza hızla bozulmamalı; bozuluyorsa zaman damgası
gecikmesi / bulanıklık / rolling shutter etkisi.

Kullanım (Humble): donus_analiz.py <sahne> <sensor_bag> [--araliklar 0,0.1,0.45,1.2]
"""

import argparse
import math
import os

import cv2
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

from bag2vlmaps import damga, okuyucu


def hiza(rgb_gri, d):
    gecerli = cv2.erode((d > 0).astype(np.uint8), np.ones((5, 5))) > 0
    dmax = cv2.dilate(d, np.ones((3, 3)))
    dmin = cv2.erode(np.where(d > 0, d, 99).astype(np.float32), np.ones((3, 3)))
    dk = gecerli & (dmax - dmin > 0.15 * dmin) & (dmax - dmin > 0.10)
    rk = cv2.Canny(cv2.GaussianBlur(rgb_gri, (5, 5), 0), 50, 150)
    if dk.sum() < 200 or rk.sum() == 0:
        return None
    uz = cv2.distanceTransform(1 - (rk > 0).astype(np.uint8), cv2.DIST_L2, 3)
    return float(np.median(uz[dk])), float(np.mean(uz[dk] <= 3)), float(cv2.Laplacian(rgb_gri, cv2.CV_64F).var())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sahne')
    ap.add_argument('sensor_bag')
    ap.add_argument('--araliklar', default='0,0.1,0.45,1.2', help='|w| aralık sınırları (rad/s)')
    a = ap.parse_args()
    od = []
    for _, v, _ in okuyucu(a.sensor_bag, ['/odom_duz']):
        m = deserialize_message(v, Odometry)
        q = m.pose.pose.orientation
        od.append((damga(m.header), math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
    od = np.array(sorted(od))
    w = np.gradient(np.unwrap(od[:, 1]), od[:, 0])
    w = np.convolve(w, np.ones(5) / 5, mode='same')                  # 0.1 s yumuşatma
    zt = np.loadtxt(os.path.join(a.sahne, 'zaman.txt'))
    sat = []
    for n, tc, td in zt:
        rgb = cv2.imread(os.path.join(a.sahne, 'rgb', f'{int(n):06d}.png'), cv2.IMREAD_GRAYSCALE)
        d = np.load(os.path.join(a.sahne, 'depth', f'{int(n):06d}.npy'))
        h = hiza(rgb, d)
        if h:
            sat.append((abs(np.interp(tc, od[:, 0], w)), (tc - td) * 1000, *h))
    sat = np.array(sat)
    sinir = [float(x) for x in a.araliklar.split(',')]
    print(f'{"|w| rad/s":>12} {"kare":>5} {"hiza_px":>8} {"<=3px":>6} {"dt_ms":>6} {"keskinlik":>10}')
    for lo, hi in zip(sinir[:-1], sinir[1:]):
        s = sat[(sat[:, 0] >= lo) & (sat[:, 0] < hi)]
        if len(s) < 5:
            continue
        print(f'{lo:5.2f}-{hi:<5.2f} {len(s):6d} {np.median(s[:, 2]):8.2f} {np.median(s[:, 3]):6.2f} '
              f'{np.median(np.abs(s[:, 1])):6.1f} {np.median(s[:, 4]):10.0f}')
    np.savetxt(os.path.join(a.sahne, 'donus_analiz.csv'), sat, delimiter=',', fmt='%.4f',
               header='w_abs_rad_s,dt_renk_derinlik_ms,hiza_medyan_px,hiza_le3px_oran,renk_keskinlik_laplas_var')


if __name__ == '__main__':
    main()
