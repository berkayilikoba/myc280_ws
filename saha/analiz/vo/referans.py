#!/usr/bin/env python3
"""Referans yörünge: her lidar taraması haritaya 2B nokta-doğru ICP ile oturtulur (harita çerçevesi, tarama anları).

lidar_poz.py'den farkı: nokta-nokta yerine nokta-doğru (harita hücreleri 5 cm; normaller 8 komşulu PCA), Huber
ağırlık, başlangıç pozu ızgara aramasıyla bulunur, tahmin EKF-eşdeğeri ölü hesaptan (her koşuda /odom + /imu var).
Çıktı bags/vo_calisma/<koşu>/referans.csv: t, x, y, yaw, uyum (8 cm içinde eşleşen nokta oranı), rms_cm.

Kullanım: referans.py [koşu ...]
"""

import math
import sys

import numpy as np
from scipy.spatial import cKDTree

from ortak import CALISMA, HARITA, KOSULAR, LIDAR, ekf_esdeger, ara, se2, sensor_oku, unse2
from lidar_poz import harita_noktalari


def normaller(M, k=8):
    _, j = cKDTree(M).query(M, k)
    N = np.zeros_like(M)
    for i, ji in enumerate(j):
        Q = M[ji] - M[ji].mean(0)
        w, v = np.linalg.eigh(Q.T @ Q)
        N[i] = v[:, 0]
    return N


def icp_dogru(P, agac, M, N, T0, esikler=(0.3, 0.15, 0.08, 0.05), it=10):
    Tm = T0.copy()
    for e in esikler:
        for _ in range(it):
            Q = P @ Tm[:2, :2].T + Tm[:2, 2]
            d, j = agac.query(Q, distance_upper_bound=e)
            k = np.isfinite(d)
            if k.sum() < 30:
                return Tm, 0.0, 99.0
            q, m, n = Q[k], M[j[k]], N[j[k]]
            r = np.sum((q - m) * n, 1)
            w = np.minimum(1.0, 0.02 / np.maximum(np.abs(r), 1e-9))          # Huber 2 cm
            # r + n·[dx, dy] + dθ (n × q) ≈ 0   (küçük açı, dönme merkezi orijin)
            A = np.c_[n, n[:, 1] * q[:, 0] - n[:, 0] * q[:, 1]]
            x = np.linalg.lstsq(A * w[:, None] ** 0.5, -r * w ** 0.5, rcond=None)[0]
            Tm = se2(x[0], x[1], x[2]) @ Tm
            if np.abs(x).max() < 1e-5:
                break
    Q = P @ Tm[:2, :2].T + Tm[:2, 2]
    d, j = agac.query(Q, distance_upper_bound=0.08)
    k = np.isfinite(d)
    rms = math.sqrt(np.mean(np.sum((Q[k] - M[j[k]]) * N[j[k]], 1) ** 2)) * 100 if k.any() else 99.0
    return Tm, float(k.mean()), rms


def main():
    adlar = sys.argv[1:] or list(KOSULAR)
    M = harita_noktalari(HARITA)
    N = normaller(M)
    agac = cKDTree(M)
    TL = se2(*LIDAR)
    for ad in adlar:
        S = sensor_oku(ad)
        E = ekf_esdeger(S)
        t0 = S['tara_t']
        P0 = S['tara'][0] @ TL[:2, :2].T + TL[:2, 2]
        en = None
        for x in np.arange(-0.3, 0.31, 0.1):
            for y in np.arange(-0.3, 0.31, 0.1):
                for a in np.radians(np.arange(-20, 21, 5)):
                    Tm, u, r = icp_dogru(P0, agac, M, N, se2(x, y, a))
                    if en is None or u > en[1] + 1e-3 or (abs(u - en[1]) <= 1e-3 and r < en[2]):
                        en = (Tm, u, r)
        print(f'{ad}: başlangıç {np.round(np.array(unse2(en[0])) * [100, 100, 180 / math.pi], 1)} '
              f'(cm, cm, °) uyum {en[1]:.2f} rms {en[2]:.1f} cm')
        C = en[0] @ np.linalg.inv(se2(*ara(E, t0[0])[:3]))
        out = []
        for t, pts in zip(t0, S['tara']):
            if t < E[0, 0] or t > E[-1, 0] or len(pts) < 50:
                continue
            To = se2(*ara(E, t)[:3])
            P = pts @ TL[:2, :2].T + TL[:2, 2]
            Tm, u, r = icp_dogru(P, agac, M, N, C @ To)
            if u > 0.6:
                C = Tm @ np.linalg.inv(To)
            out.append((t, *unse2(Tm), u, r))
        R = np.array(out)
        R[:, 3] = np.unwrap(R[:, 3])
        np.savetxt(f'{CALISMA}/{ad}/referans.csv', R, delimiter=',', fmt='%.6f', header='t,x,y,yaw,uyum,rms_cm')
        iyi = R[:, 4] > 0.6
        print(f'  {len(R)} tarama, uyum medyan {np.median(R[:, 4]):.2f}, <0.6: {(~iyi).sum()}, '
              f'rms medyan {np.median(R[iyi, 5]):.1f} cm; bitiş {R[-1, 1] * 100:+.1f} {R[-1, 2] * 100:+.1f} cm '
              f'{math.degrees(R[-1, 3] - R[0, 3]):+.1f}° (başlangıca göre yaw)')


if __name__ == '__main__':
    main()
