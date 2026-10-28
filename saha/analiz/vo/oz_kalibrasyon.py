#!/usr/bin/env python3
"""Kamera düzlem dış kalibrasyonunun (x, y) VO'dan öz kalibrasyonu — lidar/referans kullanmadan.

Kısıt: yerinde dönüşte (IMU |wz| > --wz, tekerlek |vx| < --vx) gövde ötelenmez. Kamera konumu base_link'e göre
Δ = (dx, dy) yanlışsa VO her dönüşte (I - R(θ))·Δ kadar sahte öteleme üretir. Düzlem çözümünde kamera hareketi
dış kalibrasyondan bağımsız olduğundan düzeltilmiş gövde yörüngesi W_b'(t) = W_b(t) · se2(dx, dy, 0)^-1 ile kapalı
biçimde hesaplanır (yeniden VO koşturmadan). Her dönüş aralığının uç pozları arasındaki öteleme 0 olacak şekilde
(dx, dy) en küçük kareler. Sonuç yörünge yorunge/<vo>_ozkal.csv olarak yazılır.

Kullanım: oz_kalibrasyon.py <koşu> <vo_yorunge_adi> [--kalib_kosu koşu]   (başka koşudan kalibrasyonu uygulamak için)
"""

import argparse
import os

import numpy as np

from ortak import CALISMA, KOSULAR, kaydet, se2, sensor_oku, unse2


def donusler(S, wz_esik, vx_esik, pay=0.5):
    I, O = S['imu'], S['odom']
    w = np.abs(I[:, 1]) > wz_esik
    d = np.diff(w.astype(int))
    bas, son = I[np.nonzero(d == 1)[0] + 1, 0], I[np.nonzero(d == -1)[0], 0]
    out = []
    for a in bas:
        b = son[son > a]
        if not len(b) or b[0] - a < 1.0:
            continue
        b = b[0]
        tt = np.linspace(a - pay, b + pay, 50)
        if np.abs(np.interp(tt, O[:, 0], O[:, 4])).max() < vx_esik:
            out.append((a - pay, b + pay))
    return out


def duzelt(V, dx, dy):
    D = np.linalg.inv(se2(dx, dy, 0))
    M0 = np.linalg.inv(se2(*V[0, 1:4]) @ D)
    A = np.array([(t, *unse2(M0 @ se2(x, y, a) @ D)) for t, x, y, a in V])
    A[:, 3] = np.unwrap(A[:, 3])
    return A


def kalibre(S, V, wz, vx):
    """Doğrusal: dönüş uçlarında W_i^-1 W_j = (R_ij, t_ij); düzeltilmiş öteleme t_ij + (I - R_ij)·Δ = 0."""
    A, b, n = [], [], 0
    for a, c in donusler(S, wz, vx):
        i, j = np.searchsorted(V[:, 0], [a, c])
        if j >= len(V) or i >= j:
            continue
        M = np.linalg.inv(se2(*V[i, 1:4])) @ se2(*V[j, 1:4])
        R, t = M[:2, :2], M[:2, 2]
        # D^-1 M D, D = se2(-Δ): öteleme t' = t + (I - R)·Δ  (Δ = kamera kayması)
        A.append(np.eye(2) - R)
        b.append(-t)
        n += 1
    A, b = np.vstack(A), np.concatenate(b)
    d = np.linalg.lstsq(A, b, rcond=None)[0]
    r0 = np.linalg.norm(b.reshape(-1, 2), axis=1)
    r1 = np.linalg.norm((A @ d - b).reshape(-1, 2), axis=1)
    return d, n, r0, r1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kosu', choices=list(KOSULAR))
    ap.add_argument('vo')
    ap.add_argument('--kalib_kosu', help='kalibrasyonu bu koşudan al (çapraz doğrulama)')
    ap.add_argument('--wz', type=float, default=0.15)
    ap.add_argument('--vx', type=float, default=0.03)
    a = ap.parse_args()
    k = a.kalib_kosu or a.kosu
    Vk = np.loadtxt(os.path.join(CALISMA, k, 'yorunge', a.vo + '.csv'), delimiter=',')
    d, n, r0, r1 = kalibre(sensor_oku(k), Vk, a.wz, a.vx)
    # doğrulama: düzeltme sonrası dönüş ötelemesi
    print(f'{k} {a.vo}: {n} dönüş, kamera kayması dx {d[0] * 100:+.1f} dy {d[1] * 100:+.1f} cm; dönüş ötelemesi '
          f'{np.round(r0 * 100, 1)} -> {np.round(r1 * 100, 1)} cm')
    V = np.loadtxt(os.path.join(CALISMA, a.kosu, 'yorunge', a.vo + '.csv'), delimiter=',')
    ek = '_ozkal' if k == a.kosu else f'_ozkal_{k}'
    kaydet(a.kosu, a.vo + ek, duzelt(V, *d), dict(dx=float(d[0]), dy=float(d[1]), kalib_kosu=k, donus=n))


if __name__ == '__main__':
    main()
