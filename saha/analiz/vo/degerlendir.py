#!/usr/bin/env python3
"""Tüm yörüngeleri (temel çizgiler + yorunge/*.csv) referansa göre ölçer, tabloyu yazar.

Temel çizgiler sensör bag'inden: ekf (kayıttaki /odometry/filtered, yalnız d435_1015), ekf_esdeger (/odom vx + IMU yönü,
ekf_robot.yaml'ın çevrimdışı karşılığı), odom_duz (09.10), odom (ham enkoder).
Kullanım: degerlendir.py [--md cikti.md]
"""

import argparse
import glob
import os

import numpy as np

from ortak import CALISMA, KOSULAR, degerlendir, ekf_esdeger, referans_oku, sensor_oku

SUTUN = [('ate_se2', 'ATE-SE2 cm', '{:.1f}'), ('ate_rmse', 'ATE rms cm', '{:.1f}'), ('ate_max', 'maks cm', '{:.1f}'),
         ('son', 'bitiş cm', '{:.1f}'),
         ('yaw_rmse', 'yaw rms °', '{:.2f}'), ('yaw_son', 'bitiş yaw °', '{:+.2f}'), ('rpe_1m', 'RPE/1 m cm', '{:.2f}')]


def temel(ad):
    S = sensor_oku(ad)
    Y = {'ekf_esdeger': ekf_esdeger(S)}
    for k in ('ekf', 'odom_duz', 'odom'):
        if len(S[k]):
            Y[k] = S[k][:, :4]
    return Y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--md')
    ap.add_argument('--kosu', nargs='*', default=list(KOSULAR))
    a = ap.parse_args()
    satir = []
    for ad in a.kosu:
        R = referans_oku(ad)
        Y = temel(ad)
        for f in sorted(glob.glob(f'{CALISMA}/{ad}/yorunge/*.csv')):
            Y[os.path.basename(f)[:-4]] = np.loadtxt(f, delimiter=',')
        for y, A in Y.items():
            m = degerlendir(A, R)
            satir.append((ad, y, m))
    bas = '| koşu | yöntem | ' + ' | '.join(s[1] for s in SUTUN) + ' |'
    out = [bas, '|' + '---|' * (len(SUTUN) + 2)]
    for ad, y, m in satir:
        out.append(f'| {ad} | {y} | ' + ' | '.join(s[2].format(m[s[0]]) for s in SUTUN) + ' |')
    print('\n'.join(out))
    if a.md:
        open(a.md, 'w').write('\n'.join(out) + '\n')


if __name__ == '__main__':
    main()
