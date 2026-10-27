#!/usr/bin/env python3
"""Taban tepki modeli (limo_kinematics/taban_modeli.py) uydurması, ROBOTTA kaydedilmiş bag'lerden (ağ gecikmesi yok):
komut /cmd_vel (tabanın aldığı), tepki v = /odom_duz twist × k_v (odom mesafeyi %1.9 eksik sayıyor), w = /imu gz
(jiroskop ölçeği 1.000, sim_real_fark). Izgara: gecikme, τ, ivme sınırı; kazanç en küçük karelerle. Yarısı uydurma,
yarısı doğrulama. 24.09 değerleri (gecikme v 0.30 / w 0.05 s) PC'de teleop kaydındandı: Wi-Fi gecikmesi dahildi.
Kullanım: taban_uydur.py --uydur <bag> [...] --dogrula <bag> [...] [--k_v 1.0188]"""

import argparse
import itertools

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from bag2vlmaps import damga, okuyucu
from limo_kinematics.taban_modeli import KanalParam, Kanal


def oku(bag, k_v):
    C, V, W = [], [], []
    for k, v, t in okuyucu(bag, ['/cmd_vel', '/odom_duz', '/imu']):
        if k == '/cmd_vel':
            m = deserialize_message(v, Twist)
            C.append((t * 1e-9, m.linear.x, m.angular.z))     # bag alım zamanı (robotta)
        elif k == '/odom_duz':
            m = deserialize_message(v, Odometry)
            V.append((damga(m.header), m.twist.twist.linear.x * k_v))
        else:
            m = deserialize_message(v, Imu)
            W.append((damga(m.header), m.angular_velocity.z))
    return np.array(C), np.array(sorted(V)), np.array(sorted(W))


def benzet(p, C, i, tg):
    k = Kanal(p)
    out, j = np.zeros(len(tg)), 0
    for n, t in enumerate(tg):
        while j < len(C) and C[j, 0] <= t:
            k.komut(C[j, 0], C[j, i])
            j += 1
        out[n] = k.adim(t)
    return out


def skor(p, veri, i):
    e, var = 0.0, 0.0
    for C, R in veri:
        tg = np.arange(max(C[0, 0], R[0, 0]), min(C[-1, 0], R[-1, 0]), 0.02)
        y = np.interp(tg, R[:, 0], R[:, 1])
        yh = benzet(p, C, i, tg)
        e += np.sum((y - yh) ** 2)
        var += np.sum((y - y.mean()) ** 2)
    return 1 - e / var


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uydur', nargs='+', required=True)
    ap.add_argument('--dogrula', nargs='+', default=[])
    ap.add_argument('--k_v', type=float, default=1.0188)
    a = ap.parse_args()
    U = [oku(b, a.k_v) for b in a.uydur]
    D = [oku(b, a.k_v) for b in a.dogrula]
    for ad, i, ri, izg in (('v', 1, 1, dict(gecikme=[0.0, 0.05, 0.1, 0.15, 0.2, 0.3], tau=[0.05, 0.1, 0.2, 0.3, 0.45],
                                                 ivme=[0.15, 0.3, 0.6, 1.0, 3.0], kazanc=[0.95, 1.0, 1.05], watchdog=[1.0])),
                           ('w', 2, 2, dict(gecikme=[0.0, 0.03, 0.05, 0.1, 0.2], tau=[0.05, 0.1, 0.2, 0.3],
                                            ivme=[1.0, 3.0, 6.0, 12.0], kazanc=[0.95, 1.0, 1.05], watchdog=[0.5]))):
        uv = [(C, R) for C, V, W in U for R in [V if ad == 'v' else W]]
        dv = [(C, R) for C, V, W in D for R in [V if ad == 'v' else W]]
        en = max((skor(KanalParam(*c), uv, i), c) for c in itertools.product(*izg.values()))
        p = KanalParam(*en[1])
        eski = KanalParam(0.30, 0.45, 0.15, 0.95, 1.0) if ad == 'v' else KanalParam(0.05, 0.20, 6.0, 1.05, 0.5)
        print(f'{ad}: en iyi {p}  R² uydurma {en[0]:.3f}' +
              (f', doğrulama {skor(p, dv, i):.3f} (24.09 parametreleri {skor(eski, dv, i):.3f})' if dv else ''))


if __name__ == '__main__':
    main()
