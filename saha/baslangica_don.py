#!/usr/bin/env python3
"""Robotu başlangıç işaretine (harita orijini, yaw 0) geri getirir: lidar taraması haritaya ICP ile oturtulur (odom
kaymasından bağımsız), fark odomla düzeltilir (gerekirse geri geri gider), --tekrar kez ölç-düzelt.
Kapsama turu odomla 4-6° yön kaydırıyor (27.10: tur sonunda x +8 cm, yaw -4.5°); tekrarlar aynı başlangıçtan olsun diye.
Komut cmd_vel_smoothed -> collision_monitor (Nav2 açık olmalı). Kullanım (Humble):
  baslangica_don.py <harita.yaml> [--tol_m 0.015 --tol_derece 0.5 --tekrar 3] [--sadece_olc]
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analiz'))
from duvar_testi import Duvar, wrap                                           # noqa: E402
from lidar_poz import T2, harita_noktalari, icp                               # noqa: E402

TL = T2(0.137, 0.015, math.radians(-9.66))                                    # base_link -> laser_duz


def harita_pozu(n, agac, M, k=8):
    """son k taramadan ICP pozu (x, y, yaw) medyanı + uyum"""
    sc = []
    t = time.time()
    while len(sc) < k and time.time() - t < 10:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.son_tarama is not None and (not sc or sc[-1] is not n.son_tarama):
            sc.append(n.son_tarama)
    P = []
    for m in sc:
        r = np.array(m.ranges)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        s = np.isfinite(r) & (r > 0.25) & (r < 8)
        pts = np.c_[r[s] * np.cos(a[s]), r[s] * np.sin(a[s])] @ TL[:2, :2].T + TL[:2, 2]
        en = max((icp(pts, agac, M, T2(0, 0, math.radians(y0))) for y0 in (-12, -6, 0, 6, 12)), key=lambda z: z[1])
        T, u = en
        P.append((T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0]), u))
    return np.median(np.array(P), 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('harita')
    ap.add_argument('--tol_m', type=float, default=0.015)
    ap.add_argument('--tol_derece', type=float, default=0.5)
    ap.add_argument('--tekrar', type=int, default=3)
    ap.add_argument('--hiz', type=float, default=0.05)
    ap.add_argument('--donus', type=float, default=0.25)
    ap.add_argument('--cmd_topic', default='cmd_vel_smoothed')
    ap.add_argument('--sadece_olc', action='store_true')
    a = ap.parse_args()
    a.geri_max, a.bekle = 0.0, 0.0
    M = harita_noktalari(a.harita)
    agac = cKDTree(M)
    rclpy.init()
    n = Duvar(a)
    n.son_tarama = None
    n.create_subscription(LaserScan, '/scan_sabit', lambda m: setattr(n, 'son_tarama', m), qos_profile_sensor_data)
    n.bekle_veri()
    for i in range(a.tekrar + 1):
        x, y, yaw, u = harita_pozu(n, agac, M)
        d = math.hypot(x, y)
        print(f'ölçüm {i}: x {x:+.3f} y {y:+.3f} yaw {math.degrees(yaw):+.2f}° (uyum {u:.2f})', flush=True)
        if u < 0.8:
            print('uyum düşük, hareket edilmiyor')
            break
        if a.sadece_olc or (d < a.tol_m and abs(math.degrees(yaw)) < a.tol_derece) or i == a.tekrar:
            break
        # orijin robot çerçevesinde -> odom hedefi
        ox, oy, oyaw = n.poz
        c, s = math.cos(-yaw), math.sin(-yaw)
        rx, ry = c * (-x) - s * (-y), s * (-x) + c * (-y)                       # orijin, robot çerçevesinde
        hx, hy = ox + math.cos(oyaw) * rx - math.sin(oyaw) * ry, oy + math.sin(oyaw) * rx + math.cos(oyaw) * ry
        hyaw = wrap(oyaw - yaw)
        if d >= a.tol_m:
            yon = math.atan2(hy - oy, hx - ox)
            geri = abs(wrap(yon - oyaw)) > math.pi / 2                          # arkadaysa geri geri git
            n.don(wrap(yon + math.pi) if geri else yon)
            t0 = time.time()
            while time.time() - t0 < 20:
                rclpy.spin_once(n, timeout_sec=0.02)
                px, py, pyaw = n.poz
                ileri = (hx - px) * math.cos(pyaw) + (hy - py) * math.sin(pyaw)
                if abs(ileri) < 0.004:
                    break
                n.gonder(math.copysign(max(0.02, min(a.hiz, 1.0 * abs(ileri))), ileri), 0.0)
            n.gonder()
        n.don(hyaw)
        n.gonder()
        time.sleep(0.5)
    n.gonder()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
