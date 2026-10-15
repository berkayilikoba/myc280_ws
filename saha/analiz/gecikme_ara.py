#!/usr/bin/env python3
"""Renk-derinlik zaman gecikmesi araması (dönüş kaydı): renk damgasına ofset ekleyerek hiza hatasını en aza indir.

bag2vlmaps.py ile aynı hizalama (derinlik -> dünya -> renk, odom pozlarıyla hareket telafisi), ama renk karesinin
pozu (tc + ofset) anında alınır. Dönerken (|w| > --min_w) hiza ölçütü (derinlik süreksizliği -> en yakın renk
kenarı, px) her ofset için hesaplanır; en küçük medyan = gerçek göreli gecikme. Diske sahne yazmaz.

Kullanım (Humble): gecikme_ara.py <kamera_bag> <sensor_bag> [--ofsetler -150,-100,...] [--her 2]
Pozitif ofset: renk karesi damgasından SONRA çekilmiş gibi davran (renk damgası erken).
"""

import argparse
import os

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from bag2vlmaps import (BURADA, damga, kamera_dis, derinlik_hizala, goruntu, okuyucu, poz_ara, sensor_oku, zincir)
from donus_analiz import hiza


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kamera_bag')
    ap.add_argument('sensor_bag')
    ap.add_argument('--ofsetler', default='-200,-150,-100,-75,-50,-25,0,25,50,75,100,150,200')
    ap.add_argument('--her', type=int, default=2)
    ap.add_argument('--min_w', type=float, default=0.15)
    ap.add_argument('--kalib', default=os.path.join(BURADA, '..', 'config', 'kamera_kalibrasyon.yaml'))
    a = ap.parse_args()
    ofs = [float(x) / 1000 for x in a.ofsetler.split(',')]
    O, tfs, ci = sensor_oku(a.sensor_bag)
    T_b_cl = kamera_dis(tfs, a.kalib)
    T_b_co = T_b_cl @ zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_color_frame', 'camera_color_optical_frame')
    T_b_do = T_b_cl @ zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')
    Kc, Dc = np.array(ci['color'].k).reshape(3, 3), np.array(ci['color'].d)
    Kd = np.array(ci['depth'].k).reshape(3, 3)
    Hc, Wc = ci['color'].height, ci['color'].width
    w_odom = np.convolve(np.gradient(O[:, 3], O[:, 0]), np.ones(5) / 5, mode='same')
    derin = [(damga(m.header), goruntu(m)) for m in
             (deserialize_message(v, Image) for _, v, _ in okuyucu(a.kamera_bag, ['/camera/depth/image_raw']))]
    dT = np.array([d[0] for d in derin])
    sonuc = {o: [] for o in ofs}
    i = 0
    for _, veri, _ in okuyucu(a.kamera_bag, ['/camera/color/image_raw']):
        i += 1
        if (i - 1) % a.her:
            continue
        m = deserialize_message(veri, Image)
        tc = damga(m.header)
        if abs(np.interp(tc, O[:, 0], w_odom)) < a.min_w:
            continue
        j = int(np.argmin(np.abs(dT - tc)))
        td = dT[j]
        Tw_bd = poz_ara(O, td, 0.25)
        if Tw_bd is None or abs(td - tc) > 0.03:
            continue
        gri = cv2.cvtColor(cv2.undistort(goruntu(m), Kc, Dc), cv2.COLOR_BGR2GRAY)
        for o in ofs:
            Tw_bc = poz_ara(O, tc + o, 0.25)
            if Tw_bc is None:
                continue
            T_c_d = np.linalg.inv(Tw_bc @ T_b_co) @ Tw_bd @ T_b_do
            h = hiza(gri, derinlik_hizala(derin[j][1], Kd, Kc, T_c_d, (Hc, Wc)))
            if h:
                sonuc[o].append(h[0])
    print(f'{"ofset_ms":>9} {"kare":>5} {"hiza_medyan_px":>15}')
    en = None
    for o in ofs:
        if len(sonuc[o]) >= 5:
            md = float(np.median(sonuc[o]))
            print(f'{o * 1000:+9.0f} {len(sonuc[o]):5d} {md:15.2f}')
            en = (md, o) if en is None or md < en[0] else en
    if en:
        print(f'en iyi ofset: {en[1] * 1000:+.0f} ms (hiza {en[0]:.2f} px)')


if __name__ == '__main__':
    main()
