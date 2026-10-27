#!/usr/bin/env python3
"""Sabit sahnede derinlik-renk hizası ve renk keskinliği, kamera ayarı başına (kamera_ayar_testi.sh çıktısı).

Her bag: kamera_info + tf_static (sürücünün camera_link -> depth/color zinciri). Her N. derinlik karesi en yakın renk
karesiyle eşlenir, bag2vlmaps.derinlik_hizala ile renk çerçevesine izdüşürülür (delik doldurmalı) ve donus_analiz.hiza
ile ölçülür. Farklı renk çözünürlüklerini karşılaştırabilmek için renk (ve K) önce --genislik px'e ölçeklenir
(varsayılan 640: 21.10 ölçütleriyle aynı piksel birimi).
Kullanım (Humble): sabit_hiza.py <ayar_dizini veya bag> [...] [--her 5 --genislik 640]
"""

import argparse
import glob
import os

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from bag2vlmaps import T, damga, derinlik_hizala, goruntu, okuyucu, q2R, zincir
from donus_analiz import hiza


def olc(bag, her, genislik):
    tfs, ci, renk, derin = {}, {}, [], []
    for konu, v, _ in okuyucu(bag, ['/tf_static', '/camera/color/camera_info', '/camera/depth/camera_info',
                                     '/camera/color/image_raw', '/camera/depth/image_raw']):
        if konu == '/tf_static':
            for t in deserialize_message(v, TFMessage).transforms:
                tr, q = t.transform.translation, t.transform.rotation
                tfs[(t.header.frame_id, t.child_frame_id)] = T(q2R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        elif konu.endswith('camera_info'):
            ci.setdefault(konu.split('/')[2], deserialize_message(v, CameraInfo))
        elif konu == '/camera/color/image_raw':
            m = deserialize_message(v, Image)
            renk.append((damga(m.header), m))
        else:
            m = deserialize_message(v, Image)
            derin.append((damga(m.header), m))
    Kc, Dc = np.array(ci['color'].k).reshape(3, 3), np.array(ci['color'].d)
    Kd = np.array(ci['depth'].k).reshape(3, 3)
    s = genislik / ci['color'].width
    Ko = Kc.copy()
    Ko[:2, :2] *= s
    Ko[:2, 2] = (Kc[:2, 2] + 0.5) * s - 0.5
    boyut = (int(round(ci['color'].height * s)), genislik)
    T_c_d = np.linalg.inv(zincir(tfs, 'camera_link', 'camera_color_frame', 'camera_color_optical_frame')) @ \
        zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')
    tr = np.array([t for t, _ in renk])
    out = []
    for t, m in derin[::her]:
        j = int(np.argmin(np.abs(tr - t)))
        rgb = cv2.resize(cv2.undistort(goruntu(renk[j][1]), Kc, Dc), (boyut[1], boyut[0]), interpolation=cv2.INTER_AREA)
        d = derinlik_hizala(goruntu(m), Kd, Ko, T_c_d, boyut, zmax=8.0)   # sahne kenarları 4-5 m
        for _ in range(2):   # izdüşüm ızgara deliklerini kapat (her ayara aynı işlem; kenar ölçütü 5x5 sağlam bölge ister)
            d = np.where(d > 0, d, cv2.dilate(d, np.ones((3, 3), np.uint8)))
        h = hiza(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY), d)
        if h:
            out.append((*h, float((d > 0).mean())))
    return np.array(out), f'{ci["color"].width}x{ci["color"].height}', f'{ci["depth"].width}x{ci["depth"].height}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yollar', nargs='+')
    ap.add_argument('--her', type=int, default=5)
    ap.add_argument('--genislik', type=int, default=640)
    a = ap.parse_args()
    baglar = []
    for y in a.yollar:
        baglar += sorted(os.path.dirname(p) for p in glob.glob(os.path.join(y, '*', 'metadata.yaml'))) or [y]
    print(f'{"ayar":>16} {"renk":>9} {"derinlik":>9} {"kare":>4} {"hiza_px":>7} {"<=3px":>6} {"keskinlik":>9} {"doluluk":>7}')
    for b in baglar:
        try:
            r, rc, dc = olc(b, a.her, a.genislik)
        except Exception as e:                                    # eksik konu / boş bag
            print(f'{os.path.basename(b):>16} HATA {e}')
            continue
        if not len(r):
            print(f'{os.path.basename(b):>16} {rc:>9} {dc:>9} ölçülemedi')
            continue
        print(f'{os.path.basename(b):>16} {rc:>9} {dc:>9} {len(r):4d} {np.median(r[:, 0]):7.2f} {np.mean(r[:, 1]):6.2f} '
              f'{np.median(r[:, 2]):9.0f} {np.mean(r[:, 3]):7.3f}')


if __name__ == '__main__':
    main()
