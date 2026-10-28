#!/usr/bin/env python3
"""Dil sorgusundan navigasyon hedefi: VLMaps haritasında "kapı", "sandalye" ... -> harita çerçevesinde (x, y, yaw).

vlmaps sanal ortamında: ~/vlmaps_env/bin/python vlmaps_hedef.py <vlmaps_cikti_dizini> <harita.yaml> "door" [...]
  1) VLMap yüklenir, sorgu CLIP metin özelliğiyle LSeg hücre özelliklerine eşlenir (vlmaps index_map; başka kategoriler
     karşısında en yüksek skor alan hücreler, --karsit ile verilen listeye göre).
  2) Eşleşen hücreler kuşbakışına (5 cm) indirilir, en büyük bağlı küme seçilir -> nesne merkezi.
  3) Lidar haritasında (map_server) serbest ve engellerden >= --pay m uzak hücreler arasından nesneye --mesafe m'ye en
     yakın olan seçilir; yaw nesneye bakar. Hedef Nav2'ye verilebilir (saha/hedefe_git.py x y yaw).
Pozlar lidar_poz.py ile harita çerçevesinde olmalı (vlmaps_calistir.py sahnesi --poz_dosyasi ile üretilmiş).
"""

import argparse
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path.home() / 'vlmaps'))
KARSIT = ['wall', 'floor', 'door', 'chair', 'table', 'cabinet', 'shelf', 'box', 'window', 'trash can', 'robot']


def harita_oku(yol):
    h = yaml.safe_load(open(yol))
    img = cv2.imread(os.path.join(os.path.dirname(yol), h['image']), cv2.IMREAD_UNCHANGED)
    return img, h['resolution'], h['origin']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('vlmaps_dizini')
    ap.add_argument('harita')
    ap.add_argument('sorgular', nargs='+')
    ap.add_argument('--karsit', nargs='+', default=KARSIT)
    ap.add_argument('--mesafe', type=float, default=0.6, help='nesneye istenen uzaklık (m)')
    ap.add_argument('--pay', type=float, default=0.30, help='engellerden en az uzaklık (m; LIMO yarıçapı ~0.2)')
    ap.add_argument('--cikti', help='harita üstünde nesne kümeleri ve hedefler (PNG)')
    ap.add_argument('--kavrama', action='store_true',
                    help='hedef = kol kavrama yaklaşması (limo_autonomy.kavrama_yaklasim): kavrama öncesi poz Q (Nav2) + '
                         'kavrama pozu P (nesne robot merkezinin d_kavrama önünde)')
    ap.add_argument('--d_kavrama', type=float, default=0.22, help='kol_erisim.py: 0.18-0.30 m')
    ap.add_argument('--yon', type=float, help='istenen yaklaşma yönü (derece, harita)')
    a = ap.parse_args()
    from omegaconf import OmegaConf
    from vlmaps.map.vlmap import VLMap

    veri = Path(a.vlmaps_dizini).resolve() / 'veri'
    cfg = OmegaConf.load(Path.home() / 'vlmaps/config/map_config/vlmaps.yaml')
    cfg.grid_size, cfg.cell_size = 1000, 0.05
    cfg.pose_info.pose_type = 'camera_base'
    vm = VLMap(cfg, data_dir=veri)
    vm.load_map(veri)
    vm._init_clip()
    pmin, cs = np.asarray(vm.pcd_min, float), 0.05
    gp = vm.grid_pos.astype(float)
    X, Y = pmin[0] + gp[:, 0] * cs, -(pmin[2] + gp[:, 2] * cs)        # Habitat -> ROS harita çerçevesi
    img, res, org = harita_oku(a.harita)
    if a.kavrama:
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'limo_autonomy'))
        from limo_autonomy.kavrama_yaklasim import Ayar, Harita, yaklasim_bul
        kh = Harita.yaml_oku(a.harita)
    serbest = (img > 250).astype(np.uint8)                              # map_server: 254 serbest, 0 dolu, 205 bilinmiyor
    uzak = cv2.distanceTransform(serbest, cv2.DIST_L2, 5) * res         # en yakın engele / bilinmeyene uzaklık (m)
    cv, cu = np.nonzero(uzak >= a.pay)
    cx, cy = org[0] + (cu + 0.5) * res, org[1] + (img.shape[0] - cv - 0.5) * res
    olc = 6
    gor = cv2.cvtColor(cv2.resize(img, (img.shape[1] * olc, img.shape[0] * olc), interpolation=cv2.INTER_NEAREST),
                       cv2.COLOR_GRAY2BGR)

    def pik(x, y):
        return int((x - org[0]) / res * olc), int((img.shape[0] - (y - org[1]) / res) * olc)
    renkler = [(0, 0, 255), (0, 160, 0), (255, 0, 0), (0, 140, 255), (160, 0, 160), (0, 200, 200)]
    for sorgu in a.sorgular:
        kat = list(dict.fromkeys([sorgu] + [k for k in a.karsit if k != sorgu]))
        vm.init_categories(kat)
        maske = vm.index_map(sorgu, with_init_cat=True)
        if maske.sum() < 5:
            print(f'{sorgu}: eşleşen hücre yok')
            continue
        # kuşbakışı 5 cm ızgarada en büyük bağlı küme
        ix, iy = np.round(X[maske] / cs).astype(int), np.round(Y[maske] / cs).astype(int)
        ox, oy = ix.min(), iy.min()
        izg = np.zeros((ix.max() - ox + 1, iy.max() - oy + 1), np.uint8)
        izg[ix - ox, iy - oy] = 1
        _, lab, st, _ = cv2.connectedComponentsWithStats(cv2.dilate(izg, np.ones((3, 3), np.uint8)))
        en = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        kx, ky = np.nonzero((lab == en) & (izg > 0))
        mx, my = (kx + ox).mean() * cs, (ky + oy).mean() * cs
        rk = renkler[a.sorgular.index(sorgu) % len(renkler)]
        if a.kavrama:
            y = yaklasim_bul(kh, (mx, my), Ayar(d_kavrama=a.d_kavrama, yon=None if a.yon is None else math.radians(a.yon)),
                             robot=(0.0, 0.0))
            if y is None:
                print(f'{sorgu}: merkez ({mx:+.2f}, {my:+.2f}) -> kavrama yaklaşması bulunamadı (engel / erişim)')
                continue
            (px, py, th), (qx, qy, _) = y.kavrama, y.oncesi
            for x, yy in zip((kx + ox) * cs, (ky + oy) * cs):
                cv2.circle(gor, pik(x, yy), 2, rk, -1)
            cv2.arrowedLine(gor, pik(qx, qy), pik(px, py), rk, 2, tipLength=0.3)
            cv2.circle(gor, pik(qx, qy), 4, rk, 1)
            cv2.putText(gor, sorgu, pik(mx, my), cv2.FONT_HERSHEY_SIMPLEX, 0.5, rk, 2)
            print(f'{sorgu}: merkez ({mx:+.2f}, {my:+.2f}) -> Nav2 hedefi Q x {qx:+.2f} y {qy:+.2f} yaw {math.degrees(th):+.0f}°, '
                  f'kavrama pozu P x {px:+.2f} y {py:+.2f} (düz {a.d_kavrama + 0.0:.2f}+0.35 m), açıklık {y.aciklik:.2f} m')
            continue
        d = np.hypot(cx - mx, cy - my)
        j = int(np.argmin(np.abs(d - a.mesafe)))
        hx, hy = cx[j], cy[j]
        yaw = math.atan2(my - hy, mx - hx)
        for x, y in zip((kx + ox) * cs, (ky + oy) * cs):
            cv2.circle(gor, pik(x, y), 2, rk, -1)
        cv2.arrowedLine(gor, pik(hx, hy), pik(hx + 0.3 * math.cos(yaw), hy + 0.3 * math.sin(yaw)), rk, 2, tipLength=0.4)
        cv2.putText(gor, sorgu, pik(mx, my), cv2.FONT_HERSHEY_SIMPLEX, 0.5, rk, 2)
        print(f'{sorgu}: {int(maske.sum())} hücre, en büyük küme {len(kx) * cs * cs:.2f} m², merkez ({mx:+.2f}, {my:+.2f}) -> '
              f'hedef x {hx:+.2f} y {hy:+.2f} yaw {math.degrees(yaw):+.0f}° (nesneye {d[j]:.2f} m, engelden {uzak[cv[j], cu[j]]:.2f} m)')
    if a.cikti:
        cv2.circle(gor, pik(0, 0), 5, (0, 0, 0), 2)                       # başlangıç işareti
        cv2.imwrite(a.cikti, gor)


if __name__ == '__main__':
    main()
