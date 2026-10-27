#!/usr/bin/env python3
"""Gerçek odanın doluluk haritasından Gazebo dünyası (world_to_map.py'nin tersi): dolu hücreler dikdörtgenlere
birleştirilip yükseltilmiş statik kutular olur. Dünya koordinatı = harita (map) çerçevesi; harita orijini robotun
başlangıç işareti olduğundan robot (0, 0, yaw 0)'da doğar ve sim aynı harita.yaml ile Nav2/AMCL çalıştırabilir.

Lidar 2B harita yalnız lidar yüksekliğindeki kesiti görür (masa ayakları, sandalye tekerleri, duvarlar); kutular
--yukseklik kadar yükseltilir (varsayılan 0.6 m: kamera ve lidar için yeterli). Masa üstü gibi yüksek yüzeyler yok.
Kullanım: harita_dunya.py <harita.yaml> <cikti.world> [--yukseklik 0.6] [--min_alan 2] [--aydinlik]
"""

import argparse
import os

import cv2
import numpy as np
import yaml


def dikdortgenler(dolu):
    """bool ızgara -> [(v0, u0, v1, u1)] (dahil); satır koşuları dikeyde aynı aralıkla birleştirilir"""
    H, W = dolu.shape
    acik, out = {}, []                                    # (u0, u1) -> v0
    for v in range(H + 1):
        kosular = set()
        if v < H:
            r = np.r_[0, dolu[v].astype(np.int8), 0]
            d = np.diff(r)
            kosular = set(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0] - 1))
        for k in list(acik):
            if k not in kosular:
                out.append((acik.pop(k), k[0], v - 1, k[1]))
        for k in kosular:
            acik.setdefault(k, v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('harita')
    ap.add_argument('cikti')
    ap.add_argument('--yukseklik', type=float, default=0.6)
    ap.add_argument('--min_alan', type=int, default=2, help='bundan az hücreli dolu bileşenler gürültü sayılır')
    a = ap.parse_args()
    h = yaml.safe_load(open(a.harita))
    img = cv2.imread(os.path.join(os.path.dirname(os.path.abspath(a.harita)), h['image']), cv2.IMREAD_GRAYSCALE)
    p = (255 - img.astype(float)) / 255.0 if not h.get('negate', 0) else img / 255.0
    dolu = p > h.get('occupied_thresh', 0.65)
    n, et, st, _ = cv2.connectedComponentsWithStats(dolu.astype(np.uint8), connectivity=8)
    dolu = np.isin(et, [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= a.min_alan])
    r, (ox, oy) = h['resolution'], h['origin'][:2]
    H = dolu.shape[0]
    D = dikdortgenler(dolu)
    parcalar = []
    for i, (v0, u0, v1, u1) in enumerate(D):
        cx = ox + (u0 + u1 + 1) / 2 * r
        cy = oy + (H - (v0 + v1 + 1) / 2) * r
        sx, sy = (u1 - u0 + 1) * r, (v1 - v0 + 1) * r
        g = f'<geometry><box><size>{sx:.3f} {sy:.3f} {a.yukseklik:.3f}</size></box></geometry>'
        parcalar.append(f'''      <collision name="c{i}"><pose>{cx:.3f} {cy:.3f} {a.yukseklik / 2:.3f} 0 0 0</pose>{g}</collision>
      <visual name="v{i}"><pose>{cx:.3f} {cy:.3f} {a.yukseklik / 2:.3f} 0 0 0</pose>{g}
        <material><ambient>0.75 0.73 0.70 1</ambient><diffuse>0.85 0.83 0.80 1</diffuse></material></visual>''')
    sdf = f'''<?xml version="1.0"?>
<!-- {os.path.basename(a.cikti)}: harita_dunya.py ile {os.path.abspath(a.harita)}'dan üretildi
     ({len(D)} kutu, yükseklik {a.yukseklik} m). Dünya = harita çerçevesi; robot başlangıcı (0, 0, 0). -->
<sdf version="1.6">
  <world name="lab_oda">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <physics type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>
    <model name="oda">
      <static>true</static>
      <link name="engeller">
{chr(10).join(parcalar)}
      </link>
    </model>
  </world>
</sdf>
'''
    open(a.cikti, 'w').write(sdf)
    print(f'{a.cikti}: {int(dolu.sum())} dolu hücre -> {len(D)} kutu, x {ox:.2f}..{ox + dolu.shape[1] * r:.2f}, '
          f'y {oy:.2f}..{oy + H * r:.2f} m')


if __name__ == '__main__':
    main()
