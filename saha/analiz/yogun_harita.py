#!/usr/bin/env python3
"""Yoğun referans harita (olcum_protokolu.md 1.4): lidar_poz.py ile haritaya oturtulmuş taramalar birden çok kayıttan
biriktirilir -> 2 cm doluluk haritası (map_server yaml + pgm). vlmaps_dogrula.py --harita, vlmaps_karsilastir.py,
lidar_poz.py doğrudan kullanabilir; 5 cm SLAM haritasının ölçütlere koyduğu taban kalkar.

  1) Her kayıt: /scan_sabit + lidar_poz CSV (t, x, y, yaw, uyum). Uyumu düşük (< --min_uyum) ya da dönüş hızı yüksek
     (> --max_w; tarama dönerken çarpılıyor) taramalar atılır.
  2) --incelt N: her tarama, DİĞER kayıtların taramalarından biriken nokta bulutuna (2 cm seyreltilmiş) yeniden ICP ile
     oturtulur (eşik 0.10 -> 0.03 m); N tur. Kayıtlar arası uyum (3 cm) tutarlılık ölçüsüdür.
  3) Doluluk: vuruş sayısı (uç noktalar) ve boşluk sayısı (ışın boyunca, uçtan 2 hücre önceye kadar) -> dolu: vuruş >=
     --min_vurus ve vuruş / (vuruş + boş) >= --oran; boş: yalnız boşluk görülmüş; diğerleri bilinmiyor. İnsanlar ve
     kapı gibi geçici nesneler oran ölçütüyle elenir (birden çok kayıt bu yüzden önemli).
Kullanım (Humble):
  yogun_harita.py <harita.yaml (5 cm, çerçeve için)> <cikti_dizini> --kayit <sensor_bag>,<poz.csv> [...] [--incelt 2]
Çıktı: <cikti>/yogun.yaml + yogun.pgm, noktalar.npy (dolu hücre merkezleri), yogun_ozet.yaml
"""

import argparse
import json
import math
import os

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

from bag2vlmaps import damga, okuyucu
from lidar_poz import T2, harita_noktalari, icp

LIDAR = (0.137, 0.015, math.radians(-9.66))           # base_link -> laser_duz (ortam.sh lidar_tf_yayinla)


def seyrelt(P, h):
    _, i = np.unique(np.floor(P / h).astype(np.int64), axis=0, return_index=True)
    return P[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('harita')
    ap.add_argument('cikti')
    ap.add_argument('--kayit', nargs='+', required=True, help='sensor_bag,poz.csv çiftleri')
    ap.add_argument('--cozunurluk', type=float, default=0.02)
    ap.add_argument('--min_uyum', type=float, default=0.7)
    ap.add_argument('--max_w', type=float, default=0.3, help='rad/s')
    ap.add_argument('--rmax', type=float, default=6.0, help='bundan uzak ışınlar (açısal aralık > 5 cm) kullanılmaz')
    ap.add_argument('--incelt', type=int, default=2)
    ap.add_argument('--min_vurus', type=int, default=3)
    ap.add_argument('--oran', type=float, default=0.3)
    a = ap.parse_args()
    os.makedirs(a.cikti, exist_ok=True)
    TL = T2(*LIDAR)

    taramalar, ozet = [], {'kayitlar': []}               # (kayıt no, 3x3 poz, base_link'te noktalar, menziller)
    for ki, cift in enumerate(a.kayit):
        bag, csv = cift.split(',')
        S = np.loadtxt(csv, delimiter=',', comments='#')
        S[:, 3] = np.unwrap(S[:, 3])
        w = np.abs(np.gradient(S[:, 3], S[:, 0]))
        say = [0, 0]
        for _, v, _ in okuyucu(bag, ['/scan_sabit']):
            m = deserialize_message(v, LaserScan)
            t = damga(m.header)
            say[0] += 1
            if t < S[0, 0] or t > S[-1, 0]:
                continue
            uy, wi = np.interp(t, S[:, 0], S[:, 4]), np.interp(t, S[:, 0], w)
            if uy < a.min_uyum or wi > a.max_w:
                continue
            r = np.array(m.ranges)
            ang = m.angle_min + np.arange(len(r)) * m.angle_increment
            k = np.isfinite(r) & (r > 0.25) & (r < a.rmax)
            P = np.c_[r[k] * np.cos(ang[k]), r[k] * np.sin(ang[k])] @ TL[:2, :2].T + TL[:2, 2]
            taramalar.append([ki, T2(*(np.interp(t, S[:, 0], S[:, i]) for i in (1, 2, 3))), P])
            say[1] += 1
        ozet['kayitlar'].append({'bag': bag, 'poz': csv, 'tarama': say[0], 'kullanilan': say[1]})
        print(f'{bag}: {say[1]} / {say[0]} tarama')

    def bulut(haric=None):
        return seyrelt(np.vstack([P @ T[:2, :2].T + T[:2, 2] for k, T, P in taramalar if k != haric]), a.cozunurluk)

    kayitlar = sorted({k for k, _, _ in taramalar})
    for tur in range(a.incelt):
        # her tarama yalnız DİĞER kayıtların bulutuna oturtulur (kendi noktaları buluttayken uyum anlamsız, ~1.0)
        bulutlar = {k: bulut(k if len(kayitlar) > 1 else None) for k in kayitlar}
        agaclar = {k: cKDTree(B) for k, B in bulutlar.items()}
        dx, uyum = [], []
        for tr in taramalar:
            T1, u = icp(tr[2], agaclar[tr[0]], bulutlar[tr[0]], tr[1], esikler=(0.10, 0.05, 0.03), it=6)
            if u > 0.6:
                dx.append(np.hypot(*(T1[:2, 2] - tr[1][:2, 2])))
                tr[1] = T1
            uyum.append(u)
        print(f'inceltme {tur + 1}: düzeltme medyanı {np.median(dx) * 1000:.1f} mm (%95 {np.percentile(dx, 95) * 1000:.1f}), '
              f'diğer kayıtlara uyum (3 cm) medyan {np.median(uyum):.2f}, %10 {np.percentile(uyum, 10):.2f}')
        ozet[f'inceltme_{tur + 1}'] = {'duzeltme_medyan_mm': round(float(np.median(dx)) * 1000, 1),
                                       'uyum_3cm_medyan': round(float(np.median(uyum)), 3),
                                       'uyum_3cm_p10': round(float(np.percentile(uyum, 10)), 3)}

    # doluluk ızgarası
    h = a.cozunurluk
    uc = [P @ T[:2, :2].T + T[:2, 2] for _, T, P in taramalar]
    tum = np.vstack(uc)
    lo, hi = tum.min(0) - 0.2, tum.max(0) + 0.2
    W, H = int(math.ceil((hi[0] - lo[0]) / h)), int(math.ceil((hi[1] - lo[1]) / h))
    vurus, bos = np.zeros(W * H, np.int32), np.zeros(W * H, np.int32)

    def indis(Q):
        ij = np.floor((Q - lo) / h).astype(np.int64)
        return ij[:, 1] * W + ij[:, 0]
    for (_, T, P), Q in zip(taramalar, uc):
        vurus += np.bincount(indis(Q), minlength=W * H).astype(np.int32)
        o = T[:2, 2] + T[:2, :2] @ TL[:2, 2]               # lidar merkezi (harita)
        dv = Q - o
        r = np.linalg.norm(dv, axis=1)
        n = np.maximum((r / h).astype(int) - 2, 0)          # uçtan 2 hücre önceye kadar boş
        adim = np.repeat(np.arange(len(r)), n)
        s = np.concatenate([np.arange(k) for k in n]) * h if n.sum() else np.zeros(0)
        F = o + dv[adim] / r[adim, None] * s[:, None]
        bos += np.bincount(np.unique(indis(F)), minlength=W * H).astype(np.int32)   # tarama başına bir kez
    dolu = (vurus >= a.min_vurus) & (vurus / np.maximum(vurus + bos, 1) >= a.oran)
    img = np.full(W * H, 205, np.uint8)
    img[(bos > 0) & ~dolu] = 254
    img[dolu] = 0
    img = img.reshape(H, W)[::-1]                          # pgm satır 0 = üst (y büyük)
    cv2.imwrite(os.path.join(a.cikti, 'yogun.pgm'), img)
    with open(os.path.join(a.cikti, 'yogun.yaml'), 'w') as f:
        yaml.safe_dump({'image': 'yogun.pgm', 'mode': 'trinary', 'resolution': h,
                        'origin': [round(float(lo[0]), 4), round(float(lo[1]), 4), 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.25}, f, sort_keys=False)
    jj, ii = np.divmod(np.nonzero(dolu)[0], W)
    N = np.c_[lo[0] + (ii + 0.5) * h, lo[1] + (jj + 0.5) * h]
    np.save(os.path.join(a.cikti, 'noktalar.npy'), N)

    # eski 5 cm haritayla karşılaştırma
    M = harita_noktalari(a.harita)
    d1, _ = cKDTree(M).query(N)
    d2, _ = cKDTree(N).query(M)
    ozet.update({'cozunurluk_m': h, 'tarama_toplam': len(taramalar), 'dolu_hucre': int(dolu.sum()),
                 'boyut_px': [W, H], 'yogun_to_eski_medyan_cm': round(float(np.median(d1)) * 100, 2),
                 'yogun_to_eski_5cm_ici_%': round(100 * float(np.mean(d1 < 0.05)), 1),
                 'eski_to_yogun_5cm_ici_%': round(100 * float(np.mean(d2 < 0.05)), 1)})
    with open(os.path.join(a.cikti, 'yogun_ozet.yaml'), 'w') as f:
        yaml.safe_dump(json.loads(json.dumps(ozet, default=float)), f, allow_unicode=True, sort_keys=False)
    print(yaml.safe_dump({k: v for k, v in ozet.items() if k != 'kayitlar'}, allow_unicode=True, sort_keys=False))


if __name__ == '__main__':
    main()
