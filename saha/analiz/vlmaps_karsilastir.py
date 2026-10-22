#!/usr/bin/env python3
"""İki VLMaps kuşbakışı etiket haritasını (vlmaps_calistir.py kusbakisi.npz) karşılaştırır: kamera A ↔ kamera B.

İki harita da lidar_poz.py pozuyla harita çerçevesinde olmalı. Ölçütler (5 cm hücre):
  kapsama      gözlenen hücre alanı (m²)
  uyum         ikisinin de gözlediği hücrelerde aynı kategori oranı
  duvar        lidar haritası (map_server) duvarlarına göre: kesinlik = "wall" hücrelerinin <10 cm'inde harita duvarı olan
               oranı; geri çağırma = harita duvar hücrelerinin <10 cm'inde "wall" hücresi olan oranı
Kullanım: vlmaps_karsilastir.py <A_dizini> <B_dizini> <harita.yaml> [--adlar Dabai D435] [--cikti fark.png]
"""

import argparse
import os

import cv2
import numpy as np
from scipy.spatial import cKDTree

from lidar_poz import harita_noktalari


def yukle(d):
    z = np.load(os.path.join(d, 'kusbakisi.npz'))
    lab = z['etiket']
    r, c = np.nonzero(lab >= 0)
    xy = np.c_[float(z['x0']) + r * float(z['cs']), float(z['y0']) - c * float(z['cs'])]
    return xy, lab[r, c], [str(a) for a in z['adlar']]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('harita')
    ap.add_argument('--adlar', nargs=2, default=['A', 'B'])
    ap.add_argument('--cikti')
    a = ap.parse_args()
    M = harita_noktalari(a.harita)
    agacM = cKDTree(M)
    veri = [yukle(a.a), yukle(a.b)]
    adlar = veri[0][2]
    w = adlar.index('wall')
    sonuc = {}
    for ad, (xy, lab, _) in zip(a.adlar, veri):
        duvar = xy[lab == w]
        d, _ = agacM.query(duvar) if len(duvar) else (np.array([np.inf]), None)
        dg, _ = cKDTree(duvar).query(M) if len(duvar) else (np.full(len(M), np.inf), None)
        sonuc[ad] = {'kapsama_m2': round(len(xy) * 0.0025, 2),
                     'duvar_kesinlik_10cm': round(float(np.mean(d < 0.10)), 3),
                     'duvar_geri_cagirma_10cm': round(float(np.mean(dg < 0.10)), 3),
                     'dagilim_%': {n: round(100 * float(np.mean(lab == k)), 1) for k, n in enumerate(adlar)}}
    # ortak hücreler: 5 cm ızgaraya yuvarla
    anahtar = [{(int(round(x / 0.05)), int(round(y / 0.05))): l for (x, y), l in zip(xy, lab)} for xy, lab, _ in veri]
    ortak = set(anahtar[0]) & set(anahtar[1])
    uyum = np.mean([anahtar[0][k] == anahtar[1][k] for k in ortak]) if ortak else 0.0
    print(f'ortak hücre {len(ortak)} ({len(ortak) * 0.0025:.1f} m²), etiket uyumu {uyum:.3f}')
    for ad, s in sonuc.items():
        print(f'{ad}: kapsama {s["kapsama_m2"]} m², duvar kesinlik {s["duvar_kesinlik_10cm"]}, '
              f'geri çağırma {s["duvar_geri_cagirma_10cm"]}')
        print('   ' + ', '.join(f'{n} %{v}' for n, v in s['dagilim_%'].items() if v >= 1))
    if a.cikti:   # kuşbakışı: gri = harita duvarı, mavi = A duvarı, turuncu = B duvarı
        allxy = np.vstack([M] + [v[0] for v in veri])
        lo, hi = allxy.min(0) - 0.2, allxy.max(0) + 0.2
        s = 100                                      # piksel/m
        img = np.full((int((hi[0] - lo[0]) * s) + 1, int((hi[1] - lo[1]) * s) + 1, 3), 255, np.uint8)
        def nokta(p, renk, r=2):
            for x, y in p:
                cv2.circle(img, (int((hi[1] - y) * s), int((hi[0] - x) * s)), r, renk, -1)
        nokta(M, (160, 160, 160), 3)
        for (xy, lab, _), renk in zip(veri, [(200, 90, 0), (0, 140, 255)]):
            nokta(xy[lab == w], renk, 1)
        cv2.putText(img, f'gri: harita  mavi: {a.adlar[0]} duvar  turuncu: {a.adlar[1]} duvar', (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        cv2.imwrite(a.cikti, img)


if __name__ == '__main__':
    main()
