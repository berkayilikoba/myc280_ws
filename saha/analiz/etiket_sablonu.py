#!/usr/bin/env python3
"""Oda etiketleme şablonu (olcum_protokolu.md 5): harita PNG'si üzerine metre ızgarası -> basılıp üstüne nesneler çizilir
(ya da bir görüntü düzenleyicide); köşe koordinatları ızgaradan okunup config/oda_etiket.yaml'a yazılır.

  ince çizgi her 0.25 m, kalın her 1 m (etiketli), kırmızı artı = harita orijini (robot başlangıç işareti, poz_ver.sh),
  ok = başlangıç yönü (+x). Ölçek çubuğu 1 m. Varsayılan 100 piksel/m (1 hücre 5 cm = 5 px); A4'e %100 basınca ~1:50.
Kullanım: etiket_sablonu.py <harita.yaml> <cikti.png> [--ppm 100] [--yaml_ornek ../config/oda_etiket.yaml]
"""

import argparse
import os

import cv2
import numpy as np
import yaml

ORNEK = """# Oda gerçek etiketi (ground truth), harita çerçevesinde (map, m). etiket_sablonu.py ızgarasından ya da metreyle.
# Her nesne: kategori (vlmaps_calistir.py KATEGORI adlarıyla aynı: door, chair, table, cabinet, shelf, box, window,
# trash can, ...), çokgen köşeleri [x, y] (saat yönü fark etmez), isteğe bağlı yükseklik aralığı ve not.
# Değerlendirme: etiket_degerlendir.py <vlmaps_cikti_dizini> oda_etiket.yaml
harita: ~/limo_veri/oturum_20261009_162308/harita.yaml
tarih: 2026-10-XX
olcen: ""
nesneler:
  - kategori: door
    cokgen: [[0.00, 0.00], [0.90, 0.00], [0.90, 0.05], [0.00, 0.05]]
    yukseklik_m: [0.0, 2.0]
    not: "örnek — sil"
  - kategori: table
    cokgen: [[1.0, 1.0], [2.2, 1.0], [2.2, 1.8], [1.0, 1.8]]
    yukseklik_m: [0.0, 0.75]
    not: "örnek — sil"
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('harita')
    ap.add_argument('cikti')
    ap.add_argument('--ppm', type=int, default=100, help='piksel / metre')
    ap.add_argument('--yaml_ornek', help='boş etiket dosyası şablonu yaz (varsa üzerine yazmaz)')
    a = ap.parse_args()
    h = yaml.safe_load(open(a.harita))
    img = cv2.imread(os.path.join(os.path.dirname(os.path.abspath(a.harita)), h['image']), cv2.IMREAD_GRAYSCALE)
    res, (ox, oy, _) = h['resolution'], h['origin']
    s = a.ppm * res
    g = cv2.resize(img, (int(img.shape[1] * s), int(img.shape[0] * s)), interpolation=cv2.INTER_NEAREST)
    g = np.where(g < 100, 60, np.where(g > 250, 255, 225)).astype(np.uint8)   # dolu koyu gri, bilinmeyen açık gri
    pay = 60                                                                   # etiketler için kenar boşluğu (px)
    H0, W0 = g.shape
    can = np.full((H0 + 2 * pay, W0 + 2 * pay, 3), 255, np.uint8)
    can[pay:pay + H0, pay:pay + W0] = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    x_max, y_max = ox + img.shape[1] * res, oy + img.shape[0] * res

    def pik(x, y):
        return int(round(pay + (x - ox) * a.ppm)), int(round(pay + (y_max - y) * a.ppm))
    for adim, kal, renk in ((0.25, 1, (200, 170, 140)), (1.0, 2, (150, 90, 30))):
        for x in np.arange(np.ceil(ox / adim) * adim, x_max + 1e-6, adim):
            cv2.line(can, pik(x, oy), pik(x, y_max), renk, kal)
        for y in np.arange(np.ceil(oy / adim) * adim, y_max + 1e-6, adim):
            cv2.line(can, pik(ox, y), pik(x_max, y), renk, kal)
    yaz = dict(fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(120, 60, 0), thickness=1)
    for x in np.arange(np.ceil(ox), x_max + 1e-6, 1.0):
        u, _ = pik(x, 0)
        cv2.putText(can, f'x={x + 0.0:+.0f}', (u - 22, pay - 10), **yaz)
        cv2.putText(can, f'x={x + 0.0:+.0f}', (u - 22, pay + H0 + 25), **yaz)
    for y in np.arange(np.ceil(oy), y_max + 1e-6, 1.0):
        _, v = pik(0, y)
        cv2.putText(can, f'y={y + 0.0:+.0f}', (3, v + 5), **yaz)
        cv2.putText(can, f'y={y + 0.0:+.0f}', (pay + W0 + 4, v + 5), **yaz)
    u0, v0 = pik(0, 0)
    cv2.drawMarker(can, (u0, v0), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.arrowedLine(can, (u0, v0), pik(0.5, 0), (0, 0, 255), 2, tipLength=0.25)
    cv2.putText(can, 'baslangic (0,0), +x', (u0 + 6, v0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    xb, yb = pay, can.shape[0] - 15                                            # ölçek çubuğu
    cv2.line(can, (xb, yb), (xb + a.ppm, yb), (0, 0, 0), 3)
    cv2.putText(can, '1 m', (xb + a.ppm + 6, yb + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(can, f'{os.path.basename(a.harita)}  ince 0.25 m, kalin 1 m  (harita cercevesi)',
                (xb + a.ppm + 50, yb + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.imwrite(a.cikti, can)
    print(f'{a.cikti}: {can.shape[1]}x{can.shape[0]} px, x {ox:.2f}..{x_max:.2f}, y {oy:.2f}..{y_max:.2f} m')
    if a.yaml_ornek and not os.path.exists(a.yaml_ornek):
        open(a.yaml_ornek, 'w').write(ORNEK)
        print(f'{a.yaml_ornek} yazıldı')


if __name__ == '__main__':
    main()
