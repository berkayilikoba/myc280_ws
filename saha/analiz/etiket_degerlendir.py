#!/usr/bin/env python3
"""VLMaps kuşbakışı etiketini (vlmaps_calistir.py kusbakisi.npz) elle çizilmiş oda etiketine (config/oda_etiket.yaml)
göre değerlendirir: kategori başına kesinlik / geri çağırma (5 cm hücre, --tol toleransıyla).

  kesinlik     o kategoriyle etiketlenmiş hücrelerin, aynı kategorideki bir çokgene <= tol uzak olan oranı
  geri çağırma çokgen içindeki ve VLMaps'in gözlediği hücrelerin, <= tol içinde o kategori etiketi bulunan oranı
  sorgu        --hedef ile vlmaps_hedef.py çıktısı (metin) verilirse: sorgu merkezi doğru kategorinin çokgeninde mi
Kullanım: etiket_degerlendir.py <vlmaps_cikti_dizini> <oda_etiket.yaml> [--tol 0.10] [--hedef hedef.txt]
"""

import argparse
import os
import re

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('vlmaps')
    ap.add_argument('etiket')
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--hedef', help='vlmaps_hedef.py çıktısı (stdout kaydı)')
    a = ap.parse_args()
    z = np.load(os.path.join(a.vlmaps, 'kusbakisi.npz'))
    lab, adlar, cs = z['etiket'], [str(s) for s in z['adlar']], float(z['cs'])
    r, c = np.nonzero(lab >= 0)
    XY = np.c_[float(z['x0']) + r * cs, float(z['y0']) - c * cs]
    L = lab[r, c]
    E = yaml.safe_load(open(a.etiket))['nesneler']
    # çokgenleri 1 cm ızgarada doldurup nokta kümesine çevir (uzaklık sorguları için)
    cok = {}
    for n in E:
        P = np.array(n['cokgen'], float)
        lo, hi = P.min(0), P.max(0)
        g = 0.01
        W, H = int((hi[0] - lo[0]) / g) + 2, int((hi[1] - lo[1]) / g) + 2
        m = np.zeros((H, W), np.uint8)
        cv2.fillPoly(m, [np.round((P - lo) / g).astype(np.int32)], 1)
        v, u = np.nonzero(m)
        cok.setdefault(n['kategori'], []).append(np.c_[lo[0] + u * g, lo[1] + v * g])
    gozlenen = cKDTree(XY)
    print(f'{"kategori":>10} {"vlmaps_hucre":>12} {"kesinlik":>8} {"cokgen_hucre":>12} {"geri_cagirma":>12}')
    sonuc = {}
    for kat, parcalar in cok.items():
        Q = np.vstack(parcalar)
        k = adlar.index(kat) if kat in adlar else None
        V = XY[L == k] if k is not None else np.zeros((0, 2))
        kes = float(np.mean(cKDTree(Q).query(V)[0] <= a.tol)) if len(V) else None
        Qh = np.unique(np.round(Q / cs).astype(int), axis=0) * cs                 # çokgen hücreleri (5 cm)
        Qg = Qh[gozlenen.query(Qh)[0] <= cs]                                     # VLMaps'in gözlediği kısım
        gc = float(np.mean(cKDTree(V).query(Qg)[0] <= a.tol)) if len(V) and len(Qg) else None
        sonuc[kat] = {'vlmaps_hucre': int(len(V)), 'kesinlik': kes, 'cokgen_gozlenen_hucre': int(len(Qg)), 'geri_cagirma': gc}
        print(f'{kat:>10} {len(V):12d} {kes if kes is not None else float("nan"):8.3f} {len(Qg):12d} '
              f'{gc if gc is not None else float("nan"):12.3f}')
    if a.hedef:
        for satir in open(a.hedef):
            m = re.match(r'(.+?): .*merkez \(([-+\d.]+), ([-+\d.]+)\)', satir)
            if not m or m.group(1) not in cok:
                continue
            p = np.array([[float(m.group(2)), float(m.group(3))]])
            d = float(cKDTree(np.vstack(cok[m.group(1)])).query(p)[0][0])
            sonuc.setdefault(m.group(1), {})['sorgu_merkezi_uzaklik_m'] = round(d, 3)
            print(f'sorgu {m.group(1)}: merkez çokgene {d:.2f} m ({"DOĞRU" if d <= 0.2 else "yanlış"})')
    yaml.safe_dump(sonuc, open(os.path.join(a.vlmaps, 'etiket_degerlendirme.yaml'), 'w'), allow_unicode=True, sort_keys=False)


if __name__ == '__main__':
    main()
