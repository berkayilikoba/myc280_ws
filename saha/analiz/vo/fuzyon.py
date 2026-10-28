#!/usr/bin/env python3
"""VO + IMU + tekerlek füzyonu (nedensel, çevrimiçi kullanılabilir): yön IMU'dan, mesafe ölçeği VO'dan.

Ölçümler (koşular, düz kenarlar): tekerlek mesafesi lidar referansına göre %1-1.6 kısa (EKF'in konum hatasının ana
kaynağı), D435 VO ölçeği 0.993-1.000; IMU yönü <1°. Yön VO'dan alınmaz (yerinde dönüşte VO 2-4 cm sahte öteleme ve
birkaç derece yön hatası veriyor). İki yöntem:

  vo_hiz   ileri yer değiştirme VO'nun gövde çerçevesindeki x artışı (VO kareleri arası), yerinde dönüşte (tekerlek
           |vx| < --dur) sıfır; yanal sıfır (kaymayan taban); VO boşluğunda tekerlek.
  olcek    tekerlek vx × s(t); s = düz gidişteki birikmiş VO ileri mesafesi / tekerlek mesafesi (o ana kadar,
           --min_yol metreden önce s = 1). Tekerlek pürüzsüzlüğü + VO ölçeği.

  vo_tam   gövde çerçevesindeki VO ötelemesi (x ve y, dönüşler dahil) + IMU yönü; öz kalibrasyonlu VO ile
           (oz_kalibrasyon.py) dönüşteki sahte öteleme giderilmiş olur.

Kullanım: fuzyon.py <koşu> <vo_yorunge_adi> [--ad cikti_adi]
"""

import argparse
import math
import os

import numpy as np

from ortak import CALISMA, KOSULAR, imu_yaw, kaydet, se2, sensor_oku


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kosu', choices=list(KOSULAR))
    ap.add_argument('vo')
    ap.add_argument('--dur', type=float, default=0.03, help='m/s; altında yerinde dönüş/duruş sayılır')
    ap.add_argument('--min_yol', type=float, default=0.5)
    ap.add_argument('--donus', type=float, default=0.05, help='rad/s; düz gidiş ölçek tahmini için |wz| sınırı')
    a = ap.parse_args()
    S = sensor_oku(a.kosu)
    V = np.loadtxt(os.path.join(CALISMA, a.kosu, 'yorunge', a.vo + '.csv'), delimiter=',')
    O, I, Y = S['odom'], S['imu'], imu_yaw(S)
    yaw = lambda t: np.interp(t, Y[:, 0], Y[:, 1])
    vx = lambda t: np.interp(t, O[:, 0], O[:, 4])

    # VO ileri artışları (gövde çerçevesi), her VO aralığı için
    dV = []
    for i in range(1, len(V)):
        M = np.linalg.inv(se2(*V[i - 1, 1:4])) @ se2(*V[i, 1:4])
        dV.append((V[i - 1, 0], V[i, 0], M[0, 2]))
    dV = np.array(dV)

    # --- vo_hiz
    out, x, y = [], 0.0, 0.0
    for t0, t1, dx in dV:
        if t1 - t0 > 0.3:                                   # VO boşluğu: tekerlek
            tt = np.linspace(t0, t1, max(2, int((t1 - t0) / 0.02)))
            for u, w in zip(tt[:-1], tt[1:]):
                d = 0.5 * (vx(u) + vx(w)) * (w - u)
                ym = 0.5 * (yaw(u) + yaw(w))
                x, y = x + d * math.cos(ym), y + d * math.sin(ym)
        else:
            if abs(vx(0.5 * (t0 + t1))) < a.dur:
                dx = 0.0
            ym = 0.5 * (yaw(t0) + yaw(t1))
            x, y = x + dx * math.cos(ym), y + dx * math.sin(ym)
        out.append((t1, x, y, yaw(t1)))
    Yh = np.array([(V[0, 0], 0.0, 0.0, yaw(V[0, 0]))] + out)
    Yh[:, 3] -= Yh[0, 3]
    kaydet(a.kosu, f'fz_vohiz_{a.vo}', Yh)

    # --- vo_tam
    out, W = [(V[0, 0], 0.0, 0.0, 0.0)], np.eye(3)
    for i in range(1, len(V)):
        M = np.linalg.inv(se2(*V[i - 1, 1:4])) @ se2(*V[i, 1:4])
        W = se2(*W[:2, 2], yaw(V[i - 1, 0]) - yaw(V[0, 0])) @ se2(M[0, 2], M[1, 2], 0)
        W = se2(W[0, 2], W[1, 2], yaw(V[i, 0]) - yaw(V[0, 0]))
        out.append((V[i, 0], W[0, 2], W[1, 2], yaw(V[i, 0]) - yaw(V[0, 0])))
    kaydet(a.kosu, f'fz_votam_{a.vo}', np.array(out))

    # --- olcek (tekerlek örneklerinde)
    k = (O[:, 0] >= V[0, 0]) & (O[:, 0] <= V[-1, 0])
    T = O[k, 0]
    yv, wv = 0.0, 0.0                    # düz gidişte birikmiş VO ve tekerlek ileri mesafesi
    s, x, y, out, sl = 1.0, 0.0, 0.0, [(T[0], 0.0, 0.0, 0.0)], []
    j = 0
    for u, w in zip(T[:-1], T[1:]):
        while j < len(dV) and dV[j, 1] <= w:                  # bu ana kadar biten VO aralıkları
            t0, t1, dx = dV[j]
            wz = np.interp(np.linspace(t0, t1, 4), I[:, 0], I[:, 1])
            v0, v1 = vx(t0), vx(t1)
            if t1 - t0 < 0.3 and min(v0, v1) > 0.1 and np.abs(wz).max() < a.donus:
                yv += dx
                wv += np.trapz(vx(np.linspace(t0, t1, 5)), np.linspace(t0, t1, 5))
            j += 1
        if wv > a.min_yol:
            s = yv / wv
        d = s * 0.5 * (vx(u) + vx(w)) * (w - u)
        ym = 0.5 * (yaw(u) + yaw(w))
        x, y = x + d * math.cos(ym), y + d * math.sin(ym)
        out.append((w, x, y, yaw(w) - yaw(T[0])))
        sl.append(s)
    kaydet(a.kosu, f'fz_olcek_{a.vo}', np.array(out), dict(son_olcek=float(s), vo_yol=float(yv), teker_yol=float(wv)))
    print(f'{a.kosu} {a.vo}: ölçek sonu {s:.4f} (VO {yv:.2f} m / tekerlek {wv:.2f} m düz gidiş)')


if __name__ == '__main__':
    main()
