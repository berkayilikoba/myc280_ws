#!/usr/bin/env python3
"""Özellik tabanlı RGB-D görsel odometri, düzlemsel (SE(2)) çözüm, anahtar kareye göre.

Her kare anahtar kareyle eşlenir (SIFT, oran testi + karşılıklı kontrol); anahtar karedeki eşleşmelerin 3B noktası
derinlikten (gövde çerçevesinde). Aykırılar solvePnPRansac (6 SD) ile ayıklanır, poz gövdenin düzlem hareketi
(x, y, yaw; z/roll/pitch sabit, kamera dış kalibrasyonu ile) olarak yeniden izdüşüm hatasından Huber ile çözülür.
Anahtar kare, eşleşen nokta azalınca veya hareket büyüyünce değişir; arada kayma birikmez (duruşta sıfır).

Varyantlar (aynı geçişte):
  ozellik         x, y, yaw görüntüden (düzlem kısıtlı)
  ozellik_jiro    yaw = IMU jiroskop integrali (anahtar kareye göre fark), görüntü yalnız x, y
  ozellik_pnp6    6 SD PnP (LM inceltme), sonra düzleme izdüşüm

Derinliği zemine tutarsız kareler (ortak.zemin_orani) anahtar kare yapılmaz; yalnız hedef kare olarak kullanılır.

Başarısız karede (eşleşme < --min_inlier) hareket EKF-eşdeğeri artıştan alınır, kare yeni anahtar kare olur.
Kullanım: vo_ozellik.py <koşu> [--her 1] [--ozellik sift|orb]
"""

import argparse
import math
import time

import cv2
import numpy as np
from scipy.optimize import least_squares

from ortak import KOSULAR, ara, ekf_esdeger, imu_yaw, kamera_kur, kareler, kaydet, se2, sensor_oku, unse2


def se3_duz(x, y, a):
    M = np.eye(4)
    M[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    M[:2, 3] = (x, y)
    return M


def izdus(p, Xb, K, T_c_b):
    """Anahtar kare gövdesindeki noktalar -> şimdiki kare pikselleri; p = şimdiki gövdenin anahtar karedeki pozu."""
    Tcb = T_c_b @ np.linalg.inv(se3_duz(*p))
    Xc = Xb @ Tcb[:3, :3].T + Tcb[:3, 3]
    z = np.maximum(Xc[:, 2], 1e-3)
    return np.c_[K[0, 0] * Xc[:, 0] / z + K[0, 2], K[1, 1] * Xc[:, 1] / z + K[1, 2]], Xc[:, 2]


def coz(p0, Xb, uv, K, T_c_b, yaw_sabit=None):
    if yaw_sabit is None:
        f = lambda p: (izdus(p, Xb, K, T_c_b)[0] - uv).ravel()
        r = least_squares(f, p0, loss='huber', f_scale=1.0, x_scale=[0.05, 0.05, 0.02])
        return r.x
    f = lambda q: (izdus([q[0], q[1], yaw_sabit], Xb, K, T_c_b)[0] - uv).ravel()
    r = least_squares(f, p0[:2], loss='huber', f_scale=1.0, x_scale=[0.05, 0.05])
    return np.array([r.x[0], r.x[1], yaw_sabit])


class Ozellik:
    def __init__(self, tur):
        if tur == 'sift':
            self.d = cv2.SIFT_create(nfeatures=2000, contrastThreshold=0.02)
            self.m = cv2.BFMatcher(cv2.NORM_L2)
        else:
            self.d = cv2.ORB_create(nfeatures=3000, fastThreshold=10)
            self.m = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.clahe = cv2.createCLAHE(2.0, (8, 8))

    def cikar(self, bgr):
        g = self.clahe.apply(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        kp, ds = self.d.detectAndCompute(g, None)
        return np.array([k.pt for k in kp], np.float32).reshape(-1, 2), ds

    def esle(self, d1, d2, oran=0.8):
        if d1 is None or d2 is None or len(d1) < 2 or len(d2) < 2:
            return np.zeros((0, 2), int)
        a = self.m.knnMatch(d1, d2, k=2)
        b = {m.queryIdx: m.trainIdx for m in self.m.match(d2, d1)}
        e = [(m.queryIdx, m.trainIdx) for m, n in (x for x in a if len(x) == 2)
             if m.distance < oran * n.distance and b.get(m.trainIdx) == m.queryIdx]
        return np.array(e, int).reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kosu', choices=list(KOSULAR))
    ap.add_argument('--her', type=int, default=1)
    ap.add_argument('--ozellik', default='sift', choices=['sift', 'orb'])
    ap.add_argument('--min_inlier', type=int, default=25)
    ap.add_argument('--gecikme_ms', type=float, default=0.0, help='derinlik damgası düzeltmesi')
    ap.add_argument('--kf_inlier', type=int, default=60, help='bundan az eşleşmede anahtar kare yenilenir')
    ap.add_argument('--kf_t', type=float, default=0.30)
    ap.add_argument('--kf_yaw', type=float, default=20.0)
    ap.add_argument('--zmax', type=float, default=4.0)
    ap.add_argument('--ek', default='', help='çıktı adına ek')
    ap.add_argument('--bas', type=float, help='koşu başından saniye (hata ayıklama)')
    ap.add_argument('--son', type=float)
    ap.add_argument('--ayikla', action='store_true')
    a = ap.parse_args()

    S = sensor_oku(a.kosu)
    kam = kamera_kur(a.kosu, S)
    K, T_b_c = kam['K'], kam['T_b_c']
    T_c_b = np.linalg.inv(T_b_c)
    E = ekf_esdeger(S)
    Y = imu_yaw(S)
    oz = Ozellik(a.ozellik)
    ad = f'ozellik{"_orb" if a.ozellik == "orb" else ""}{a.ek}'

    var = ('serbest', 'jiro', 'pnp6')
    kf = None                  # dict: t, uv, ds, X (gövde), dünya pozları (varyant başına), son tahmin
    cikti = {v: [] for v in var}
    say = dict(kare=0, basarisiz=0, kf=0, kotu_derinlik=0)
    inl = []
    t_bas = time.time()
    t0 = S['odom'][0, 0]
    for t, bgr, dm, zr in kareler(a.kosu, S, kam, her=a.her, gecikme=a.gecikme_ms / 1000,
                                  bas=None if a.bas is None else t0 + a.bas,
                                  son=None if a.son is None else t0 + a.son):
        say['kare'] += 1
        iyi = abs(zr - 1) <= 0.1               # derinlik zemine tutarlı: anahtar kare olabilir
        say['kotu_derinlik'] += not iyi
        uv, ds = oz.cikar(bgr)
        z = np.zeros(len(uv))
        if len(uv):
            ui, vi = np.clip(np.round(uv[:, 0]).astype(int), 0, 639), np.clip(np.round(uv[:, 1]).astype(int), 0, 479)
            z = dm[vi, ui]
        gecerli = (z > 0.25) & (z < a.zmax)
        Xc = np.c_[(uv[:, 0] - K[0, 2]) * z / K[0, 0], (uv[:, 1] - K[1, 2]) * z / K[1, 1], z]
        Xb = Xc @ T_b_c[:3, :3].T + T_b_c[:3, 3]
        yaw_imu = float(np.interp(t, Y[:, 0], Y[:, 1]))
        ekf_t = se2(*ara(E, t)[:3])

        def yeni_kf(pozlar):
            say['kf'] += 1
            return dict(t=t, ds=ds[gecerli] if ds is not None else None, X=Xb[gecerli], uv=uv[gecerli],
                        poz=pozlar, yaw_imu=yaw_imu, ekf=ekf_t, son={v: np.zeros(3) for v in var}, gecersiz=False)

        if not cikti['serbest']:
            for v in var:
                cikti[v].append((t, 0.0, 0.0, 0.0))
            if iyi:
                kf = yeni_kf({v: np.eye(3) for v in var})
            continue

        sonuc, n_in, e = {}, 0, np.zeros((0, 2), int)
        tahmin = dyaw = None
        if kf is not None and not kf['gecersiz']:
            e = oz.esle(kf['ds'], ds)
            tahmin = np.array(unse2(np.linalg.inv(kf['ekf']) @ ekf_t))     # EKF-eşdeğeri göreli hareket
            dyaw = yaw_imu - kf['yaw_imu']
        if len(e) >= a.min_inlier:
            X3, u2 = kf['X'][e[:, 0]], uv[e[:, 1]].astype(np.float64)
            Xo = X3 @ T_c_b[:3, :3].T + T_c_b[:3, 3]
            ok, rv, tv, idx = cv2.solvePnPRansac(Xo, u2, K, None, iterationsCount=300, reprojectionError=3.0,
                                                 flags=cv2.SOLVEPNP_SQPNP)
            if ok and idx is not None and len(idx) >= a.min_inlier:
                i = idx.ravel()
                for v in ('serbest', 'jiro'):
                    p = coz(kf['son'][v] if np.any(kf['son'][v]) else tahmin, X3[i], u2[i], K, T_c_b,
                            dyaw if v == 'jiro' else None)
                    r, _ = izdus(p, X3, K, T_c_b)
                    j = np.nonzero(np.hypot(*(r - u2).T) < 2.5)[0]
                    if len(j) >= a.min_inlier:
                        p = coz(p, X3[j], u2[j], K, T_c_b, dyaw if v == 'jiro' else None)
                    sonuc[v] = p
                    n_in = max(n_in, len(j))
                rv, tv = cv2.solvePnPRefineLM(Xo[i], u2[i], K, None, rv, tv)
                Tc = np.eye(4)
                Tc[:3, :3], Tc[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
                M = T_b_c @ np.linalg.inv(Tc) @ T_c_b                   # şimdiki gövde, anahtar kare gövdesinde
                sonuc['pnp6'] = np.array([M[0, 3], M[1, 3], math.atan2(M[1, 0], M[0, 0])])
        if not sonuc or n_in < a.min_inlier:
            if kf is not None and not kf['gecersiz']:
                say['basarisiz'] += 1
            for v in var:
                adim = np.linalg.inv(se2(*ara(E, cikti[v][-1][0])[:3])) @ ekf_t
                if v == 'jiro':                                         # yön jiroskoptan
                    x, y_, _ = unse2(adim)
                    adim = se2(x, y_, yaw_imu - float(np.interp(cikti[v][-1][0], Y[:, 0], Y[:, 1])))
                cikti[v].append((t, *unse2(se2(*cikti[v][-1][1:]) @ adim)))
            if iyi:
                kf = yeni_kf({v: se2(*cikti[v][-1][1:]) for v in var})
            elif kf is not None:
                kf['gecersiz'] = True
            inl.append(0)
            continue
        inl.append(n_in)
        for v in var:
            kf['son'][v] = sonuc[v]
            cikti[v].append((t, *unse2(kf['poz'][v] @ se2(*sonuc[v]))))
        p = sonuc['serbest']
        if a.ayikla:
            print(f'{t - t0:6.2f} kf {say["kf"]} n {len(e)}/{n_in} göreli {np.round(p * [100, 100, 57.3], 2)} '
                  f'ekf {np.round(tahmin * [100, 100, 57.3], 2)} zemin {zr:.2f}')
        if iyi and (n_in < a.kf_inlier or math.hypot(p[0], p[1]) > a.kf_t or abs(math.degrees(p[2])) > a.kf_yaw):
            kf = yeni_kf({v: se2(*cikti[v][-1][1:]) for v in var})
        if say['kare'] % 300 == 0:
            print(f'  {say["kare"]} kare, {time.time() - t_bas:.0f} s, kf {say["kf"]}, başarısız {say["basarisiz"]}, '
                  f'eşleşme medyan {int(np.median(inl[-300:]))}', flush=True)

    for v in var:
        Yv = np.array(cikti[v])
        Yv[:, 3] = np.unwrap(Yv[:, 3])
        kaydet(a.kosu, ad + ('' if v == 'serbest' else '_' + v), Yv,
               dict(kare=say['kare'], basarisiz=say['basarisiz'], anahtar_kare=say['kf'],
                    kotu_derinlik=say['kotu_derinlik'], eslesme_medyan=int(np.median(inl)),
                    sure_s=round(time.time() - t_bas)))
    print(f'{a.kosu} {ad}: {say}, eşleşme medyan {int(np.median(inl))}, {time.time() - t_bas:.0f} s')


if __name__ == '__main__':
    main()
