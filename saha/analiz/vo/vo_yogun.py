#!/usr/bin/env python3
"""Yoğun RGB-D görsel odometri (Open3D tensor, CUDA): fotometrik + geometrik (hibrit) hizalama, anahtar kareye göre.

Her kare anahtar kareye çok ölçekli hibrit RGB-D odometri ile hizalanır (başlangıç: önceki çözüm). Sonuç 6 SD kamera
hareketi; gövdeye taşınıp düzleme izdüşürülür. Derinliği zemine tutarsız kareler (ortak.zemin_orani) atlanır: yoğun
yöntem iki karenin de derinliğini kullanır.

Varyantlar: yogun (x, y, yaw görüntüden), yogun_jiro (yaw IMU jiroskobundan, öteleme görüntüden).
Kullanım: vo_yogun.py <koşu> [--yontem hibrit|nokta_duzlem|yogunluk]
"""

import argparse
import math
import time

import numpy as np
import open3d as o3d
import open3d.core as o3c

from ortak import KOSULAR, ara, ekf_esdeger, imu_yaw, kamera_kur, kareler, kaydet, se2, sensor_oku, unse2

YONTEM = {'hibrit': o3d.t.pipelines.odometry.Method.Hybrid,
          'nokta_duzlem': o3d.t.pipelines.odometry.Method.PointToPlane,
          'yogunluk': o3d.t.pipelines.odometry.Method.Intensity}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kosu', choices=list(KOSULAR))
    ap.add_argument('--yontem', default='hibrit', choices=list(YONTEM))
    ap.add_argument('--zmax', type=float, default=4.0)
    ap.add_argument('--kf_t', type=float, default=0.25)
    ap.add_argument('--kf_yaw', type=float, default=15.0)
    ap.add_argument('--kf_uyum', type=float, default=0.25,
                    help='uyum (fitness) bunun altına inince anahtar kare yenilenir')
    ap.add_argument('--min_uyum', type=float, default=0.1)
    ap.add_argument('--ek', default='')
    a = ap.parse_args()

    dev = o3c.Device('CUDA:0' if o3c.cuda.is_available() else 'CPU:0')
    S = sensor_oku(a.kosu)
    kam = kamera_kur(a.kosu, S)
    K, T_b_c = kam['K'], kam['T_b_c']
    T_c_b = np.linalg.inv(T_b_c)
    E = ekf_esdeger(S)
    Y = imu_yaw(S)
    Ki = o3c.Tensor(K, o3c.float64)
    kriter = [o3d.t.pipelines.odometry.OdometryConvergenceCriteria(n) for n in (20, 10, 5)]
    ad = ('yogun' if a.yontem == 'hibrit' else f'yogun_{a.yontem}') + a.ek

    def rgbd(bgr, dm):
        renk = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(bgr[:, :, ::-1]))).to(dev)
        d = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(dm, dtype=np.float32))).to(dev)
        return o3d.t.geometry.RGBDImage(renk.to(o3c.float32, scale=1.0 / 255), d)

    def taban(M):                       # kamera hareketi (şimdiki kamera anahtar kamerada) -> gövde, düzleme
        B = T_b_c @ M @ T_c_b
        return np.array([B[0, 3], B[1, 3], math.atan2(B[1, 0], B[0, 0])])

    var = ('serbest', 'jiro')
    cikti = {v: [] for v in var}
    kf = None
    say = dict(kare=0, atlanan=0, basarisiz=0, kf=0)
    uy = []
    t_bas = time.time()
    for t, bgr, dm, zr in kareler(a.kosu, S, kam):
        say['kare'] += 1
        if abs(zr - 1) > 0.1:
            say['atlanan'] += 1
            continue
        dm = np.where(dm < a.zmax, dm, 0).astype(np.float32)
        cur = rgbd(bgr, dm)
        yaw_imu = float(np.interp(t, Y[:, 0], Y[:, 1]))
        ekf_t = se2(*ara(E, t)[:3])

        def yeni_kf():
            say['kf'] += 1
            return dict(rgbd=cur, poz={v: se2(*cikti[v][-1][1:]) for v in var}, yaw=yaw_imu, ekf=ekf_t,
                        M=np.eye(4))
        if kf is None:
            for v in var:
                cikti[v].append((t, 0.0, 0.0, 0.0))
            kf = yeni_kf()
            continue
        # başlangıç: önceki çözüm; yoksa EKF-eşdeğeri göreli hareketten
        M0 = kf['M']
        if np.allclose(M0, np.eye(4)):
            p = unse2(np.linalg.inv(kf['ekf']) @ ekf_t)
            B = np.eye(4)
            B[:2, :2], B[:2, 3] = se2(*p)[:2, :2], p[:2]
            M0 = T_c_b @ B @ T_b_c
        try:
            # kaynak = şimdiki, hedef = anahtar: dönüşüm şimdiki kamera noktalarını anahtar kameraya taşır
            r = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(
                cur, kf['rgbd'], Ki, o3c.Tensor(M0, o3c.float64), 1.0, a.zmax, kriter, YONTEM[a.yontem])
            M, uyum = r.transformation.cpu().numpy(), r.fitness
        except RuntimeError:
            M, uyum = None, 0.0
        uy.append(uyum)
        if M is None or uyum < a.min_uyum:
            say['basarisiz'] += 1
            for v in var:
                adim = np.linalg.inv(se2(*ara(E, cikti[v][-1][0])[:3])) @ ekf_t
                if v == 'jiro':
                    x, y_, _ = unse2(adim)
                    adim = se2(x, y_, yaw_imu - float(np.interp(cikti[v][-1][0], Y[:, 0], Y[:, 1])))
                cikti[v].append((t, *unse2(se2(*cikti[v][-1][1:]) @ adim)))
            kf = yeni_kf()
            continue
        kf['M'] = M
        p = taban(M)
        cikti['serbest'].append((t, *unse2(kf['poz']['serbest'] @ se2(*p))))
        cikti['jiro'].append((t, *unse2(kf['poz']['jiro'] @ se2(p[0], p[1], yaw_imu - kf['yaw']))))
        if uyum < a.kf_uyum or math.hypot(p[0], p[1]) > a.kf_t or abs(math.degrees(p[2])) > a.kf_yaw:
            kf = yeni_kf()
        if say['kare'] % 300 == 0:
            print(f'  {say["kare"]} kare, {time.time() - t_bas:.0f} s, kf {say["kf"]}, başarısız {say["basarisiz"]}, '
                  f'uyum medyan {np.median(uy[-300:]):.2f}', flush=True)

    for v in var:
        Yv = np.array(cikti[v])
        Yv[:, 3] = np.unwrap(Yv[:, 3])
        kaydet(a.kosu, ad + ('' if v == 'serbest' else '_jiro'), Yv,
               dict(say, uyum_medyan=float(np.median(uy)), sure_s=round(time.time() - t_bas)))
    print(f'{a.kosu} {ad}: {say}, uyum medyan {np.median(uy):.2f}, {time.time() - t_bas:.0f} s')


if __name__ == '__main__':
    main()
