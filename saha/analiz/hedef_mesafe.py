#!/usr/bin/env python3
"""Bilinen mesafe testi (olcum_protokolu.md 1.2): robot sabit, düz levha lazer metreyle ölçülen mesafelere taşınır.

Lidardan bağımsız mutlak referans. Kayıt: kayit_baslat.sh (sensor/ + kamera/), robot hiç hareket etmez.
Duraklar: lidar ön mesafesinin (on_mesafe) >= --min_s s boyunca 1 cm içinde sabit kaldığı aralıklar (levha
taşınırken değişir). Her durak en yakın --gercek değerine eşlenir (adımlar 0.5 m olduğundan karışmaz; >10 cm uyarı).
  gercek_b    = gercek + --ref_x            (lazer metrenin sıfır noktası base_link x'te; ör. ön tampon, ölçülecek)
  lidar_b     = base_link önündeki (|y| < 5 cm) lidar noktalarının x medyanı (laser_duz TF'i ile; x 0.137 tahmin)
  kamera_b    = merkez 10x10 piksel derinlik medyanı + kamera x (kalibrasyon dosyası / tf_static)
  hata        = ölçülen_b - gercek_b
Her sensör için hata = a + b·gercek doğrusu: a = sabit kısım (sensör konumu + ref_x hatası, cm), b = ölçek hatası (%).
b, ref_x ve konum tahminlerinden bağımsızdır: asıl aranan budur. Kamera-lidar farkının doğrusu da verilir.
Kullanım (Humble): hedef_mesafe.py <kayit_dizini> --gercek 0.5 1.0 1.5 2.0 2.5 3.0 --ref_x 0.16 [--kamerasiz]
Çıktı: <kayit>/hedef_mesafe.yaml, hedef_mesafe.png
"""

import argparse
import json
import math
import os

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage

from bag2vlmaps import T, damga, kamera_dis, okuyucu, q2R, zincir

BURADA = os.path.dirname(os.path.abspath(__file__))
LIDAR = (0.137, 0.015, -0.1686)        # base_link -> laser_duz x, y, yaw (ortam.sh lidar_tf_yayinla)


def on_mesafe(aci, r, lidar=LIDAR, yari=0.05):
    """base_link'in tam önündeki yüzeye uzaklık (base_link x'i boyunca): tarama gövde çerçevesine çevrilir, |y| < yari
    olan noktaların x medyanı. 27.10 düzeltmesi: /scan_sabit laser_duz çerçevesinde ve bu çerçeve gövdeye göre -9.66°
    dönük; eskiden lidar açısı 0 ± 3° "ön" alınıyordu = gövdenin 9.66° sağı -> dik yüzeyde menzil %1.4 uzun."""
    k = np.isfinite(r) & (r > 0.05)
    a = aci[k] + lidar[2]
    x, y = lidar[0] + r[k] * np.cos(a), lidar[1] + r[k] * np.sin(a)
    s = (x > 0.05) & (np.abs(y) < yari)
    return float(np.median(x[s])) if s.sum() >= 2 else None


def duraklar(t, r, min_s, tol=0.01):
    """r(t) sabit aralıklar: [(t1, t2, medyan)]"""
    out, i = [], 0
    while i < len(t):
        j = i
        while j + 1 < len(t) and abs(r[j + 1] - np.median(r[i:j + 2])) < tol:
            j += 1
        if t[j] - t[i] >= min_s:
            out.append((t[i] + 0.3, t[j] - 0.3, float(np.median(r[i:j + 1]))))
            i = j + 1
        else:
            i += 1
    birles = []                                          # aynı mesafede bölünmüş aralıkları birleştir
    for d in out:
        if birles and abs(birles[-1][2] - d[2]) < 0.02:
            birles[-1] = (birles[-1][0], d[1], (birles[-1][2] + d[2]) / 2)
        else:
            birles.append(d)
    return birles


def dogru(x, y):
    if len(x) < 2:
        return None, None, None
    b, a = np.polyfit(x, y, 1)
    rms = float(np.sqrt(np.mean((y - (a + b * x)) ** 2)))
    return float(a), float(b), rms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kayit')
    ap.add_argument('--gercek', type=float, nargs='+', required=True, help='lazer metre mesafeleri (m)')
    ap.add_argument('--ref_x', type=float, default=0.0, help='lazer metre sıfır noktasının base_link x konumu (m)')
    ap.add_argument('--kalib', default=os.path.join(BURADA, '..', 'config', 'kamera_kalibrasyon.yaml'))
    ap.add_argument('--min_s', type=float, default=3.0)
    ap.add_argument('--yari', type=float, default=0.05, help='lidar ön şerit yarı genişliği (m; levha 60 cm)')
    ap.add_argument('--kamerasiz', action='store_true')
    a = ap.parse_args()
    sb, kb = os.path.join(a.kayit, 'sensor'), os.path.join(a.kayit, 'kamera')

    tl, rl, tfs, Kd, lidar = [], [], {}, None, LIDAR
    for _, veri, _ in okuyucu(sb, ['/tf_static']):
        for tt in deserialize_message(veri, TFMessage).transforms:
            if (tt.header.frame_id, tt.child_frame_id) == ('base_link', 'laser_duz'):
                tr, q = tt.transform.translation, tt.transform.rotation
                lidar = (tr.x, tr.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
                break
    print(f'lidar TF x {lidar[0]:.3f} y {lidar[1]:.3f} yaw {math.degrees(lidar[2]):+.2f}°')
    for konu, veri, _ in okuyucu(sb, ['/scan_sabit', '/tf_static', '/camera/depth/camera_info']):
        if konu == '/scan_sabit':
            m = deserialize_message(veri, LaserScan)
            r = np.array(m.ranges)
            d = on_mesafe(m.angle_min + np.arange(len(r)) * m.angle_increment, r, lidar, a.yari)
            if d is not None:
                tl.append(damga(m.header))
                rl.append(d)
        elif konu == '/tf_static':
            for tt in deserialize_message(veri, TFMessage).transforms:
                tr, q = tt.transform.translation, tt.transform.rotation
                tfs[(tt.header.frame_id, tt.child_frame_id)] = T(q2R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        elif Kd is None:
            Kd = np.array(deserialize_message(veri, CameraInfo).k).reshape(3, 3)
    tl, rl = np.array(tl), np.array(rl)
    D = duraklar(tl, rl, a.min_s)
    print(f'{len(tl)} tarama, {len(D)} durak: ' + ', '.join(f'{d[2]:.3f}' for d in D))

    kam = [[] for _ in D]
    if not a.kamerasiz:
        T_b_do = kamera_dis(tfs, a.kalib) @ zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')
        kx = T_b_do[0, 3]
        for _, veri, _ in okuyucu(kb, ['/camera/depth/image_raw']):
            m = deserialize_message(veri, Image)
            t = damga(m.header)
            for i, (t1, t2, _) in enumerate(D):
                if t1 <= t <= t2:
                    Z = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
                    H, W = Z.shape
                    c = Z[H // 2 - 5:H // 2 + 5, W // 2 - 5:W // 2 + 5].astype(float) / 1000
                    if (c > 0).sum() >= 20:
                        kam[i].append(np.median(c[c > 0]) + kx)

    gercek = np.array(sorted(a.gercek))
    satir = []
    print(f'{"gercek":>7} {"gercek_b":>8} {"lidar_b":>8} {"l_hata_cm":>9} {"kamera_b":>8} {"k_hata_cm":>9} {"k_sigma_mm":>10}')
    for (t1, t2, rmed), kk in zip(D, kam):
        lb = rmed
        g = gercek[int(np.argmin(np.abs(gercek + a.ref_x - lb)))]
        gb = float(g + a.ref_x)
        if abs(gb - lb) > 0.10:
            print(f'UYARI: durak {lb:.3f} m en yakın gerçek {gb:.3f} m ile {abs(gb - lb) * 100:.0f} cm farklı')
        s = {'gercek': float(g), 'gercek_b': round(gb, 4), 'lidar_b': round(lb, 4), 'lidar_hata_cm': round((lb - gb) * 100, 2),
             'sure_s': round(t2 - t1, 1)}
        if kk:
            kb_ = float(np.median(kk))
            s.update({'kamera_b': round(kb_, 4), 'kamera_hata_cm': round((kb_ - gb) * 100, 2),
                      'kamera_sigma_mm': round(float(np.std(kk)) * 1000, 2), 'kamera_kare': len(kk)})
        satir.append(s)
        print(f'{g:7.3f} {gb:8.3f} {lb:8.3f} {s["lidar_hata_cm"]:+9.2f} {s.get("kamera_b", float("nan")):8.3f} '
              f'{s.get("kamera_hata_cm", float("nan")):+9.2f} {s.get("kamera_sigma_mm", float("nan")):10.2f}')
    if len({s['gercek'] for s in satir}) < len(satir):
        print('UYARI: birden çok durak aynı gerçek mesafeye eşlendi (levha arada durdu mu?) — tümü uydurmaya katıldı')

    sonuc = {'ref_x': a.ref_x, 'lidar_tf': list(lidar), 'duraklar': satir, 'dogrular': {}}
    G = np.array([s['gercek'] for s in satir])
    seriler = {'lidar': np.array([s['lidar_hata_cm'] for s in satir]) / 100}
    kmask = np.array(['kamera_hata_cm' in s for s in satir], bool)
    if kmask.sum() >= 2:
        seriler['kamera'] = np.array([s['kamera_hata_cm'] for s in satir if 'kamera_hata_cm' in s]) / 100
        seriler['kamera-lidar'] = seriler['kamera'] - seriler['lidar'][kmask]
    for ad, y in seriler.items():
        x = G if ad == 'lidar' else G[kmask]
        a0, b, rms = dogru(x, y)
        if a0 is None:
            continue
        sonuc['dogrular'][ad] = {'sabit_cm': round(a0 * 100, 2), 'olcek_%': round(b * 100, 3), 'artik_rms_mm': round(rms * 1000, 1)}
        print(f'{ad:>13}: hata = {a0 * 100:+.2f} cm + {b * 100:+.3f}% × mesafe   (artık RMS {rms * 1000:.1f} mm)')
    with open(os.path.join(a.kayit, 'hedef_mesafe.yaml'), 'w') as f:
        yaml.safe_dump(json.loads(json.dumps(sonuc, default=float)), f, allow_unicode=True, sort_keys=False)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4))
        for ad, y in seriler.items():
            x = G if ad == 'lidar' else G[kmask]
            ax.plot(x, y * 100, 'o', label=ad)
            d = sonuc['dogrular'].get(ad)
            if d:
                xx = np.array([x.min(), x.max()])
                ax.plot(xx, d['sabit_cm'] + d['olcek_%'] * xx, '-', alpha=0.6)
        ax.axhline(0, color='k', lw=0.5)
        ax.set_xlabel('gerçek mesafe (m)')
        ax.set_ylabel('hata (cm)')
        ax.legend()
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(a.kayit, 'hedef_mesafe.png'), dpi=120)
    except ImportError:
        pass


if __name__ == '__main__':
    main()
