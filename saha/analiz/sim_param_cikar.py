#!/usr/bin/env python3
"""Gerçek LIMO kayıtlarından simülasyon parametreleri (sim = gerçek): kamera, IMU, odom hatası.

Girdi: bir ya da birden çok "sensor_bag[,lidar_poz.csv]" (kayit_baslat.sh sensor/ + lidar_poz.py çıktısı) ve isteğe
bağlı kamera bag (kare hızı için) ve derinlik_karakter.py JSON'u (derinlik gürültüsü).
  kamera   camera_info (çözünürlük, fx/fy/cx/cy, yatay/dikey FOV, bozulma), tf_static zinciri (camera_link -> optik
           çerçeveler), kare hızı; derinlik gürültüsü σ(z) = a·z² uydurması (sabit sahne σ ölçümlerinden)
  imu      yayın hızı; durağan aralıklarda jiroskop/ivme σ ve sapma; jiroskop nicemleme adımı (ardışık farkların modu)
  odom     /odom_duz yayın hızı; lidar pozuna (ICP, referans) göre 1 s pencerelerde dönüş ölçeği k_w = Δyaw_lidar /
           Δyaw_odom (yerinde dönüş pencereleri) ve öteleme ölçeği k_v = yol_lidar / yol_odom (düz pencereler);
           ayrıca rad başına yön hatası artığı (σ) ve tur sonu yön kayması
Kullanım (Humble): sim_param_cikar.py --kayit <sensor>[,<poz.csv>] ... [--kamera_bag <kamera>] [--derinlik_json j]
                    [--cikti ../config/sim_gercek_param.yaml]
"""

import argparse
import json
import math
import os

import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Imu
from tf2_msgs.msg import TFMessage

from bag2vlmaps import T, damga, okuyucu, q2R, zincir


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def rpy(M):
    R = M[:3, :3]
    return [round(math.atan2(R[2, 1], R[2, 2]), 5), round(-math.asin(max(-1, min(1, R[2, 0]))), 5),
            round(math.atan2(R[1, 0], R[0, 0]), 5)]


def kamera(bag, kamera_bag, derinlik_json):
    out, tfs = {}, {}
    for konu, v, _ in okuyucu(bag, ['/camera/color/camera_info', '/camera/depth/camera_info', '/tf_static']):
        if konu == '/tf_static':
            for t in deserialize_message(v, TFMessage).transforms:
                tr, q = t.transform.translation, t.transform.rotation
                tfs[(t.header.frame_id, t.child_frame_id)] = T(q2R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
            continue
        ad = konu.split('/')[2]
        if ad in out:
            continue
        m = deserialize_message(v, CameraInfo)
        K = np.array(m.k).reshape(3, 3)
        out[ad] = {'genislik': m.width, 'yukseklik': m.height, 'fx': round(K[0, 0], 3), 'fy': round(K[1, 1], 3),
                   'cx': round(K[0, 2], 3), 'cy': round(K[1, 2], 3),
                   'yatay_fov_rad': round(2 * math.atan(m.width / 2 / K[0, 0]), 5),
                   'dikey_fov_rad': round(2 * math.atan(m.height / 2 / K[1, 1]), 5),
                   'bozulma': [round(float(d), 5) for d in m.d], 'model': m.distortion_model, 'cerceve': m.header.frame_id}
    if not out:
        return None
    zin = {}
    for a, b in (('camera_link', 'camera_color_optical_frame'), ('camera_link', 'camera_depth_optical_frame'),
                 ('base_link', 'camera_link')):
        try:
            M = zincir(tfs, a, b)
            zin[f'{a}->{b}'] = {'xyz': [round(float(v), 5) for v in M[:3, 3]], 'rpy': rpy(M)}
        except KeyError:
            pass
    out['tf'] = zin
    if kamera_bag:
        meta = yaml.safe_load(open(os.path.join(kamera_bag, 'metadata.yaml')))['rosbag2_bagfile_information']
        sure = meta['duration']['nanoseconds'] * 1e-9
        for t in meta['topics_with_message_count']:
            ad = t['topic_metadata']['name'].split('/')
            if len(ad) > 2 and ad[2] in out and ad[-1] == 'image_raw':
                out[ad[2]]['hz'] = round(t['message_count'] / sure, 2)
    if derinlik_json:
        d = json.load(open(derinlik_json))
        z, s = [], []
        for v in d.values():
            for (a, b) in ((0.3, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0)):
                k = f'sigma_mm_{a:g}-{b:g}m'
                if v.get(k):
                    z.append((a + b) / 2)
                    s.append(v[k] / 1000)
        z, s = np.array(z), np.array(s)
        a_ = float(np.sum(s * z ** 2) / np.sum(z ** 4))                       # en küçük kareler σ = a z²
        out['depth']['gurultu'] = {'model': 'sigma = a * z^2', 'a': round(a_, 5),
                                   'artik_rms_mm': round(float(np.sqrt(np.mean((s - a_ * z ** 2) ** 2))) * 1000, 2),
                                   'kaynak': os.path.basename(derinlik_json)}
        ort = np.mean([v['doluluk'] for v in d.values()])
        out['depth']['gecerli_oran_sabit_sahne'] = round(float(ort), 3)
    return out


def imu(bag):
    t, g, a = [], [], []
    for _, v, _ in okuyucu(bag, ['/imu']):
        m = deserialize_message(v, Imu)
        t.append(damga(m.header))
        g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        a.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    if len(t) < 100:
        return None
    t, g, a = np.array(t), np.array(g), np.array(a)
    o = np.argsort(t)
    t, g, a = t[o], g[o], a[o]
    # durağan: 1 s pencerede |gz| < 0.01 ve ivme normunun std'si < 0.05
    pen = int(round(1.0 / np.median(np.diff(t))))
    dur = np.zeros(len(t), bool)
    for i in range(0, len(t) - pen, pen // 2):
        s = slice(i, i + pen)
        if np.abs(g[s, 2]).max() < 0.01 and np.linalg.norm(a[s], axis=1).std() < 0.05:
            dur[s] = True
    dg = np.abs(np.diff(g[:, 2]))
    dg = dg[dg > 1e-6]
    adim = float(np.median(dg[dg < np.percentile(dg, 50)])) if len(dg) else None
    return {'hz': round(float((len(t) - 1) / (t[-1] - t[0])), 1), 'durgun_ornek': int(dur.sum()),
            'jiroskop_sigma': [round(float(v), 5) for v in g[dur].std(0)] if dur.sum() > 50 else None,
            'jiroskop_sapma': [round(float(v), 6) for v in g[dur].mean(0)] if dur.sum() > 50 else None,
            'ivme_sigma': [round(float(v), 4) for v in a[dur].std(0)] if dur.sum() > 50 else None,
            'ivme_ortalama': [round(float(v), 3) for v in a[dur].mean(0)] if dur.sum() > 50 else None,
            'jiroskop_nicem_adimi_rad_s': round(adim, 6) if adim else None}


def odom(bag, poz_csv, pencere=1.0):
    O = []
    for _, v, _ in okuyucu(bag, ['/odom_duz']):
        m = deserialize_message(v, Odometry)
        p = m.pose.pose.position
        O.append((damga(m.header), p.x, p.y, yaw_q(m.pose.pose.orientation)))
    if len(O) < 100:
        return None
    O = np.array(sorted(O))
    O[:, 3] = np.unwrap(O[:, 3])
    sonuc = {'hz': round(float((len(O) - 1) / (O[-1, 0] - O[0, 0])), 1)}
    if not poz_csv:
        return sonuc
    L = np.loadtxt(poz_csv, delimiter=',', comments='#')
    L[:, 3] = np.unwrap(L[:, 3])
    t0, t1 = max(O[0, 0], L[0, 0]), min(O[-1, 0], L[-1, 0])
    ts = np.arange(t0, t1, pencere)
    oi = np.c_[[np.interp(ts, O[:, 0], O[:, i]) for i in (1, 2, 3)]].T
    li = np.c_[[np.interp(ts, L[:, 0], L[:, i]) for i in (1, 2, 3, 4)]].T
    dyo, dyl = np.diff(oi[:, 2]), np.diff(li[:, 2])
    do = np.hypot(*np.diff(oi[:, :2], axis=0).T)
    dl = np.hypot(*np.diff(li[:, :2], axis=0).T)
    iyi = (li[1:, 3] > 0.7) & (li[:-1, 3] > 0.7)
    don = iyi & (np.abs(dyo) > 0.1 * pencere) & (do < 0.02 * pencere)          # yerinde dönüş
    duz = iyi & (do > 0.08 * pencere) & (np.abs(dyo) < 0.02 * pencere)          # düz sürüş
    if don.sum() > 5:
        kw = float(np.sum(dyl[don] * dyo[don]) / np.sum(dyo[don] ** 2))
        art = dyl[don] - kw * dyo[don]
        sonuc.update({'donus_olcegi_k_w': round(kw, 4), 'donus_pencere': int(don.sum()),
                      'donus_artik_sigma_rad_per_rad': round(float(np.std(art) / np.sqrt(np.mean(np.abs(dyo[don])))), 4)})
    if duz.sum() > 5:
        kv = float(np.sum(dl[duz] * do[duz]) / np.sum(do[duz] ** 2))
        sonuc.update({'oteleme_olcegi_k_v': round(kv, 4), 'duz_pencere': int(duz.sum()),
                      'duz_yon_kaymasi_rad_per_m': round(float(np.sum(dyl[duz] - dyo[duz]) / np.sum(do[duz])), 4)})
    sonuc['toplam_donus_odom_rad'] = round(float(np.sum(np.abs(dyo))), 2)
    sonuc['tur_sonu_yon_farki_derece'] = round(math.degrees((li[-1, 2] - li[0, 2]) - (oi[-1, 2] - oi[0, 2])), 2)
    return sonuc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kayit', nargs='+', required=True, help='sensor_bag[,lidar_poz.csv]')
    ap.add_argument('--kamera_bag')
    ap.add_argument('--derinlik_json')
    ap.add_argument('--cikti')
    a = ap.parse_args()
    sonuc = {'kayitlar': [], 'kamera': None, 'imu': [], 'odom': []}
    for k in a.kayit:
        bag, csv = (k.split(',') + [None])[:2]
        ad = os.path.basename(os.path.dirname(os.path.abspath(bag)))
        sonuc['kayitlar'].append(ad)
        if sonuc['kamera'] is None:
            sonuc['kamera'] = kamera(bag, a.kamera_bag, a.derinlik_json)
        i = imu(bag)
        if i:
            sonuc['imu'].append({'kayit': ad, **i})
        o = odom(bag, csv)
        if o:
            sonuc['odom'].append({'kayit': ad, **o})
        print(f'{ad}: imu {i and i["hz"]} Hz, odom {o}')
    ow = [o for o in sonuc['odom'] if 'donus_olcegi_k_w' in o]
    if ow:   # pencere sayısıyla ağırlıklı özet
        w = np.array([o['donus_pencere'] for o in ow])
        sonuc['odom_ozet'] = {'donus_olcegi_k_w': round(float(np.average([o['donus_olcegi_k_w'] for o in ow], weights=w)), 4),
                              'donus_artik_sigma_rad_per_rad': round(float(np.average([o['donus_artik_sigma_rad_per_rad'] for o in ow], weights=w)), 4)}
        ov = [o for o in sonuc['odom'] if 'oteleme_olcegi_k_v' in o]
        if ov:
            w = np.array([o['duz_pencere'] for o in ov])
            sonuc['odom_ozet'].update({
                'oteleme_olcegi_k_v': round(float(np.average([o['oteleme_olcegi_k_v'] for o in ov], weights=w)), 4),
                'duz_yon_kaymasi_rad_per_m': round(float(np.average([o['duz_yon_kaymasi_rad_per_m'] for o in ov], weights=w)), 4)})
    txt = yaml.safe_dump(json.loads(json.dumps(sonuc, default=float)), allow_unicode=True, sort_keys=False)
    if a.cikti:
        open(a.cikti, 'w').write('# sim_param_cikar.py çıktısı: gerçek LIMO kayıtlarından simülasyon parametreleri\n' + txt)
    print(txt)


if __name__ == '__main__':
    main()
