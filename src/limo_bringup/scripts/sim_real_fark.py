#!/usr/bin/env python3
"""Sim-gerçek farkı: gerçek bir turun komutları simde (lab_oda dünyası, aynı başlangıç) açık çevrim oynatılır, sim
tepkisi kaydedilir ve gerçek kayıtla karşılaştırılır.

  oynat <sensor_bag> <cikti.npz> [--sure 240]
      Simde çalışır (sim + sensor_modeli açık, Nav2 KAPALI; FASTRTPS SHM profili). Önce 5 s durağan kayıt (lidar,
      derinlik, IMU), sonra bag'deki /cmd_vel (tabanın gerçekten aldığı komut) bag zamanlamasıyla sim saatine göre
      yayınlanır. Kaydedilen: /odom_gercek (gerçek konum), /odom, /imu, /scan (durağan), /camera/depth/image_raw (durağan).
  karsilastir <sensor_bag> <lidar_poz.csv> <sim.npz> [--sahne vlmaps_sahne/v2a_lidar] [--cikti rapor.yaml]
      Gerçek: lidar pozu (referans konum), /odom_duz, /imu, /scan_sabit (ilk durağan taramalar); sim: npz.
      Ölçütler: komut -> hız tepkisi (v, w; kazanç, gecikme, RMSE), odom ölçek hatası, IMU gürültüsü ve jiroskop-
      gerçek dönüş hızı, başlangıç pozunda lidar menzilleri (açı başına), derinlik (--sahne: gerçek renge hizalı kare 0).
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import yaml

ANALIZ = os.path.expanduser('~/limo_robot/limo_ws/saha/analiz')


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def bag_oku(bag, konular):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    tipler = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[k for k in konular if k in tipler]))
    while r.has_next():
        k, v, t = r.read_next()
        yield k, deserialize_message(v, get_message(tipler[k])), t * 1e-9


# ------------------------------------------------------------------ oynat (simde)
def oynat(a):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu, LaserScan
    cmd = [(t, m.linear.x, m.angular.z) for _, m, t in bag_oku(a.sensor_bag, ['/cmd_vel'])]
    cmd = np.array(cmd)
    hareket = np.nonzero((np.abs(cmd[:, 1]) > 1e-3) | (np.abs(cmd[:, 2]) > 1e-3))[0]
    i0 = max(0, hareket[0] - 5)
    cmd = cmd[i0:]
    cmd[:, 0] -= cmd[0, 0]
    cmd = cmd[cmd[:, 0] <= a.sure]
    print(f'{len(cmd)} komut, {cmd[-1, 0]:.0f} s')
    rclpy.init()
    n = rclpy.create_node('sim_real_fark', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    pub = n.create_publisher(Twist, 'cmd_vel', 10)
    K = {'gercek': [], 'odom': [], 'imu': [], 'scan': [], 'derinlik': [], 'dinfo': None}
    simdi = lambda: n.get_clock().now().nanoseconds * 1e-9
    durgun = [True]
    n.create_subscription(Odometry, 'odom_gercek', lambda m: K['gercek'].append(
        (simdi(), m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation),
         m.twist.twist.linear.x, m.twist.twist.angular.z)), 100)
    n.create_subscription(Odometry, 'odom', lambda m: K['odom'].append(
        (simdi(), m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation))), 100)
    n.create_subscription(Imu, 'imu', lambda m: K['imu'].append(
        (simdi(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
         m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)), qos_profile_sensor_data)
    n.create_subscription(LaserScan, 'scan', lambda m: durgun[0] and K['scan'].append(
        (m.angle_min, m.angle_increment, np.array(m.ranges, np.float32))), qos_profile_sensor_data)
    n.create_subscription(Image, '/camera/depth/image_raw', lambda m: durgun[0] and len(K['derinlik']) < 20 and
                          K['derinlik'].append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()),
                          qos_profile_sensor_data)
    from sensor_msgs.msg import CameraInfo
    n.create_subscription(CameraInfo, '/camera/depth/camera_info', lambda m: K.__setitem__('dinfo', np.array(m.k)),
                          qos_profile_sensor_data)
    while simdi() == 0:
        rclpy.spin_once(n, timeout_sec=0.05)
    t0 = simdi()
    while simdi() - t0 < 5:                                        # durağan kayıt
        rclpy.spin_once(n, timeout_sec=0.01)
    durgun[0] = False
    t0, j = simdi(), 0
    print('oynatılıyor')
    while j < len(cmd):
        rclpy.spin_once(n, timeout_sec=0.002)
        while j < len(cmd) and simdi() - t0 >= cmd[j, 0]:
            m = Twist()
            m.linear.x, m.angular.z = float(cmd[j, 1]), float(cmd[j, 2])
            pub.publish(m)
            j += 1
    for _ in range(200):
        pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.01)
    np.savez_compressed(a.cikti, gercek=np.array(K['gercek']), odom=np.array(K['odom']), imu=np.array(K['imu']),
                        cmd=cmd, t_basla=t0, scan_amin=K['scan'][0][0], scan_ainc=K['scan'][0][1],
                        scan=np.stack([s[2] for s in K['scan']]), derinlik=np.stack(K['derinlik']),
                        dinfo=K['dinfo'])
    print(f'{a.cikti}: gerçek konum {len(K["gercek"])}, odom {len(K["odom"])}, imu {len(K["imu"])}, '
          f'tarama {len(K["scan"])}, derinlik {len(K["derinlik"])}')


# ------------------------------------------------------------------ karsilastir
def hiz_tepki(t, v, tc, vc, gecikmeler=np.arange(0.0, 0.8, 0.02)):
    """v(t) ~ k * vc(t - d): en iyi gecikme d ve kazanç k, R², RMSE (yalnız komutlu bölgeler değil, tümü)"""
    en = None
    for d in gecikmeler:
        c = np.interp(t - d, tc, vc)
        if np.dot(c, c) < 1e-9:
            continue
        k = float(np.dot(c, v) / np.dot(c, c))
        art = v - k * c
        r2 = 1 - np.var(art) / max(np.var(v), 1e-12)
        if en is None or r2 > en[2]:
            en = (float(d), k, float(r2), float(np.sqrt(np.mean(art ** 2))))
    return {'gecikme_s': round(en[0], 2), 'kazanc': round(en[1], 3), 'r2': round(en[2], 3), 'rmse': round(en[3], 4)}


def turev(t, x, pen=0.2):
    """pen saniyelik merkezi fark (lidar pozu/gerçek konum gürültüsünü bastırır)"""
    dt = np.median(np.diff(t))
    k = max(1, int(round(pen / 2 / dt)))
    d = np.full_like(x, np.nan)
    d[k:-k] = (x[2 * k:] - x[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    return d


def govde_hiz(t, x, y, yaw, pen=0.2):
    """-> (t, v, w); aynı damgalı örnekler (sim geri çağrıları toplu işlenince) ayıklanır"""
    t, i = np.unique(t, return_index=True)
    x, y, yaw = x[i], y[i], np.unwrap(yaw[i])
    vx, vy, w = turev(t, x, pen), turev(t, y, pen), turev(t, yaw, pen)
    v = vx * np.cos(yaw) + vy * np.sin(yaw)
    return t, v, w


def karsilastir(a):
    S = np.load(a.sim, allow_pickle=True)
    sonuc = {}
    # --- gerçek veriler
    O, I, C, scans = [], [], [], []
    for k, m, t in bag_oku(a.sensor_bag, ['/odom_duz', '/imu', '/cmd_vel', '/scan_sabit']):
        if k == '/odom_duz':
            O.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation)))
        elif k == '/imu':
            I.append((t, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        elif k == '/cmd_vel':
            C.append((t, m.linear.x, m.angular.z))
        elif len(scans) < 400:
            scans.append((t, m.angle_min, m.angle_increment, np.array(m.ranges, np.float32)))
    O, I, C = np.array(O), np.array(I), np.array(C)
    L = np.loadtxt(a.lidar_poz, delimiter=',', comments='#')
    hareket = C[np.nonzero((np.abs(C[:, 1]) > 1e-3) | (np.abs(C[:, 2]) > 1e-3))[0][0], 0]
    sure = float(S['cmd'][-1, 0])
    t_r0 = hareket - float(S['cmd'][np.nonzero((np.abs(S['cmd'][:, 1]) > 1e-3) | (np.abs(S['cmd'][:, 2]) > 1e-3))[0][0], 0])
    # ortak zaman: komut dizisinin başı = 0
    G = S['gercek']
    ts = G[:, 0] - float(S['t_basla'])
    k = (ts >= 0) & (ts <= sure)
    ts, G = ts[k], G[k]
    tr = L[:, 0] - t_r0
    kr = (tr >= 0) & (tr <= sure)
    tr, L = tr[kr], L[kr]
    tc = C[:, 0] - t_r0
    tr_, vr, wr = govde_hiz(tr, L[:, 1], L[:, 2], L[:, 3], 0.4)     # lidar pozu ~6 Hz düzeltmeli -> 0.4 s pencere
    ts_, vs, ws = govde_hiz(ts, G[:, 1], G[:, 2], G[:, 3], 0.2)
    ok_r, ok_s = np.isfinite(vr), np.isfinite(vs)
    sonuc['taban_tepki'] = {
        'gercek_v': hiz_tepki(tr_[ok_r], vr[ok_r], tc, C[:, 1]), 'sim_v': hiz_tepki(ts_[ok_s], vs[ok_s], tc, C[:, 1]),
        'gercek_w': hiz_tepki(tr_[ok_r], wr[ok_r], tc, C[:, 2]), 'sim_w': hiz_tepki(ts_[ok_s], ws[ok_s], tc, C[:, 2])}
    # aynı zaman ızgarasında gerçek - sim hız farkı
    tg = np.arange(1.0, sure - 1.0, 0.1)
    dv = np.interp(tg, ts_[ok_s], vs[ok_s]) - np.interp(tg, tr_[ok_r], vr[ok_r])
    dw = np.interp(tg, ts_[ok_s], ws[ok_s]) - np.interp(tg, tr_[ok_r], wr[ok_r])
    sonuc['hiz_farki_sim_eksi_gercek'] = {'v_rmse_m_s': round(float(np.sqrt(np.mean(dv ** 2))), 4),
                                          'w_rmse_rad_s': round(float(np.sqrt(np.mean(dw ** 2))), 4),
                                          'v_ort': round(float(dv.mean()), 4), 'w_ort': round(float(dw.mean()), 4)}
    # yörünge (açık çevrim; başlangıç farkı dahil)
    yol_r = float(np.sum(np.hypot(np.diff(L[:, 1]), np.diff(L[:, 2]))))
    yol_s = float(np.sum(np.hypot(np.diff(G[:, 1]), np.diff(G[:, 2]))))
    don_r, don_s = float(np.unwrap(L[:, 3])[-1] - np.unwrap(L[:, 3])[0]), float(np.unwrap(G[:, 3])[-1] - np.unwrap(G[:, 3])[0])
    sonuc['yorunge'] = {'yol_m_gercek': round(yol_r, 3), 'yol_m_sim': round(yol_s, 3),
                        'net_donus_derece_gercek': round(math.degrees(don_r), 1),
                        'net_donus_derece_sim': round(math.degrees(don_s), 1),
                        'son_konum_gercek': [round(float(v), 3) for v in L[-1, 1:4]],
                        'son_konum_sim': [round(float(v), 3) for v in G[-1, 1:4]],
                        'baslangic_gercek': [round(float(v), 3) for v in L[0, 1:4]],
                        'baslangic_sim': [round(float(v), 3) for v in G[0, 1:4]]}
    # odom ölçek hatası (1 s pencereler)
    def odom_olcek(to, oo, tt, tt_xyz):
        pts = np.arange(max(to[0], tt[0]), min(to[-1], tt[-1]), 1.0)
        oy = np.interp(pts, to, np.unwrap(oo[:, 2]))
        ty = np.interp(pts, tt, np.unwrap(tt_xyz[:, 2]))
        oxy = np.c_[np.interp(pts, to, oo[:, 0]), np.interp(pts, to, oo[:, 1])]
        txy = np.c_[np.interp(pts, tt, tt_xyz[:, 0]), np.interp(pts, tt, tt_xyz[:, 1])]
        do, dt_ = np.diff(oy), np.diff(ty)
        so, st = np.hypot(*np.diff(oxy, axis=0).T), np.hypot(*np.diff(txy, axis=0).T)
        don = (np.abs(do) > 0.1) & (so < 0.02)
        duz = (so > 0.08) & (np.abs(do) < 0.02)
        return {'k_w': round(float(np.sum(dt_[don] * do[don]) / np.sum(do[don] ** 2)), 4) if don.sum() > 5 else None,
                'k_v': round(float(np.sum(st[duz] * so[duz]) / np.sum(so[duz] ** 2)), 4) if duz.sum() > 5 else None,
                'tur_sonu_yon_farki_derece': round(math.degrees((ty[-1] - ty[0]) - (oy[-1] - oy[0])), 2)}
    So = S['odom']
    sonuc['odom'] = {'gercek': odom_olcek(O[:, 0] - t_r0, O[:, 1:4], tr, L[:, 1:4]),
                     'sim': odom_olcek(So[:, 0] - float(S['t_basla']), So[:, 1:4], ts, G[:, 1:4])}
    # IMU: durağan gürültü + jiroskop z'nin gerçek dönüş hızına oranı/farkı
    def imu_ozet(Ii, t0, tt, w_ref):
        ti = Ii[:, 0] - t0
        dur = ti < -0.5
        hare = (ti > 0) & (ti < sure)
        wref = np.interp(ti[hare], tt, w_ref)
        gz = Ii[hare, 3]
        ok = np.isfinite(wref)
        return {'hz': round(float((len(Ii) - 1) / (Ii[-1, 0] - Ii[0, 0])), 1),
                'durgun_jiro_sigma': [round(float(v), 5) for v in Ii[dur, 1:4].std(0)] if dur.sum() > 50 else None,
                'durgun_ivme_sigma': [round(float(v), 4) for v in Ii[dur, 4:7].std(0)] if dur.sum() > 50 else None,
                'durgun_ivme_ort': [round(float(v), 3) for v in Ii[dur, 4:7].mean(0)] if dur.sum() > 50 else None,
                'hareket_gz_olcek': round(float(np.dot(gz[ok], wref[ok]) / np.dot(wref[ok], wref[ok])), 4),
                'hareket_gz_artik_rms': round(float(np.std(gz[ok] - wref[ok])), 4),
                'hareket_ivme_sigma': [round(float(v), 3) for v in Ii[hare, 4:7].std(0)]}
    sonuc['imu'] = {'gercek': imu_ozet(I, t_r0, tr_[ok_r], wr[ok_r]), 'sim': imu_ozet(S['imu'], float(S['t_basla']), ts_[ok_s], ws[ok_s])}
    # lidar: başlangıçta (durağan) açı başına medyan menzil
    def lidar_medyan(amin, ainc, R):
        bins = np.arange(-180, 180, 1.0)
        out = np.full(len(bins), np.nan)
        cnt = np.zeros(len(bins))
        aci = np.degrees(amin + np.arange(R.shape[1]) * ainc)
        aci = (aci + 180) % 360 - 180
        b = np.clip(np.floor(aci + 180).astype(int), 0, 359)
        gecerli = np.isfinite(R) & (R > 0.05) & (R < 12)
        for i in range(360):
            v = R[:, b == i][gecerli[:, b == i]]
            cnt[i] = gecerli[:, b == i].mean() if (b == i).any() else 0
            if len(v):
                out[i] = np.median(v)
        return out, cnt
    rs = [s for s in scans if s[0] < hareket - 0.5]
    Rr = np.stack([s[3] for s in rs if len(s[3]) == len(rs[0][3])])
    mr, cr = lidar_medyan(rs[0][1], rs[0][2], Rr)
    # başlangıç yönü farkı: aynı dünya noktası gerçekte (yaw_g) ve simde (yaw_s) farklı ışın açısında görünür
    dyaw = float(L[0, 3] - G[0, 3])
    ms, cs = lidar_medyan(float(S['scan_amin']) - dyaw, float(S['scan_ainc']), S['scan'])
    ik = np.isfinite(mr) & np.isfinite(ms)
    fark = ms[ik] - mr[ik]
    sonuc['lidar_baslangic'] = {
        'aci_kutusu': int(ik.sum()), 'medyan_fark_cm': round(float(np.median(fark)) * 100, 2),
        'mutlak_fark_medyan_cm': round(float(np.median(np.abs(fark))) * 100, 2),
        '5cm_ici_%': round(100 * float(np.mean(np.abs(fark) < 0.05)), 1),
        '20cm_disi_%': round(100 * float(np.mean(np.abs(fark) > 0.20)), 1),
        'gecerli_oran_gercek': round(float(cr[cr > 0].mean()), 3), 'gecerli_oran_sim': round(float(cs[cs > 0].mean()), 3),
        'tarama_sayisi': [int(len(Rr)), int(len(S['scan']))],
        'baslangic_yon_farki_derece_duzeltildi': round(math.degrees(dyaw), 2),
        'baslangic_konum_farki_cm_duzeltilmedi': round(100 * math.hypot(L[0, 1] - G[0, 1], L[0, 2] - G[0, 2]), 1)}
    np.savez(os.path.splitext(a.cikti or 'sim_real_fark.yaml')[0] + '_lidar.npz', gercek=mr, sim=ms)
    # derinlik: sahne kare 0 (gerçek, renge hizalı) vs sim (renge hizala)
    if a.sahne:
        sys.path.insert(0, ANALIZ)
        from bag2vlmaps import derinlik_hizala
        cfg = yaml.safe_load(open(os.path.join(a.sahne, 'vlmaps_config.yaml')))
        Ko = np.array(cfg['cam_calib_mat']).reshape(3, 3)
        dr = np.load(os.path.join(a.sahne, 'depth', '000000.npy'))
        Kd = S['dinfo'].reshape(3, 3)
        T = np.eye(4)
        T[0, 3] = -0.0148                                              # renk optik çerçevesinde derinlik kamerası (x sağ)
        dsim = [derinlik_hizala(z, Kd, Ko, T, dr.shape, zmax=8.0) for z in S['derinlik'][:10]]
        ds = np.median(np.stack(dsim), 0)
        ikisi = (dr > 0) & (ds > 0)
        f = ds[ikisi] - dr[ikisi]
        bant = {}
        for z0, z1 in ((0.3, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0)):
            m = (dr[ikisi] >= z0) & (dr[ikisi] < z1)
            if m.sum() > 100:
                bant[f'{z0:g}-{z1:g}m'] = {'piksel_%': round(100 * float(m.mean()), 1),
                                           'medyan_fark_cm': round(float(np.median(f[m])) * 100, 1),
                                           'mutlak_medyan_cm': round(float(np.median(np.abs(f[m]))) * 100, 1)}
        sonuc['derinlik_baslangic'] = {'gecerli_gercek': round(float((dr > 0).mean()), 3),
                                       'gecerli_sim': round(float((ds > 0).mean()), 3),
                                       'ikisi_gecerli': round(float(ikisi.mean()), 3),
                                       'mutlak_fark_medyan_cm': round(float(np.median(np.abs(f))) * 100, 1),
                                       '10cm_ici_%': round(100 * float(np.mean(np.abs(f) < 0.10)), 1), 'bantlar': bant}
        np.savez_compressed(os.path.splitext(a.cikti or 'sim_real_fark.yaml')[0] + '_derinlik.npz', gercek=dr, sim=ds)
    txt = yaml.safe_dump(json.loads(json.dumps(sonuc, default=float)), allow_unicode=True, sort_keys=False)
    if a.cikti:
        open(a.cikti, 'w').write(txt)
    print(txt)


def main():
    ap = argparse.ArgumentParser()
    alt = ap.add_subparsers(dest='komut', required=True)
    o = alt.add_parser('oynat')
    o.add_argument('sensor_bag')
    o.add_argument('cikti')
    o.add_argument('--sure', type=float, default=240.0)
    k = alt.add_parser('karsilastir')
    k.add_argument('sensor_bag')
    k.add_argument('lidar_poz')
    k.add_argument('sim')
    k.add_argument('--sahne')
    k.add_argument('--cikti')
    a = ap.parse_args()
    oynat(a) if a.komut == 'oynat' else karsilastir(a)


if __name__ == '__main__':
    main()
