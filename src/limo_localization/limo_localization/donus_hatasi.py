"""
Başlangıca dönüş hatası: tur öncesi ve sonrası lidar taramaları arasında ICP.

drive_pattern turu kaydedilmiş bir sensör bag'inde (/scan, /cmd_vel, /odom, /odometry/filtered)
ilk komuttan önceki ~3 s ve son komuttan sonraki ~3 s taramaları üst üste bindirilir. ICP'nin
verdiği göreli poz robotun gerçek bitiş pozudur; odometri kaynaklarının bitiş pozuyla farkı
onların tur hatasıdır.

Bitiş taramasının başlangıçla aynı yöne bakması gerekir: robot 90° dönük bitirirse ortak görülen
tek duvar boyunca çözüm kayar (x belirsiz). Tur yarıda kesilip robot sonradan yerinde döndürüldüyse
bitiş taramaları ikinci bag'den alınır (EKF aynı düğümse pozlar ortaktır).

Lidar dış kalibrasyonu varsayılanı kayıtlardan ölçülen değerdir:
base_link -> lidar = (0.137, 0.015) m, yaw -9.66° (fabrika TF'i yanlış).

Kullanım:
    ros2 run limo_localization donus_hatasi <sensor_bag> [--son <ikinci_bag>]
        [--lazer 0.137 0.015 -9.66]
"""

import argparse
import math

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

ODOM_TOPICS = ['/odometry/filtered', '/odom']


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while r.has_next():
        name, data, t = r.read_next()
        if name in out:
            out[name].append((t * 1e-9, deserialize_message(data, types[name])))
    return out


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def unse2(T):
    return T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])


def scan_pts(m, X):
    """Taramayı taban çerçevesinde 2B noktalara çevirir (X: base_link -> lidar)."""
    r = np.asarray(m.ranges)
    a = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (r > 0.15) & (r < 8.0)
    p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.ones(ok.sum())])
    return (X @ p)[:2].T


def icp(src, dst, T0, iters=60):
    """Kaynak noktaları (src) hedefe (dst) hizalayan T; dönüş: T, rms (m), 5 cm içi oranı."""
    tree = cKDTree(dst)
    T = T0.copy()
    for it in range(iters):
        p = (T[:2, :2] @ src.T).T + T[:2, 2]
        d, j = tree.query(p)
        k = d < (max(0.05, np.percentile(d, 70)) if it > 5 else 0.5)
        if k.sum() < 10:
            break
        a, b = p[k], dst[j[k]]
        ca, cb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = Vt.T @ U.T
        dT = np.eye(3)
        dT[:2, :2] = R
        dT[:2, 2] = cb - R @ ca
        T = dT @ T
        if np.abs(dT[:2, 2]).max() < 1e-5 and abs(math.atan2(R[1, 0], R[0, 0])) < 1e-6:
            break
    p = (T[:2, :2] @ src.T).T + T[:2, 2]
    d, _ = tree.query(p)
    near = d[d < 0.1]
    rms = math.sqrt(np.mean(near ** 2)) if len(near) else float('inf')
    return T, rms, float(np.mean(d < 0.05))


def pose_at(msgs, t):
    i = min(range(len(msgs)), key=lambda k: abs(msgs[k][0] - t))
    p = msgs[i][1].pose.pose
    return se2(p.position.x, p.position.y, yaw_of(p.orientation))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bag', help='tur sensör bag dizini')
    ap.add_argument('--son', help='bitiş taramaları için ikinci bag (verilmezse aynı bag)')
    ap.add_argument('--lazer', nargs=3, type=float, default=[0.137, 0.015, -9.66],
                    metavar=('X', 'Y', 'YAW_DEG'), help='base_link -> lidar')
    a = ap.parse_args()
    X = se2(a.lazer[0], a.lazer[1], math.radians(a.lazer[2]))

    d = read(a.bag, ['/scan', '/cmd_vel'] + ODOM_TOPICS)
    moving = [t for t, m in d['/cmd_vel'] if abs(m.linear.x) > 1e-4 or abs(m.angular.z) > 1e-4]
    t0, t1 = moving[0], moving[-1]
    s0 = [m for t, m in d['/scan'] if t0 - 3.0 < t < t0 - 0.2]
    if a.son:
        e = read(a.son, ['/scan'] + ODOM_TOPICS)
        te = e['/scan'][-1][0]
        s1 = [m for t, m in e['/scan'] if te - 2.5 < t <= te]
    else:
        e, te = d, t1 + 2.0
        s1 = [m for t, m in d['/scan'] if t1 + 1.0 < t < t1 + 4.0]
    print(f'hareket {t1 - t0:.1f} s; başlangıç taraması {len(s0)}, bitiş {len(s1)}')
    A = np.vstack([scan_pts(m, X) for m in s0])
    B = np.vstack([scan_pts(m, X) for m in s1])

    Tn, _, _ = icp(scan_pts(s0[-1], X), scan_pts(s0[0], X), np.eye(3))
    xn, yn, thn = unse2(Tn)
    print(f'gürültü tabanı (duruşta iki tarama): {math.hypot(xn, yn) * 100:.2f} cm, '
          f'{math.degrees(thn):.2f}°')

    rels = {k: np.linalg.inv(pose_at(d[k], t0 - 0.3)) @ pose_at(e[k], te)
            for k in ODOM_TOPICS if d[k] and e[k]}
    init = rels.get('/odometry/filtered', rels.get('/odom'))
    # yerel minimuma karşı: odometri tahmini etrafında birkaç başlangıç, en çok eşleşen
    sols = []
    for dx in (-0.3, 0.0, 0.3):
        for dy in (-0.3, 0.0, 0.3):
            T, rms, inl = icp(B, A, init @ se2(dx, dy, 0.0))
            if math.isfinite(rms):
                sols.append((inl, rms, T))
    inl, rms, T = max(sols, key=lambda s: s[0])
    agree = sum(np.linalg.norm(S[:2, 2] - T[:2, 2]) < 0.02 for _, _, S in sols)
    gx, gy, gth = unse2(T)
    print(f'ICP (gerçek) bitiş: x={gx * 100:.1f} y={gy * 100:.1f} cm yaw={math.degrees(gth):.2f}°'
          f'  | rms {rms * 100:.1f} cm, 5 cm içi %{inl * 100:.0f}, '
          f'{agree}/{len(sols)} başlangıç aynı çözüm')
    if inl < 0.85 or agree < len(sols) // 2:
        print('UYARI: eşleşme zayıf/belirsiz (bitiş yönü başlangıçtan farklı olabilir)')
    print(f'başlangıca uzaklık (gerçek): {math.hypot(gx, gy) * 100:.1f} cm')
    for name, rel in rels.items():
        x, y, th = unse2(rel)
        ex, ey, eth = unse2(np.linalg.inv(T) @ rel)
        print(f'{name:20s} bitiş: x={x * 100:6.1f} y={y * 100:6.1f} cm '
              f'yaw={math.degrees(th):7.2f}°  -> hata {math.hypot(ex, ey) * 100:.1f} cm, '
              f'{math.degrees(eth):+.2f}°')


if __name__ == '__main__':
    main()
