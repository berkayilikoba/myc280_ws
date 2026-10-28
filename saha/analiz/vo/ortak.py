"""Görsel odometri denemeleri için ortak parçalar: koşu tanımları, bag okuma, kare akışı, SE(2), değerlendirme.

Koşular bags/ altındaki üç kameralı dikdörtgen turu (2 m x 1 m, 0.15 m/s, yerinde 90° dönüşler):
  dabai_0910a  09.10 12:21 Orbbec Dabai, renk ~20 Hz, derinlik 640x400 (renge hizalanır)
  dabai_0910b  09.10 16:28 Orbbec Dabai, renk 27 Hz, derinlik 30 Hz, lidar TF'i düzeltilmiş oturum
  d435_1015    15.10 13:30 RealSense D435, renk ~12 Hz, renge hizalı derinlik 24 Hz, EKF ile sürüldü

Kare akışı sqlite'tan doğrudan okur (rosbag2_py sıralı okuyucusu ileri-geri gezinemiyor): her renk karesine başlık
damgası en yakın derinlik karesi eşlenir, derinlik renk optik çerçevesine taşınır ve iki damga arasındaki gövde dönüşü
odom/IMU ile telafi edilir. Gerekli ortam: source /opt/ros/humble/setup.bash (rclpy serileştirme, mesaj tipleri).
"""

import math
import os
import sqlite3
import struct
import sys

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

ANALIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ANALIZ)
from bag2vlmaps import T, derinlik_hizala, q2R, rpy2R, zincir  # noqa: E402

KOK = os.path.abspath(os.path.join(ANALIZ, '..', '..'))
BAGS = os.path.join(KOK, 'bags')
CALISMA = os.path.join(BAGS, 'vo_calisma')
HARITA = os.path.join(BAGS, 'LIMO_2026-10-09_dikdortgen_lidarduz', 'harita', 'harita.yaml')

KOSULAR = {
    'dabai_0910a': dict(
        kok='LIMO_2026-10-09_dikdortgen/bag/kamera_tur_20261009_122158',
        renk='/camera/color/image_raw', derinlik='/camera/depth/image_raw', hizali=False,
        kalib='saha/config/kamera_kalibrasyon.yaml', son_donus=None),
    'dabai_0910b': dict(
        kok='LIMO_2026-10-09_dikdortgen_lidarduz/bag/dikdortgen_20261009_162849',
        renk='/camera/color/image_raw', derinlik='/camera/depth/image_raw', hizali=False,
        kalib='saha/config/kamera_kalibrasyon.yaml', son_donus=None),
    'd435_1015': dict(
        kok='LIMO_2026-10-15_dikdortgen_realsense/bag/dikdortgen_rs_20261015_133037',
        renk='/camera/color/image_raw', derinlik='/camera/aligned_depth_to_color/image_raw', hizali=True,
        kalib='saha/config/kamera_kalibrasyon_d435.yaml', son_donus='sensor_son_donus'),
}
LIDAR = (0.137, 0.015, math.radians(-9.66))        # base_link -> laser_duz (ölçülen; kayıtlardaki TF yanlış)


# ---------------------------------------------------------------- SE(2)
def se2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def unse2(M):
    return M[0, 2], M[1, 2], math.atan2(M[1, 0], M[0, 0])


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def ara(A, t):
    """(t, x, y, yaw...) dizisinde doğrusal ara değer (yaw açılmış olmalı)."""
    return np.array([np.interp(t, A[:, 0], A[:, i]) for i in range(1, A.shape[1])]).T


# ---------------------------------------------------------------- bag okuma
def _db(klasor):
    d = os.path.join(BAGS, klasor)
    return os.path.join(d, [f for f in os.listdir(d) if f.endswith('.db3')][0])


def _baglan(klasor):
    c = sqlite3.connect(f'file:{_db(klasor)}?mode=ro', uri=True)
    return c, {n: i for i, n in c.execute('select id, name from topics')}


def _damga(veri):
    """CDR başlığının ilk alanı std_msgs/Header.stamp (4 bayt kapsül + sec int32 + nanosec uint32)."""
    s, ns = struct.unpack_from('<iI', veri, 4)
    return s + ns * 1e-9


def sensor_oku(ad):
    """Sensör bag'i -> sözlük (önbellekli): odom, odom_duz, ekf (t x y yaw vx wz), imu (t wz yaw), taramalar, tf."""
    yol = os.path.join(CALISMA, ad, 'sensor.npz')
    if os.path.exists(yol):
        z = np.load(yol, allow_pickle=True)
        return {k: z[k].item() if z[k].dtype == object and z[k].shape == () else z[k] for k in z.files}
    k = KOSULAR[ad]
    c, konu = _baglan(os.path.join(k['kok'], 'sensor'))
    ters = {i: n for n, i in konu.items()}
    od = {'/odom': [], '/odom_duz': [], '/odometry/filtered': []}
    imu, tara, tfs, ci = [], [], {}, {}
    for tid, veri in c.execute('select topic_id, data from messages order by timestamp'):
        n = ters[tid]
        if n in od:
            m = deserialize_message(veri, Odometry)
            p = m.pose.pose.position
            od[n].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, yaw_q(m.pose.pose.orientation),
                          m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif n == '/imu':
            m = deserialize_message(veri, Imu)
            imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z, yaw_q(m.orientation)))
        elif n == '/scan':
            m = deserialize_message(veri, LaserScan)
            r = np.array(m.ranges)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment
            g = np.isfinite(r) & (r > 0.25) & (r < 8)
            tara.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         np.c_[r[g] * np.cos(a[g]), r[g] * np.sin(a[g])].astype(np.float32)))
        elif n == '/tf_static':
            for t in deserialize_message(veri, TFMessage).transforms:
                tr, q = t.transform.translation, t.transform.rotation
                tfs[(t.header.frame_id, t.child_frame_id)] = T(q2R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        elif n.endswith('camera_info') and n not in ci:
            m = deserialize_message(veri, CameraInfo)
            ci[n] = dict(K=np.array(m.k).reshape(3, 3), D=np.array(m.d), w=m.width, h=m.height)

    def dizi(L):
        if not L:
            return np.zeros((0, 6))
        A = np.array(sorted(L))
        A = A[np.r_[True, np.diff(A[:, 0]) > 0]]
        A[:, 3 if A.shape[1] > 3 else 2] = np.unwrap(A[:, 3 if A.shape[1] > 3 else 2])
        return A
    S = dict(odom=dizi(od['/odom']), odom_duz=dizi(od['/odom_duz']), ekf=dizi(od['/odometry/filtered']),
             imu=dizi(imu), tara_t=np.array([t for t, _ in tara]),
             tara=np.array([p for _, p in tara] + [None], dtype=object)[:-1], tfs=tfs, ci=ci)
    os.makedirs(os.path.dirname(yol), exist_ok=True)
    np.savez(yol, **S)
    return S


def imu_yaw(S):
    """IMU jiroskop integrali (t, yaw) — EKF'in yön kaynağı ile aynı bilgi (IMU yaw = ∫wz)."""
    I = S['imu']
    y = np.r_[0, np.cumsum(0.5 * (I[1:, 1] + I[:-1, 1]) * np.diff(I[:, 0]))]
    return np.c_[I[:, 0], y]


def ekf_esdeger(S):
    """robot_localization ekf_robot.yaml'ın çevrimdışı karşılığı: /odom vx (vy=0) + IMU yönü, ölü hesap.
    d435_1015'te kayıttaki /odometry/filtered ile karşılaştırılarak doğrulanır."""
    O, Y = S['odom'], imu_yaw(S)
    t = O[:, 0]
    yaw = np.interp(t, Y[:, 0], Y[:, 1])
    dt = np.diff(t, prepend=t[0])
    ym = np.r_[yaw[0], 0.5 * (yaw[1:] + yaw[:-1])]
    x = np.cumsum(O[:, 4] * np.cos(ym) * dt)
    y = np.cumsum(O[:, 4] * np.sin(ym) * dt)
    return np.c_[t, x, y, yaw]


# ---------------------------------------------------------------- kamera
def kamera_kur(ad, S):
    """K (renk, bozulmasız), T_base_optik (4x4), derinlik hizalama parametreleri."""
    k = KOSULAR[ad]
    kal = yaml.safe_load(open(os.path.join(KOK, k['kalib'])))['base_link_to_camera_link']
    T_b_cl = T(rpy2R(*kal['rotation_rpy_rad']), kal['translation_m'])
    ci = S['ci']
    Kc, Dc = ci['/camera/color/camera_info']['K'], ci['/camera/color/camera_info']['D']
    tfs = S['tfs']
    if k['hizali']:
        T_cl_co = zincir(tfs, 'camera_link', 'camera_color_frame', 'camera_color_optical_frame')
        T_cl_do = zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')
        c, konu = _baglan(os.path.join(k['kok'], 'kamera'))
        m = deserialize_message(c.execute('select data from messages where topic_id=? limit 1',
                                          (konu['/camera/depth/camera_info'],)).fetchone()[0], CameraInfo)
        hiz = None
        # sürücünün hizalamadan yayınladığı kareler için (bkz. zemin_orani): ham derinlik -> renk
        ham = dict(Kd=np.array(m.k).reshape(3, 3), T_c_d=np.linalg.inv(T_cl_co) @ T_cl_do)
    else:
        T_cl_co = zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_color_frame', 'camera_color_optical_frame')
        T_cl_do = zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')
        hiz = dict(Kd=ci['/camera/depth/camera_info']['K'], T_c_d=np.linalg.inv(T_cl_co) @ T_cl_do)
        ham = None
    return dict(K=Kc, D=Dc, T_b_c=T_b_cl @ T_cl_co, hiz=hiz, ham=ham, boyut=(480, 640))


def _goruntu(veri):
    m = deserialize_message(veri, Image)
    if m.encoding in ('16UC1', 'mono16'):
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
    g = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
    return cv2.cvtColor(g, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else g.copy()


def zemin_orani(dm, kam, h_base=0.15, satir=(400, 480), sutun=(160, 480)):
    """Görüntü altındaki zemin piksellerinde ölçülen / beklenen derinlik (medyan). Beklenen: optik ışının z_base = -h
    düzlemiyle kesişimi (dış kalibrasyondan). Sağlam karede ~1; d435_1015'te sürücünün hizalamadan yayınladığı
    'aligned_depth' karelerinde ~0.55 (zemin 0.61 yerine 0.35 m) — bu kareler anahtar kare olursa ölçek yarılanıyor."""
    K, Tbc = kam['K'], kam['T_b_c']
    v, u = np.mgrid[satir[0]:satir[1]:4, sutun[0]:sutun[1]:4]
    d = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1).reshape(-1, 3)
    db = d @ Tbc[:3, :3].T
    lam = (-h_base - Tbc[2, 3]) / np.minimum(db[:, 2], -1e-6)
    z = dm[v.ravel(), u.ravel()]
    k = (z > 0) & (lam > 0) & (lam < 3)
    return float(np.median(z[k] / lam[k])) if k.sum() > 50 else float('nan')


def kareler(ad, S, kam, her=1, esik_ms=40, bas=None, son=None, gecikme=0.0):
    """(t_renk, bgr (bozulmasız), derinlik_m (renge hizalı, float32), zemin_orani) üreteci, zaman sırasıyla.
    Zemin oranı 1'den uzaksa (|z-1| > 0.1) derinlik güvenilmez: kare anahtar kare yapılmamalı.
    Derinlik renk anına taşınır: iki damga arasındaki gövde hareketi (EKF-eşdeğeri, öteleme + dönüş). Dabai'de akışlar
    eşzamansız (±15 ms); telafisiz her anahtar kare v·dt kadar kayma ekliyordu. gecikme: derinlik damgasına eklenen
    sabit düzeltme (s), damga ile gerçek pozlama arasındaki fark için."""
    k = KOSULAR[ad]
    c, konu = _baglan(os.path.join(k['kok'], 'kamera'))
    liste = {}
    for ad_k in (k['renk'], k['derinlik']):
        q = c.execute('select id, substr(data, 1, 12) from messages where topic_id=?', (konu[ad_k],)).fetchall()
        liste[ad_k] = np.array(sorted((_damga(b), i) for i, b in q))
    R, D = liste[k['renk']], liste[k['derinlik']]
    E = ekf_esdeger(S)
    K, Dc = kam['K'], kam['D']
    harita = None
    if np.any(np.abs(Dc) > 1e-6):
        harita = cv2.initUndistortRectifyMap(K, Dc, None, K, (640, 480), cv2.CV_32FC1)
    secili = R[::her]
    if bas is not None:
        secili = secili[secili[:, 0] >= bas]
    if son is not None:
        secili = secili[secili[:, 0] <= son]
    for tr, ir in secili:
        j = np.searchsorted(D[:, 0], tr)
        aday = [i for i in (j - 1, j) if 0 <= i < len(D)]
        jd = min(aday, key=lambda i: abs(D[i, 0] - tr))
        td, idd = D[jd]
        if abs(td - tr) * 1000 > esik_ms:
            continue
        bgr = _goruntu(c.execute('select data from messages where id=?', (int(ir),)).fetchone()[0])
        dmm = _goruntu(c.execute('select data from messages where id=?', (int(idd),)).fetchone()[0])
        if harita is not None:
            bgr = cv2.remap(bgr, *harita, cv2.INTER_LINEAR)
        # derinlik anındaki gövdeden renk anındaki gövdeye: T_b(tr)^-1 T_b(td)
        G = np.linalg.inv(se2(*ara(E, tr)[:3])) @ se2(*ara(E, td + gecikme)[:3])
        Tb = np.eye(4)
        Tb[:2, :2], Tb[:2, 3] = G[:2, :2], G[:2, 2]
        dyaw = math.atan2(G[1, 0], G[0, 0])
        Tdon = np.linalg.inv(kam['T_b_c']) @ Tb @ kam['T_b_c']

        def hizala_ham(p):
            return derinlik_hizala(dmm, p['Kd'], K, Tdon @ p['T_c_d'], kam['boyut'])
        if kam['hiz'] is not None:
            dm = hizala_ham(kam['hiz'])
        elif abs(dyaw) > 1e-4 or np.hypot(*G[:2, 2]) > 2e-4:
            dm = derinlik_hizala(dmm, K, K, Tdon, kam['boyut'])
        else:
            dm = dmm.astype(np.float32) / 1000.0
        z = zemin_orani(dm, kam)
        if kam['ham'] is not None and z < 0.8:          # 'aligned' konuda hizalanmamış kare
            dm = hizala_ham(kam['ham'])
            z = zemin_orani(dm, kam)
        yield tr, bgr, dm, z


# ---------------------------------------------------------------- değerlendirme
def referans_oku(ad):
    return np.loadtxt(os.path.join(CALISMA, ad, 'referans.csv'), delimiter=',')


def hizala(Y, R):
    """Odometri yörüngesini (odom çerçevesi) referansın ilk ortak anındaki pozuna oturtur (yalnız başlangıç)."""
    t0 = max(Y[0, 0], R[0, 0])
    A = se2(*ara(R, t0)[:3]) @ np.linalg.inv(se2(*ara(Y, t0)[:3]))
    P = np.c_[Y[:, 1:3], np.ones(len(Y))] @ A.T
    return np.c_[Y[:, 0], P[:, :2], Y[:, 3] + math.atan2(A[1, 0], A[0, 0])]


def degerlendir(Y, R, uyum_min=0.6):
    """Y (t x y yaw, odom çerçevesi) vs referans (t x y yaw uyum, harita). Hata ölçüleri cm / derece."""
    Yh = hizala(Y, R)
    R = R[(R[:, 4] >= uyum_min) & (R[:, 0] >= Yh[0, 0]) & (R[:, 0] <= Yh[-1, 0])]
    P = ara(Yh, R[:, 0])
    e = np.hypot(P[:, 0] - R[:, 1], P[:, 1] - R[:, 2])
    ey = np.degrees((P[:, 2] - R[:, 3] + np.pi) % (2 * np.pi) - np.pi)
    # göreli hata: referansta 1 m'lik yol parçaları, uç pozlar arası göreli dönüşüm farkı
    s = np.r_[0, np.cumsum(np.hypot(np.diff(R[:, 1]), np.diff(R[:, 2])))]
    rpe = []
    for i in range(len(R)):
        j = np.searchsorted(s, s[i] + 1.0)
        if j >= len(R):
            break
        Dr = np.linalg.inv(se2(*R[i, 1:4])) @ se2(*R[j, 1:4])
        Dy = np.linalg.inv(se2(*P[i])) @ se2(*P[j])
        rpe.append(np.hypot(*(Dr[:2, 2] - Dy[:2, 2])))
    # SE(2) en iyi uyum (TUM ATE gibi, ölçeksiz): başlangıç pozundaki referans yanlılığından bağımsız
    A, B = P[:, :2], R[:, 1:3]
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    Rm = (U @ Vt).T
    if np.linalg.det(Rm) < 0:
        Vt[1] *= -1
        Rm = (U @ Vt).T
    e2 = np.linalg.norm((A - ca) @ Rm.T + cb - B, axis=1)
    return dict(ate_rmse=100 * math.sqrt(np.mean(e ** 2)), ate_max=100 * e.max(), son=100 * e[-1],
                ate_se2=100 * math.sqrt(np.mean(e2 ** 2)),
                yaw_rmse=math.sqrt(np.mean(ey ** 2)), yaw_son=ey[-1],
                rpe_1m=100 * float(np.median(rpe)) if rpe else float('nan'), yol=s[-1])


def kaydet(ad, yontem, Y, ek=None):
    d = os.path.join(CALISMA, ad, 'yorunge')
    os.makedirs(d, exist_ok=True)
    np.savetxt(os.path.join(d, f'{yontem}.csv'), Y, delimiter=',', fmt='%.6f', header='t,x,y,yaw')
    if ek:
        with open(os.path.join(d, f'{yontem}.yaml'), 'w') as f:
            yaml.safe_dump(ek, f, allow_unicode=True)
