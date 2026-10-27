#!/usr/bin/env python3
"""Nav2 ölçüm koşusu: hedef turunu sürer, sonuçları JSON/YAML'a yazar.

Simülasyonda /odom_gercek (diff, 27.10.2026'dan beri; yoksa /odom) Gazebo'nun GERÇEK konumudur (diff_drive ve ackermann_drive
eklentilerinde odometry_source=1). Harita dünya dosyasından üretildiği ve robot
(0,0,0)'da doğduğu için map == odom == dünya koordinatı; hedef hatası doğrudan
/odom'dan ölçülür, AMCL'in kendi tahmininden değil.

Her hedef için ölçülenler:
  status                       SUCCEEDED / FAILED / CANCELED / TIMEOUT
  sim_time_s / sure_s          hedefe gidiş süresi (simülasyon saati)
  recoveries / kurtarma_sayisi Nav2 feedback.number_of_recoveries (en büyük değer)
  pos_err_m                    son konumun hedefe uzaklığı (gerçek konum)
  yaw_err_deg                  son yönelim hatası
  varis_hatasi_gercek_m        hedef ↔ /odom_gercek (Nav2 SUCCEEDED anında gerçek varış hatası)
  varis_yon_hatasi_gercek_deg  hedef ↔ /odom_gercek gerçek yön hatası (derece)
  lokalizasyon_hatasi_m        o anda map->base_footprint TF ile /odom_gercek arasındaki fark
  path_len_m                   gerçek konumdan birikmiş yol uzunluğu
  min_clear_m / min_aciklik_m  tarama noktalarının footprint poligonuna en küçük uzaklığı
                               (sim LiDAR range_min 0.2 m — lazerin çok yakını kör nokta)
  reverse_s                    geri vites (cmd_vel linear.x < -0.01) süresi
  planner_fail                 /rosout'ta "failed to create plan" sayısı
  ctrl_fail                    /rosout'ta controller_server ERROR sayısı

Kullanım:
  nav_bench.py --mode diff --out sonuc.json [--timeout 120] [--hedefler tur.yaml]
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros
import yaml

# rcl_interfaces/Log sabitleri Humble'da bytes (b'('); m.level ise int
_ERROR_LEVEL = Log.ERROR[0] if isinstance(Log.ERROR, bytes) else int(Log.ERROR)

# Varsayılan hedef turu (map/dünya koordinatı, derece). Dünya: 8x8 m kapalı alan,
# y=0.6'da x∈[-2,2] bölme duvarı, (2,-2) ve (-2,-1.5)'te sütunlar.
TOUR = [
    ('duz_ileri', 2.5, 0.0, 0.0),
    ('sutun_yani', 3.0, -3.0, -90.0),
    ('bati_guney', -3.0, -3.0, 180.0),
    ('bolme_etrafi_kuzey', -3.0, 2.5, 90.0),
    ('bolme_ustu_dogu', 0.0, 2.5, 0.0),
    ('baslangica_donus', 0.0, 0.0, 0.0),
]

# base_link'e göre sabit lazer yerleşimi (Faz 1: laser_link 0.105 0 0.08, yaw -0.15)
LASER_X, LASER_Y, LASER_YAW = 0.105, 0.0, -0.15
FOOTPRINTS = {
    'ackermann': [(-0.161, -0.11), (-0.161, 0.11), (0.161, 0.11), (0.161, -0.11)],
    'diff': [(-0.161, -0.11), (-0.161, 0.11), (0.161, 0.11), (0.161, -0.11)],
}


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def poly_clearance(px, py, poly):
    """Noktanın poligona uzaklığı; içerideyse 0."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py) and px < (x2 - x1) * (py - y1) / (y2 - y1) + x1:
            inside = not inside
    if inside:
        return 0.0
    return min(seg_dist(px, py, *poly[i], *poly[(i + 1) % n]) for i in range(n))


def load_tour_yaml(path):
    """YAML dosyasından başlangıç pozu ve hedef listesini yükler.

    Desteklenen YAML yapıları:
      1) {'baslangic': [x, y, yaw] veya {'x': .., 'y': .., 'yaw_derece': ..},
          'hedefler': [{'ad': .., 'x': .., 'y': .., 'yaw_derece': ..}, ...]}
      2) [{'ad': .., 'x': .., 'y': .., 'yaw_derece': ..}, ...]
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    baslangic = (0.0, 0.0, 0.0)
    if isinstance(data, dict):
        if 'baslangic' in data and data['baslangic'] is not None:
            b = data['baslangic']
            if isinstance(b, dict):
                bx = float(b.get('x', 0.0))
                by = float(b.get('y', 0.0))
                byaw = float(b.get('yaw_derece', b.get('yaw_deg', b.get('yaw', 0.0))))
                baslangic = (bx, by, byaw)
            elif isinstance(b, (list, tuple)):
                bx = float(b[0]) if len(b) > 0 else 0.0
                by = float(b[1]) if len(b) > 1 else 0.0
                byaw = float(b[2]) if len(b) > 2 else 0.0
                baslangic = (bx, by, byaw)
        raw_list = data.get('hedefler', data.get('goals', data.get('waypoints', [])))
    elif isinstance(data, list):
        raw_list = data
    else:
        raise ValueError(f'Geçersiz YAML verisi: {path}')

    tour = []
    for i, item in enumerate(raw_list):
        if isinstance(item, dict):
            name = str(item.get('ad', item.get('name', item.get('goal', f'hedef_{i}'))))
            x = float(item['x'])
            y = float(item['y'])
            yaw = float(item.get('yaw_derece', item.get('yaw_deg', item.get('yaw', 0.0))))
            tour.append((name, x, y, yaw))
        elif isinstance(item, (list, tuple)):
            name = str(item[0])
            x = float(item[1])
            y = float(item[2])
            yaw = float(item[3]) if len(item) > 3 else 0.0
            tour.append((name, x, y, yaw))
        else:
            raise ValueError(f'Geçersiz hedef tanımı: {item}')

    return tour, baslangic


class Monitor:
    def __init__(self, node, footprint):
        self.node = node
        self.fp = footprint
        self.pose = None          # (x, y, yaw)
        self.path_len = 0.0
        self.min_clear = float('inf')
        self.reverse_s = 0.0
        self.last_cmd_t = None
        self.last_cmd_vx = 0.0
        self._gercek_var = False  # /odom_gercek geldi mi (diff sim gerçek konum)
        self.planner_fail = 0
        self.ctrl_fail = 0
        self.traj = []            # (t, x, y, yaw_deg, vx) ~2 Hz
        self.last_traj_t = -1e9
        self.plans = []           # her yeni /plan için (uzunluk_m, cusp sayısı)
        self.plan_paths = []      # ilk 4 planın seyreltilmiş geometrisi [(x, y)]
        # Yalpalama ölçüsü: hareket hâlinde (vx > 0.05) yaw hızı karesi ve işaret değişimi (±0.05 histerezis)
        self.wz_sq, self.wz_n, self.wz_flips, self.wz_sign, self.move_len = 0.0, 0, 0, 0, 0.0
        self.last_cmd_wz = 0.0
        self.last_amcl_pose = None

        # TF dinleyicisi (map -> base_footprint / base_link dönüşümü için)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # diff sim (27.10.2026'dan beri): /odom gerçek tabanın hatalı odom'unu taklit eder, gerçek konum /odom_gercek.
        # İkisi de gelirse /odom_gercek kullanılır (Ackermann'da yalnız /odom var ve o gerçek konumdur).
        node.create_subscription(Odometry, '/odom', lambda m: None if self._gercek_var else self._odom(m), 20)
        node.create_subscription(Odometry, '/odom_gercek', self._odom_gercek, 20)
        node.create_subscription(Path, '/plan', self._plan, 10)
        # AMCL tahmini vs gerçek konum: map == dünya == odom olduğundan fark = lokalizasyon hatası
        amcl_qos = rclpy.qos.QoSProfile(depth=5, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, amcl_qos)
        self.loc_err_max = 0.0
        self.loc_yaw_err_max = 0.0
        self.loc_samples = []     # (t, dxy, dyaw_deg)
        node.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        node.create_subscription(Twist, '/cmd_vel', self._cmd, 20)
        node.create_subscription(Log, '/rosout', self._log, 100)

    def reset_goal(self):
        self.loc_err_max = 0.0
        self.loc_yaw_err_max = 0.0
        self.loc_samples = []
        self.traj = []
        self.last_traj_t = -1e9
        self.plans = []
        self.plan_paths = []
        self.wz_sq, self.wz_n, self.wz_flips, self.wz_sign, self.move_len = 0.0, 0, 0, 0, 0.0
        self.path_len = 0.0
        self.min_clear = float('inf')
        self.reverse_s = 0.0
        self.planner_fail = 0
        self.ctrl_fail = 0

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _odom_gercek(self, m):
        self._gercek_var = True
        self._odom(m)

    def _odom(self, m):
        p = m.pose.pose
        cur = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        if self.pose is not None:
            step = math.hypot(cur[0] - self.pose[0], cur[1] - self.pose[1])
            self.path_len += step
            vx, wz = m.twist.twist.linear.x, m.twist.twist.angular.z
            if vx > 0.05:   # gerçek robot ayarı en fazla 0.2 m/s; eşik ona göre
                self.move_len += step
                self.wz_sq += wz * wz
                self.wz_n += 1
                sg = 1 if wz > 0.05 else (-1 if wz < -0.05 else 0)
                if sg and self.wz_sign and sg != self.wz_sign:
                    self.wz_flips += 1
                if sg:
                    self.wz_sign = sg
        self.pose = cur
        t = self.now()
        if t - self.last_traj_t >= 0.5:
            self.last_traj_t = t
            self.traj.append((round(t, 2), round(cur[0], 3), round(cur[1], 3),
                              round(math.degrees(cur[2]), 1), round(m.twist.twist.linear.x, 3),
                              round(self.last_cmd_vx, 3), round(self.last_cmd_wz, 3)))

    def _amcl(self, m):
        p = m.pose.pose
        ax, ay, ayaw = p.position.x, p.position.y, yaw_from_quat(p.orientation)
        self.last_amcl_pose = (ax, ay, ayaw)
        if self.pose is None:
            return
        gx, gy, gyaw = self.pose
        dxy = math.hypot(ax - gx, ay - gy)
        dyaw = abs(math.degrees(wrap(ayaw - gyaw)))
        self.loc_err_max = max(self.loc_err_max, dxy)
        self.loc_yaw_err_max = max(self.loc_yaw_err_max, dyaw)
        self.loc_samples.append((round(self.now(), 2), round(dxy, 3), round(dyaw, 1)))

    def get_map_base_tf(self):
        """map -> base_footprint (veya base_link) TF dönüşümünü (x, y, yaw) döndürür."""
        for target_frame in ('base_footprint', 'base_link'):
            try:
                t = self.tf_buffer.lookup_transform(
                    'map',
                    target_frame,
                    rclpy.time.Time()
                )
                tr = t.transform.translation
                rot = t.transform.rotation
                return (tr.x, tr.y, yaw_from_quat(rot))
            except Exception:
                continue
        if self.last_amcl_pose is not None:
            return self.last_amcl_pose
        return None

    def get_lokalizasyon_hatasi(self):
        """O anda map->base_footprint TF ile /odom_gercek arasındaki Öklid farkı (m)."""
        if self.pose is None:
            return None
        gx, gy, _ = self.pose
        tf_pose = self.get_map_base_tf()
        if tf_pose is not None:
            return round(math.hypot(tf_pose[0] - gx, tf_pose[1] - gy), 3)
        return None

    def _plan(self, m):
        # cusp = yol boyunca ileri/geri yön değişimi (pozun yönelimine göre hareket yönü)
        ps = m.poses
        length, cusps, prev_dir = 0.0, 0, 0
        for a, b in zip(ps, ps[1:]):
            dx = b.pose.position.x - a.pose.position.x
            dy = b.pose.position.y - a.pose.position.y
            d = math.hypot(dx, dy)
            length += d
            if d < 1e-4:
                continue
            th = yaw_from_quat(a.pose.orientation)
            dirn = 1 if math.cos(math.atan2(dy, dx) - th) >= 0 else -1
            if prev_dir and dirn != prev_dir:
                cusps += 1
            prev_dir = dirn
        self.plans.append((round(length, 2), cusps))
        if len(self.plan_paths) < 4:
            geo, acc, last = [], 0.0, None
            for a in ps:
                x, y = a.pose.position.x, a.pose.position.y
                if last is None or math.hypot(x - last[0], y - last[1]) >= 0.1:
                    geo.append((round(x, 2), round(y, 2)))
                    last = (x, y)
            self.plan_paths.append({'t': round(self.now(), 2), 'len': round(length, 2),
                                    'cusps': cusps, 'xy': geo})

    def _scan(self, m):
        c, s = math.cos(LASER_YAW), math.sin(LASER_YAW)
        a = m.angle_min
        best = self.min_clear
        for r in m.ranges:
            if m.range_min < r < m.range_max and math.isfinite(r):
                lx, ly = r * math.cos(a), r * math.sin(a)
                bx = LASER_X + c * lx - s * ly
                by = LASER_Y + s * lx + c * ly
                if abs(bx) < 1.0 and abs(by) < 1.0:     # yalnızca yakın noktalar
                    best = min(best, poly_clearance(bx, by, self.fp))
            a += m.angle_increment
        self.min_clear = best

    def _cmd(self, m):
        t = self.now()
        if self.last_cmd_t is not None and self.last_cmd_vx < -0.01:
            self.reverse_s += max(0.0, min(0.5, t - self.last_cmd_t))
        self.last_cmd_t, self.last_cmd_vx, self.last_cmd_wz = t, m.linear.x, m.angular.z

    def _log(self, m):
        msg = m.msg.lower()
        if m.name.startswith('planner_server') and 'failed to create plan' in msg:
            self.planner_fail += 1
        if m.name.startswith('controller_server') and m.level >= _ERROR_LEVEL:
            self.ctrl_fail += 1


def make_pose(nav, x, y, yaw_deg):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = nav.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    yaw = math.radians(yaw_deg)
    ps.pose.orientation.z = math.sin(yaw / 2)
    ps.pose.orientation.w = math.cos(yaw / 2)
    return ps


def spin_for(nav, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(nav, timeout_sec=0.05)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=list(FOOTPRINTS), required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=120.0, help='hedef başına sim saniyesi')
    ap.add_argument('--label', default='')
    ap.add_argument('--hedefler', default=None, help='Hedef turu YAML dosyası yolu')
    a = ap.parse_args(argv)

    tour = TOUR
    baslangic = (0.0, 0.0, 0.0)
    if a.hedefler:
        if not os.path.exists(a.hedefler):
            print(f'HATA: Hedef turu dosyası bulunamadı: {a.hedefler}', file=sys.stderr)
            return 2
        tour, baslangic = load_tour_yaml(a.hedefler)
        if not tour:
            print(f'HATA: Hedef listesi boş: {a.hedefler}', file=sys.stderr)
            return 2

    rclpy.init()
    nav = BasicNavigator(node_name='nav_bench')
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    mon = Monitor(nav, FOOTPRINTS[a.mode])

    # sim saati akana ve gerçek konum gelene kadar bekle
    t0 = time.monotonic()
    while (mon.pose is None or mon.now() == 0.0) and time.monotonic() - t0 < 120:
        rclpy.spin_once(nav, timeout_sec=0.2)
    if mon.pose is None:
        print('HATA: /odom gelmedi', file=sys.stderr)
        return 3

    bx, by, byaw = baslangic
    nav.setInitialPose(make_pose(nav, bx, by, byaw))
    nav.waitUntilNav2Active(localizer='amcl')
    spin_for(nav, 2.0)

    results = []
    wall0, sim0 = time.monotonic(), mon.now()
    for name, gx, gy, gyaw in tour:
        mon.reset_goal()
        start_sim = mon.now()
        max_rec = 0
        nav.goToPose(make_pose(nav, gx, gy, gyaw))
        status = None
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.05)
            fb = nav.getFeedback()
            if fb is not None:
                max_rec = max(max_rec, fb.number_of_recoveries)
            if mon.now() - start_sim > a.timeout:
                nav.cancelTask()
                spin_for(nav, 1.0)
                status = 'TIMEOUT'
                break
        if status is None:
            r = nav.getResult()
            status = {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.CANCELED: 'CANCELED',
                      TaskResult.FAILED: 'FAILED'}.get(r, str(r))
        spin_for(nav, 0.5)
        x, y, yaw = mon.pose

        sure_s = round(mon.now() - start_sim, 2)
        kurtarma_sayisi = int(max_rec)
        pos_err = round(math.hypot(x - gx, y - gy), 3)
        yaw_err = round(abs(math.degrees(wrap(yaw - math.radians(gyaw)))), 1)
        min_aciklik = None if math.isinf(mon.min_clear) else round(mon.min_clear, 3)
        lok_hatasi = mon.get_lokalizasyon_hatasi()

        res = {
            'goal': name, 'target': [gx, gy, gyaw], 'status': status,
            'sim_time_s': sure_s,
            'sure_s': sure_s,
            'recoveries': kurtarma_sayisi,
            'kurtarma_sayisi': kurtarma_sayisi,
            'pos_err_m': pos_err,
            'yaw_err_deg': yaw_err,
            'varis_hatasi_gercek_m': pos_err,
            'varis_yon_hatasi_gercek_deg': yaw_err,
            'lokalizasyon_hatasi_m': lok_hatasi,
            'path_len_m': round(mon.path_len, 2),
            'min_clear_m': min_aciklik,
            'min_aciklik_m': min_aciklik,
            'reverse_s': round(mon.reverse_s, 1),
            'planner_fail': mon.planner_fail,
            'ctrl_fail': mon.ctrl_fail,
            'final_pose': [round(x, 3), round(y, 3), round(math.degrees(yaw), 1)],
            'loc_err_max_m': round(mon.loc_err_max, 3),
            'loc_yaw_err_max_deg': round(mon.loc_yaw_err_max, 1),
            'loc_samples': mon.loc_samples,
            'n_plans': len(mon.plans),
            'first_plan': mon.plans[0] if mon.plans else None,
            'max_plan_cusps': max((c for _, c in mon.plans), default=None),
            'traj': mon.traj,
            'plan_paths': mon.plan_paths,
            'wz_rms_moving': round(math.sqrt(mon.wz_sq / mon.wz_n), 3) if mon.wz_n else None,
            'wz_flips_per_m': round(mon.wz_flips / mon.move_len, 2) if mon.move_len > 0.5 else None,
        }
        results.append(res)
        print(json.dumps({k: v for k, v in res.items() if k not in ('traj', 'loc_samples', 'plan_paths')},
                         ensure_ascii=False), flush=True)

    wall = time.monotonic() - wall0
    sim = mon.now() - sim0
    ok = [r for r in results if r['status'] == 'SUCCEEDED']
    basari_orani = round(len(ok) / len(results), 3) if results else 0.0
    ok_varis = [r['varis_hatasi_gercek_m'] for r in ok if r.get('varis_hatasi_gercek_m') is not None]
    ort_varis = round(sum(ok_varis) / len(ok_varis), 3) if ok_varis else None
    maks_varis = round(max(ok_varis), 3) if ok_varis else None

    lok_hatalari = [r['lokalizasyon_hatasi_m'] for r in results if r.get('lokalizasyon_hatasi_m') is not None]
    ort_lok = round(sum(lok_hatalari) / len(lok_hatalari), 3) if lok_hatalari else None
    toplam_sure = round(sum(r['sim_time_s'] for r in results), 1)

    summary = {
        'label': a.label, 'mode': a.mode,
        'succeeded': len(ok), 'total': len(results),
        'total_sim_time_s': toplam_sure,
        'mean_pos_err_m': round(sum(r['pos_err_m'] for r in ok) / len(ok), 3) if ok else None,
        'total_recoveries': sum(r['recoveries'] for r in results),
        'total_planner_fail': sum(r['planner_fail'] for r in results),
        'min_clear_m': min((r['min_clear_m'] for r in results if r['min_clear_m'] is not None),
                           default=None),
        'real_time_factor': round(sim / wall, 2) if wall > 0 else None,
        # Yeni özet ölçütleri (Türkçe ve eş anlamlılar)
        'basari_orani': basari_orani,
        'success_rate': basari_orani,
        'ortalama_varis_hatasi_gercek_m': ort_varis,
        'ort_varis_hatasi_gercek_m': ort_varis,
        'maks_varis_hatasi_gercek_m': maks_varis,
        'max_varis_hatasi_gercek_m': maks_varis,
        'ortalama_lokalizasyon_hatasi_m': ort_lok,
        'ort_lokalizasyon_hatasi_m': ort_lok,
        'toplam_sure_s': toplam_sure,
    }
    out_data = {'summary': summary, 'goals': results}
    with open(a.out, 'w', encoding='utf-8') as f:
        if a.out.endswith(('.yaml', '.yml')):
            yaml.dump(out_data, f, allow_unicode=True, sort_keys=False)
        else:
            json.dump(out_data, f, indent=2, ensure_ascii=False)
    print('OZET ' + json.dumps(summary, ensure_ascii=False), flush=True)
    nav.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
