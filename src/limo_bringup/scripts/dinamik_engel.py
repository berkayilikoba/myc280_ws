#!/usr/bin/env python3
"""Dinamik Engel (Yürüyen İnsan) ve Nav2 Tepki Ölçümü.

Simülasyonda Gazebo varlık servisleri (/spawn_entity, /gazebo/set_entity_state, /delete_entity)
kullanılarak bir insan silindiri (r=0.20m, h=1.70m) oluşturulur ve 20 Hz'de doğrusal
yol noktaları boyunca sabit hızla hareket ettirilir.
Aynı anda Nav2 NavigateToPose hedefi gönderilir ve robotun tepkisi ölçülür:
  - hedefe_ulasti_mi / status (SUCCEEDED / FAILED / CANCELED / TIMEOUT)
  - sure_s (simülasyon süresi)
  - robot-engel en küçük merkez mesafesi (m)
  - robot ayak izi - silindir yüzeyi en küçük açıklığı (m)
  - çarpışma var mı (açıklık < 0)
  - collision_monitor durdurma sayısı (collision_monitor_state veya cmd_vel fallback)
  - Nav2 kurtarma sayısı (bt_navigator recoveries)
  - varış hatası ve yol uzunluğu

Kullanım:
  dinamik_engel.py [--senaryo AD] [--senaryolar YAML] [--mode diff] [--zaman_asimi 120] [--cikti SONUC.yaml]
"""

import argparse
import math
import os
import signal
import sys
import time
import yaml

# ROS 2 modüllerini korumalı içe aktar (ROS'suz ortamda saf fonksiyon testleri için)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

try:
    from nav2_msgs.msg import CollisionMonitorState
    COLLISION_MONITOR_MSG_AVAILABLE = True
except ImportError:
    COLLISION_MONITOR_MSG_AVAILABLE = False


# ==============================================================================
# Saf Geometri ve Yörünge Yardımcıları (ROS Bağımsız)
# ==============================================================================

# LIMO robot gövde ayak izi poligonu (base_link çerçevesinde ±0.161 x ±0.11 m)
DEFAULT_FOOTPRINT = [
    (-0.161, -0.11),
    (-0.161, 0.11),
    (0.161, 0.11),
    (0.161, -0.11)
]

DEFAULT_CYLINDER_RADIUS = 0.20  # metre


def yaw_from_quat(q):
    """Kuaterniyondan z ekseni etrafındaki yaw açısını (radyan) hesaplar."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Açıyı [-pi, pi] aralığına sarar."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def seg_dist(px, py, ax, ay, bx, by):
    """Bir noktanın (px, py) [A, B] doğru parçasına olan en küçük Öklid uzaklığı."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_inside_polygon(px, py, poly):
    """Nokta poligonun içinde mi (ray casting algoritması)."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py) and px < (x2 - x1) * (py - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def point_polygon_distance(px, py, poly):
    """Noktanın poligon kenarlarına olan en küçük uzaklığı."""
    n = len(poly)
    return min(seg_dist(px, py, *poly[i], *poly[(i + 1) % n]) for i in range(n))


def transform_footprint(footprint, robot_pose):
    """Gövde ayak izi köşelerini dünya/harita çerçevesine dönüştürür.
    robot_pose: (rx, ry, ryaw)
    """
    rx, ry, ryaw = robot_pose
    c, s = math.cos(ryaw), math.sin(ryaw)
    return [(rx + c * bx - s * by, ry + s * bx + c * by) for bx, by in footprint]


def footprint_to_cylinder_clearance(robot_pose, footprint, cyl_center, cyl_radius=DEFAULT_CYLINDER_RADIUS):
    """Robot ayak izi poligonu ile silindir yüzeyi arasındaki en küçük açıklık (m).

    Açıklık > 0: Temas yok, serbest alan.
    Açıklık = 0: Silindir yüzeyi ayak izi poligonuna teğet.
    Açıklık < 0: Çarpışma / penetrasyon (|açıklık| penetrasyon derinliğidir).
    """
    poly = transform_footprint(footprint, robot_pose)
    cx, cy = cyl_center
    d_edge = point_polygon_distance(cx, cy, poly)
    if point_inside_polygon(cx, cy, poly):
        # Silindir merkezi robot ayak izi içinde kalmışsa derin çarpışma
        return -(cyl_radius + d_edge)
    return d_edge - cyl_radius


def center_distance(pos1, pos2):
    """İki nokta arasındaki Öklid uzaklığı."""
    return math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])


def interpolate_trajectory(waypoints, speed, elapsed_time, delay=0.0):
    """Yol noktaları boyunca sabit hızla doğrusal yörünge enterpolasyonu.

    Döndürür: (x, y, yaw)
      - elapsed_time <= delay ise ilk yol noktasında bekler, yaw ilk segment yönüdür.
      - speed <= 0 ise ilk yol noktasında sabit kalır.
      - waypoints tek nokta ise o noktada durur (yaw = 0.0).
      - Tüm yol noktaları tamamlandığında son noktada durur, yaw son segment yönüdür.
    """
    if not waypoints:
        raise ValueError("Yol noktaları listesi boş olamaz.")

    if len(waypoints) == 1:
        return (float(waypoints[0][0]), float(waypoints[0][1]), 0.0)

    # İlk segment doğrultusu (yürüyüş yönü)
    dx0 = waypoints[1][0] - waypoints[0][0]
    dy0 = waypoints[1][1] - waypoints[0][1]
    yaw0 = math.atan2(dy0, dx0)

    if elapsed_time <= delay or speed <= 0.0:
        return (float(waypoints[0][0]), float(waypoints[0][1]), yaw0)

    d = speed * (elapsed_time - delay)

    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        seg_len = math.hypot(dx, dy)
        seg_yaw = math.atan2(dy, dx)

        if seg_len == 0.0:
            continue

        if d <= seg_len:
            alpha = d / seg_len
            x = p1[0] + alpha * dx
            y = p1[1] + alpha * dy
            return (x, y, seg_yaw)

        d -= seg_len

    # Son noktaya varıldı
    p_last = waypoints[-1]
    p_prev = waypoints[-2]
    last_yaw = math.atan2(p_last[1] - p_prev[1], p_last[0] - p_prev[0])
    return (float(p_last[0]), float(p_last[1]), last_yaw)


class StopDetector:
    """Collision monitor durdurma olaylarını yükselen kenar mantığıyla sayar."""

    def __init__(self, vel_threshold=0.005):
        self.vel_threshold = vel_threshold
        self.stop_count = 0
        self.is_stopped = False
        self.using_state_msg = False

    def update_from_state(self, action_type):
        """nav2_msgs/CollisionMonitorState eylemi ile güncelle (STOP = 1)."""
        self.using_state_msg = True
        stopped = (action_type == 1 or action_type == 'STOP')
        if stopped and not self.is_stopped:
            self.stop_count += 1
        self.is_stopped = stopped
        return self.stop_count

    def update_from_cmd_vel(self, smoothed_vx, smoothed_wz, cmd_vx, cmd_wz):
        """cmd_vel_smoothed != 0 iken cmd_vel == 0 ise durdurma anıdır."""
        if self.using_state_msg:
            return self.stop_count

        smoothed_active = (abs(smoothed_vx) > self.vel_threshold or abs(smoothed_wz) > self.vel_threshold)
        cmd_zero = (abs(cmd_vx) <= self.vel_threshold and abs(cmd_wz) <= self.vel_threshold)

        stopped = smoothed_active and cmd_zero
        if stopped and not self.is_stopped:
            self.stop_count += 1
        self.is_stopped = stopped
        return self.stop_count

    def reset(self):
        self.stop_count = 0
        self.is_stopped = False
        self.using_state_msg = False


def load_scenarios(yaml_path):
    """Senaryo YAML dosyasını okur ve standart sözlük yapısına dönüştürür."""
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Senaryo dosyası bulunamadı: {yaml_path}")

    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("Geçersiz YAML formatı: Kök nesne bir sözlük olmalıdır.")

    raw_scenarios = data.get('senaryolar', data)
    scenarios = {}

    for name, sdata in raw_scenarios.items():
        if not isinstance(sdata, dict):
            continue

        # robot_hedef ayrıştırma ([x, y, yaw_deg] veya dict)
        hedef_raw = sdata.get('robot_hedef', [0.0, 0.0, 0.0])
        if isinstance(hedef_raw, dict):
            gx = float(hedef_raw.get('x', 0.0))
            gy = float(hedef_raw.get('y', 0.0))
            gyaw = float(hedef_raw.get('yaw_derece', hedef_raw.get('yaw_deg', hedef_raw.get('yaw', 0.0))))
            robot_hedef = (gx, gy, gyaw)
        elif isinstance(hedef_raw, (list, tuple)):
            gx = float(hedef_raw[0]) if len(hedef_raw) > 0 else 0.0
            gy = float(hedef_raw[1]) if len(hedef_raw) > 1 else 0.0
            gyaw = float(hedef_raw[2]) if len(hedef_raw) > 2 else 0.0
            robot_hedef = (gx, gy, gyaw)
        else:
            robot_hedef = (0.0, 0.0, 0.0)

        # engel bilgileri
        engel_data = sdata.get('engel', {})
        yol_raw = engel_data.get('yol', [])
        waypoints = []
        for wp in yol_raw:
            if isinstance(wp, (list, tuple)) and len(wp) >= 2:
                waypoints.append((float(wp[0]), float(wp[1])))

        speed = float(engel_data.get('hiz', 0.0))
        delay = float(engel_data.get('gecikme_s', 0.0))
        aciklama = sdata.get('aciklama', '')

        scenarios[name] = {
            'aciklama': aciklama,
            'robot_hedef': robot_hedef,
            'waypoints': waypoints,
            'speed': speed,
            'delay': delay
        }

    return scenarios


def format_terminal_table(results):
    """Ölçüm sonuçlarını terminal için okunabilir ASCII tablosuna dönüştürür."""
    if not results:
        return "Sonuç bulunamadı."

    headers = [
        "Senaryo", "Durum", "Süre(s)", "Min Açıklık(m)",
        "Çarpışma", "Min Merkez(m)", "Durdurma", "Kurtarma", "Varış Hatası(m)"
    ]
    rows = []
    for r in results:
        min_acik = f"{r['min_aciklik_m']:.3f}" if r.get('min_aciklik_m') is not None else "-"
        min_mrk = f"{r['min_merkez_mesafesi_m']:.3f}" if r.get('min_merkez_mesafesi_m') is not None else "-"
        carp = "EVET" if r.get('carpisma') else "HAYIR"
        varis_h = f"{r['varis_hatasi_m']:.3f}" if r.get('varis_hatasi_m') is not None else "-"
        rows.append([
            str(r.get('senaryo', '')),
            str(r.get('status', '')),
            f"{float(r.get('sure_s', 0.0)):.1f}",
            min_acik,
            carp,
            min_mrk,
            str(r.get('durdurma_sayisi', 0)),
            str(r.get('kurtarma_sayisi', 0)),
            varis_h
        ])

    col_widths = [len(h) for h in headers]
    for row in rows:
        for i, val in enumerate(row):
            col_widths[i] = max(col_widths[i], len(str(val)))

    sep = "+" + "+".join("-" * (w + 2) for w in col_widths) + "+"
    header_str = "|" + "|".join(f" {headers[i]:<{col_widths[i]}} " for i in range(len(headers))) + "|"

    lines = ["", sep, header_str, sep]
    for row in rows:
        line = "|" + "|".join(f" {row[i]:<{col_widths[i]}} " for i in range(len(row))) + "|"
        lines.append(line)
    lines.append(sep)
    lines.append("")
    return "\n".join(lines)


# ==============================================================================
# ROS 2 Gazebo ve Nav2 Ölçüm Yürütücüsü
# ==============================================================================

def make_pose(clock_node, x, y, yaw_deg):
    """map çerçevesinde PoseStamped üretir."""
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = clock_node.get_clock().now().to_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


class DynamicObstacleRunner:
    """Gazebo'da dinamik engeli yürütür ve Nav2 koşusunu ölçer."""

    def __init__(self, nav, footprint=DEFAULT_FOOTPRINT, model_sdf_path=None, entity_name='insan_silindir'):
        self.nav = nav
        self.footprint = footprint
        self.entity_name = entity_name
        self.model_sdf_path = model_sdf_path
        self.model_xml = None
        if model_sdf_path and os.path.exists(model_sdf_path):
            with open(model_sdf_path, 'r', encoding='utf-8') as f:
                self.model_xml = f.read()

        self.pose = None  # (x, y, yaw)
        self.path_len = 0.0
        self._gercek_var = False
        self.last_smoothed_vx = 0.0
        self.last_smoothed_wz = 0.0
        self.last_cmd_vx = 0.0
        self.last_cmd_wz = 0.0
        self.stop_detector = StopDetector()

        # Gazebo servis istemcileri
        self.spawn_client = self.nav.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.nav.create_client(DeleteEntity, '/delete_entity')
        self.set_state_client = self.nav.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Abonelikler
        self.nav.create_subscription(Odometry, '/odom', lambda m: None if self._gercek_var else self._odom(m), 20)
        self.nav.create_subscription(Odometry, '/odom_gercek', self._odom_gercek, 20)
        self.nav.create_subscription(Twist, '/cmd_vel_smoothed', self._cmd_vel_smoothed, 20)
        self.nav.create_subscription(Twist, '/cmd_vel', self._cmd_vel, 20)
        self.cmd_vel_pub = self.nav.create_publisher(Twist, '/cmd_vel', 10)

        if COLLISION_MONITOR_MSG_AVAILABLE:
            self.nav.create_subscription(
                CollisionMonitorState,
                '/collision_monitor_state',
                self._collision_monitor_state,
                10
            )

    def now(self):
        """Simülasyon saatini saniye cinsinden döndürür."""
        return self.nav.get_clock().now().nanoseconds * 1e-9

    def _odom_gercek(self, m):
        self._gercek_var = True
        self._odom(m)

    def _odom(self, m):
        p = m.pose.pose
        cur = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        if self.pose is not None:
            self.path_len += math.hypot(cur[0] - self.pose[0], cur[1] - self.pose[1])
        self.pose = cur

    def _cmd_vel_smoothed(self, m):
        self.last_smoothed_vx = m.linear.x
        self.last_smoothed_wz = m.angular.z

    def _cmd_vel(self, m):
        self.last_cmd_vx = m.linear.x
        self.last_cmd_wz = m.angular.z

    def _collision_monitor_state(self, m):
        action = getattr(m, 'action_type', None)
        if action is not None:
            self.stop_detector.update_from_state(action)

    def spawn_obstacle(self, initial_pose):
        """Engeli Gazebo'da doğurur. initial_pose: (x, y, yaw)"""
        if not self.model_xml:
            self.nav.get_logger().error(f"SDF model dosyası yüklenemedi: {self.model_sdf_path}")
            return False

        # Varsa eskisi silinsin
        self.delete_obstacle()

        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.nav.get_logger().error("/spawn_entity servisi bulunamadı!")
            return False

        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.model_xml
        req.reference_frame = 'world'
        req.initial_pose.position.x = float(initial_pose[0])
        req.initial_pose.position.y = float(initial_pose[1])
        req.initial_pose.position.z = 0.0
        yaw = float(initial_pose[2])
        req.initial_pose.orientation.z = math.sin(yaw / 2.0)
        req.initial_pose.orientation.w = math.cos(yaw / 2.0)

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self.nav, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.nav.get_logger().info(f"Model '{self.entity_name}' başarıyla doğuruldu.")
            return True
        else:
            msg = future.result().status_message if future.result() else 'Zaman aşımı'
            self.nav.get_logger().warn(f"Model doğurulamadı: {msg}")
            return False

    def delete_obstacle(self):
        """Engeli Gazebo'dan siler."""
        if not self.delete_client.wait_for_service(timeout_sec=2.0):
            return False
        req = DeleteEntity.Request()
        req.name = self.entity_name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self.nav, future, timeout_sec=3.0)
        return True

    def set_obstacle_state_async(self, x, y, yaw, vx=0.0, vy=0.0):
        """20 Hz döngüsü için asenkron engel konumu ve hızı güncellemesi."""
        if not self.set_state_client.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = self.entity_name
        req.state.reference_frame = 'world'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.0
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.twist.linear.x = float(vx)
        req.state.twist.linear.y = float(vy)
        self.set_state_client.call_async(req)

    def stop_robot(self):
        """Robota sıfır hız gönderir."""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def run_scenario(self, scenario_name, scenario_cfg, timeout=120.0):
        """Tek bir dinamik engel senaryosunu baştan sona icra eder."""
        robot_hedef = scenario_cfg['robot_hedef']
        gx, gy, gyaw = robot_hedef
        waypoints = scenario_cfg['waypoints']
        speed = scenario_cfg.get('speed', 0.0)
        delay = scenario_cfg.get('delay', 0.0)

        self.nav.get_logger().info(
            f"=== Senaryo Başlatılıyor: {scenario_name} (Hedef: {robot_hedef}, Engel hız: {speed} m/s) ==="
        )

        init_x, init_y, init_yaw = interpolate_trajectory(waypoints, speed, 0.0, delay=delay)
        if not self.spawn_obstacle((init_x, init_y, init_yaw)):
            self.nav.get_logger().warn("Engel doğurulamadı, senaryoya devam ediliyor.")

        # Metrikleri sıfırla
        self.path_len = 0.0
        self.stop_detector.reset()
        min_center_dist = float('inf')
        min_clearance = float('inf')
        has_collision = False
        max_recoveries = 0

        # Başlamadan önce kısa stabilizasyon beklemesi
        end_wait = time.monotonic() + 0.5
        while time.monotonic() < end_wait:
            rclpy.spin_once(self.nav, timeout_sec=0.05)

        # Nav2 hedefini gönder
        target_pose = make_pose(self.nav, gx, gy, gyaw)
        self.nav.goToPose(target_pose)

        start_sim = self.now()
        last_update_sim = start_sim
        status = None

        try:
            while not self.nav.isTaskComplete():
                rclpy.spin_once(self.nav, timeout_sec=0.02)
                now_sim = self.now()
                if now_sim == 0.0:
                    continue

                elapsed = now_sim - start_sim

                # 20 Hz kontrol döngüsü (~50 ms)
                if now_sim - last_update_sim >= 0.05:
                    last_update_sim = now_sim
                    ox, oy, oyaw = interpolate_trajectory(waypoints, speed, elapsed, delay=delay)
                    vx = speed * math.cos(oyaw) if (elapsed > delay and speed > 0) else 0.0
                    vy = speed * math.sin(oyaw) if (elapsed > delay and speed > 0) else 0.0
                    self.set_obstacle_state_async(ox, oy, oyaw, vx, vy)

                    # Metrik hesaplamaları
                    if self.pose is not None:
                        rx, ry, ryaw = self.pose
                        d_c = center_distance((rx, ry), (ox, oy))
                        if d_c < min_center_dist:
                            min_center_dist = d_c

                        c_fp = footprint_to_cylinder_clearance(
                            self.pose, self.footprint, (ox, oy), cyl_radius=DEFAULT_CYLINDER_RADIUS
                        )
                        if c_fp < min_clearance:
                            min_clearance = c_fp
                        if c_fp < 0.0:
                            has_collision = True

                    # Durdurma sayacı (cmd_vel fallback kontrolü)
                    self.stop_detector.update_from_cmd_vel(
                        self.last_smoothed_vx, self.last_smoothed_wz,
                        self.last_cmd_vx, self.last_cmd_wz
                    )

                # Kurtarma sayısı
                fb = self.nav.getFeedback()
                if fb is not None:
                    max_recoveries = max(max_recoveries, getattr(fb, 'number_of_recoveries', 0))

                # Zaman aşımı kontrolü
                if elapsed > timeout:
                    self.nav.get_logger().warn(f"Senaryo zaman aşımına uğradı ({timeout} s)")
                    self.nav.cancelTask()
                    status = 'TIMEOUT'
                    break

        except KeyboardInterrupt:
            self.nav.get_logger().info("Kullanıcı tarafından durduruldu (Ctrl-C)")
            self.nav.cancelTask()
            status = 'CANCELED'
            raise

        finally:
            # Senaryo bittiğinde veya kesildiğinde engeli sil ve robota durma komutu gönder
            self.delete_obstacle()
            self.stop_robot()

        if status is None:
            r = self.nav.getResult()
            status = {
                TaskResult.SUCCEEDED: 'SUCCEEDED',
                TaskResult.CANCELED: 'CANCELED',
                TaskResult.FAILED: 'FAILED'
            }.get(r, str(r))

        sure_s = round(self.now() - start_sim, 2)
        rx, ry, ryaw = self.pose if self.pose is not None else (0.0, 0.0, 0.0)
        pos_err = round(math.hypot(rx - gx, ry - gy), 3)
        yaw_err = round(abs(math.degrees(wrap(ryaw - math.radians(gyaw)))), 1)
        min_aciklik = None if math.isinf(min_clearance) else round(min_clearance, 3)
        min_merkez = None if math.isinf(min_center_dist) else round(min_center_dist, 3)

        result_dict = {
            'senaryo': scenario_name,
            'status': status,
            'durum': status,
            'sure_s': sure_s,
            'sim_time_s': sure_s,
            'min_aciklik_m': min_aciklik,
            'en_kucuk_aciklik_m': min_aciklik,
            'min_clear_m': min_aciklik,
            'carpisma': has_collision,
            'carpisma_var_mi': has_collision,
            'min_merkez_mesafesi_m': min_merkez,
            'en_kucuk_merkez_mesafesi_m': min_merkez,
            'durdurma_sayisi': self.stop_detector.stop_count,
            'collision_monitor_durdurma': self.stop_detector.stop_count,
            'kurtarma_sayisi': int(max_recoveries),
            'recoveries': int(max_recoveries),
            'varis_hatasi_m': pos_err,
            'pos_err_m': pos_err,
            'varis_yon_hatasi_deg': yaw_err,
            'yaw_err_deg': yaw_err,
            'yol_uzunlugu_m': round(self.path_len, 2),
            'path_len_m': round(self.path_len, 2),
            'son_konum': [round(rx, 3), round(ry, 3), round(math.degrees(ryaw), 1)]
        }
        return result_dict


# ==============================================================================
# Ana Giriş Noktası
# ==============================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(description="Dinamik Engel (Yürüyen İnsan) ve Nav2 Tepki Ölçümü")
    ap.add_argument('--senaryo', default='hepsi', help='Çalıştırılacak senaryo adı (karsidan, yandan_kesen, duran_engel, takip) veya "hepsi"')
    ap.add_argument('--senaryolar', default=None, help='Senaryolar YAML dosya yolu')
    ap.add_argument('--model', default=None, help='insan_silindir model.sdf dosya yolu')
    ap.add_argument('--mode', choices=['diff', 'ackermann'], default='diff', help='Robot sürüş modu')
    ap.add_argument('--zaman_asimi', type=float, default=120.0, help='Senaryo zaman aşımı süresi (s)')
    ap.add_argument('--cikti', default=None, help='Sonuçların kaydedileceği YAML dosya yolu')
    a = ap.parse_args(argv)

    if not ROS_AVAILABLE:
        print("HATA: ROS 2 ortamı (rclpy) bulunamadı. Lütfen ROS 2 çalışma alanını kaynaklayın (source).", file=sys.stderr)
        return 1

    # Senaryo YAML yolu belirleme
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path = a.senaryolar or os.path.join(script_dir, 'dinamik_engel_senaryolar.yaml')
    try:
        scenarios = load_scenarios(yaml_path)
    except Exception as e:
        print(f"HATA: Senaryo dosyası okunamadı ({yaml_path}): {e}", file=sys.stderr)
        return 2

    # Model SDF yolu belirleme
    model_path = a.model
    candidate_paths = [model_path] if model_path else []
    if not model_path:
        candidate_paths = [
            os.path.abspath(os.path.join(script_dir, '..', '..', 'limo_car', 'models', 'insan_silindir', 'model.sdf')),
            '/home/gunes/limo_robot/limo_ws/src/limo_car/models/insan_silindir/model.sdf'
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            candidate_paths.insert(0, os.path.join(get_package_share_directory('limo_car'), 'models', 'insan_silindir', 'model.sdf'))
        except Exception:
            pass

        for p in candidate_paths:
            if os.path.exists(p):
                model_path = p
                break

    if not model_path or not os.path.exists(model_path):
        print(f"UYARI: insan_silindir model.sdf bulunamadı. Aranan yollar: {candidate_paths}", file=sys.stderr)

    # Seçilen senaryolar
    if a.senaryo in ('hepsi', 'all', ''):
        run_list = list(scenarios.keys())
    elif a.senaryo in scenarios:
        run_list = [a.senaryo]
    else:
        print(f"HATA: Geçersiz senaryo adı '{a.senaryo}'. Mevcut senaryolar: {list(scenarios.keys())}", file=sys.stderr)
        return 3

    # ROS başlat
    rclpy.init()
    nav = BasicNavigator(node_name='dinamik_engel_runner')
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    runner = DynamicObstacleRunner(
        nav=nav,
        footprint=DEFAULT_FOOTPRINT,
        model_sdf_path=model_path,
        entity_name='insan_silindir'
    )

    # Simülasyon saati ve /odom gelene kadar bekle
    t0 = time.monotonic()
    nav.get_logger().info("Simülasyon saati ve /odom_gercek bekleniyor...")
    while (runner.pose is None or runner.now() == 0.0) and time.monotonic() - t0 < 60:
        rclpy.spin_once(nav, timeout_sec=0.2)

    if runner.pose is None:
        nav.get_logger().warn("Uyarı: /odom henüz gelmedi, devam ediliyor.")

    # Nav2'nin hazır olmasını bekle
    try:
        nav.waitUntilNav2Active()
    except Exception as e:
        nav.get_logger().warn(f"waitUntilNav2Active uyarısı: {e}")

    results = []
    try:
        for sname in run_list:
            scfg = scenarios[sname]
            res = runner.run_scenario(sname, scfg, timeout=a.zaman_asimi)
            results.append(res)
    except KeyboardInterrupt:
        nav.get_logger().info("Testler kullanıcı tarafından kesildi.")
    finally:
        runner.delete_obstacle()
        runner.stop_robot()

    # Terminal tablosu
    table_str = format_terminal_table(results)
    print(table_str)

    # YAML çıktısı
    if a.cikti:
        out_dir = os.path.dirname(os.path.abspath(a.cikti))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        out_data = {
            'ozet': {
                'toplam_senaryo': len(results),
                'basarili_sayisi': sum(1 for r in results if r['status'] == 'SUCCEEDED'),
                'carpisma_sayisi': sum(1 for r in results if r['carpisma']),
                'toplam_sure_s': round(sum(r['sure_s'] for r in results), 1)
            },
            'senaryolar': {r['senaryo']: r for r in results}
        }
        with open(a.cikti, 'w', encoding='utf-8') as f:
            yaml.dump(out_data, f, allow_unicode=True, sort_keys=False)
        print(f"Sonuçlar kaydedildi: {a.cikti}")

    nav.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
