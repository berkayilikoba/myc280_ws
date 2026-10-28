"""
Odometri/RTAB-Map kaydı için lidar korumalı sürüş deseni.

Desenler (ikisi de başlangıç noktasına geri döner):
  gidis_donus: 360° yerinde dönüş -> ileri D m -> 180° dönüş -> ileri D m -> 180° dönüş
  dikdortgen:  D m -> sola 90° -> W m -> sola 90° -> D m -> sola 90° -> W m -> sola 90°
               Yön hedefleri başlangıç yönüne göre mutlak tutulur (dönüş taşmaları birikmez),
               düz giderken yön P-kontrolle hedefte tutulur.

Mesafe ve açı odom_topic'ten ölçülür; gerçek hareketle farkı zaten araştırdığımız hata.
Güvenlik: ön sektörde (±fwd_sector_deg) safety_dist'ten yakın engel, dönüşte
turn_safety_dist'ten yakın engel, 0.5 s'den eski /scan veya adım süresinin aşılması
robotu durdurur ve betiği sonlandırır. Lidar arkayı görmez; desen geri gitmez.

Kullanım:
    ros2 run limo_localization drive_pattern --ros-args -p distance:=0.8
    # 2 m x 1 m dikdörtgen, EKF çıktısıyla
    ros2 run limo_localization drive_pattern --ros-args -p pattern:=dikdortgen \
        -p distance:=2.0 -p width:=1.0 -p odom_topic:=odometry/filtered
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DrivePattern(Node):

    def __init__(self):
        super().__init__('drive_pattern')
        self.distance = self.declare_parameter('distance', 0.8).value
        self.lin_speed = self.declare_parameter('linear_speed', 0.1).value
        self.ang_speed = self.declare_parameter('angular_speed', 0.3).value
        self.safety_dist = self.declare_parameter('safety_dist', 0.45).value
        self.turn_safety_dist = self.declare_parameter('turn_safety_dist', 0.25).value
        self.fwd_sector = math.radians(self.declare_parameter('fwd_sector_deg', 30.0).value)
        self.pause = self.declare_parameter('pause', 2.0).value
        self.pattern = self.declare_parameter('pattern', 'gidis_donus').value
        self.width = self.declare_parameter('width', 1.0).value
        self.heading_gain = self.declare_parameter('heading_gain', 1.5).value
        self.max_heading_rate = self.declare_parameter('max_heading_rate', 0.2).value
        self.slow_turn_speed = self.declare_parameter('slow_turn_speed', 0.2).value
        odom_topic = self.declare_parameter('odom_topic', 'odom').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.odom = None
        self.scan = None
        self.scan_time = None

        if self.pattern == 'dikdortgen':
            self.steps = [('fwd', self.distance), ('turn', math.pi / 2),
                          ('fwd', self.width), ('turn', math.pi / 2)] * 2
        elif self.pattern == 'gidis_donus':
            self.steps = [('turn', 2 * math.pi), ('fwd', self.distance), ('turn', math.pi),
                          ('fwd', self.distance), ('turn', math.pi)]
        else:
            raise ValueError(f'bilinmeyen desen: {self.pattern}')
        self.goal = None          # dikdortgen: mutlak yön hedefi (rad, sarmalanmamış)
        self.target = None        # bu adımın hedefi (dikdortgen dönüşlerinde düzeltilmiş)
        self.step = -1
        self.start = None
        self.travelled = 0.0
        self.last = None
        self.step_t0 = None
        self.pause_until = None
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_scan(self, m):
        self.scan = m
        self.scan_time = self.get_clock().now()

    def min_range(self, sector):
        m = self.scan
        best = float('inf')
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if abs(wrap(a)) <= sector and math.isfinite(r) and r > max(m.range_min, 0.05):
                best = min(best, r)
        return best

    def stop(self, reason):
        self.pub.publish(Twist())
        self.get_logger().info(reason)
        self.done = True

    def tick(self):
        if self.done:
            self.pub.publish(Twist())
            return
        if self.odom is None or self.scan is None:
            return
        age = (self.get_clock().now() - self.scan_time).nanoseconds * 1e-9
        if age > 0.5:
            return self.stop(f'DUR: /scan {age:.1f} s eski')
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pause_until is not None:
            self.pub.publish(Twist())
            if now < self.pause_until:
                return
            self.pause_until = None
            self.step += 1
            if self.step >= len(self.steps):
                x, y, th = self.odom
                return self.stop(f'BİTTİ. odom son poz: x={x:.3f} y={y:.3f} '
                                 f'yaw={math.degrees(th):.1f}°')
            self.last = self.odom
            self.travelled = 0.0
            self.step_t0 = now
            kind, self.target = self.steps[self.step]
            if self.goal is not None and kind == 'turn':
                # önceki dönüşün taşması/eksiği bu dönüşte telafi edilir
                self.goal += self.target
                self.target = wrap(self.goal - self.odom[2]) % (2 * math.pi)
            self.get_logger().info(
                f'adım {self.step + 1}/{len(self.steps)}: {kind} {self.target:.3f}')
            return
        if self.step < 0:
            self.pause_until = now + self.pause
            if self.pattern == 'dikdortgen':
                self.goal = self.odom[2]
            self.get_logger().info(f'başlangıç odom: {self.odom}')
            return

        kind, target = self.steps[self.step]
        if self.target is not None:
            target = self.target
        x, y, th = self.odom
        lx, ly, lth = self.last
        self.travelled += math.hypot(x - lx, y - ly) if kind == 'fwd' else abs(wrap(th - lth))
        self.last = self.odom
        speed = self.lin_speed if kind == 'fwd' else self.ang_speed
        if now - self.step_t0 > 2.0 * target / speed + 5.0:
            return self.stop(
                f'DUR: adım süresi aşıldı ({kind}, {self.travelled:.2f}/{target:.2f})')

        cmd = Twist()
        if kind == 'fwd':
            d = self.min_range(self.fwd_sector)
            if d < self.safety_dist:
                return self.stop(f'DUR: önde engel {d:.2f} m')
            cmd.linear.x = self.lin_speed
            if self.goal is not None:
                err = wrap(self.goal - th)
                cmd.angular.z = max(-self.max_heading_rate,
                                    min(self.max_heading_rate, self.heading_gain * err))
        else:
            d = self.min_range(math.pi)
            if d < self.turn_safety_dist:
                return self.stop(f'DUR: dönüşte yakın engel {d:.2f} m')
            cmd.angular.z = self.ang_speed
            if self.goal is not None and target - self.travelled < math.radians(10):
                cmd.angular.z = min(self.ang_speed, self.slow_turn_speed)
        if self.travelled >= target:
            self.pub.publish(Twist())
            self.get_logger().info(f'  ölçülen (odom): {self.travelled:.3f} / hedef {target:.3f}')
            self.pause_until = now + self.pause
            return
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = DrivePattern()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
