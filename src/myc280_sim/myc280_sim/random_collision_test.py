#!/usr/bin/env python3
"""Rastgele konum -> çarpışma testi -> çarpışma varsa hata yaz, yoksa RViz'de git.

fake:=true  (varsayılan): düğüm /joint_states yayınlar (kontrolcü gerekmez).
fake:=false: /arm_group_controller/joint_trajectory'ye komut gönderir.
"""
import random
import time

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
EXTRA_JOINTS = ['front_left_wheel', 'front_right_wheel',
                'rear_left_wheel', 'rear_right_wheel']      # sabit 0 yayınlanır
URDF_LIMITS = [2.9322, 2.3562, 2.6180, 2.5307, 2.8798, 3.0543]   # +/- rad


class RandomCollisionTest(Node):
    def __init__(self):
        super().__init__('random_collision_test')
        self.declare_parameter('group', 'arm_group')
        self.declare_parameter('fake', True)
        self.declare_parameter('controller_topic', '/arm_group_controller/joint_trajectory')
        self.declare_parameter('limit_ratio', 0.9)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('path_steps', 15)

        self.group = self.get_parameter('group').value
        self.fake = bool(self.get_parameter('fake').value)
        self.ratio = float(self.get_parameter('limit_ratio').value)
        self.speed = float(self.get_parameter('speed').value)
        self.hold = float(self.get_parameter('hold_time').value)
        self.steps = int(self.get_parameter('path_steps').value)

        self.cli = self.create_client(GetStateValidity, '/check_state_validity')

        if self.fake:
            self.current = {j: 0.0 for j in JOINTS}
            self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.create_timer(1.0 / 30.0, self.publish_js)
        else:
            self.current = {}
            self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
            self.pub = self.create_publisher(
                JointTrajectory, self.get_parameter('controller_topic').value, 10)

    # ---- sahte donanım ----
    def publish_js(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + EXTRA_JOINTS
        msg.position = [self.current[j] for j in JOINTS] + [0.0] * len(EXTRA_JOINTS)
        self.js_pub.publish(msg)

    def js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.current[n] = p

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def sample(self):
        return [random.uniform(-l * self.ratio, l * self.ratio) for l in URDF_LIMITS]

    # ---- çarpışma kontrolü ----
    def check(self, q):
        req = GetStateValidity.Request()
        req.group_name = self.group
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            raise RuntimeError('check_state_validity yanıt vermedi')
        pairs = sorted({f'{c.contact_body_1} <-> {c.contact_body_2}' for c in res.contacts})
        return res.valid, pairs

    def check_path(self, q_from, q_to):
        for i in range(1, self.steps + 1):
            a = i / self.steps
            q = [f + a * (t - f) for f, t in zip(q_from, q_to)]
            ok, pairs = self.check(q)
            if not ok:
                return False, a, pairs
        return True, 1.0, []

    # ---- hareket ----
    def go_to(self, q):
        cur = [self.current[j] for j in JOINTS]
        dur = max(max(abs(t - c) for t, c in zip(q, cur)) / self.speed, 0.5)

        if self.fake:
            t0 = time.time()
            while rclpy.ok():
                a = min((time.time() - t0) / dur, 1.0)
                for j, c, t in zip(JOINTS, cur, q):
                    self.current[j] = c + a * (t - c)
                rclpy.spin_once(self, timeout_sec=0.02)
                if a >= 1.0:
                    break
            self.spin_for(self.hold)
        else:
            msg = JointTrajectory()
            msg.joint_names = JOINTS
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
            msg.points = [pt]
            self.pub.publish(msg)
            self.spin_for(dur + self.hold)

    def run(self):
        self.get_logger().info('/check_state_validity bekleniyor (move_group açık mı?)...')
        if not self.cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('/check_state_validity yok. demo.launch.py çalışıyor mu?')
            return

        if self.fake:
            self.get_logger().info('Sahte donanım modu: /joint_states bu düğüm tarafından yayınlanıyor.')
            self.spin_for(1.0)
        else:
            self.get_logger().info('/joint_states bekleniyor...')
            last = time.time()
            while rclpy.ok() and not all(j in self.current for j in JOINTS):
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - last > 5.0:
                    self.get_logger().warn('Hâlâ /joint_states gelmiyor (kol eklemleri eksik).')
                    last = time.time()

        n = 0
        while rclpy.ok():
            n += 1
            q = self.sample()
            txt = ', '.join(f'{x:+.2f}' for x in q)

            ok, pairs = self.check(q)
            if not ok:
                self.get_logger().error(f'#{n} ÇARPIŞMA (hedef pozisyon): [{txt}]')
                for p in pairs:
                    self.get_logger().error(f'      {p}')
                self.spin_for(0.3)
                continue

            cur = [self.current[j] for j in JOINTS]
            ok, frac, pairs = self.check_path(cur, q)
            if not ok:
                self.get_logger().error(
                    f'#{n} ÇARPIŞMA (yolda, %{frac * 100:.0f} noktasında): [{txt}]')
                for p in pairs:
                    self.get_logger().error(f'      {p}')
                self.spin_for(0.3)
                continue

            self.get_logger().info(f'#{n} Çarpışma yok, gidiliyor: [{txt}]')
            self.go_to(q)


def main():
    rclpy.init()
    node = RandomCollisionTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
