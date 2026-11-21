#!/usr/bin/env python3
"""Küpü RViz'de rastgele bir noktaya at -> çarpışma testi -> çarpışma varsa hata yaz,
yoksa Cartesian path ile küpe ilerle, home pozisyonuna dön ve sonraki küpe geç.

Her turda:
1. Küp sahneye eklenir (random:=true ise rastgele konum, değilse cube_x/y/z).
2. Küp robotun/gövdenin içine düştüyse: ÇARPIŞMA (küp başlangıç konumunda).
3. Hazır poz, hazır poza giden yol ve Cartesian yaklaşma yolu çarpışma için kontrol edilir
   (MoveIt /check_state_validity). Çarpışan link çiftleri terminale yazılır.
4. Çarpışma yoksa kol hazır poza gider, Cartesian path ile küpe ilerler, geri çekilip
   home pozisyonuna döner ve bir sonraki küpe geçer.

Kontrolcü yok: düğüm /joint_states'i kendisi yayınlar (sahte donanım).
Konumlar `frame` (varsayılan joint1 = kolun tabanı) çerçevesindedir.

Çarpışmayı bilerek denemek için: -p stop_distance:=-0.02 (flanş küpün içine girer).
"""
import itertools
import math
import random
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene, RobotState
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetPositionIK,
                             GetStateValidity)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
WHEELS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
HOME_Q = [0.0] * len(JOINTS)
CUBE_ID = 'cube'


# ---------------- yönelim adayları ----------------
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qaxis_y(deg):
    h = math.radians(deg) / 2.0
    return (0.0, math.sin(h), 0.0, math.cos(h))


def det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def mat_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s,
                (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s,
                (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s,
            (m[1][0] - m[0][1]) / s)


def make_candidates():
    """İlk 24: eksen hizalı yönelimler. Sonraki 48: aynıları +30 / -30 derece eğimli."""
    base = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = [[0, 0, 0] for _ in range(3)]
            for i, (p, s) in enumerate(zip(perm, signs)):
                m[i][p] = s
            if det3(m) == 1:
                base.append(mat_to_quat(m))
    return [qmul(qaxis_y(t), q) for t in (0, 30, -30) for q in base]


def lerp_path(a, b, n=12):
    return [[x + (i / n) * (y - x) for x, y in zip(a, b)] for i in range(1, n + 1)]


# ---------------- düğüm ----------------
class CubeApproach(Node):
    def __init__(self):
        super().__init__('cube_approach')
        defaults = [('group', 'arm_group'), ('ee_link', 'joint6_flange'), ('frame', 'joint1'),
                    ('random', True),             # true: her turda rastgele küp konumu
                    ('cube_x', 0.17), ('cube_y', 0.0), ('cube_z', 0.10),   # random:=false için
                    ('cube_size', 0.04),
                    ('r_min', 0.10), ('r_max', 0.26),                      # rastgele konum aralığı
                    ('z_min', 0.0), ('z_max', 0.25),
                    ('approach', 'top'),          # top: yukarıdan in, front: önden yaklaş
                    ('stop_distance', 0.05),      # flanş küp yüzeyine bu kadar kala durur (m)
                    ('approach_len', 0.08),       # Cartesian doğrunun uzunluğu (m)
                    ('num_candidates', 24),       # denenecek uç yönelimi sayısı (max 72)
                    ('hold_time', 3.0),
                    ('joint_speed', 0.6),         # hazır poza / home'a giderken (rad/s)
                    ('path_speed', 0.25)]         # Cartesian hareket sırasında (rad/s)
        for name, val in defaults:
            self.declare_parameter(name, val)
        p = self.get_parameter
        self.group = p('group').value
        self.ee = p('ee_link').value
        self.frame = p('frame').value
        self.random = bool(p('random').value)
        self.fixed_cube = (float(p('cube_x').value), float(p('cube_y').value),
                           float(p('cube_z').value))
        self.size = float(p('cube_size').value)
        self.r_min, self.r_max = float(p('r_min').value), float(p('r_max').value)
        self.z_min, self.z_max = float(p('z_min').value), float(p('z_max').value)
        self.approach = p('approach').value
        self.stop = float(p('stop_distance').value)
        self.app_len = float(p('approach_len').value)
        self.hold = float(p('hold_time').value)
        self.joint_speed = float(p('joint_speed').value)
        self.path_speed = float(p('path_speed').value)
        self.cands = make_candidates()[:int(p('num_candidates').value)]
        self.cube = self.fixed_cube

        self.current = {j: 0.0 for j in JOINTS}
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / 30.0, self.publish_js)

        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.valid_cli = self.create_client(GetStateValidity, '/check_state_validity')

    # ---- sahte donanım ----
    def publish_js(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + WHEELS
        msg.position = [self.current[j] for j in JOINTS] + [0.0] * len(WHEELS)
        self.js_pub.publish(msg)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def play(self, qs, speed):
        """Eklem konumları listesini (JOINTS sırasıyla) sabit hızla oynatır."""
        if not qs:
            return
        timed, t = [(0.0, qs[0])], 0.0
        for a, b in zip(qs, qs[1:]):
            t += max(max(abs(x - y) for x, y in zip(a, b)) / speed, 0.01)
            timed.append((t, b))
        t0, i = time.time(), 0
        while rclpy.ok():
            el = time.time() - t0
            if el >= timed[-1][0]:
                self.current = dict(zip(JOINTS, timed[-1][1]))
                break
            while timed[i + 1][0] < el:
                i += 1
            (ta, qa), (tb, qb) = timed[i], timed[i + 1]
            a = (el - ta) / (tb - ta)
            self.current = dict(zip(JOINTS, [x + a * (y - x) for x, y in zip(qa, qb)]))
            rclpy.spin_once(self, timeout_sec=0.01)

    def q_list(self):
        return [self.current[j] for j in JOINTS]

    # ---- servis yardımcıları ----
    def call(self, cli, req, timeout=5.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def robot_state(self, joint_dict):
        rs = RobotState()
        rs.joint_state.name = list(joint_dict.keys())
        rs.joint_state.position = [float(v) for v in joint_dict.values()]
        return rs

    def pose(self, xyz, quat):
        ps = Pose()
        ps.position.x, ps.position.y, ps.position.z = [float(v) for v in xyz]
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = \
            [float(v) for v in quat]
        return ps

    # ---- küp ----
    def add_cube(self, xyz):
        co = CollisionObject()
        co.header.frame_id = self.frame
        co.id = CUBE_ID
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.size] * 3
        co.primitives = [box]
        co.primitive_poses = [self.pose(xyz, (0.0, 0.0, 0.0, 1.0))]
        co.operation = CollisionObject.ADD       # aynı id varsa değiştirir

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        scene.object_colors = [ObjectColor(id=CUBE_ID,
                                           color=ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0))]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = self.call(self.scene_cli, req)
        return res is not None and res.success

    def random_cube(self):
        r = math.sqrt(random.uniform(self.r_min ** 2, self.r_max ** 2))
        a = random.uniform(-math.pi, math.pi)
        return (round(r * math.cos(a), 3), round(r * math.sin(a), 3),
                round(random.uniform(self.z_min, self.z_max), 3))

    # ---- IK / Cartesian / çarpışma kontrolü ----
    def ik(self, xyz, quat, seed):
        """Çarpışma kontrolü KAPALI: çarpışmayı ayrıca check() ile raporluyoruz."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group
        r.ik_link_name = self.ee
        r.pose_stamped = PoseStamped()
        r.pose_stamped.header.frame_id = self.frame
        r.pose_stamped.pose = self.pose(xyz, quat)
        r.robot_state = self.robot_state(seed)
        r.avoid_collisions = False
        r.timeout = Duration(sec=0, nanosec=100_000_000)
        res = self.call(self.ik_cli, req)
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        if not all(j in sol for j in JOINTS):
            return None
        return {j: sol[j] for j in JOINTS}

    def cartesian(self, start, xyz, quat):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame
        req.start_state = self.robot_state(start)
        req.group_name = self.group
        req.link_name = self.ee
        req.waypoints = [self.pose(xyz, quat)]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        return self.call(self.cart_cli, req, timeout=10.0)

    def check(self, q):
        """(geçerli_mi, çarpışan_çiftler)"""
        req = GetStateValidity.Request()
        req.group_name = self.group
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in q]
        res = self.call(self.valid_cli, req)
        if res is None:
            raise RuntimeError('check_state_validity yanıt vermedi')
        pairs = sorted({f'{c.contact_body_1} <-> {c.contact_body_2}' for c in res.contacts})
        return res.valid, pairs

    def path_check(self, qs, max_pts=25):
        """Yoldan eşit aralıklı en fazla max_pts nokta kontrol eder (son nokta dahil)."""
        n = len(qs)
        if n == 0:
            return True, 1.0, []
        idx = sorted({round(i * (n - 1) / (max_pts - 1)) for i in range(max_pts)}) \
            if n > max_pts else range(n)
        for i in idx:
            ok, pairs = self.check(qs[i])
            if not ok:
                return False, i / max(n - 1, 1), pairs
        return True, 1.0, []

    # ---- planlama ----
    def targets(self):
        cx, cy, _ = self.cube
        if self.approach == 'front':
            n = max(math.hypot(cx, cy), 1e-6)
            d = (cx / n, cy / n, 0.0)
        else:
            d = (0.0, 0.0, -1.0)           # yukarıdan aşağı
        gap = self.size / 2.0 + self.stop
        final = tuple(c - di * gap for c, di in zip(self.cube, d))
        pre = tuple(f - di * self.app_len for f, di in zip(final, d))
        return pre, final

    def plan(self):
        pre, final = self.targets()
        home_dict = dict(zip(JOINTS, HOME_Q))
        first_col, n_ik = None, 0
        # top modunda tek sabit yönelim: X etrafında 180° -> tool0 Z ekseni düz aşağı (küpe dik)
        top_quat = (1.0, 0.0, 0.0, 0.0)   # (x, y, z, w)
        cands_to_try = [top_quat] if self.approach == 'top' else self.cands
        for quat in cands_to_try:
            sol = self.ik(pre, quat, home_dict)
            if sol is None:
                continue
            n_ik += 1
            q_pre = [sol[j] for j in JOINTS]

            ok, pairs = self.check(q_pre)
            if not ok:
                first_col = first_col or ('hazır poz', pairs)
                continue

            res = self.cartesian(sol, final, quat)
            if (res is None or res.fraction < 0.99
                    or not res.solution.joint_trajectory.points):
                continue            # yol boyunca IK çözülemedi (erişim sorunu, çarpışma değil)
            jt = res.solution.joint_trajectory
            idx = [list(jt.joint_names).index(j) for j in JOINTS]
            path = [[pt.positions[i] for i in idx] for pt in jt.points]

            ok, frac, pairs = self.path_check(path)
            if not ok:
                first_col = first_col or (f'Cartesian yaklaşma yolu (%{frac * 100:.0f})', pairs)
                continue

            ok, frac, pairs = self.path_check(lerp_path(self.q_list(), q_pre))
            if not ok:
                first_col = first_col or (f'hazır poza giderken (%{frac * 100:.0f})', pairs)
                continue

            return {'ok': True, 'q_pre': q_pre, 'path': path}

        if first_col:
            return {'ok': False, 'reason': 'collision', 'where': first_col[0],
                    'pairs': first_col[1]}
        return {'ok': False, 'reason': 'unreachable', 'n_ik': n_ik}

    def return_home(self, n, ret_path):
        """Geri çekilip home pozisyonuna ulaşılabilir/çarpışmasız dönüşü dener."""
        ret_ok, frac, pairs = self.path_check(ret_path)
        if not ret_ok:
            self.get_logger().warn(
                f'#{n} Geri çekilme yolunda çarpışma (%{frac * 100:.0f}), '
                f'doğrudan home dönüşü deneniyor: {pairs}')
        else:
            self.play(ret_path, self.path_speed)

        to_home = lerp_path(self.q_list(), HOME_Q)
        home_ok, frac, pairs = self.path_check(to_home)
        home_valid, home_pairs = self.check(HOME_Q)
        if home_ok and home_valid:
            self.play(to_home, self.joint_speed)
            self.get_logger().info(f'#{n} Home pozisyonuna dönüldü.')
            return True

        self.get_logger().warn(
            f'#{n} Home pozisyonuna dönüşte sorun (geçerli={home_valid}, '
            f'çiftler={pairs or home_pairs}); mevcut konumda kalınıyor.')
        return False

    def report_collision(self, n, where, pairs):
        x, y, z = self.cube
        self.get_logger().error(
            f'#{n} ÇARPIŞMA ({where}): küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        for pr in pairs:
            self.get_logger().error(f'      {pr}')

    # ---- bir tur ----
    def cycle(self, n):
        self.cube = self.random_cube() if self.random else self.fixed_cube
        x, y, z = self.cube
        if not self.add_cube(self.cube):
            self.get_logger().error(f'#{n} Küp sahneye eklenemedi.')
            return False
        self.spin_for(0.5)                          # RViz küpü göstersin

        ok, pairs = self.check(self.q_list())       # küp robotun/gövdenin içine mi düştü?
        if not ok:
            self.report_collision(n, 'küp başlangıç konumunda', pairs)
            self.spin_for(self.hold)
            return False

        res = self.plan()
        if not res['ok']:
            if res['reason'] == 'collision':
                self.report_collision(n, res['where'], res['pairs'])
            else:
                self.get_logger().warn(
                    f'#{n} Erişilemez (IK çözümü yok / yol bulunamadı): '
                    f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
            self.spin_for(self.hold)
            return False

        self.get_logger().info(
            f'#{n} Çarpışma yok, gidiliyor: küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        self.play([self.q_list(), res['q_pre']], self.joint_speed)
        self.spin_for(0.3)
        self.play(res['path'], self.path_speed)           # Cartesian path ile ilerle
        self.get_logger().info(f'#{n} Hedefe varıldı.')
        self.spin_for(self.hold)

        self.return_home(n, list(reversed(res['path'])))
        return True

    def run(self):
        for cli, name in ((self.ik_cli, '/compute_ik'),
                          (self.cart_cli, '/compute_cartesian_path'),
                          (self.scene_cli, '/apply_planning_scene'),
                          (self.valid_cli, '/check_state_validity')):
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f'{name} yok. demo.launch.py çalışıyor mu?')
                return
        self.spin_for(1.0)      # home pozunu yayınla

        n = 0
        while rclpy.ok():
            n += 1
            self.cycle(n)
            if not self.random:
                break
        self.get_logger().info('Bitti. Çıkmak için Ctrl+C.')


def main():
    rclpy.init()
    node = CubeApproach()
    try:
        node.run()
        rclpy.spin(node)        # /joint_states yayını sürsün
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()