#!/usr/bin/env python3
import itertools
import math
import random
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, ObjectColor,
                             PlanningScene, PlanningSceneComponents, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan,
                             GetPlanningScene, GetPositionIK, GetStateValidity)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
WHEELS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
HOME_Q = [0.0] * len(JOINTS)
CUBE_ID = 'cube'


def multiplyQuat(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quatAxisY(deg):
    halfRad = math.radians(deg) / 2.0
    return (0.0, math.sin(halfRad), 0.0, math.cos(halfRad))


def det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def matToQuat(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
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


def makeCandidates():
    base = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = [[0, 0, 0] for _ in range(3)]
            for i, (p, s) in enumerate(zip(perm, signs)):
                m[i][p] = s
            if det3(m) == 1:
                base.append(matToQuat(m))
    return [multiplyQuat(quatAxisY(t), q) for t in (0, 30, -30) for q in base]


def lerpPath(startQ, endQ, steps=40):
    return [[x + (i / steps) * (y - x) for x, y in zip(startQ, endQ)]
            for i in range(0, steps + 1)]


def densify(qs, stepsPerSegment=10):
    """OMPL yolunun seyrek noktalarını sık noktalara böler (hem kontrol hem oynatma için)."""
    if len(qs) < 2:
        return list(qs)
    out = []
    for a, b in zip(qs, qs[1:]):
        out.extend(lerpPath(a, b, stepsPerSegment)[:-1])
    out.append(list(qs[-1]))
    return out


class CubeApproach(Node):
    def __init__(self):
        super().__init__('cube_approach')
        defaults = [('group', 'arm_group'), ('ee_link', 'joint6_flange'), ('frame', 'joint1'),
                    ('random', True),
                    ('cube_x', 0.17), ('cube_y', 0.0), ('cube_z', 0.10),
                    ('cube_size', 0.04),
                    ('r_min', 0.10), ('r_max', 0.26),
                    ('z_min', 0.0), ('z_max', 0.25),
                    ('approach', 'top'),
                    ('stop_distance', 0.05),
                    ('approach_len', 0.08),
                    ('num_candidates', 24),
                    ('check_points', 0),      # 0 = yolun tüm noktaları kontrol edilir
                    ('use_ompl', True),       # lerp yolu çarpışırsa OMPL ile yeniden planla
                    ('planning_time', 2.0),
                    ('debug', False),
                    ('hold_time', 5.0),
                    ('pause_time', 3.0),
                    ('move_duration', 5.0)]
        for name, val in defaults:
            self.declare_parameter(name, val)
        p = self.get_parameter
        self.group = p('group').value
        self.ee = p('ee_link').value
        self.frame = p('frame').value
        self.random = bool(p('random').value)
        self.fixedCube = (float(p('cube_x').value), float(p('cube_y').value),
                          float(p('cube_z').value))
        self.size = float(p('cube_size').value)
        self.rMin, self.rMax = float(p('r_min').value), float(p('r_max').value)
        self.zMin, self.zMax = float(p('z_min').value), float(p('z_max').value)
        self.approach = p('approach').value
        self.stop = float(p('stop_distance').value)
        self.appLen = float(p('approach_len').value)
        self.checkPoints = int(p('check_points').value)
        self.useOmpl = bool(p('use_ompl').value)
        self.planTime = float(p('planning_time').value)
        self.debug = bool(p('debug').value)
        self.hold = float(p('hold_time').value)
        self.pause = float(p('pause_time').value)
        self.moveDur = float(p('move_duration').value)
        self.cands = makeCandidates()[:int(p('num_candidates').value)]
        self.cube = self.fixedCube

        self.current = {j: 0.0 for j in JOINTS}
        self.jsPub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / 30.0, self.publishJointStates)

        self.ikCli = self.create_client(GetPositionIK, '/compute_ik')
        self.cartCli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.sceneCli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validCli = self.create_client(GetStateValidity, '/check_state_validity')
        self.getSceneCli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.planCli = self.create_client(GetMotionPlan, '/plan_kinematic_path')

    def publishJointStates(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + WHEELS
        msg.position = [self.current[j] for j in JOINTS] + [0.0] * len(WHEELS)
        self.jsPub.publish(msg)

    def spinFor(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def playDuration(self, qs, duration):
        if not qs or len(qs) < 2:
            return
        dists = [0.0]
        for a, b in zip(qs, qs[1:]):
            dists.append(dists[-1] + max(max(abs(x - y) for x, y in zip(a, b)), 1e-6))
        total = dists[-1]
        timed = [(d / total * duration, q) for d, q in zip(dists, qs)]

        t0, i = time.time(), 0
        while rclpy.ok():
            el = time.time() - t0
            if el >= duration:
                self.current = dict(zip(JOINTS, qs[-1]))
                break
            while i + 1 < len(timed) - 1 and timed[i + 1][0] < el:
                i += 1
            (ta, qa), (tb, qb) = timed[i], timed[i + 1]
            dt = tb - ta
            alpha = (el - ta) / dt if dt > 0 else 1.0
            alpha = min(max(alpha, 0.0), 1.0)
            self.current = dict(zip(JOINTS,
                                    [x + alpha * (y - x) for x, y in zip(qa, qb)]))
            rclpy.spin_once(self, timeout_sec=0.01)

    def getJointList(self):
        return [self.current[j] for j in JOINTS]

    def callService(self, cli, req, timeout=5.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def buildRobotState(self, jointDict):
        rs = RobotState()
        rs.is_diff = True
        rs.joint_state.name = list(jointDict.keys())
        rs.joint_state.position = [float(v) for v in jointDict.values()]
        return rs

    def buildPose(self, xyz, quat):
        ps = Pose()
        ps.position.x, ps.position.y, ps.position.z = [float(v) for v in xyz]
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = \
            [float(v) for v in quat]
        return ps

    def addCube(self, xyz):
        co = CollisionObject()
        co.header.frame_id = self.frame
        co.id = CUBE_ID
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.size] * 3
        co.primitives = [box]
        co.primitive_poses = [self.buildPose(xyz, (0.0, 0.0, 0.0, 1.0))]
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        scene.object_colors = [ObjectColor(id=CUBE_ID,
                                           color=ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0))]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = self.callService(self.sceneCli, req)
        return res is not None and res.success

    def randomCube(self):
        r = math.sqrt(random.uniform(self.rMin ** 2, self.rMax ** 2))
        a = random.uniform(-math.pi, math.pi)
        return (round(r * math.cos(a), 3), round(r * math.sin(a), 3),
                round(random.uniform(self.zMin, self.zMax), 3))

    def debugScene(self):
        if not self.getSceneCli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('[debug] /get_planning_scene servisi yok.')
            return
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                                     | PlanningSceneComponents.WORLD_OBJECT_NAMES)
        res = self.callService(self.getSceneCli, req)
        if res is None:
            self.get_logger().warn('[debug] /get_planning_scene yanıt vermedi.')
            return
        ids = [o.id for o in res.scene.world.collision_objects]
        self.get_logger().info(f'[debug] Sahnedeki nesneler: {ids}')
        acm = res.scene.allowed_collision_matrix
        names = list(acm.entry_names)
        if CUBE_ID not in names:
            self.get_logger().info(
                f"[debug] ACM'de '{CUBE_ID}' girdisi yok (çarpışma normal kontrol edilir).")
            return
        row = acm.entry_values[names.index(CUBE_ID)].enabled
        allowed = [n for n, e in zip(names, row) if e]
        self.get_logger().warn(
            f"[debug] ACM'de küple çarpışmasına İZİN verilen linkler: {allowed}")

    def computeIk(self, xyz, quat, seed):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group
        r.ik_link_name = self.ee
        r.pose_stamped = PoseStamped()
        r.pose_stamped.header.frame_id = self.frame
        r.pose_stamped.pose = self.buildPose(xyz, quat)
        r.robot_state = self.buildRobotState(seed)
        r.avoid_collisions = True          # DÜZELTME: kendi kendine / küple çarpışan çözümleri ele
        r.timeout = Duration(sec=0, nanosec=100_000_000)
        res = self.callService(self.ikCli, req)
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        if not all(j in sol for j in JOINTS):
            return None
        return {j: sol[j] for j in JOINTS}

    def computeCartesian(self, start, xyz, quat):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame
        req.start_state = self.buildRobotState(start)
        req.group_name = self.group
        req.link_name = self.ee
        req.waypoints = [self.buildPose(xyz, quat)]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True        # DÜZELTME
        return self.callService(self.cartCli, req, timeout=10.0)

    def checkValidity(self, q):
        req = GetStateValidity.Request()
        req.group_name = ''
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = JOINTS + WHEELS
        req.robot_state.joint_state.position = [float(x) for x in q] + [0.0] * len(WHEELS)
        res = self.callService(self.validCli, req)
        if res is None:
            raise RuntimeError('check_state_validity yanıt vermedi')
        pairs = sorted({f'{c.contact_body_1} <-> {c.contact_body_2}' for c in res.contacts})
        if not res.valid and not pairs:
            pairs = ['(geçersiz durum, ama temas bilgisi dönmedi)']
        return res.valid, pairs

    def checkPath(self, qs):
        n = len(qs)
        if n == 0:
            return True, 1.0, []
        m = self.checkPoints
        if m <= 1 or n <= m:
            idx = list(range(n))
        else:
            idx = sorted({round(i * (n - 1) / (m - 1)) for i in range(m)} | {0, n - 1})
        for i in idx:
            ok, pairs = self.checkValidity(qs[i])
            if not ok:
                return False, i / max(n - 1, 1), pairs
        return True, 1.0, []

    def planJointSpace(self, startQ, goalQ):
        """OMPL ile çarpışmasız eklem-uzayı yolu. Başarısızsa None."""
        if not self.planCli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetMotionPlan.Request()
        mr = req.motion_plan_request
        mr.group_name = self.group
        mr.start_state = self.buildRobotState(dict(zip(JOINTS, startQ)))
        goal = Constraints()
        for j, v in zip(JOINTS, goalQ):
            goal.joint_constraints.append(JointConstraint(
                joint_name=j, position=float(v),
                tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0))
        mr.goal_constraints = [goal]
        mr.num_planning_attempts = 5
        mr.allowed_planning_time = self.planTime
        mr.max_velocity_scaling_factor = 1.0
        mr.max_acceleration_scaling_factor = 1.0
        res = self.callService(self.planCli, req, timeout=self.planTime + 5.0)
        if res is None or res.motion_plan_response.error_code.val != 1:
            return None
        jt = res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            return None
        idx = [list(jt.joint_names).index(j) for j in JOINTS]
        raw = [[pt.positions[i] for i in idx] for pt in jt.points]
        return densify(raw)

    def connect(self, startQ, goalQ):
        """startQ -> goalQ çarpışmasız yol: önce düz eklem enterpolasyonu, olmazsa OMPL."""
        straight = lerpPath(startQ, goalQ)
        ok, _, _ = self.checkPath(straight)
        if ok:
            return straight
        if not self.useOmpl:
            return None
        path = self.planJointSpace(startQ, goalQ)
        if path is None:
            return None
        ok, _, _ = self.checkPath(path)
        return path if ok else None

    def getTargets(self):
        cx, cy, _ = self.cube
        if self.approach == 'front':
            norm = max(math.hypot(cx, cy), 1e-6)
            direction = (cx / norm, cy / norm, 0.0)
        else:
            direction = (0.0, 0.0, -1.0)
        gap = self.size / 2.0 + self.stop
        final = tuple(c - di * gap for c, di in zip(self.cube, direction))
        pre = tuple(f - di * self.appLen for f, di in zip(final, direction))
        return pre, final

    def planTrajectory(self):
        """Çarpışmasız ilk adayı döndürür. Çarpışmalı adaylar elenir."""
        pre, final = self.getTargets()
        homeDict = dict(zip(JOINTS, HOME_Q))
        topQuat = (1.0, 0.0, 0.0, 0.0)
        # DÜZELTME: 'top' için de tek yönelimde kalma, diğer adaylar yedek olsun
        candsToTry = [topQuat] + self.cands if self.approach == 'top' else self.cands

        startQ = self.getJointList()
        rejected = []          # (aday no, sebep, çarpışan çiftler)

        for k, quat in enumerate(candsToTry):
            sol = self.computeIk(pre, quat, homeDict)
            if sol is None:
                continue
            qPre = [sol[j] for j in JOINTS]

            res = self.computeCartesian(sol, final, quat)
            if (res is None or res.fraction < 0.99
                    or not res.solution.joint_trajectory.points):
                continue

            jt = res.solution.joint_trajectory
            idx = [list(jt.joint_names).index(j) for j in JOINTS]
            path = [[pt.positions[i] for i in idx] for pt in jt.points]

            ok, pairs = self.checkValidity(qPre)
            if not ok:
                rejected.append((k, 'hazır poz', pairs))
                continue

            ok, frac, pairs = self.checkPath(path)
            if not ok:
                rejected.append((k, f'Cartesian yaklaşma yolu (%{frac * 100:.0f})', pairs))
                continue

            toPre = self.connect(startQ, qPre)
            if toPre is None:
                rejected.append((k, 'hazır poza giderken çarpışmasız yol yok', []))
                continue

            return {'found': True, 'qPre': qPre, 'toPre': toPre, 'path': path,
                    'rejected': rejected}

        return {'found': False, 'rejected': rejected}

    def reportCollision(self, n, where, pairs):
        x, y, z = self.cube
        self.get_logger().error(
            f'#{n} ÇARPIŞMA ({where}): küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        for pr in pairs:
            self.get_logger().error(f'      {pr}')

    def returnHome(self, n, retPath, retPre):
        # retPath ve retPre, gidişte doğrulanmış yolların tersidir -> çarpışmasız
        self.playDuration(retPath, self.moveDur)
        self.playDuration(retPre, self.moveDur)
        self.get_logger().info(f'#{n} Home pozisyonuna dönüldü.')

    def runCycle(self, n):
        self.cube = self.randomCube() if self.random else self.fixedCube
        x, y, z = self.cube
        if not self.addCube(self.cube):
            self.get_logger().error(f'#{n} Küp sahneye eklenemedi.')
            return False
        self.spinFor(0.8)
        if self.debug:
            self.debugScene()

        ok, pairs = self.checkValidity(self.getJointList())
        if not ok:
            self.reportCollision(n, 'küp başlangıç konumunda', pairs)
            self.spinFor(self.hold)
            self.spinFor(self.pause)
            return False

        res = self.planTrajectory()
        if not res['found']:
            if res['rejected']:
                for k, where, pairs in res['rejected']:
                    self.reportCollision(n, f'aday {k}: {where}', pairs)
                self.get_logger().warn(
                    f'#{n} Çarpışmasız çözüm yok, hareket oynatılmıyor: '
                    f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
            else:
                self.get_logger().warn(
                    f'#{n} Erişilemez (IK çözümü yok / yol bulunamadı): '
                    f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
            self.spinFor(self.hold)
            self.spinFor(self.pause)
            return False

        if res['rejected']:
            self.get_logger().info(
                f'#{n} {len(res["rejected"])} çarpışmalı aday elendi.')
        self.get_logger().info(
            f'#{n} Çarpışma yok, gidiliyor: '
            f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')

        self.playDuration(res['toPre'], self.moveDur)
        self.spinFor(0.3)

        self.playDuration(res['path'], self.moveDur)
        self.get_logger().info(f'#{n} Hedefe varıldı.')

        self.spinFor(self.hold)
        self.returnHome(n, list(reversed(res['path'])), list(reversed(res['toPre'])))
        self.spinFor(self.pause)
        return True

    def runLoop(self):
        for cli, name in ((self.ikCli, '/compute_ik'),
                          (self.cartCli, '/compute_cartesian_path'),
                          (self.sceneCli, '/apply_planning_scene'),
                          (self.validCli, '/check_state_validity')):
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f'{name} yok. demo.launch.py çalışıyor mu?')
                return
        self.spinFor(1.0)

        n = 0
        while rclpy.ok():
            n += 1
            self.runCycle(n)
            if not self.random:
                break
        self.get_logger().info('Bitti. Çıkmak için Ctrl+C.')


def main():
    rclpy.init()
    node = CubeApproach()
    try:
        node.runLoop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()