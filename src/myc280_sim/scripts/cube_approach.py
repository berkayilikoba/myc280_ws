#!/usr/bin/env python3
"""import itertools
#gemini orijinal
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


def lerpPath(startQ, endQ, steps=20):
    return [[x + (i / steps) * (y - x) for x, y in zip(startQ, endQ)]
            for i in range(1, steps + 1)]


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

    def computeIk(self, xyz, quat, seed):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group
        r.ik_link_name = self.ee
        r.pose_stamped = PoseStamped()
        r.pose_stamped.header.frame_id = self.frame
        r.pose_stamped.pose = self.buildPose(xyz, quat)
        r.robot_state = self.buildRobotState(seed)
        r.avoid_collisions = False
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
        req.avoid_collisions = False
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
        return res.valid, pairs

    def checkPath(self, qs, maxPts=None):
        n = len(qs)
        if n == 0:
            return True, 1.0, []
        idx = sorted({round(i * (n - 1) / (maxPts - 1)) for i in range(maxPts)}) \
            if (maxPts and n > maxPts) else range(n)
        for i in idx:
            ok, pairs = self.checkValidity(qs[i])
            if not ok:
                return False, i / max(n - 1, 1), pairs
        return True, 1.0, []

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
        pre, final = self.getTargets()
        homeDict = dict(zip(JOINTS, HOME_Q))
        topQuat = (1.0, 0.0, 0.0, 0.0)
        candsToTry = [topQuat] if self.approach == 'top' else self.cands

        for quat in candsToTry:
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

            collisions = []
            ok, pairs = self.checkValidity(qPre)
            if not ok:
                collisions.append(('hazır poz', pairs))

            ok, frac, pairs = self.checkPath(path)
            if not ok:
                collisions.append((f'Cartesian yaklaşma yolu (%{frac * 100:.0f})', pairs))

            ok, frac, pairs = self.checkPath(lerpPath(self.getJointList(), qPre))
            if not ok:
                collisions.append((f'hazır poza giderken (%{frac * 100:.0f})', pairs))

            return {'found': True,
                    'hasCollision': len(collisions) > 0,
                    'collisions': collisions,
                    'qPre': qPre,
                    'path': path}

        return {'found': False}

    def reportCollision(self, n, where, pairs):
        x, y, z = self.cube
        self.get_logger().error(
            f'#{n} ÇARPIŞMA ({where}): küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        for pr in pairs:
            self.get_logger().error(f'      {pr}')

    def returnHome(self, n, retPath):
        retOk, frac, pairs = self.checkPath(retPath)
        if not retOk:
            self.get_logger().warn(
                f'#{n} Geri çekilme yolunda çarpışma (%{frac * 100:.0f}): {pairs}')
        self.playDuration(retPath, self.moveDur)

        toHome = lerpPath(self.getJointList(), HOME_Q)
        homeOk, _, pairs = self.checkPath(toHome)
        homeValid, homePairs = self.checkValidity(HOME_Q)
        if not homeOk:
            self.get_logger().warn(f'#{n} Home yolunda çarpışma: {pairs}')
        if not homeValid:
            self.get_logger().warn(f'#{n} Home pozunda çarpışma: {homePairs}')
        self.playDuration(toHome, self.moveDur)
        self.get_logger().info(f'#{n} Home pozisyonuna dönüldü.')

    def runCycle(self, n):
        self.cube = self.randomCube() if self.random else self.fixedCube
        x, y, z = self.cube
        if not self.addCube(self.cube):
            self.get_logger().error(f'#{n} Küp sahneye eklenemedi.')
            return False
        self.spinFor(0.8)

        ok, pairs = self.checkValidity(self.getJointList())
        if not ok:
            self.reportCollision(n, 'küp başlangıç konumunda', pairs)
            self.spinFor(self.hold)
            self.spinFor(self.pause)
            return False

        res = self.planTrajectory()
        if not res['found']:
            self.get_logger().warn(
                f'#{n} Erişilemez (IK çözümü yok / yol bulunamadı): '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
            self.spinFor(self.hold)
            self.spinFor(self.pause)
            return False

        if res['hasCollision']:
            for where, pairs in res['collisions']:
                self.reportCollision(n, where, pairs)
            self.get_logger().warn(
                f'#{n} ÇARPIŞMA var ama hareket yine de gösteriliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        else:
            self.get_logger().info(
                f'#{n} Çarpışma yok, gidiliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')

        self.playDuration([self.getJointList(), res['qPre']], self.moveDur)
        self.spinFor(0.3)

        self.playDuration(res['path'], self.moveDur)
        self.get_logger().info(f'#{n} Hedefe varıldı.')

        self.spinFor(self.hold)
        self.returnHome(n, list(reversed(res['path'])))
        self.spinFor(self.pause)
        return not res['hasCollision']

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

#!/usr/bin/env python3
#merged claude 
"""
"""import itertools
import math
import random
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (CollisionObject, ObjectColor, PlanningScene,
                             PlanningSceneComponents, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetPlanningScene,
                             GetPositionIK, GetStateValidity)
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
        r.avoid_collisions = False
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
        req.avoid_collisions = False
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
        pre, final = self.getTargets()
        homeDict = dict(zip(JOINTS, HOME_Q))
        topQuat = (1.0, 0.0, 0.0, 0.0)
        candsToTry = [topQuat] if self.approach == 'top' else self.cands

        for quat in candsToTry:
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

            # Oynatılacak yolun kendisi: kontrol edilen = oynatılan
            toPre = lerpPath(self.getJointList(), qPre)

            collisions = []
            ok, pairs = self.checkValidity(qPre)
            if not ok:
                collisions.append(('hazır poz', pairs))

            ok, pairs = self.checkValidity(path[-1])
            if not ok:
                collisions.append(('HEDEF poz (yolun son noktası)', pairs))

            ok, frac, pairs = self.checkPath(path)
            if not ok:
                collisions.append((f'Cartesian yaklaşma yolu (%{frac * 100:.0f})', pairs))

            ok, frac, pairs = self.checkPath(toPre)
            if not ok:
                collisions.append((f'hazır poza giderken (%{frac * 100:.0f})', pairs))

            return {'found': True,
                    'hasCollision': len(collisions) > 0,
                    'collisions': collisions,
                    'qPre': qPre,
                    'toPre': toPre,
                    'path': path}

        return {'found': False}

    def reportCollision(self, n, where, pairs):
        x, y, z = self.cube
        self.get_logger().error(
            f'#{n} ÇARPIŞMA ({where}): küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        for pr in pairs:
            self.get_logger().error(f'      {pr}')

    def returnHome(self, n, retPath):
        retOk, frac, pairs = self.checkPath(retPath)
        if not retOk:
            self.get_logger().warn(
                f'#{n} Geri çekilme yolunda çarpışma (%{frac * 100:.0f}): {pairs}')
        self.playDuration(retPath, self.moveDur)

        toHome = lerpPath(self.getJointList(), HOME_Q)
        homeOk, frac, pairs = self.checkPath(toHome)
        homeValid, homePairs = self.checkValidity(HOME_Q)
        if not homeOk:
            self.get_logger().warn(
                f'#{n} Home yolunda çarpışma (%{frac * 100:.0f}): {pairs}')
        if not homeValid:
            self.get_logger().warn(f'#{n} Home pozunda çarpışma: {homePairs}')
        self.playDuration(toHome, self.moveDur)
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
            self.get_logger().warn(
                f'#{n} Erişilemez (IK çözümü yok / yol bulunamadı): '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
            self.spinFor(self.hold)
            self.spinFor(self.pause)
            return False

        if res['hasCollision']:
            for where, pairs in res['collisions']:
                self.reportCollision(n, where, pairs)
            self.get_logger().warn(
                f'#{n} ÇARPIŞMA var ama hareket yine de gösteriliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        else:
            self.get_logger().info(
                f'#{n} Çarpışma yok, gidiliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')

        self.playDuration(res['toPre'], self.moveDur)
        self.spinFor(0.3)

        self.playDuration(res['path'], self.moveDur)
        self.get_logger().info(f'#{n} Hedefe varıldı.')

        self.spinFor(self.hold)
        self.returnHome(n, list(reversed(res['path'])))
        self.spinFor(self.pause)
        return not res['hasCollision']

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

"""

#claude 

"""#!/usr/bin/env python3
import itertools
import math
import random
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (CollisionObject, ObjectColor, PlanningScene,
                             PlanningSceneComponents, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetPlanningScene,
                             GetPositionIK, GetStateValidity)
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

    base = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = [[0, 0, 0] for _ in range(3)]
            for i, (p, s) in enumerate(zip(perm, signs)):
                m[i][p] = s
            if det3(m) == 1:
                base.append(mat_to_quat(m))
    return [qmul(qaxis_y(t), q) for t in (0, 30, -30) for q in base]


def lerp_path(a, b, n=40):
    return [[x + (i / n) * (y - x) for x, y in zip(a, b)] for i in range(0, n + 1)]


# ---------------- düğüm ----------------
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
                    ('check_points', 100),        # yol başına kontrol edilecek en fazla nokta
                    ('debug', False),             # ACM / sahne teşhisi
                    ('hold_time', 5.0),           # hedefte bekleme süresi (sn)
                    ('pause_time', 3.0),          # turlar arası bekleme süresi (sn)
                    ('move_duration', 5.0)]       # her hareket segmenti süresi (sn)
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
        self.check_points = max(int(p('check_points').value), 2)
        self.debug = bool(p('debug').value)
        self.hold = float(p('hold_time').value)
        self.pause = float(p('pause_time').value)
        self.move_dur = float(p('move_duration').value)
        self.cands = make_candidates()[:int(p('num_candidates').value)]
        self.cube = self.fixed_cube

        self.current = {j: 0.0 for j in JOINTS}
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / 30.0, self.publish_js)

        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.valid_cli = self.create_client(GetStateValidity, '/check_state_validity')
        self.getscene_cli = self.create_client(GetPlanningScene, '/get_planning_scene')

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

    def play_duration(self, qs, duration):
        if not qs or len(qs) < 2:
            return
        # Kümülatif eklem uzayı mesafesi üzerinden orantılı zamanlama
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
        co.operation = CollisionObject.ADD

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

    def debug_scene(self):
        
        if not self.getscene_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('[debug] /get_planning_scene servisi yok.')
            return
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                                     | PlanningSceneComponents.WORLD_OBJECT_NAMES)
        res = self.call(self.getscene_cli, req)
        if res is None:
            self.get_logger().warn('[debug] /get_planning_scene yanıt vermedi.')
            return
        ids = [o.id for o in res.scene.world.collision_objects]
        self.get_logger().info(f'[debug] Sahnedeki nesneler: {ids}')
        acm = res.scene.allowed_collision_matrix
        names = list(acm.entry_names)
        if CUBE_ID not in names:
            self.get_logger().info(f"[debug] ACM'de '{CUBE_ID}' girdisi yok (varsayılan: çarpışma kontrol edilir).")
            return
        row = acm.entry_values[names.index(CUBE_ID)].enabled
        allowed = [n for n, e in zip(names, row) if e]
        self.get_logger().warn(f"[debug] ACM'de küple çarpışmasına İZİN verilen linkler: {allowed}")

    # ---- IK / Cartesian / çarpışma kontrolü ----
    def ik(self, xyz, quat, seed):
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
        req = GetStateValidity.Request()
        req.group_name = ''   # boş = tüm robot linkleri kontrol edilir
        req.robot_state.joint_state.name = JOINTS + WHEELS
        req.robot_state.joint_state.position = [float(x) for x in q] + [0.0] * len(WHEELS)
        res = self.call(self.valid_cli, req)
        if res is None:
            raise RuntimeError('check_state_validity yanıt vermedi')
        pairs = sorted({f'{c.contact_body_1} <-> {c.contact_body_2}' for c in res.contacts})
        if not res.valid and not pairs:
            pairs = ['(geçersiz durum, ama temas bilgisi dönmedi)']
        return res.valid, pairs

    def path_check(self, qs):
        
   
        n = len(qs)
        if n == 0:
            return True, 1.0, []
        if n <= self.check_points:
            idx = list(range(n))
        else:
            m = self.check_points
            idx = sorted({round(i * (n - 1) / (m - 1)) for i in range(m)} | {0, n - 1})
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
            d = (0.0, 0.0, -1.0)
        gap = self.size / 2.0 + self.stop
        final = tuple(c - di * gap for c, di in zip(self.cube, d))
        pre = tuple(f - di * self.app_len for f, di in zip(final, d))
        return pre, final

    def plan(self):

        pre, final = self.targets()
        home_dict = dict(zip(JOINTS, HOME_Q))

        top_quat = (1.0, 0.0, 0.0, 0.0)
        cands_to_try = [top_quat] if self.approach == 'top' else self.cands

        for quat in cands_to_try:
            sol = self.ik(pre, quat, home_dict)
            if sol is None:
                continue

            q_pre = [sol[j] for j in JOINTS]

            res = self.cartesian(sol, final, quat)
            if (res is None or res.fraction < 0.99
                    or not res.solution.joint_trajectory.points):
                continue

            jt = res.solution.joint_trajectory
            idx = [list(jt.joint_names).index(j) for j in JOINTS]
            path = [[pt.positions[i] for i in idx] for pt in jt.points]

            # Oynatılacak yolların kendisi (kontrol edilen = oynatılan)
            to_pre = lerp_path(self.q_list(), q_pre)

            collisions = []

            ok, pairs = self.check(q_pre)
            if not ok:
                collisions.append(('hazır poz', pairs))

            ok, pairs = self.check(path[-1])
            if not ok:
                collisions.append(('HEDEF poz (yolun son noktası)', pairs))

            ok, frac, pairs = self.path_check(path)
            if not ok:
                collisions.append((f'Cartesian yaklaşma yolu (%{frac * 100:.0f})', pairs))

            ok, frac, pairs = self.path_check(to_pre)
            if not ok:
                collisions.append((f'hazır poza giderken (%{frac * 100:.0f})', pairs))

            return {'found': True,
                    'has_collision': len(collisions) > 0,
                    'collisions': collisions,
                    'q_pre': q_pre,
                    'to_pre': to_pre,
                    'path': path}

        return {'found': False}

    def report_collision(self, n, where, pairs):
        x, y, z = self.cube
        self.get_logger().error(
            f'#{n} ÇARPIŞMA ({where}): küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        for pr in pairs:
            self.get_logger().error(f'      {pr}')

    def return_home(self, n, ret_path):
        ret_ok, frac, pairs = self.path_check(ret_path)
        if not ret_ok:
            self.get_logger().warn(
                f'#{n} Geri çekilme yolunda çarpışma (%{frac * 100:.0f}): {pairs}')
        self.play_duration(ret_path, self.move_dur)

        to_home = lerp_path(self.q_list(), HOME_Q)
        home_ok, frac, pairs = self.path_check(to_home)
        if not home_ok:
            self.get_logger().warn(
                f'#{n} Home yolunda çarpışma (%{frac * 100:.0f}): {pairs}')
        self.play_duration(to_home, self.move_dur)
        self.get_logger().info(f'#{n} Home pozisyonuna dönüldü.')

    # ---- bir tur ----
    def cycle(self, n):
        self.cube = self.random_cube() if self.random else self.fixed_cube
        x, y, z = self.cube
        if not self.add_cube(self.cube):
            self.get_logger().error(f'#{n} Küp sahneye eklenemedi.')
            return False
        self.spin_for(0.5)                          # RViz küpü göstersin
        if self.debug:
            self.debug_scene()

        # Küp robotun/gövdenin içine mi düştü?
        ok, pairs = self.check(self.q_list())
        if not ok:
            self.report_collision(n, 'küp başlangıç konumunda', pairs)
            self.spin_for(self.hold)
            self.spin_for(self.pause)
            return False

        res = self.plan()
        if not res['found']:
            self.get_logger().warn(
                f'#{n} Erişilemez (IK çözümü yok / yol bulunamadı): '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
            self.spin_for(self.hold)
            self.spin_for(self.pause)
            return False

        if res['has_collision']:
            for where, pairs in res['collisions']:
                self.report_collision(n, where, pairs)
            self.get_logger().warn(
                f'#{n} ÇARPIŞMA var ama hareket yine de gösteriliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        else:
            self.get_logger().info(
                f'#{n} Çarpışma yok, gidiliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')

        # ---- Hareketi her zaman göster ----
        # Home -> hazır poz (kontrol edilen yolun aynısı)
        self.play_duration(res['to_pre'], self.move_dur)
        self.spin_for(0.3)

        # Hazır poz -> hedef (Cartesian path)
        self.play_duration(res['path'], self.move_dur)
        self.get_logger().info(f'#{n} Hedefe varıldı.')

        # Hedefte bekle
        self.spin_for(self.hold)

        # Geri çekil ve home'a dön
        self.return_home(n, list(reversed(res['path'])))

        # Turlar arası bekleme
        self.spin_for(self.pause)
        return not res['has_collision']

    def run(self):
        for cli, name in ((self.ik_cli, '/compute_ik'),
                          (self.cart_cli, '/compute_cartesian_path'),
                          (self.scene_cli, '/apply_planning_scene'),
                          (self.valid_cli, '/check_state_validity')):
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f'{name} yok. demo.launch.py çalışıyor mu?')
                return
        self.spin_for(1.0)

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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()"""
""""""

#!/usr/bin/env python3
"""Küpü RViz'de rastgele bir noktaya at -> çarpışma testi -> çarpışma varsa hata yaz,
yoksa Cartesian path ile küpe ilerle, home pozisyonuna dön ve sonraki küpe geç.

Kontrol edilen yollar, oynatılan yollarla birebir aynıdır.
Teşhis için: -p debug:=true  (sahnedeki nesneleri ve ACM'de küple ilgili izinleri yazar)
Tüm yol noktalarını kontrol etmek için check_points:=0 (varsayılan).
"""
#!/usr/bin/env python3
"""Küpü RViz'de rastgele bir noktaya at -> çarpışma testi -> çarpışma varsa hata yaz,
yoksa Cartesian path ile küpe ilerle, home pozisyonuna dön ve sonraki küpe geç.

Kontrol edilen yollar, oynatılan yollarla birebir aynıdır.
Teşhis için: -p debug:=true  (sahnedeki nesneleri ve ACM'de küple ilgili izinleri yazar)
Tüm yol noktalarını kontrol etmek için check_points:=0 (varsayılan).
"""
#!/usr/bin/env python3
"""Küpü RViz'de rastgele bir noktaya at -> çarpışma testi -> çarpışma varsa hata yaz,
yoksa Cartesian path ile küpe ilerle, home pozisyonuna dön ve sonraki küpe geç.

Kontrol edilen yollar, oynatılan yollarla birebir aynıdır.
Teşhis için: -p debug:=true  (sahnedeki nesneleri ve ACM'de küple ilgili izinleri yazar)
Tüm yol noktalarını kontrol etmek için check_points:=0 (varsayılan).
"""
"""import itertools
import math
import random
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, ObjectColor,
                             PlanningScene, PlanningSceneComponents, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetPlanningScene,
                             GetPositionIK, GetStateValidity)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
WHEELS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
HOME_Q = [0.0] * len(JOINTS)
CUBE_ID = 'cube'
GRIPPER_ID = 'gripper'


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


def quatAxisX(deg):
    halfRad = math.radians(deg) / 2.0
    return (math.sin(halfRad), 0.0, 0.0, math.cos(halfRad))


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
                    ('gripper', True),            # gripper'ı sahneye ekle (RViz + çarpışma)
                    ('gripper_w', 0.06),          # flanş x ekseni boyunca genişlik (m)
                    ('gripper_d', 0.04),          # flanş y ekseni boyunca derinlik (m)
                    ('gripper_len', 0.08),        # flanş z ekseni boyunca uzunluk (m)
                    ('gripper_touch_links', ['joint6_flange']),  # çarpışması yok sayılan linkler
                    ('ik_timeout', 0.5),          # IK çözüm süresi (sn)
                    ('ik_seeds', 6),              # home dışında denenecek rastgele başlangıç sayısı
                    ('top_tilt', True),           # 'top' modunda eğimli yönelimleri de dene
                    ('check_points', 0),      # 0 = yolun tüm noktaları kontrol edilir
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
        self.useGripper = bool(p('gripper').value)
        self.gripDims = (float(p('gripper_w').value), float(p('gripper_d').value),
                         float(p('gripper_len').value))
        self.gripTouch = list(p('gripper_touch_links').value)
        self.ikTimeout = float(p('ik_timeout').value)
        self.ikSeeds = int(p('ik_seeds').value)
        self.topTilt = bool(p('top_tilt').value)
        self.checkPoints = int(p('check_points').value)
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

    def attachGripper(self):
    
        w, d, ln = self.gripDims
        co = CollisionObject()
        co.header.frame_id = self.ee
        co.id = GRIPPER_ID
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [w, d, ln]
        co.primitives = [box]
        co.primitive_poses = [self.buildPose((0.0, 0.0, ln / 2.0), (0.0, 0.0, 0.0, 1.0))]
        co.operation = CollisionObject.ADD

        aco = AttachedCollisionObject()
        aco.link_name = self.ee
        aco.object = co
        aco.touch_links = self.gripTouch   # bu linklerle çarpışma sayılmaz (flanşa bağlı)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        scene.object_colors = [ObjectColor(id=GRIPPER_ID,
                                           color=ColorRGBA(r=0.1, g=0.6, b=1.0, a=1.0))]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = self.callService(self.sceneCli, req)
        ok = res is not None and res.success
        if ok:
            self.get_logger().info(
                f'Gripper eklendi: {w:.3f} x {d:.3f} x {ln:.3f} m, '
                f'{self.ee} çerçevesinde +z yönünde.')
        else:
            self.get_logger().error('Gripper sahneye eklenemedi.')
        return ok

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
        r.avoid_collisions = False
        r.timeout = Duration(sec=int(self.ikTimeout),
                             nanosec=int((self.ikTimeout % 1.0) * 1e9))
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
        req.avoid_collisions = False
        return self.callService(self.cartCli, req, timeout=10.0)

    def checkValidity(self, q):
        req = GetStateValidity.Request()
        req.group_name = ''
        # is_diff=True ŞART: aksi halde sahnedeki attached (gripper) cisim durumdan silinir
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

    def getTargets(self):
        cx, cy, _ = self.cube
        if self.approach == 'front':
            norm = max(math.hypot(cx, cy), 1e-6)
            direction = (cx / norm, cy / norm, 0.0)
        else:
            direction = (0.0, 0.0, -1.0)
        gap = self.size / 2.0 + self.stop
        if self.useGripper:
            gap += self.gripDims[2]   # hedef flanşa göre; gripper ucu stop mesafesinde kalsın
        final = tuple(c - di * gap for c, di in zip(self.cube, direction))
        pre = tuple(f - di * self.appLen for f, di in zip(final, direction))
        return pre, final

    def planTrajectory(self):
        pre, final = self.getTargets()
        homeDict = dict(zip(JOINTS, HOME_Q))
        topQuat = (1.0, 0.0, 0.0, 0.0)
        if self.approach == 'top':
            candsToTry = [topQuat]
            if self.topTilt:
                for a in (20, -20, 40, -40):
                    candsToTry.append(multiplyQuat(quatAxisX(a), topQuat))
                    candsToTry.append(multiplyQuat(quatAxisY(a), topQuat))
        else:
            candsToTry = self.cands
        seeds = [homeDict] + [{j: random.uniform(-2.0, 2.0) for j in JOINTS}
                              for _ in range(self.ikSeeds)]
        ikFail = cartFail = 0
        bestFrac = 0.0

        for quat in candsToTry:
            sol = None
            for seed in seeds:
                sol = self.computeIk(pre, quat, seed)
                if sol is not None:
                    break
            if sol is None:
                ikFail += 1
                continue

            qPre = [sol[j] for j in JOINTS]
            res = self.computeCartesian(sol, final, quat)
            if res is None or not res.solution.joint_trajectory.points:
                cartFail += 1
                continue
            bestFrac = max(bestFrac, res.fraction)
            if res.fraction < 0.99:
                cartFail += 1
                continue

            jt = res.solution.joint_trajectory
            idx = [list(jt.joint_names).index(j) for j in JOINTS]
            path = [[pt.positions[i] for i in idx] for pt in jt.points]

            # Oynatılacak yolun kendisi: kontrol edilen = oynatılan
            toPre = lerpPath(self.getJointList(), qPre)

            collisions = []
            ok, pairs = self.checkValidity(qPre)
            if not ok:
                collisions.append(('hazır poz', pairs))

            ok, pairs = self.checkValidity(path[-1])
            if not ok:
                collisions.append(('HEDEF poz (yolun son noktası)', pairs))

            ok, frac, pairs = self.checkPath(path)
            if not ok:
                collisions.append((f'Cartesian yaklaşma yolu (%{frac * 100:.0f})', pairs))

            ok, frac, pairs = self.checkPath(toPre)
            if not ok:
                collisions.append((f'hazır poza giderken (%{frac * 100:.0f})', pairs))

            return {'found': True,
                    'hasCollision': len(collisions) > 0,
                    'collisions': collisions,
                    'qPre': qPre,
                    'toPre': toPre,
                    'path': path}

        info = (f'{len(candsToTry)} yönelim denendi: IK çözümsüz {ikFail}, '
                f'Cartesian yol yetersiz {cartFail} (en iyi tamamlanma %{bestFrac * 100:.0f}); '
                f'hazır poz {tuple(round(v, 3) for v in pre)}, '
                f'hedef {tuple(round(v, 3) for v in final)}')
        return {'found': False, 'info': info}

    def reportCollision(self, n, where, pairs):
        x, y, z = self.cube
        self.get_logger().error(
            f'#{n} ÇARPIŞMA ({where}): küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        for pr in pairs:
            self.get_logger().error(f'      {pr}')

    def returnHome(self, n, retPath):
        retOk, frac, pairs = self.checkPath(retPath)
        if not retOk:
            self.get_logger().warn(
                f'#{n} Geri çekilme yolunda çarpışma (%{frac * 100:.0f}): {pairs}')
        self.playDuration(retPath, self.moveDur)

        toHome = lerpPath(self.getJointList(), HOME_Q)
        homeOk, frac, pairs = self.checkPath(toHome)
        homeValid, homePairs = self.checkValidity(HOME_Q)
        if not homeOk:
            self.get_logger().warn(
                f'#{n} Home yolunda çarpışma (%{frac * 100:.0f}): {pairs}')
        if not homeValid:
            self.get_logger().warn(f'#{n} Home pozunda çarpışma: {homePairs}')
        self.playDuration(toHome, self.moveDur)
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
            self.get_logger().warn(
                f'#{n} Erişilemez: küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]\n'
                f'      {res["info"]}')
            self.spinFor(self.hold)
            self.spinFor(self.pause)
            return False

        if res['hasCollision']:
            for where, pairs in res['collisions']:
                self.reportCollision(n, where, pairs)
            self.get_logger().warn(
                f'#{n} ÇARPIŞMA var ama hareket yine de gösteriliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')
        else:
            self.get_logger().info(
                f'#{n} Çarpışma yok, gidiliyor: '
                f'küp ({x:+.3f}, {y:+.3f}, {z:+.3f}) [{self.frame}]')

        self.playDuration(res['toPre'], self.moveDur)
        self.spinFor(0.3)

        self.playDuration(res['path'], self.moveDur)
        self.get_logger().info(f'#{n} Hedefe varıldı.')

        self.spinFor(self.hold)
        self.returnHome(n, list(reversed(res['path'])))
        self.spinFor(self.pause)
        return not res['hasCollision']

    def runLoop(self):
        for cli, name in ((self.ikCli, '/compute_ik'),
                          (self.cartCli, '/compute_cartesian_path'),
                          (self.sceneCli, '/apply_planning_scene'),
                          (self.validCli, '/check_state_validity')):
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f'{name} yok. demo.launch.py çalışıyor mu?')
                return
        self.spinFor(1.0)
        if self.useGripper:
            self.attachGripper()
            self.spinFor(0.5)

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
    main()"""