"""Kavrama yaklaşması yürütücüsü: nesne konumu (harita) -> kavrama_yaklasim -> Nav2 (kavrama öncesi poz Q) -> odomla
yerinde hizalanma + düz son yaklaşma (P) -> nesnenin base_footprint'teki konumunu kol tarafına yayınlar.

Konular:
  giriş  /kavrama/hedef            geometry_msgs/PointStamped (frame map; z = nesne yüksekliği, yerden)
  çıkış  /kavrama/durum            std_msgs/String  (PLAN, NAV, HIZALA, YAKLAS, HAZIR, HATA: ...)
         /kavrama/nesne_robotta    geometry_msgs/PointStamped (frame base_footprint) — kol bu noktayı kavrar
         /kavrama/plan             geometry_msgs/PoseArray [Q, P] (RViz)
Son yaklaşma Nav2'siz yapılır (collision_monitor nesneye 6 cm yaklaşmaya izin vermez): son_cmd_topic (varsayılan
cmd_vel; gerçek robotta da sürücü girişi) üzerinden, odom çerçevesinde hedef tutularak, 0.05 m/s.
İnce konumlama: Q'da ve varışta lidar taraması haritaya ICP ile oturtulur (AMCL simde ~8 cm / 4° yanılıyordu, 27.10;
ICP bu odada ~1 cm). Son yaklaşma hedefi ve kola bildirilen nesne konumu ICP pozundan hesaplanır.
Kullanım: ros2 run limo_autonomy kavrama --ros-args -p harita:=<harita.yaml> [-p d_kavrama:=0.22] [-p yon_derece:=..]
          ros2 topic pub --once /kavrama/hedef geometry_msgs/PointStamped "{header: {frame_id: map}, point: {x: 1, y: 1}}"
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from limo_autonomy.kavrama_yaklasim import Ayar, Harita, icp2d, yaklasim_bul


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def poz(x, y, th):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.z, p.orientation.w = math.sin(th / 2), math.cos(th / 2)
    return p


class KavramaDugumu(Node):

    def __init__(self):
        super().__init__('kavrama')
        p = self.declare_parameter
        self.harita = Harita.yaml_oku(p('harita', '').value)
        yon = p('yon_derece', float('nan')).value
        self.ayar = Ayar(d_kavrama=p('d_kavrama', 0.22).value, yanal=p('yanal', 0.0).value, d_on=p('d_on', 0.35).value,
                         pay=p('pay', 0.05).value, r_nesne=p('r_nesne', 0.12).value,
                         yon=None if math.isnan(yon) else math.radians(yon))
        self.v_son, self.w_max = p('son_hiz', 0.05).value, p('donus_hiz', 0.4).value
        self.tol_m, self.tol_aci = p('tol_m', 0.01).value, math.radians(p('tol_derece', 1.0).value)
        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)
        self.p_cmd = self.create_publisher(Twist, p('son_cmd_topic', 'cmd_vel').value, 10)
        self.p_durum = self.create_publisher(String, '/kavrama/durum', 10)
        self.p_nesne = self.create_publisher(PointStamped, '/kavrama/nesne_robotta', 10)
        self.p_plan = self.create_publisher(PoseArray, '/kavrama/plan', 10)
        self.odom = None
        self.tarama = None
        self.harita_nok = self.harita.dolu_noktalar()
        self.harita_agac = cKDTree(self.harita_nok)
        self.create_subscription(Odometry, 'odom', self._odom, 50, callback_group=self.cb)
        self.create_subscription(LaserScan, p('scan_topic', 'scan').value, lambda m: setattr(self, 'tarama', m),
                                 qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(PointStamped, '/kavrama/hedef', self._hedef, 10, callback_group=self.cb)
        self.mesgul = threading.Lock()
        self.get_logger().info(f'kavrama yürütücüsü hazır: {self.ayar}')

    def durum(self, s):
        self.get_logger().info(s)
        self.p_durum.publish(String(data=s))

    def _odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation))

    def icp_poz(self, k=6):
        """son k taramayı haritaya ICP ile oturt (başlangıç: TF map->base_footprint) -> (x, y, yaw, uyum, odom pozu)"""
        x0, y0, t0 = self.tf2d('map', 'base_footprint')
        sonuc, son = [], None
        bas = time.time()
        while len(sonuc) < k and time.time() - bas < 5:
            m = self.tarama
            if m is None or m is son:
                time.sleep(0.03)
                continue
            son, od = m, self.odom
            lx, ly, lt = self.tf2d('base_footprint', m.header.frame_id)
            r = np.array(m.ranges)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment + lt
            s = np.isfinite(r) & (r > 0.25) & (r < 8)
            P = np.c_[lx + r[s] * np.cos(a[s]), ly + r[s] * np.sin(a[s])]
            ct, st = math.cos(t0), math.sin(t0)
            T, u = icp2d(P, self.harita_agac, self.harita_nok, np.array([[ct, -st, x0], [st, ct, y0], [0, 0, 1.0]]))
            sonuc.append((T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0]), u, od))
        if not sonuc:
            raise RuntimeError('tarama yok')
        i = int(np.argmax([s[3] for s in sonuc]))                 # en iyi uyumlu tarama (robot dururken)
        return sonuc[i]

    def tf2d(self, hedef, kaynak):
        t = self.tf.lookup_transform(hedef, kaynak, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)).transform
        return t.translation.x, t.translation.y, yaw_q(t.rotation)

    def _hedef(self, m):
        if not self.mesgul.acquire(blocking=False):
            self.durum('HATA: önceki görev sürüyor')
            return
        try:
            self.calis(m)
        except Exception as e:                                  # görev hatası düğümü düşürmesin
            self.durum(f'HATA: {e}')
        finally:
            self.p_cmd.publish(Twist())
            self.mesgul.release()

    def calis(self, m):
        nx, ny, nz = m.point.x, m.point.y, m.point.z
        rx, ry, _ = self.tf2d('map', 'base_footprint')
        self.durum(f'PLAN nesne ({nx:+.2f}, {ny:+.2f}, z {nz:.2f}), robot ({rx:+.2f}, {ry:+.2f})')
        y = yaklasim_bul(self.harita, (nx, ny), self.ayar, robot=(rx, ry))
        if y is None:
            self.durum('HATA: geçerli yaklaşma yok (engel / erişim)')
            return
        (px, py, th), (qx, qy, _) = y.kavrama, y.oncesi
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = [poz(qx, qy, th), poz(px, py, th)]
        self.p_plan.publish(pa)
        self.durum(f'NAV Q ({qx:+.2f}, {qy:+.2f}, {math.degrees(th):+.0f}°) -> P ({px:+.2f}, {py:+.2f}), açıklık {y.aciklik:.2f}')
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('navigate_to_pose eylem sunucusu yok')
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose = poz(qx, qy, th)
        fut = self.nav.send_goal_async(g)
        while not fut.done():
            time.sleep(0.05)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError('Nav2 hedefi reddetti')
        rf = gh.get_result_async()
        while not rf.done():
            time.sleep(0.1)
        if rf.result().status != 4:                            # GoalStatus.STATUS_SUCCEEDED
            raise RuntimeError(f'Nav2 başarısız (durum {rf.result().status})')
        # son yaklaşma: ICP pozu (harita) + o andaki odom pozu -> P'yi odom çerçevesine çevir, odomla sür
        time.sleep(0.5)
        ix, iy, ith, iu, (dx, dy, dth) = self.icp_poz()
        ax, ay, ath = self.tf2d('map', 'base_footprint')
        self.durum(f'ICP Q: ({ix:+.3f}, {iy:+.3f}, {math.degrees(ith):+.1f}°) uyum {iu:.2f}; AMCL farkı '
                   f'{math.hypot(ax - ix, ay - iy) * 100:.1f} cm, {math.degrees(wrap(ath - ith)):+.1f}°')
        if iu < 0.6:
            raise RuntimeError(f'ICP uyumu düşük ({iu:.2f})')
        myaw = wrap(dth - ith)                                    # map -> odom (ICP'ye göre)
        c, s = math.cos(myaw), math.sin(myaw)
        mx, my = dx - (c * ix - s * iy), dy - (s * ix + c * iy)
        ox, oy = mx + c * px - s * py, my + s * px + c * py
        oth = wrap(th + myaw)
        # P'ye doğru dön, P'ye düz git, θ'ya dön: Nav2 Q'ya yanal sapmayla varsa da (AMCL hatası) P tam tutulur
        x, y, _ = self.odom
        yon = math.atan2(oy - y, ox - x)
        self.durum(f'HIZALA (P {math.hypot(ox - x, oy - y) * 100:.0f} cm, yön sapması {math.degrees(wrap(yon - oth)):+.0f}°)')
        self.don(yon)
        self.durum('YAKLAS')
        self.duz(ox, oy, yon)
        self.don(oth)
        # sonuç: nesnenin base_footprint'teki konumu (kol bunu kavrar), ICP pozundan
        time.sleep(0.5)
        bx, by, byaw, bu, _ = self.icp_poz()
        c, s = math.cos(-byaw), math.sin(-byaw)
        out = PointStamped()
        out.header.frame_id = 'base_footprint'
        out.header.stamp = self.get_clock().now().to_msg()
        out.point.x, out.point.y = c * (nx - bx) - s * (ny - by), s * (nx - bx) + c * (ny - by)
        out.point.z = nz
        self.p_nesne.publish(out)
        hp = math.hypot(bx - px, by - py)
        ha = math.degrees(wrap(byaw - th))
        self.durum(f'HAZIR nesne robotta ({out.point.x:+.3f}, {out.point.y:+.3f}, {nz:.2f}) m; '
                   f'P hatası {hp * 100:.1f} cm, {ha:+.1f}° (ICP, uyum {bu:.2f})')

    def don(self, hedef):
        t0 = time.time()
        while time.time() - t0 < 20:
            if self.odom is None:
                time.sleep(0.05)
                continue
            e = wrap(hedef - self.odom[2])
            if abs(e) < self.tol_aci:
                break
            cmd = Twist()
            cmd.angular.z = math.copysign(max(0.08, min(self.w_max, 1.2 * abs(e))), e)
            self.p_cmd.publish(cmd)
            time.sleep(0.02)
        self.p_cmd.publish(Twist())
        time.sleep(0.3)

    def duz(self, hx, hy, hth):
        t0 = time.time()
        while time.time() - t0 < 30:
            x, y, yaw = self.odom
            ileri = (hx - x) * math.cos(hth) + (hy - y) * math.sin(hth)
            yanal = -(hx - x) * math.sin(hth) + (hy - y) * math.cos(hth)
            if ileri < self.tol_m:
                break
            cmd = Twist()
            cmd.linear.x = max(0.02, min(self.v_son, 0.8 * ileri))
            cmd.angular.z = max(-0.3, min(0.3, 1.5 * wrap(hth - yaw) + 2.0 * yanal))
            self.p_cmd.publish(cmd)
            time.sleep(0.02)
        self.p_cmd.publish(Twist())
        time.sleep(0.3)


def main(args=None):
    rclpy.init(args=args)
    n = KavramaDugumu()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
