"""Simülasyon düğümü: Gazebo'nun kusursuz sensörlerini gerçek LIMO'nun ölçülmüş kusurlarıyla yeniden yayınlar.

Değerler gerçek kayıtlardan (saha/analiz/sim_param_cikar.py -> saha/config/sim_gercek_param.yaml, 27.10.2026, 5 kayıt):
  odom   /odom_gercek (planar_move, Gazebo gerçek konumu) -> /odom + TF odom->base_footprint. Gerçek tabanın odom'u
         dönüşü %0.46 fazla (k_w = Δyaw_gerçek/Δyaw_odom = 0.9954), mesafeyi %1.9 eksik (k_v = 1.0188) sayıyor.
         Bir kapsama turunda (31.5 rad dönüş) gerçekte -8..-10° yön kayması = bu ölçek hatası. Rastgele kısım küçük:
         ölçekten sonra tur sonu farkı ±2° / ~30 rad -> σ 0.006 rad/√rad (1 s pencerelerdeki 0.027'nin çoğu lidar
         pozunun kendi gürültüsü).
  imu    /imu_ham -> /imu: jiroskop 0.2°/s (0.00349 rad/s) adımlarla nicemlenir (gerçek taban kartı böyle veriyor);
         Gazebo gürültüsü (σ) URDF'te ayarlı.
  derinlik /camera/depth_sim/depth/image_raw (32FC1, m) -> /camera/depth/image_raw (16UC1, mm) + camera_info:
         σ(z) = a·z² (a = 0.00176 1/m, D435 848x480 sabit sahne ölçümü), geçerli aralık [z_min, z_max], derinlik
         süreksizliklerinde gölge şeridi (gerçek D435'te nesne kenarında geçersiz bant) ve rastgele boşluk.
odom_hatasi:=false ile /odom = gerçek konum (eski davranış; Nav2 ayar düzeneği için).
bolum: 'odom_imu' ya da 'derinlik' (iki ayrı süreç: 1.6 MB derinlik mesajını Python'da çözmek tek iş parçacıklı düğümde
odom/IMU'yu 15 Hz'e düşürüyordu, 27.10.2026 ölçümü) ya da 'hepsi'.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import TransformBroadcaster


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class OdomHatasi:
    """Gerçek konum artışlarını ölçülmüş ölçek hatası + gürültüyle tümleştirir (saf, ROS'suz; test edilebilir)."""

    def __init__(self, k_w=0.9954, k_v=1.0188, sigma_w=0.006, tohum=0):
        self.k_w, self.k_v, self.sigma_w = k_w, k_v, sigma_w
        self.rng = np.random.default_rng(tohum)
        self.onceki = None
        self.x = self.y = self.yaw = 0.0

    def adim(self, x, y, yaw):
        if self.onceki is None:
            self.onceki = (x, y, yaw)
            return self.x, self.y, self.yaw
        px, py, pyaw = self.onceki
        self.onceki = (x, y, yaw)
        dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
        # ileri / yanal artış, gerçek gövde çerçevesinde
        c, s = math.cos(pyaw), math.sin(pyaw)
        ileri, yanal = c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py)
        dyaw_o = dyaw / self.k_w + self.rng.normal(0.0, self.sigma_w * math.sqrt(abs(dyaw))) if dyaw else 0.0
        ileri, yanal = ileri / self.k_v, yanal / self.k_v
        orta = self.yaw + dyaw_o / 2
        self.x += math.cos(orta) * ileri - math.sin(orta) * yanal
        self.y += math.sin(orta) * ileri + math.cos(orta) * yanal
        self.yaw = math.atan2(math.sin(self.yaw + dyaw_o), math.cos(self.yaw + dyaw_o))
        return self.x, self.y, self.yaw


def derinlik_bozu(z, a=0.00176, z_min=0.28, z_max=10.0, golge_esik=0.08, bosluk=0.02, rng=None):
    """z: (H, W) float32 metre (NaN/inf = yok). Dönüş: uint16 mm, gerçek D435 benzeri gürültü ve boşluklarla."""
    rng = rng or np.random.default_rng()
    z = np.where(np.isfinite(z), z, 0.0).astype(np.float32)
    gec = (z >= z_min) & (z <= z_max)
    zn = z + rng.normal(0.0, 1.0, z.shape).astype(np.float32) * (a * z * z)
    # gölge: yatay komşuyla bağıl fark > golge_esik olan kenarın uzak tarafı (D435 sol IR - projektör paralaksı)
    if golge_esik > 0:
        fark = np.zeros_like(z)
        fark[:, 1:] = (z[:, 1:] - z[:, :-1]) / np.maximum(z[:, 1:], 1e-3)
        kenar = fark > golge_esik                                   # sağdaki piksel daha uzak
        golge = kenar.copy()
        for k in range(1, 4):                                       # ~3 piksellik bant
            golge[:, k:] |= kenar[:, :-k]
        gec &= ~golge
    if bosluk > 0:
        gec &= rng.random(z.shape) >= bosluk
    return np.where(gec, np.clip(np.round(zn * 1000.0), 0, 65535), 0).astype(np.uint16)


class SensorModeliDugumu(Node):

    def __init__(self):
        super().__init__('sensor_modeli')
        p = self.declare_parameter
        bolum = p('bolum', 'hepsi').value
        oi, dr = bolum in ('hepsi', 'odom_imu'), bolum in ('hepsi', 'derinlik')
        self.odom_hatasi = p('odom_hatasi', True).value
        self.oh = OdomHatasi(p('k_w', 0.9954).value, p('k_v', 1.0188).value, p('sigma_w', 0.006).value)
        self.nicem = p('jiroskop_nicem', 0.00349).value
        self.dp = dict(a=p('derinlik_a', 0.00176).value, z_min=p('derinlik_z_min', 0.28).value,
                       z_max=p('derinlik_z_max', 10.0).value, golge_esik=p('derinlik_golge_esik', 0.08).value,
                       bosluk=p('derinlik_bosluk', 0.02).value)
        self.rng = np.random.default_rng(1)
        if oi:
            self.tf = TransformBroadcaster(self)
            self.p_odom = self.create_publisher(Odometry, 'odom', 20)
            self.p_imu = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom_gercek', self._odom, 50)
            self.create_subscription(Imu, 'imu_ham', self._imu, qos_profile_sensor_data)
            self.get_logger().info(f'odom/IMU modeli: odom_hatasi={self.odom_hatasi} k_w={self.oh.k_w} '
                                   f'k_v={self.oh.k_v} σw={self.oh.sigma_w}; jiroskop nicem {self.nicem}')
        if dr:
            self.p_d = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
            self.p_di = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
            self.create_subscription(Image, '/camera/depth_sim/depth/image_raw', self._derinlik, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/depth_sim/depth/camera_info', self._dinfo,
                                     qos_profile_sensor_data)
            self.get_logger().info(f'derinlik modeli: {self.dp}')

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if self.odom_hatasi:
            x, y, yaw = self.oh.adim(p.x, p.y, yaw_q(q))
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.pose.orientation.x = m.pose.pose.orientation.y = 0.0
            m.twist.twist.linear.x /= self.oh.k_v
            m.twist.twist.angular.z /= self.oh.k_w
        self.p_odom.publish(m)
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = m.child_frame_id
        t.transform.translation.x, t.transform.translation.y = m.pose.pose.position.x, m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)

    def _imu(self, m):
        if self.nicem > 0:
            av = m.angular_velocity
            av.x, av.y, av.z = (round(v / self.nicem) * self.nicem for v in (av.x, av.y, av.z))
        self.p_imu.publish(m)

    def _derinlik(self, m):
        if m.encoding != '32FC1':
            return
        z = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        out = Image()
        out.header, out.height, out.width = m.header, m.height, m.width
        out.encoding, out.is_bigendian, out.step = '16UC1', 0, m.width * 2
        out.data = derinlik_bozu(z, rng=self.rng, **self.dp).tobytes()
        self.p_d.publish(out)

    def _dinfo(self, m):
        self.p_di.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = SensorModeliDugumu()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
