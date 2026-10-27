#!/usr/bin/env python3
"""Sahte nesne algısı düğümü (simülasyonda nesne algısı yer tutucusu).

Gerçek algı katmanı (kamera/YOLO vb.) henüz devrede değilken simülasyon ortamında
/kavrama/nesne_algi konusunu üretir (gerçek algı ekibinin arayüzü ile birebir aynıdır).

Varsayımlar ve Geometri:
  - Robotun GERÇEK pozu /odom_gercek (nav_msgs/Odometry) konusundan alınır.
    Gazebo planar_move eklentisi robotun dünya çerçevesindeki gerçek pozunu yayınlar;
    /odom_gercek başlangıcının dünya (harita) orijini ile aynı olduğu varsayılır.
  - Kamera robot merkezinin (base_footprint) kamera_ofset (varsayılan 0.084 m) önündedir.
    Görüş alanı (FOV) ve menzil testi doğrudan kamera noktasından yapılır.
  - Nesne kameranın görüş açısı (gorus_acisi=70.0°) ve menzili (menzil_min=0.12 m, menzil_max=1.5 m)
    içindeyse; kayıp olasılığı (kayip_olasiligi=0.1) ile rastgele atlanarak, nesnenin
    base_footprint'teki konumuna Gaussian xy gürültüsü (sigma=0.01 m) eklenir ve
    geometry_msgs/PointStamped olarak yayınlanır.
  - Zaman damgası: header.stamp = son alınan /odom_gercek mesajının stamp'i.
  - Yükseklik: point.z = nesne_z.
  - Görüş durumu: /kavrama/nesne_algi_durum (std_msgs/String) ile 1 Hz frekansında
    'GORUYOR' veya 'GORMUYOR' olarak yayınlanır.
"""

import math
from typing import Sequence, Tuple

import numpy as np
from geometry_msgs.msg import PointStamped, Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def yaw_q(q: Quaternion) -> float:
    """geometry_msgs Quaternion -> düzlemsel yaw açısı (radyan)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def gorus_testi(
    robot_pose: Sequence[float],
    nesne_xy: Sequence[float],
    kamera_ofset: float = 0.084,
    menzil_min: float = 0.12,
    menzil_max: float = 1.5,
    gorus_acisi: float = 70.0,
    nesne_z: float | None = None,
    kamera_z: float = 0.18,
    dikey_yari_aci: float = 21.0,
) -> Tuple[bool, Tuple[float, float]]:
    """Robot ve nesne konumuna göre nesnenin kamera görüş alanında olup olmadığını test eder.

    Args:
        robot_pose: (rx, ry, ryaw) robotun dünya çerçevesindeki konumu ve yönelimi (m, m, rad).
        nesne_xy: (nx, ny) nesnenin dünya çerçevesindeki konumu (m, m).
        kamera_ofset: Kameranın robot merkezine (base_footprint) göre ileri yöndeki ofseti (m).
        menzil_min: Kameranın asgari algılama mesafesi (m).
        menzil_max: Kameranın azami algılama mesafesi (m).
        gorus_acisi: Kameranın toplam yatay görüş açısı (derece).
        nesne_z: Nesnenin z yüksekliği (m, isteğe bağlı).
        kamera_z: Kameranın yerden yüksekliği (m, varsayılan 0.18 m).
        dikey_yari_aci: Kameranın dikey görüş yarı açısı (derece veya radyan, varsayılan 21°).

    Returns:
        (gorunur, (x_r, y_r)):
            gorunur: Nesne kameranın görüş açısı ve menzil sınırları içinde mi (bool).
            (x_r, y_r): Nesnenin robotun base_footprint çerçevesindeki konumu (m, m).
    """
    rx, ry, ryaw = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    nx, ny = float(nesne_xy[0]), float(nesne_xy[1])

    # 1. Nesnenin robot gövde çerçevesine (base_footprint) göre konumu (x_r, y_r)
    dx = nx - rx
    dy = ny - ry

    cos_th = math.cos(ryaw)
    sin_th = math.sin(ryaw)

    # base_footprint koordinatları (-ryaw dönüşümü)
    x_r = dx * cos_th + dy * sin_th
    y_r = -dx * sin_th + dy * cos_th

    # 2. Kamera merkezine göre konum (kamera robot merkezinin kamera_ofset m önünde)
    x_cam = x_r - float(kamera_ofset)
    y_cam = y_r

    # Kamera optik eksenine göre yatay mesafe ve yatay açı
    d_yatay = math.hypot(x_cam, y_cam)
    aci = math.atan2(y_cam, x_cam)

    # 3. Yatay görüş açısı testi (toplam açı gorus_acisi derece -> yarı açı gorus_acisi / 2)
    yarim_aci = math.radians(float(gorus_acisi) * 0.5)
    aci_tamam = abs(aci) <= yarim_aci

    # 4. Dikey görüş açısı (yükselme açısı) testi
    if nesne_z is not None:
        nz_val = float(nesne_z)
    elif len(nesne_xy) >= 3:
        nz_val = float(nesne_xy[2])
    else:
        nz_val = None

    if nz_val is not None:
        dz = nz_val - float(kamera_z)
        d = math.hypot(d_yatay, dz)
        aci_dikey = math.atan2(dz, d_yatay)
        dikey_sinir = (
            math.radians(float(dikey_yari_aci))
            if abs(float(dikey_yari_aci)) > math.pi
            else abs(float(dikey_yari_aci))
        )
        dikey_tamam = abs(aci_dikey) <= dikey_sinir
    else:
        d = d_yatay
        dikey_tamam = True

    # 5. Menzil testi
    menzil_tamam = float(menzil_min) <= d <= float(menzil_max)

    gorunur = bool(menzil_tamam and aci_tamam and dikey_tamam)
    return gorunur, (x_r, y_r)


class SahteAlgiDugumu(Node):
    """Simülasyonda nesne algısını simüle eden ROS 2 düğümü."""

    def __init__(self) -> None:
        super().__init__('sahte_algi')

        # Düğüm parametreleri
        p = self.declare_parameter
        p('nesne_x', 0.0)
        p('nesne_y', 0.0)
        p('nesne_z', 0.0)
        p('hz', 5.0)
        p('sigma', 0.01)
        p('menzil_min', 0.12)
        p('menzil_max', 1.5)
        p('gorus_acisi', 70.0)
        p('kayip_olasiligi', 0.1)
        p('cerceve', 'base_footprint')
        p('tohum', 0)
        p('kamera_ofset', 0.084)
        p('kamera_z', 0.18)
        p('dikey_yari_aci', 21.0)
        p('odom_topic', '/odom_gercek')

        tohum = int(self.get_parameter('tohum').value)
        self.rng = np.random.default_rng(tohum if tohum >= 0 else None)

        self.son_odom: Odometry | None = None
        self.goruyor: bool = False

        # Yayıncılar
        self.pub_algi = self.create_publisher(PointStamped, '/kavrama/nesne_algi', 10)
        self.pub_durum = self.create_publisher(String, '/kavrama/nesne_algi_durum', 10)

        # /odom_gercek aboneliği
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        # Döngü zamanlayıcıları
        hz = float(self.get_parameter('hz').value)
        algi_periyot = 1.0 / max(1e-3, hz)
        self.timer_algi = self.create_timer(algi_periyot, self._algi_dongusu)
        self.timer_durum = self.create_timer(1.0, self._durum_dongusu)

        self.get_logger().info(
            f"sahte_algi düğümü hazır: hz={hz}, sigma={self.get_parameter('sigma').value}, "
            f"cerceve='{self.get_parameter('cerceve').value}'"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self.son_odom = msg

    def _algi_dongusu(self) -> None:
        if self.son_odom is None:
            self.goruyor = False
            return

        nx = float(self.get_parameter('nesne_x').value)
        ny = float(self.get_parameter('nesne_y').value)
        nz = float(self.get_parameter('nesne_z').value)
        sigma = float(self.get_parameter('sigma').value)
        menzil_min = float(self.get_parameter('menzil_min').value)
        menzil_max = float(self.get_parameter('menzil_max').value)
        gorus_acisi = float(self.get_parameter('gorus_acisi').value)
        kayip_olasiligi = float(self.get_parameter('kayip_olasiligi').value)
        cerceve = str(self.get_parameter('cerceve').value)
        kamera_ofset = float(self.get_parameter('kamera_ofset').value)
        kamera_z = float(self.get_parameter('kamera_z').value)
        dikey_yari_aci = float(self.get_parameter('dikey_yari_aci').value)

        pos = self.son_odom.pose.pose.position
        ori = self.son_odom.pose.pose.orientation
        robot_pose = (pos.x, pos.y, yaw_q(ori))

        gorunur, (x_r, y_r) = gorus_testi(
            robot_pose=robot_pose,
            nesne_xy=(nx, ny),
            kamera_ofset=kamera_ofset,
            menzil_min=menzil_min,
            menzil_max=menzil_max,
            gorus_acisi=gorus_acisi,
            nesne_z=nz,
            kamera_z=kamera_z,
            dikey_yari_aci=dikey_yari_aci,
        )

        self.goruyor = gorunur

        if gorunur:
            # Kayıp olasılığı ile kare atlama
            if kayip_olasiligi > 0.0 and float(self.rng.random()) < kayip_olasiligi:
                return

            # Gaussian xy gürültüsü
            gx = float(self.rng.normal(0.0, sigma)) if sigma > 0.0 else 0.0
            gy = float(self.rng.normal(0.0, sigma)) if sigma > 0.0 else 0.0

            msg = PointStamped()
            msg.header.stamp = self.son_odom.header.stamp
            msg.header.frame_id = cerceve
            msg.point.x = x_r + gx
            msg.point.y = y_r + gy
            msg.point.z = nz
            self.pub_algi.publish(msg)

    def _durum_dongusu(self) -> None:
        durum_str = 'GORUYOR' if self.goruyor else 'GORMUYOR'
        self.pub_durum.publish(String(data=durum_str))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    dugum = SahteAlgiDugumu()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
