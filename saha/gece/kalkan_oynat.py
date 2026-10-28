#!/usr/bin/env python3
"""kalkan_oynat.py BAG_DIZINI: kaydedilmiş /scan + /odom'u hareketli_kalkan saf fonksiyonlarından geçirir (çevrimdışı).
Her taramada: robot pozu, hızlı izler (|v| > 0.1) ve karar. Kalkanın neden tetiklenmediğini görmek için teşhis aracı."""
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from limo_autonomy.hareketli_kalkan import (IzTakipci, kalkan_karari, kume_merkezleri_odoma, kumele, poz_aradeger,
                                            tarama_noktalari, yaw_q)

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
gecmis, v_odom, wz, takip, gercek = [], (0.0, 0.0), 0.0, IzTakipci(), None
while r.has_next():
    konu, veri, _ = r.read_next()
    if konu == '/odom':
        m = deserialize_message(veri, Odometry); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        yaw = yaw_q(m.pose.pose.orientation); gecmis.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        gecmis = gecmis[-200:]
        vx = m.twist.twist.linear.x; v_odom = (vx * math.cos(yaw), vx * math.sin(yaw)); wz = m.twist.twist.angular.z
    elif konu == '/odom_gercek':
        m = deserialize_message(veri, Odometry); gercek = (m.pose.pose.position.x, m.pose.pose.position.y)
    elif konu == '/scan':
        m = deserialize_message(veri, LaserScan); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        poz = poz_aradeger(gecmis, t)
        if poz is None:
            print(f'{t:.2f} POZ YOK (odom tamponu {gecmis[0][0] if gecmis else None}..{gecmis[-1][0] if gecmis else None})')
            continue
        pts = tarama_noktalari(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, 5.0,
                               lidar_x=0.137, lidar_y=0.015, lidar_yaw=-0.1686)
        mer, yar = kumele(pts, esik=0.10, min_nokta=3) if len(pts) else ([], [])
        sec = [i for i, rr in enumerate(yar) if rr <= 0.35]
        mo = kume_merkezleri_odoma([mer[i] for i in sec], poz)
        takip.guncelle(mo, [yar[i] for i in sec], t, robot_wz=wz)
        k = kalkan_karari(takip.izler(), v_odom, poz[:2])
        hizli = [(round(i.x, 2), round(i.y, 2), round(math.hypot(i.vx, i.vy), 2), i.gorulme, round(i.iz_yasi_s, 2))
                 for i in takip.izler() if math.hypot(i.vx, i.vy) > 0.1 or (3.0 < i.x and 0 < i.y < 2.2)]
        print(f'{t:.2f} robot ({poz[0]:.2f},{poz[1]:.2f}) kume {len(mer)}/{len(sec)} karar {k[0]} {k[2]} izler {hizli[:4]}')
