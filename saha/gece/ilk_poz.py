#!/usr/bin/env python3
"""ilk_poz.py [x y yaw]: AMCL'e başlangıç pozu ver — /initialpose abonesi görünene kadar bekler (en çok 90 s), 3 kez yayınlar.
ros2 CLI (daemon) bayatken `ros2 topic pub -w 1` aboneyi görmeyip takılıyordu; burada doğrudan DDS keşfi kullanılır."""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yaw = (float(v) for v in (sys.argv[1:4] if len(sys.argv) >= 4 else (0, 0, 0)))
rclpy.init(); n = rclpy.create_node('ilk_poz'); p = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
t0 = time.time()
while p.get_subscription_count() == 0 and time.time() - t0 < 90:
    rclpy.spin_once(n, timeout_sec=0.5)
if p.get_subscription_count() == 0:
    print('ilk_poz: /initialpose abonesi yok (AMCL açık mı?)'); sys.exit(1)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x, m.pose.pose.position.y = x, y
m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
m.pose.covariance[0] = m.pose.covariance[7] = 0.0025; m.pose.covariance[35] = 0.003
for _ in range(3):
    # stamp 0: AMCL en son TF'i kullanır (duvar saati sim saatinde 'gelecek' sayılıp reddediliyordu)
    p.publish(m); rclpy.spin_once(n, timeout_sec=1.0)
print(f'ilk_poz: verildi ({x}, {y}, {yaw}) — abone {p.get_subscription_count()}')
