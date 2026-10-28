#!/usr/bin/env python3
"""costmap_anlik.py CIKTI_DIZINI HEDEF_X HEDEF_Y: robot hedefin 0.8 m yakınında 4 s'den uzun hareketsiz kalırsa
(takılma) yerel costmap'i PNG olarak kaydeder (ilk 2 takılma). Kırmızı: ölümcül (>=253), turuncu: yazılı (>=99),
mavi: robot. Ayrıca aynı anın /scan noktaları yeşil. Teşhis aracı (bati_dar voxel takılması)."""
import math, os, sys, time
import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

out, hx, hy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
os.makedirs(out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('costmap_anlik', parameter_overrides=[
    rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
tfb = Buffer(); TransformListener(tfb, n)
son = {'cm': None, 'scan': None, 'v': 1.0}
n.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: son.__setitem__('cm', m), 1)
n.create_subscription(LaserScan, '/scan', lambda m: son.__setitem__('scan', m), rclpy.qos.qos_profile_sensor_data)
n.create_subscription(Odometry, '/odom', lambda m: son.__setitem__('v', abs(m.twist.twist.linear.x) + abs(m.twist.twist.angular.z)), 10)


def poz(hedef, kaynak):
    t = tfb.lookup_transform(hedef, kaynak, rclpy.time.Time())
    q = t.transform.rotation
    return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def kaydet(i):
    import cv2
    cm = son['cm']; W, H, r = cm.info.width, cm.info.height, cm.info.resolution
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
    g = np.array(cm.data, dtype=np.int16).reshape(H, W)
    img = np.full((H, W, 3), 255, np.uint8)
    img[g > 0] = (200, 200, 200); img[g >= 99] = (0, 140, 255); img[g >= 253] = (0, 0, 255)
    rx, ry, ryaw = poz(cm.header.frame_id, 'base_link')
    s = son['scan']
    if s is not None:
        lx, ly, lyaw = poz(cm.header.frame_id, s.header.frame_id)
        for k, d in enumerate(s.ranges):
            if math.isfinite(d) and s.range_min < d < s.range_max:
                a = s.angle_min + k * s.angle_increment + lyaw
                u, v = int((lx + d * math.cos(a) - ox) / r), int((ly + d * math.sin(a) - oy) / r)
                if 0 <= u < W and 0 <= v < H:
                    img[v, u] = (0, 180, 0)
    for dx, dy in [(-0.161, -0.11), (-0.161, 0.11), (0.161, 0.11), (0.161, -0.11)]:
        u = int((rx + dx * math.cos(ryaw) - dy * math.sin(ryaw) - ox) / r)
        v = int((ry + dx * math.sin(ryaw) + dy * math.cos(ryaw) - oy) / r)
        if 0 <= u < W and 0 <= v < H:
            img[v, u] = (255, 0, 0)
    img = cv2.resize(np.flipud(img), (W * 8, H * 8), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(f'{out}/takilma_{i}.png', img)
    mx, my, _ = poz('map', 'base_link')
    print(f'takilma_{i}: map ({mx:.2f}, {my:.2f}), olumcul hucre {(g >= 253).sum()}', flush=True)


t_dur = None; sayi = 0; t0 = time.time()
while rclpy.ok() and sayi < 2 and time.time() - t0 < 1500:
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        mx, my, _ = poz('map', 'base_link')
    except Exception:
        continue
    yakin = math.hypot(mx - hx, my - hy) < 0.8
    if yakin and son['v'] < 0.03 and son['cm'] is not None:
        t_dur = t_dur or time.time()
        if time.time() - t_dur > 4.0:
            kaydet(sayi); sayi += 1; t_dur = None; time.sleep(8.0)
    else:
        t_dur = None
