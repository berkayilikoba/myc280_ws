#!/usr/bin/env python3
"""bekle_aktif.py DUGUM [zaman_asimi_s]: lifecycle düğümü 'active' olana kadar bekle (çıkış 0), yoksa 1.
`ros2 lifecycle get --no-daemon` keşfi ara sıra düğümü göremeyip "Nav2 açılmadı" diye yanlış başarısızlık
veriyordu (Nav2 aslında açıktı); burada tek düğüm sürekli açık kalıp get_state servisini doğrudan çağırır."""
import sys, time
import rclpy
from lifecycle_msgs.srv import GetState
dugum = sys.argv[1].lstrip('/'); sure = float(sys.argv[2]) if len(sys.argv) > 2 else 180.0
rclpy.init(); n = rclpy.create_node('bekle_aktif'); c = n.create_client(GetState, f'/{dugum}/get_state')
t0 = time.time()
while time.time() - t0 < sure:
    if c.wait_for_service(timeout_sec=1.0):
        f = c.call_async(GetState.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=2.0)
        if f.done() and f.result() is not None and f.result().current_state.label == 'active':
            print(f'bekle_aktif: {dugum} active ({time.time() - t0:.0f} s)'); sys.exit(0)
    time.sleep(1.0)
print(f'bekle_aktif: {dugum} {sure:.0f} s içinde active olmadı'); sys.exit(1)
