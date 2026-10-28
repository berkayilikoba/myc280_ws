#!/usr/bin/env python3
"""insan_gecir.py MODEL_SDF X0 Y0 X1 Y1 [HIZ]: /kavrama/durum 'SON_YAKLASMA' olunca insan silindirini (X0,Y0)->(X1,Y1)
doğrultusunda HIZ m/s ile yürütür (tepkisiz, set_entity_state), sonra siler. Robot (odom_gercek) ile en yakın merkez
mesafesini ve o anki robot hızını yazar. N24 (kavrama son yaklaşması kalkana uyuyor mu) sim doğrulaması için."""
import math, sys, time
import rclpy
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sdf, x0, y0, x1, y1 = sys.argv[1], *(float(v) for v in sys.argv[2:6])
hiz = float(sys.argv[6]) if len(sys.argv) > 6 else 0.6
rclpy.init(); n = rclpy.create_node('insan_gecir')
sp, de, st = (n.create_client(SpawnEntity, '/spawn_entity'), n.create_client(DeleteEntity, '/delete_entity'),
              n.create_client(SetEntityState, '/gazebo/set_entity_state'))
d = {'durum': '', 'robot': None, 'v': 0.0, 'kalkan': []}
n.create_subscription(String, '/kavrama/durum', lambda m: d.__setitem__('durum', m.data), 10)
n.create_subscription(String, '/hareketli_kalkan/durum', lambda m: d['kalkan'].append(m.data), 10)


def od(m):
    d['robot'] = (m.pose.pose.position.x, m.pose.pose.position.y); d['v'] = abs(m.twist.twist.linear.x)


n.create_subscription(Odometry, '/odom_gercek', od, 20)


def cagir(cli, req, sure=5.0):
    cli.wait_for_service(timeout_sec=10.0); f = cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=sure); return f.result()


def konum(x, y, vx, vy):
    r = SetEntityState.Request(); r.state.name = 'insan'; r.state.reference_frame = 'world'
    r.state.pose.position.x, r.state.pose.position.y = x, y; r.state.pose.orientation.w = 1.0
    r.state.twist.linear.x, r.state.twist.linear.y = vx, vy; st.call_async(r)


r = SpawnEntity.Request(); r.name = 'insan'; r.xml = open(sdf).read(); r.reference_frame = 'world'
r.initial_pose.position.x, r.initial_pose.position.y = x0, y0; cagir(sp, r)
print('insan hazır, SON_YAKLASMA bekleniyor', flush=True)
t0 = time.time()
while 'SON_YAKLASMA' not in d['durum'] and time.time() - t0 < 300:
    rclpy.spin_once(n, timeout_sec=0.1)
print(f'tetiklendi: {d["durum"][:60]}', flush=True)
L = math.hypot(x1 - x0, y1 - y0); ux, uy = (x1 - x0) / L, (y1 - y0) / L
en_yakin, v_yakin, ts = 9.0, 0.0, time.time()
while True:
    s = (time.time() - ts) * hiz
    if s > L:
        break
    x, y = x0 + ux * s, y0 + uy * s; konum(x, y, ux * hiz, uy * hiz)
    rclpy.spin_once(n, timeout_sec=0.05)
    if d['robot']:
        m = math.hypot(x - d['robot'][0], y - d['robot'][1])
        if m < en_yakin:
            en_yakin, v_yakin = m, d['v']
dr = DeleteEntity.Request(); dr.name = 'insan'; cagir(de, dr)
print(f'SONUC en_yakin_merkez {en_yakin:.3f} m, o an robot hızı {v_yakin:.3f} m/s; kalkan durumları: {d["kalkan"][:8]}', flush=True)
