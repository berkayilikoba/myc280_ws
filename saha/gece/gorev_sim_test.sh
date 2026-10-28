#!/bin/bash
# gorev_sim_test.sh [cikti_dizini]: 3B odada (1) /kavra eylem arayüzü, (2) "elma" görevinin uçtan uca akışı
# (kavra -> taşı -> bırak -> eve dön; kol sahte). Sim + Nav2/AMCL + kavrama.launch (sahte algı elma). Sonunda temizler.
WS=$HOME/limo_robot/limo_ws; OUT=${1:-$WS/saha/gece/cikti/gorev_$(date +%H%M%S)}; mkdir -p $OUT
source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91} ROS_LOCALHOST_ONLY=0 GAZEBO_MASTER_URI=http://localhost:11391
export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/src/limo_bringup/config/fastdds_sim_shm.xml
SH=$WS/install/limo_bringup/share/limo_bringup; HARITA=$SH/maps/lab_oda.yaml
ros2 daemon stop >/dev/null 2>&1   # bayat daemon CLI keşfini bozuyordu
fastdds shm clean >/dev/null 2>&1   # kill -9 sonrası zombi SHM segmentleri DDS keşfini bozuyordu (yalnız kullanılmayanları siler)
PIDS=()
temizle() { for p in "${PIDS[@]}"; do kill -- -$p 2>/dev/null; done; sleep 3; for p in "${PIDS[@]}"; do kill -9 -- -$p 2>/dev/null; done
            ps -eo pid,args | awk '$2 ~ /^gzserver/ {print $1}' | xargs -r kill -9; }
trap temizle EXIT
setsid ros2 launch limo_bringup limo_sim_nav2.launch.py headless:=true use_rviz:=false localization:=amcl \
  map:=$HARITA world:=$WS/install/limo_car/share/limo_car/worlds/lab_oda_3b.world > $OUT/sim.log 2>&1 & PIDS+=($!)
# Sıra önemli (kilitlenme): AMCL başlangıç pozu gelmeden map->odom yayınlamaz; Nav2 planner etkinleşmesi bu TF'i
# bekler. Önce AMCL'i bekle, pozu ver, SONRA bt_navigator'ı bekle.
python3 $WS/saha/gece/bekle_aktif.py /amcl 180
python3 $WS/saha/gece/ilk_poz.py 0 0 0 || exit 1
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 180
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 5 || { echo "Nav2 açılmadı"; exit 1; }
# 1) eylem arayüzü: dolap (sahte algı dolabın konumunda)
setsid ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true sahte_algi:=true nesne_x:=3.73 nesne_y:=2.18 nesne_z:=0.30 \
  > $OUT/kavrama1.log 2>&1 & K1=$!; PIDS+=($K1); sleep 8
echo "== 1) /kavra eylemi (dolap)"
timeout 300 ros2 action send_goal /kavra limo_interfaces/action/Kavra \
  "{nesne: {header: {frame_id: map}, point: {x: 3.73, y: 2.18, z: 0.30}}, yon_derece: .nan, algisiz_izinli: true}" --feedback \
  > $OUT/eylem.log 2>&1; echo "cikis $?"; grep -E "durum:|basarili|neden|tahmini|sure_s|Goal (accepted|finished)|status" $OUT/eylem.log | tail -12
kill -- -$K1 2>/dev/null; sleep 4
# 2) görev: elma (doğu duvarı dibinde, kavrama.launch varsayılanı)
setsid ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true sahte_algi:=true > $OUT/kavrama2.log 2>&1 & PIDS+=($!); sleep 8
echo "== 2) /gorev/istek elma"
timeout 900 python3 - > $OUT/gorev.log 2>&1 <<'PY'
import rclpy, time
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node('gorev_test'); son = []
n.create_subscription(String, '/gorev/durum', lambda m: (son.append(m.data), print(f'{time.strftime("%H:%M:%S")} {m.data}', flush=True)), 10)
p = n.create_publisher(String, '/gorev/istek', 10)
t = time.time()
while time.time() - t < 3: rclpy.spin_once(n, timeout_sec=0.1)
p.publish(String(data='elma')); print('istek gönderildi: elma', flush=True)
t = time.time()
while time.time() - t < 840:
    rclpy.spin_once(n, timeout_sec=0.2)
    if son and (son[-1].startswith('TAMAM') or son[-1].startswith('BASARISIZ') or son[-1].startswith('HATA') or son[-1].startswith('BOSTA')) and len(son) > 2:
        break
print('SON:', son[-1] if son else 'durum yok')
PY
grep -vE "^\s*$" $OUT/gorev.log | uniq | tail -25
echo "çıktı: $OUT"
