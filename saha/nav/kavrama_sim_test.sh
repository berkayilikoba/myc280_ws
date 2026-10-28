#!/bin/bash
# kavrama_sim_test.sh [hedefler.yaml] [cikti_dizini]: 3B odada uçtan uca kavrama ölçümü (sim + Nav2/AMCL + kavrama +
# sahte algı + kavrama_bench). Her hedef için sahte algı o nesnenin konumuyla yeniden başlatılır. Sonunda her şeyi kapatır.
WS=$HOME/limo_robot/limo_ws
HEDEF=${1:-$WS/src/limo_bringup/scripts/kavrama_hedefler_lab_oda.yaml}; OUT=${2:-$WS/saha/nav/cikti/kavrama_$(date +%H%M%S)}
mkdir -p $OUT
source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91} ROS_LOCALHOST_ONLY=0 GAZEBO_MASTER_URI=http://localhost:11391
export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/src/limo_bringup/config/fastdds_sim_shm.xml
SH=$WS/install/limo_bringup/share/limo_bringup; HARITA=$SH/maps/lab_oda.yaml
ros2 daemon stop >/dev/null 2>&1   # bayat daemon CLI keşfini bozuyordu
PIDS=()
temizle() { for p in "${PIDS[@]}"; do kill -- -$p 2>/dev/null; done; sleep 3; for p in "${PIDS[@]}"; do kill -9 -- -$p 2>/dev/null; done
            ps -eo pid,args | awk '$2 ~ /^gzserver/ {print $1}' | xargs -r kill -9; }
trap temizle EXIT
setsid ros2 launch limo_bringup limo_sim_nav2.launch.py headless:=true use_rviz:=false localization:=amcl \
  map:=$HARITA world:=$WS/install/limo_car/share/limo_car/worlds/lab_oda_3b.world > $OUT/sim.log 2>&1 & PIDS+=($!)
# Nav2 hazır: bt_navigator aktif
# Sıra önemli (kilitlenme): AMCL başlangıç pozu gelmeden map->odom yayınlamaz; Nav2 planner etkinleşmesi bu TF'i
# bekler. Önce AMCL'i bekle, pozu ver, SONRA bt_navigator'ı bekle.
for i in $(seq 1 60); do sleep 3; ros2 lifecycle get --no-daemon /amcl 2>/dev/null | grep -q active && break; done
python3 $WS/saha/gece/ilk_poz.py 0 0 0 || exit 1
for i in $(seq 1 60); do sleep 3; ros2 lifecycle get --no-daemon /bt_navigator 2>/dev/null | grep -q active && break; done
ros2 lifecycle get --no-daemon /bt_navigator 2>/dev/null | grep -q active || { echo "Nav2 açılmadı"; exit 1; }
setsid ros2 run limo_autonomy kavrama --ros-args -p use_sim_time:=true -p harita:=$HARITA > $OUT/kavrama.log 2>&1 & PIDS+=($!)
sleep 5
python3 - "$HEDEF" > $OUT/hedefler.txt <<'PY'
import sys, yaml
for h in yaml.safe_load(open(sys.argv[1])): print(h['ad'], h['x'], h['y'], h['z'])
PY
while read ad x y z; do
  echo "== $ad ($x, $y, $z)"
  setsid ros2 run limo_autonomy sahte_algi --ros-args -p use_sim_time:=true -p nesne_x:=$x -p nesne_y:=$y -p nesne_z:=$z \
    > $OUT/algi_$ad.log 2>&1 & AP=$!
  python3 - "$HEDEF" "$ad" > $OUT/tek_$ad.yaml <<'PY'
import sys, yaml
print(yaml.safe_dump([h for h in yaml.safe_load(open(sys.argv[1])) if h['ad'] == sys.argv[2]]))
PY
  python3 $WS/src/limo_bringup/scripts/kavrama_bench.py --hedefler $OUT/tek_$ad.yaml --cikti $OUT/sonuc_$ad.yaml \
    --zaman_asimi 240 2>&1 | tail -25 | tee $OUT/bench_$ad.log
  kill -- -$AP 2>/dev/null; sleep 2
done < $OUT/hedefler.txt
echo "çıktı: $OUT"
