#!/bin/bash
# tur_sim_test.sh [cikti_dizini] [ek launch arg...]: 3B odada nav_tur_lab_oda.yaml gezinme turu (nav_bench.py --hedefler).
# Örnek stres: tur_sim_test.sh cikti/tur_vio odom_profil:=vio_kotu
WS=$HOME/limo_robot/limo_ws; OUT=${1:-$WS/saha/gece/cikti/tur_$(date +%H%M%S)}; shift; EK="$@"; mkdir -p $OUT
source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91} ROS_LOCALHOST_ONLY=0 GAZEBO_MASTER_URI=http://localhost:11391
export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/src/limo_bringup/config/fastdds_sim_shm.xml
SH=$WS/install/limo_bringup/share/limo_bringup; HARITA=$SH/maps/lab_oda.yaml
ros2 daemon stop >/dev/null 2>&1   # bayat daemon CLI keşfini bozuyordu
PIDS=()
temizle() { for p in "${PIDS[@]}"; do kill -- -$p 2>/dev/null; done; sleep 3; for p in "${PIDS[@]}"; do kill -9 -- -$p 2>/dev/null; done
            ps -eo pid,args | awk '$2 ~ /^gzserver/ {print $1}' | xargs -r kill -9; }
trap temizle EXIT
setsid ros2 launch limo_bringup limo_sim_nav2.launch.py headless:=true use_rviz:=false localization:=$(echo "$EK" | grep -q "localization:=yok" && echo yok || echo amcl) \
  map:=$HARITA world:=$WS/install/limo_car/share/limo_car/worlds/lab_oda_3b.world $(echo "$EK" | sed "s/localization:=yok//") > $OUT/sim.log 2>&1 & PIDS+=($!)
# Sıra önemli (kilitlenme): AMCL başlangıç pozu gelmeden map->odom yayınlamaz; Nav2 planner etkinleşmesi bu TF'i
# bekler. Önce AMCL'i bekle, pozu ver, SONRA bt_navigator'ı bekle.
if ! echo "$EK" | grep -q "localization:=yok"; then   # yok: AMCL yok, map->odom sabit
python3 $WS/saha/gece/bekle_aktif.py /amcl 180
python3 $WS/saha/gece/ilk_poz.py 0 0 0 || exit 1
fi
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 180
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 5 || { echo "Nav2 açılmadı"; exit 1; }
echo "== tur ($EK)"
timeout 1500 python3 $WS/src/limo_bringup/scripts/nav_bench.py --mode diff --out $OUT/tur.json --timeout 150 \
  --hedefler $WS/src/limo_bringup/scripts/nav_tur_lab_oda.yaml $(echo "$EK" | grep -q "localization:=yok" && echo "--lokalizasyon yok") > $OUT/tur.log 2>&1; echo "cikis $?"
tail -25 $OUT/tur.log
echo "çıktı: $OUT"
