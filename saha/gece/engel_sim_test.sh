#!/bin/bash
# engel_sim_test.sh SENARYO [cikti_dizini] [ek launch arg...]: 3B odada dinamik engel senaryosu (dinamik_engel.py).
WS=$HOME/limo_robot/limo_ws; SEN=$1; OUT=${2:-$WS/saha/gece/cikti/engel_${1}_$(date +%H%M%S)}; shift 2; EK="$@"; mkdir -p $OUT
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
  map:=$HARITA world:=$WS/install/limo_car/share/limo_car/worlds/lab_oda_3b.world $EK > $OUT/sim.log 2>&1 & PIDS+=($!)
# Sıra önemli (kilitlenme): AMCL başlangıç pozu gelmeden map->odom yayınlamaz; Nav2 planner etkinleşmesi bu TF'i
# bekler. Önce AMCL'i bekle, pozu ver, SONRA bt_navigator'ı bekle.
python3 $WS/saha/gece/bekle_aktif.py /amcl 180
python3 $WS/saha/gece/ilk_poz.py 0 0 0 || exit 1
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 180
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 5 || { echo "Nav2 açılmadı"; exit 1; }
echo "== engel $SEN ($EK)"
timeout 400 python3 $WS/src/limo_bringup/scripts/dinamik_engel.py --senaryo $SEN \
  --senaryolar $WS/src/limo_bringup/scripts/dinamik_engel_senaryolar.yaml \
  --model $WS/install/limo_car/share/limo_car/models/insan_silindir/model.sdf --zaman_asimi 150 --cikti $OUT/sonuc.yaml > $OUT/engel.log 2>&1
echo "cikis $?"; tail -15 $OUT/engel.log
