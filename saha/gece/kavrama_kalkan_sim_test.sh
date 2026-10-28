#!/bin/bash
# kavrama_kalkan_sim_test.sh [cikti_dizini]: kavrama son yaklaşmasında robotun önünden insan geçer (N24 doğrulaması).
# Sim + Nav2/AMCL (hareketli_kalkan:=true) + kavrama.launch (sahte algı, doğu duvarı elması) + insan_gecir.py + /kavra.
WS=$HOME/limo_robot/limo_ws; OUT=${1:-$WS/saha/gece/cikti/kk_$(date +%H%M%S)}; mkdir -p $OUT
source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91} ROS_LOCALHOST_ONLY=0 GAZEBO_MASTER_URI=http://localhost:11391
export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/src/limo_bringup/config/fastdds_sim_shm.xml
SH=$WS/install/limo_bringup/share/limo_bringup; HARITA=$SH/maps/lab_oda.yaml
ros2 daemon stop >/dev/null 2>&1
bash $WS/saha/gece/shm_temizle.sh
PIDS=()
temizle() { for p in "${PIDS[@]}"; do kill -- -$p 2>/dev/null; done; sleep 3; for p in "${PIDS[@]}"; do kill -9 -- -$p 2>/dev/null; done
            ps -eo pid,args | awk '$2 ~ /^gzserver/ {print $1}' | xargs -r kill -9; }
trap temizle EXIT
setsid ros2 launch limo_bringup limo_sim_nav2.launch.py headless:=true use_rviz:=false localization:=amcl hareketli_kalkan:=true \
  map:=$HARITA world:=$WS/install/limo_car/share/limo_car/worlds/lab_oda_3b.world > $OUT/sim.log 2>&1 & PIDS+=($!)
python3 $WS/saha/gece/bekle_aktif.py /amcl 180
python3 $WS/saha/gece/ilk_poz.py 0 0 0 || exit 1
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 180 || { echo "Nav2 açılmadı"; exit 1; }
setsid ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true sahte_algi:=true > $OUT/kavrama.log 2>&1 & PIDS+=($!); sleep 8
# İnsan Q (5.05, 0.80) ile nesne (5.62, 0.80) arasından, kuzeyden güneye geçer (yaklaşma ekseni +x, y=0.8)
[ -n "$KAYIT" ] && { setsid ros2 bag record -o $OUT/kayit /scan /odom /odom_gercek /hareketli_kalkan/durum /kavrama/durum > $OUT/bag.log 2>&1 & PIDS+=($!); sleep 2; }
python3 $WS/saha/gece/insan_gecir.py $WS/install/limo_car/share/limo_car/models/insan_silindir/model.sdf \
  5.35 1.9 5.35 -0.2 0.5 > $OUT/insan.log 2>&1 & IP=$!
timeout 400 ros2 action send_goal /kavra limo_interfaces/action/Kavra \
  "{nesne: {header: {frame_id: map}, point: {x: 5.62, y: 0.8, z: 0.10}}, yon_derece: .nan, algisiz_izinli: true}" \
  > $OUT/eylem.log 2>&1
wait $IP
grep -E "basarili|neden|tahmini_hata_cm|Goal finished" $OUT/eylem.log | tail -4; cat $OUT/insan.log
grep -E "KALKAN|kalkan" $OUT/kavrama.log | sed 's/.*\]: //' | uniq | head -5
echo "çıktı: $OUT"
