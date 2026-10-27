#!/bin/bash
# isik_dene.sh <sahne_ambient> <isik_diffuse> <malzeme_ambient> <etiket>: ışık ayarlı dünya kopyası -> sim -> E (3 kare)
cd ~/limo_robot/limo_ws/saha/oda; S=/tmp/claude-1000/-home-gunes-limo-robot/38148ada-f9ad-4910-bc3b-6072460e873b/scratchpad
D=cikti/isik_$4; mkdir -p $D; cp -r cikti/doku/materials $D/
sed -i "s/ambient 0.8 0.8 0.8 1.0/ambient $3 $3 $3 1.0/" $D/materials/scripts/oda.material
sed -e "s#<ambient>0.65 0.65 0.65 1</ambient>#<ambient>$1 $1 $1 1</ambient>#" -e "s#<diffuse>0.55 0.55 0.52 1</diffuse>#<diffuse>$2 $2 $2 1</diffuse>#g" \
    -e "s#cikti/doku/materials#$D/materials#g" cikti/lab_oda_3b.world > $D/dunya.world
sed -i "s#file://$PWD/$D#file://$PWD/$D#g" $D/dunya.world
for p in $(ps -eo pid,cmd | awk '/^ *[0-9]+ (gzserve[r]|.*ros2 launch limo_descriptio[n]|.*sensor_model[i]|.*taban_model[i]|.*robot_state_publishe[r]|.*joint_state_publishe[r])/ {print $1}'); do kill $p 2>/dev/null; done; sleep 4
for p in $(ps -eo pid,cmd | awk '/^ *[0-9]+ gzserve[r]/ {print $1}'); do kill -9 $p; done
setsid ./sim_ac.sh $PWD/$D/dunya.world > $S/sim_$4.log 2>&1 < /dev/null &
for i in $(seq 1 60); do grep -qE "Spawn status|ERROR" $S/sim_$4.log && break; sleep 3; done; sleep 10
source /opt/ros/humble/setup.bash; source ~/limo_robot/limo_ws/install/setup.bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/limo_robot/limo_ws/src/limo_bringup/config/fastdds_sim_shm.xml
timeout 600 python3 oda_dogrula.py ~/limo_veri/vlmaps_sahne/v2a_lidar $D/dogrula --kareler 280,560,840 2>&1 | grep -oE "Kare [0-9]+ tamamlandı.*"
