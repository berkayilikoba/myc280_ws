#!/bin/bash
# sim_ac.sh <dunya.world> [ek launch argümanları]: kollu LIMO simi (ekransız), ayrı ROS domain'i (91) ve büyük SHM profili
source /opt/ros/humble/setup.bash; source ~/limo_robot/limo_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91} ROS_LOCALHOST_ONLY=0 GAZEBO_MASTER_URI=http://localhost:11391
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/limo_robot/limo_ws/src/limo_bringup/config/fastdds_sim_shm.xml
W=${1:?dunya.world}; shift
exec ros2 launch limo_description gazebo_limo_mycobot.launch.py headless:=true use_rviz:=False gui:=False world:=$W "$@"
