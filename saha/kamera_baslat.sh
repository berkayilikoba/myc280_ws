#!/usr/bin/env bash
# Robottaki derinlik kamerasını açar (tip lsusb'den: ortam.sh kamera_tipi). SURE saniye sonra kendiliğinden kapanır.
#   dabai: Orbbec Dabai DC1 — renk 640x480 + derinlik 640x400, 30 Hz; nokta bulutu ve IR kapalı.
#   d435 : Intel RealSense D435 — renk + derinlik PROFIL (varsayılan 640x480x30); derinlik konusu
#          /camera/depth/image_rect_raw -> /camera/depth/image_raw yeniden adlandırılır (kayıt/analiz aynı konular).
#          EK ile ek parametre: ör. EK="-p align_depth.enable:=true -p spatial_filter.enable:=true"
# Kullanım: [SURE=1200] [PROFIL=640x480x30] [EK="..."] kamera_baslat.sh        Kapatmak: kamera_dur.sh
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
SURE=${SURE:-1200}; PROFIL=${PROFIL:-640x480x30}; EK=${EK:-}
ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "orbbec_camer[a]|realsense2_camera_nod[e]" >/dev/null' && { echo "kamera zaten acik"; exit 0; }
TIP=$(kamera_tipi); echo "kamera: $TIP"
if [ "$TIP" = d435 ]; then
  robot_cmd "(timeout -s INT $SURE ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/camera -r __node:=camera \
      -p depth_module.profile:=$PROFIL -p rgb_camera.profile:=$PROFIL -p enable_infra1:=false -p enable_infra2:=false \
      -p pointcloud.enable:=false -p publish_tf:=true -p tf_publish_rate:=0.0 \
      -r /camera/depth/image_rect_raw:=/camera/depth/image_raw $EK > /tmp/kamera.log 2>&1 &); sleep 10; \
    grep -m3 -iE 'error|fail|usb' /tmp/kamera.log; ros2 topic list --no-daemon 2>/dev/null | grep -E '^/camera/'" | sed 's/^/robot: /'
else
  robot_cmd "(timeout -s INT $SURE ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=false enable_ir:=false > /tmp/kamera.log 2>&1 &); sleep 12; grep -m2 -iE 'error|fail' /tmp/kamera.log; ros2 topic list --no-daemon 2>/dev/null | grep -E 'camera/(color|depth)/image_raw'" \
    | sed 's/^/robot: /'
  echo "Not: 'Failed to set 2035' hatası zararsız (kayıt etkilenmiyor)."
fi
