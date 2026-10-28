#!/usr/bin/env bash
# Robottaki derinlik kamerasını açar (tip lsusb'den: ortam.sh kamera_tipi). SURE saniye sonra kendiliğinden kapanır.
#   dabai: Orbbec Dabai DC1 — renk 640x480 + derinlik 640x400, 30 Hz; nokta bulutu ve IR kapalı.
#   d435 : Intel RealSense D435 — renk + derinlik PROFIL (varsayılan 640x480x30); derinlik konusu
#          /camera/depth/image_rect_raw -> /camera/depth/image_raw yeniden adlandırılır (kayıt/analiz aynı konular).
#          EK ile ek parametre: ör. EK="-p align_depth.enable:=true -p spatial_filter.enable:=true"
#          D435_AYAR=vlmaps: VLMaps kaydı için önerilen (21.10 bulguları): derinlik 848x480 (D435 için önerilen
#          çözünürlük), renk 640x480, uzamsal + zamansal süzgeç, High Accuracy ön ayarı (depth_module.visual_preset 3,
#          27.10 robotta doğrulandı: 0 Custom (sürücü varsayılanı), 1 Default, 3 High Accuracy, 4 High Density).
#          Ayrıca kayıtta dönüşler <= 0.2 rad/s olmalı (kapsama_turu.py --spin_w 0.2).
# Kullanım: [SURE=1200] [PROFIL=640x480x30] [DPROFIL=..] [D435_AYAR=vlmaps] [EK="..."] kamera_baslat.sh   Kapatmak: kamera_dur.sh
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
SURE=${SURE:-1200}; EK=${EK:-}
if [ "${D435_AYAR:-}" = vlmaps ]; then
  DPROFIL=${DPROFIL:-848x480x30}
  EK="-p spatial_filter.enable:=true -p temporal_filter.enable:=true -p depth_module.visual_preset:=3 $EK"
elif [ "${D435_AYAR:-}" = vlmaps2 ]; then
  # 27.10: zamansal süzgeç kareler arası ortalama -> hareket halinde derinlik kenarı gecikiyor (turda hiza 3.8-6 px,
  # eski 1.9); yalnız uzamsal süzgeç + lazer 360 (sabit test 27.10: doluluk 0.48 -> 0.54, 2-4 m 0.73 -> 0.84; zamansal
  # süzgecin "σ yarıya" kazancı zamansal σ ölçütünün doğal sonucu, hareket halinde geometriye katkısı yok)
  PROFIL=${PROFIL:-1280x720x15}; DPROFIL=${DPROFIL:-848x480x15}
  EK="-p spatial_filter.enable:=true -p depth_module.visual_preset:=3 -p depth_module.laser_power:=${LAZER:-360.0} $EK"
fi
PROFIL=${PROFIL:-640x480x30}; DPROFIL=${DPROFIL:-$PROFIL}
ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "orbbec_camer[a]|realsense2_camera_nod[e]" >/dev/null' && { echo "kamera zaten acik"; exit 0; }
TIP=$(kamera_tipi); echo "kamera: $TIP"
if [ "$TIP" = d435 ]; then
  robot_cmd "(timeout -s INT $SURE ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/camera -r __node:=camera \
      -p depth_module.profile:=$DPROFIL -p rgb_camera.profile:=$PROFIL -p enable_infra1:=false -p enable_infra2:=false \
      -p pointcloud.enable:=false -p publish_tf:=true -p tf_publish_rate:=0.0 \
      -r /camera/depth/image_rect_raw:=/camera/depth/image_raw $EK > /tmp/kamera.log 2>&1 &); sleep 10; \
    grep -m3 -iE 'error|fail|usb' /tmp/kamera.log; ros2 topic list --no-daemon 2>/dev/null | grep -E '^/camera/'" | sed 's/^/robot: /'
else
  robot_cmd "(timeout -s INT $SURE ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=false enable_ir:=false > /tmp/kamera.log 2>&1 &); sleep 12; grep -m2 -iE 'error|fail' /tmp/kamera.log; ros2 topic list --no-daemon 2>/dev/null | grep -E 'camera/(color|depth)/image_raw'" \
    | sed 's/^/robot: /'
  echo "Not: 'Failed to set 2035' hatası zararsız (kayıt etkilenmiyor)."
fi
