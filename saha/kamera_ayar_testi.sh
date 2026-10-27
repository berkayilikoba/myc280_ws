#!/bin/bash
# Derinlik kamerası (Orbbec Dabai / RealSense D435, lsusb'den) ayar karşılaştırması: robot DURURKEN aynı sahneden her ayarla kısa kamera kaydı (robotta ~/limo_kayit/ayar_<zaman>/<ad>).
# Kullanım: kamera_ayar_testi.sh [saniye]   Sonra: analiz/derinlik_karakter.py <dizin>
source "$(dirname "$0")/ortam.sh"
SN=${1:-8}; Z=$(date +%Y%m%d_%H%M%S); D=/home/agilex/limo_kayit/ayar_$Z
scp -q -o BatchMode=yes "$(dirname "$0")/config/kamera_qos.yaml" "$ROBOT:/tmp/" || exit 1
TIP=$(kamera_tipi); echo "kamera: $TIP"
if [ "$TIP" = d435 ]; then
  # 848x480: D435 derinlik için önerilen çözünürlük; hw_hiza: sürücü derinliği renge hizalar (aligned_depth_to_color)
  declare -A AYAR=(
    [varsayilan]="-p depth_module.profile:=640x480x30 -p rgb_camera.profile:=640x480x30"
    [cozunurluk_848]="-p depth_module.profile:=848x480x30 -p rgb_camera.profile:=640x480x30"
    [hw_hiza]="-p depth_module.profile:=640x480x30 -p rgb_camera.profile:=640x480x30 -p align_depth.enable:=true -r /camera/depth/image_rect_raw:=/camera/depth/image_ham -r /camera/depth/camera_info:=/camera/depth/camera_info_ham -r /camera/aligned_depth_to_color/image_raw:=/camera/depth/image_raw -r /camera/aligned_depth_to_color/camera_info:=/camera/depth/camera_info"
    [suzgecli]="-p depth_module.profile:=640x480x30 -p rgb_camera.profile:=640x480x30 -p spatial_filter.enable:=true -p temporal_filter.enable:=true"
    [lazer_yok]="-p depth_module.profile:=640x480x30 -p rgb_camera.profile:=640x480x30 -p depth_module.emitter_enabled:=0"
    [yuksek_dogruluk]="-p depth_module.profile:=640x480x30 -p rgb_camera.profile:=640x480x30 -p depth_module.visual_preset:=3"
    [vlmaps_onerilen]="-p depth_module.profile:=848x480x30 -p rgb_camera.profile:=640x480x30 -p depth_module.visual_preset:=3 -p spatial_filter.enable:=true -p temporal_filter.enable:=true"
    # 27.10 kayıt profili (renk 1280x720@15, derinlik 848x480@15): ön ayar ve süzgeç etkisini ayırmak için
    [p720_temel]="-p depth_module.profile:=848x480x15 -p rgb_camera.profile:=1280x720x15"
    [p720_ha]="-p depth_module.profile:=848x480x15 -p rgb_camera.profile:=1280x720x15 -p depth_module.visual_preset:=3"
    [p720_suzgec]="-p depth_module.profile:=848x480x15 -p rgb_camera.profile:=1280x720x15 -p spatial_filter.enable:=true -p temporal_filter.enable:=true"
    [p720_uzamsal]="-p depth_module.profile:=848x480x15 -p rgb_camera.profile:=1280x720x15 -p depth_module.visual_preset:=3 -p spatial_filter.enable:=true"
    [p720_uzamsal_lazer]="-p depth_module.profile:=848x480x15 -p rgb_camera.profile:=1280x720x15 -p depth_module.visual_preset:=3 -p spatial_filter.enable:=true -p depth_module.laser_power:=360.0"
    [p720_hepsi]="-p depth_module.profile:=848x480x15 -p rgb_camera.profile:=1280x720x15 -p depth_module.visual_preset:=3 -p spatial_filter.enable:=true -p temporal_filter.enable:=true"
  )
  SIRA="${SIRA:-varsayilan cozunurluk_848 hw_hiza suzgecli lazer_yok yuksek_dogruluk vlmaps_onerilen}"
  # ayara özel -r kuralları temel kuraldan önce (ROS 2: ilk eşleşen yeniden adlandırma kuralı geçerli)
  baslat() { echo "(timeout -s INT 60 ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/camera -r __node:=camera $1 -p enable_infra1:=false -p enable_infra2:=false -p pointcloud.enable:=false -r /camera/depth/image_rect_raw:=/camera/depth/image_raw > /tmp/kamera_$2.log 2>&1 &)"; }
  durdur="pkill -INT -f realsense2_camera_nod[e]; sleep 4; pkill -9 -f realsense2_camera_nod[e]"
else
  declare -A AYAR=(
    [varsayilan]=""
    [hw_hiza]="depth_registration:=true"
    [filtre_yok]="enable_soft_filter:=false"
    [ldp_yok]="enable_ldp:=false"
  )
  SIRA="varsayilan hw_hiza filtre_yok ldp_yok"
  baslat() { echo "(timeout -s INT 60 ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=false enable_ir:=false $1 > /tmp/kamera_$2.log 2>&1 &)"; }
  durdur="pkill -INT -f orbbec_camera_node; pkill -INT -f 'dabai.launch'; sleep 4; pkill -9 -f orbbec_camera_node"
fi
ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "orbbec_camer[a]|realsense2_camera_nod[e]" >/dev/null' && { echo "HATA: kamera acik, once kamera_dur.sh"; exit 3; }
for ad in $SIRA; do
  echo "== $ad ${AYAR[$ad]}"
  robot_cmd "$(baslat "${AYAR[$ad]}" $ad); sleep 14; mkdir -p $D
    timeout -s INT $SN ros2 bag record -o $D/$ad --qos-profile-overrides-path /tmp/kamera_qos.yaml /camera/color/image_raw /camera/depth/image_raw /camera/color/camera_info /camera/depth/camera_info /tf_static > /dev/null 2>&1
    $durdur; ros2 bag info $D/$ad 2>/dev/null | grep -E 'Topic: /camera/(color|depth)/image_raw' | sed 's/Serialization.*//'"
done
echo "$D"
