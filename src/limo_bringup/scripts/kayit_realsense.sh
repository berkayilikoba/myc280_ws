#!/usr/bin/env bash
# Gerçek LIMO'da (Foxy) RealSense D435 + sensör kaydı; robotun üzerinde çalıştırılır (ham görüntü Wi-Fi'dan geçmez).
# İki bag yazar: <dizin>/kamera (renk + ham derinlik, ~46 MB/s) ve <dizin>/sensor (odom, imu, scan, EKF, tf ...).
# Ctrl+C / SIGINT ile ikisi de kapanır.
#
# Kamera hizalamasız başlatılmalı: align_depth.enable:=true Orin Nano'da düğümü tek çekirdeğe kilitliyor,
# renk 30 yerine ~12 Hz yayınlanıyor (15.10.2026 ölçümü). Hizalama kayıttan sonra
# /camera/depth/camera_info + /camera/color/camera_info + /camera/extrinsics/depth_to_color ile yapılır.
#   ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 \
#     depth_module.profile:=640x480x30 align_depth.enable:=false pointcloud.enable:=false
#
# Kullanım: kayit_realsense.sh <dizin>
set -u
D=${1:?kullanım: kayit_realsense.sh <dizin>}
mkdir -p "$D"
CAM_INFO="/camera/color/camera_info /camera/depth/camera_info /camera/extrinsics/depth_to_color"

ros2 bag record -o "$D/kamera" /camera/color/image_raw /camera/depth/image_rect_raw $CAM_INFO \
  > "$D/kamera.log" 2>&1 &
K=$!
ros2 bag record -o "$D/sensor" /odom /imu /scan /odometry/filtered /cmd_vel /tf /tf_static /limo_status \
  $CAM_INFO > "$D/sensor.log" 2>&1 &
S=$!
trap 'kill -INT $K $S 2>/dev/null' INT TERM
wait
