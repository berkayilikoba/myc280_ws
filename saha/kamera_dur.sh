#!/usr/bin/env bash
# Robottaki derinlik kamerasını (Orbbec ya da RealSense) kapatır.
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
ssh -o BatchMode=yes "$ROBOT" 'pkill -INT -f "dabai[.]launch"; pkill -INT -f "realsense2_camera_nod[e]"'; sleep 3
ssh -o BatchMode=yes "$ROBOT" 'pkill -9 -f "realsense2_camera_nod[e]"; pgrep -fa "orbbec_camer[a]|realsense2_camera_nod[e]" || echo "kamera kapali"'
