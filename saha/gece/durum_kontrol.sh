#!/bin/bash
# durum_kontrol.sh: gece çalışmasının genel durumu (15 dk'da bir). Takılı bekleyici, açık sim, ajan görevleri, test ilerlemesi.
WS=$HOME/limo_robot/limo_ws; G=$WS/saha/gece
echo "=== $(date +%H:%M:%S) genel durum"
echo "-- ajan görevleri"; ajan durum 2>&1 | grep -v "^-" | awk 'NR==1 || /calisiyor|hata|kayip/' | cut -c1-120
echo "-- sim / ROS süreçleri"
echo "gzserver: $(ps -eo args | awk '$1=="gzserver"' | wc -l)   ros2 launch: $(ps -eo args | grep -c '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch')"
echo "-- çalışan test betikleri (süre)"
ps -eo pid,etime,args | awk '/bash (engel|tur|gorev)_sim_test\.sh|bash kavrama_sim_test\.sh/ && !/awk/ {print "  "$1, $2, $4, $5, $6}'
echo "-- şüpheli bekleyiciler (pgrep/while döngüsü, 30 dk'dan eski)"
ps -eo pid,etimes,args | awk '$2>1800 && /while (pgrep|ps -eo args)/ && !/awk/ {print "  UYARI", $1, int($2/60)"dk", substr($0, index($0,$3), 90)}'
echo "-- son çıktı dosyaları (son 15 dk)"
find $G/cikti $WS/saha/nav/cikti -maxdepth 2 -newermt "-15 min" -type f 2>/dev/null | sed "s|$WS/||" | head -8
echo "-- son test çıktısı satırları"
for f in $(ls -t $G/cikti_*.log 2>/dev/null | head -2); do echo "  [$(basename $f)] $(grep -vE '^\s*$|INFO' $f | tail -1 | cut -c1-110)"; done
echo "-- disk: $(df -h $WS | awk 'NR==2{print $4" boş"}')"
