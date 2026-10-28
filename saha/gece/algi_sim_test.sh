#!/bin/bash
# algi_sim_test.sh [cikti_dizini]: 3B odada kameradan renk algısıyla (renk_algi) uçtan uca kavrama doğrulaması.
# Akış: Sim + Nav2/AMCL -> ilk poz -> elma modelini Gazebo'ya doğur -> kavrama.launch (algi:=renk)
#       -> 10 s algı durumu denetimi -> /kavra eylemi -> Gazebo gerçeği ve algı hatası raporu -> temizlik.
WS=$HOME/limo_robot/limo_ws
OUT=${1:-$WS/saha/gece/cikti/algi_$(date +%H%M%S)}
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91}
export ROS_LOCALHOST_ONLY=0
export GAZEBO_MASTER_URI=http://localhost:11391
export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/src/limo_bringup/config/fastdds_sim_shm.xml
SH=$WS/install/limo_bringup/share/limo_bringup
HARITA=$SH/maps/lab_oda.yaml

ros2 daemon stop >/dev/null 2>&1   # bayat daemon CLI keşfini bozuyordu
fastdds shm clean >/dev/null 2>&1   # kill -9 sonrası zombi SHM segmentleri DDS keşfini bozuyordu (yalnız kullanılmayanları siler)

PIDS=()
temizle() {
  echo "== Temizleniyor..."
  for p in "${PIDS[@]}"; do kill -- -$p 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -9 -- -$p 2>/dev/null; done
  ps -eo pid,args | awk '$2 ~ /^gzserver/ {print $1}' | xargs -r kill -9
}
trap temizle EXIT

echo "== 1) Gazebo simülasyonu ve Nav2 başlatılıyor..."
setsid ros2 launch limo_bringup limo_sim_nav2.launch.py headless:=true use_rviz:=false localization:=amcl \
  map:=$HARITA world:=$WS/install/limo_car/share/limo_car/worlds/lab_oda_3b.world > "$OUT/sim.log" 2>&1 &
PIDS+=($!)

# Sıra önemli (kilitlenme): AMCL başlangıç pozu gelmeden map->odom yayınlamaz; Nav2 planner etkinleşmesi bu TF'i
# bekler. Önce AMCL'i bekle, pozu ver, SONRA bt_navigator'ı bekle (bekleme bekle_aktif.py: get_state servisi).
echo "== AMCL bekleniyor..."
python3 $WS/saha/gece/bekle_aktif.py /amcl 180

echo "== Başlangıç pozu veriliyor..."
python3 "$WS/saha/gece/ilk_poz.py" 0 0 0 || exit 1

echo "== bt_navigator bekleniyor..."
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 180
python3 $WS/saha/gece/bekle_aktif.py /bt_navigator 5 || { echo "Nav2 açılmadı"; exit 1; }
echo "== Nav2 hazır."

# 2) Nav2 hazır olduktan sonra elma modelini gazebo'ya doğur
echo "== 2) Elma modeli Gazebo'ya doğuruluyor (-x 5.62 -y 0.8 -z 0.04)..."
ros2 run gazebo_ros spawn_entity.py -entity elma \
  -file "$WS/install/limo_car/share/limo_car/models/elma/model.sdf" \
  -x 5.62 -y 0.8 -z 0.04 > "$OUT/spawn_elma.log" 2>&1
sleep 2

# 3) kavrama.launch use_sim_time:=true algi:=renk hata_ayiklama:=true
echo "== 3) kavrama.launch (algi:=renk, hata_ayiklama:=true) başlatılıyor..."
setsid ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true algi:=renk hata_ayiklama:=true \
  > "$OUT/kavrama.log" 2>&1 &
KAVRAMA_PID=$!
PIDS+=($KAVRAMA_PID)
sleep 6

# 4) 10 s içinde /kavrama/nesne_algi_durum 'GORUYOR' gelmezse bunu raporla ama devam et
echo "== 4) /kavrama/nesne_algi_durum kontrolü (10 s)..."
python3 - > "$OUT/durum_kontrol.log" 2>&1 <<'PY'
import time, rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('durum_kontrol_test')
goruyor = False

def cb(msg):
    global goruyor
    if msg.data == 'GORUYOR':
        goruyor = True

sub = node.create_subscription(String, '/kavrama/nesne_algi_durum', cb, 10)
t0 = time.time()
while time.time() - t0 < 10.0 and not goruyor:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()

if goruyor:
    print("DURUM: GORUYOR (robot başlangıçta elmayı gördü)")
else:
    print("DURUM: 10 s içinde GORUYOR gelmedi (robot başlangıçta elmayı görmeyebilir — algısız yedekle Q'ya gider, son yaklaşmada görür)")
PY
cat "$OUT/durum_kontrol.log"

# 5) Algı ve odom izleyiciyi arka planda başlat
echo "== 5) Algı kaydedici başlatılıyor..."
python3 "$WS/saha/gece/algi_kaydedici.py" "$OUT/algi_verileri.json" > "$OUT/kaydedici.log" 2>&1 &
RECORDER_PID=$!
PIDS+=($RECORDER_PID)
sleep 2

# 6) /kavra eylemini gönder
echo "== 6) /kavra eylemi gönderiliyor..."
timeout 400 ros2 action send_goal /kavra limo_interfaces/action/Kavra \
  "{nesne: {header: {frame_id: map}, point: {x: 5.62, y: 0.8, z: 0.04}}, yon_derece: .nan, algisiz_izinli: true}" --feedback \
  > "$OUT/eylem.log" 2>&1
EYLEM_STATUS=$?
echo "Eylem tamamlandı (çıkış kodu: $EYLEM_STATUS)"
sleep 3

# Kaydediciyi kapat ve verileri diske yazdır
if [ -n "$RECORDER_PID" ]; then
  kill -INT "$RECORDER_PID" 2>/dev/null
  wait "$RECORDER_PID" 2>/dev/null
fi

# 7) Sonuçları raporla
echo "== 7) Test sonuçları analiz ediliyor..."
python3 "$WS/saha/gece/algi_kaydedici.py" --rapor "$OUT" | tee "$OUT/ozet.txt"

echo "Çıktı dizini: $OUT"
