# Gerçek LIMO'ya bağlanma, komut gönderme ve veri kaydı

Robotta (Jetson Orin Nano, Ubuntu 20.04) **ROS 2 Foxy** doğrudan kurulu. PC'de **ROS 2 Humble** var.
İkisi aynı Wi-Fi'da, aynı `ROS_DOMAIN_ID` ile birbirini otomatik bulur. Docker ya da köprü gerekmez.

```
 PC  (Humble, 192.168.0.199)                 LIMO (Foxy, 192.168.0.200?)
 ┌──────────────────────────┐   Wi-Fi     ┌──────────────────────────────┐
 │ ros2 topic pub /cmd_vel  │ ──────────► │ limo_base  (seri port, motor)│
 │ ros2 bag record          │ ◄────────── │ lidar sürücüsü               │
 │ ROS_DOMAIN_ID=42         │  /odom /imu │ ROS_DOMAIN_ID=42             │
 └──────────────────────────┘    /scan    └──────────────────────────────┘
```

**Domain kuralı:** Aynı numarayı kullanan herkes aynı topic'leri görür. Arkadaşımız **77** kullanıyor
ve orada Nav2'si açık kalabiliyor. Nav2 de `/cmd_vel` yayınlıyor. Onunla çakışmamak için biz kendi
numaramızı (örnekte **42**) kullanıyoruz: hem robotta hem PC'de aynı numara olmalı.

## 0. Robotu bul

```bash
ping -c3 192.168.0.200
```

IP değiştiyse robotun ekranından bir terminalde `hostname -I` ile öğrenilir.

## 1. Robotta sürücüyü başlat (SSH)

```bash
ssh KULLANICI@192.168.0.200
```

Robotun içinde:

```bash
source /opt/ros/foxy/setup.bash
source ~/limo_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 node list
```

`ros2 node list` boş olmalı. Boş değilse bu domain'de başka biri çalışıyordur, başka numara seçin.

Sürücüyü başlatın. Launch dosyasının adını bilmiyorsanız önce listeleyin:

```bash
ls ~/limo_ros2_ws/install/limo_bringup/share/limo_bringup/launch/ ~/limo_ros2_ws/install/limo_base/share/limo_base/launch/
```

```bash
ros2 launch limo_bringup limo_start.launch.py
```

Bu terminal açık kalsın; sürücü burada çalışır. Ctrl+C ile durur.

## 2. PC'den bağlantıyı test et

Yeni bir PC terminalinde:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
ros2 topic list
```

`/odom`, `/imu`, `/scan` görünmeli. Veri akıyor mu:

```bash
ros2 topic hz /odom
```

```bash
ros2 topic echo --once /imu
```

`topic list` boşsa: domain numaraları aynı mı, iki makine aynı ağda mı (ping), robottaki sürücü çalışıyor mu?
Topic görünüp veri gelmiyorsa: `ros2 topic info -v /odom` ile QoS uyumsuzluğuna bakın (Foxy ↔ Humble).

## 3. Robota komut gönder (güvenli şekilde)

Önce robotun önünü açın, ilk denemeyi düşük hızda yapın. Komut kesilince LIMO durur. Bu yüzden
komutu saniyede 10 kez gönderiyoruz (`-r 10`) ve **Ctrl+C ile durduruyoruz**:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Yerinde dönme:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
```

Belirli süre sürmek (`timeout` süre dolunca durdurur):

```bash
timeout 10 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

Başka bir yayıncı var mı kontrolü (Nav2 gibi). Burada yalnız sizin `pub` komutunuz görünmeli:

```bash
ros2 topic info -v /cmd_vel
```

## 3b. Kamera (Orbbec DaBai DC1)

`dabai_dcl.py` bu kamerada çalışmıyor (olmayan IMU'yu açmaya çalışıyor). Çalışan komut:

```bash
ros2 launch orbbec_camera dabai.launch.py color_width:=640 color_height:=480 color_fps:=15 depth_width:=640 depth_height:=400 depth_fps:=15 depth_format:=Y11 enable_ir:=false depth_registration:=true
```

Ham görüntü Wi-Fi'dan PC'ye ulaşmıyor (kare ~900 KB, RELIABLE QoS). Görüntülü kayıtları **robotta** alın.

## 4. Veri kaydı

**Kamera ile birlikte kayıt (robotta, iki ayrı süreç).** Tek `ros2 bag record` hem ~21 MB/s görüntü
hem 100 Hz IMU yazınca IMU/odom mesajlarının ~%20'sini kaçırıyor (2026-10-09 ölçümü; kayıt yokken
sürücü kusursuz 50/100 Hz). Hafif ve ağır topic'leri ayrı süreçlerde kaydedin:

```bash
mkdir -p ~/bags/kayit_01 && cd ~/bags/kayit_01
ros2 bag record -o sensor /odom /imu /scan /cmd_vel /tf /tf_static /limo_status &
ros2 bag record -o kamera /camera/color/image_raw /camera/color/camera_info /camera/depth/image_raw /camera/depth/camera_info /tf_static
```

Kamera kaydı dakikada ~1.3 GB tutar; robotun diski ortak, kısa tutun ve işi bitince silin.
PC'ye kopyalamak için (kesilirse kaldığı yerden devam eder):

```bash
rsync -a --partial --info=progress2 agilex@192.168.0.200:~/bags/kayit_01 ~/limo_bags/
```

**Sadece odometri/IMU kaydı** PC'den de alınabilir (kayıt `~/limo_bags` altına):

```bash
mkdir -p ~/limo_bags && cd ~/limo_bags
ros2 bag record -o duz_2m_01 /odom /imu /cmd_vel /scan /tf /tf_static
```

Ayrı terminalden hareketi gönderin, bitince kayıt terminalinde Ctrl+C. Kontrol:

```bash
ros2 bag info ~/limo_bags/duz_2m_01
```

Wi-Fi zayıfsa kayıt mesaj kaçırabilir. `ros2 bag info`'daki mesaj sayısı beklenenden azsa
(`/odom` ≈ 50 Hz × süre), kaydı robotta alıp `scp` ile PC'ye kopyalayın.

### Odometri hata ölçümü için deney seti

Gerçek konumu ölçmek için yerde bant ve şerit metre yeterli. Her deneyi 3–5 kez tekrarlayın.

| Deney | Komut | Ölçülecek |
|---|---|---|
| Düz gidiş | `timeout 10 ... "{linear: {x: 0.2}}"` (≈2 m) | Gerçek mesafe ve yan kayma (metre) ve `/odom` son konumu |
| Yerinde dönüş | `timeout 21 ... "{angular: {z: 0.3}}"` (≈1 tur) | Gerçek açı (bant işaretine göre) ve `/odom` yaw |
| Bekleme | Hiç komut yok, 60 s | IMU yaw kayması (gyro bias) |
| Kare | 4 × (düz 1 m + 90° dönüş) | Başlangıç noktasına göre kapanma hatası |

Her koşu için dosya adını ve elle ölçtüğünüz değeri not edin (ör. `duz_2m_01: 1.93 m, sola 4 cm`).
Simülasyona eklenecek gürültü (ölçek hatası, yaw bias, gyro gürültüsü) bu tablodan çıkarılacak.

## Hızlı sorun giderme

| Belirti | Sebep / çözüm |
|---|---|
| `ping` gitmiyor | Robot kapalı, farklı ağda ya da IP değişmiş |
| `ros2 topic list` boş | `ROS_DOMAIN_ID` farklı, `ROS_LOCALHOST_ONLY=1` kalmış, ya da sürücü çalışmıyor |
| Topic var, veri yok | QoS uyumsuzluğu (`ros2 topic info -v`) veya multicast engelli ağ |
| Robot komutla kısa hareket edip duruyor | Komutu `--once` ile tek sefer gönderdiniz; `-r 10` kullanın |
| Robot kendi kendine hareket ediyor / komut dinlemiyor | Aynı domain'de Nav2 gibi başka bir `/cmd_vel` yayıncısı var (`ros2 topic info -v /cmd_vel`) |

## Kaydı PC'de güvenle oynatma

Bag'de `/cmd_vel` var. Robotla aynı domain'de oynatırsanız kayıttaki hareket komutları **robota gider**.
Her zaman yalıtılmış domain + yalnız bu makine + `/cmd_vel` hariç oynatın:

```bash
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1
ros2 bag play ~/limo_bags/rtab_01 --clock --topics /odom /imu /tf /tf_static /scan /camera/color/image_raw /camera/color/camera_info /camera/depth/image_raw /camera/depth/camera_info
```

`--start-offset` ile ortadan başlatırsanız baştaki `/tf_static` atlanır ve kamera TF'i kaybolur;
baştan oynatın.
