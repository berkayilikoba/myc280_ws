# Simülasyon = gerçek (27.10.2026)

Amaç: Gazebo'daki LIMO (diff / FOUR_DIFF, myCobot'lu) gerçek robotla aynı sensör verisini ve kusurlarını üretsin; algı,
görsel odometri ve Nav2 kodu sim ile gerçekte değişmeden çalışsın. Önceki kalibrasyon (IMU gürültüsü, lidar, taban
tepki modeli): [sensor_modeli.md](sensor_modeli.md).

## Kaynak ölçümler

`saha/analiz/sim_param_cikar.py` → `saha/config/sim_gercek_param.yaml` (5 kayıt: 14.10, 21.10, 27.10; poz referansı
lidar ICP `lidar_poz.py`; derinlik gürültüsü `kamera_ayar_testi.sh` + `derinlik_karakter.py`).

| | Gerçek (ölçülen) | Sim (önce) | Sim (şimdi) |
|---|---|---|---|
| Kamera | RealSense D435 | yok (yorum satırı; eski Kinect bloğu) | D435 modeli `limo_d435.xacro` |
| Renk | 1280×720, fx 908.97 (FOV 70.3°), 13.5–13.9 Hz, bozulma 0 | — | 1280×720, fx 908.98, 15 Hz |
| Derinlik | 848×480, fx 429.73 (FOV 89.2°), 16UC1 mm | — | 848×480, fx 429.73, 16UC1 mm, ~12 Hz |
| Derinlik gürültüsü | σ = 0.00176·z² (1–2 m 4.6 mm, 2–4 m 22 mm) | — | aynı (`sensor_modeli`) |
| Derinlik boşlukları | sabit sahnede geçerli %48–57; nesne kenarında gölge şeridi | — | z ∉ [0.28, 10] m, kenar gölgesi 3 px, %2 rastgele |
| Kamera çerçeveleri | camera_link → camera_{color,depth}_frame → *_optical_frame; renk y +0.0148 | camera_link, depth_link | gerçekle aynı |
| Kamera montajı | base_link → camera_link (0.084, 0.0175, 0.028), roll 0.0015, pitch −0.0191 | (0.084, 0, 0.03) | gerçekle aynı |
| Lidar konumu | (0.137, 0.015), yaw −0.1686 (ICP) | (0.105, 0), yaw −0.15 | gerçekle aynı (z 0.08 ölçülmedi) |
| IMU | 92 Hz, jiroskop 0.2°/s nicemli | 83 Hz, nicemsiz | 83 Hz, 0.00349 rad/s nicemli |
| Odom (dönüş) | Δyaw_gerçek/Δyaw_odom = 0.9954 (beş kayıtta 0.992–0.998) | kusursuz (Gazebo konumu) | k_w 0.9954 + σ 0.006 rad/√rad |
| Odom (öteleme) | yol_gerçek/yol_odom = 1.0188 (1.018–1.020) | kusursuz | k_v 1.0188 |
| Konular | /camera/color/image_raw, /camera/depth/image_raw, /imu, /odom, /scan | /limo/... | gerçekle aynı |

Odom bulgusu gerçek robot için de önemli: kapsama turundaki −8…−10° yön kayması neredeyse tamamen bu sabit dönüş
ölçeği hatası (31.5 rad × −0.46 %). Gerçek `odom_duzelt.py`'de yaw artışı 0.9954 ile çarpılırsa büyük kısmı gider
(denenmedi).

## Simde yapı

- `limo_description/urdf/limo_d435.xacro`: D435 gövdesi, çerçeveler, iki Gazebo sensörü (renk → `/camera/color/*`,
  derinlik → `/camera/depth_sim/*` 32FC1).
- `limo_kinematics/sensor_modeli_dugumu.py` (iki süreç, `bolum`):
  - `odom_imu`: `/odom_gercek` (planar_move, Gazebo gerçek konumu) → `/odom` + TF odom→base_footprint (ölçek hatası);
    `/imu_ham` → `/imu` (nicemleme). `odom_hatasi:=false` ile `/odom` = gerçek konum.
  - `derinlik`: `/camera/depth_sim/depth/image_raw` → `/camera/depth/image_raw` (16UC1, gürültü/boşluk).
- `nav_bench.py` gerçek konumu artık `/odom_gercek`'ten okur (yoksa `/odom`).
- **Fast DDS:** büyük görüntüler varsayılan SHM bölümüne (0.5 MB) sığmıyor, UDP'de kayboluyordu (0–0.4 kare/s).
  Simi ve izleyicileri `FASTRTPS_DEFAULT_PROFILES_FILE=src/limo_bringup/config/fastdds_sim_shm.xml` ile çalıştırın
  (16 MB SHM; `ROS_LOCALHOST_ONLY=0`, ayrı `ROS_DOMAIN_ID`).

```bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/limo_bringup/config/fastdds_sim_shm.xml
ros2 launch limo_description gazebo_limo_mycobot.launch.py headless:=true use_rviz:=False
```

## Doğrulama (27.10)

- `test_urdf_consistency.py` 22/22 (diff: ölçülen lidar/kamera konumu, D435 çerçeve ağacı), `test_sensor_modeli.py` 4/4.
- Canlı sim: renk 15 Hz 1280×720, derinlik 12.3 Hz 848×480 16UC1, IMU 83 Hz (0.00349 adım), odom 50 Hz;
  `camera_info` fx gerçekle aynı. Yerinde 15 rad dönüş: odom gerçekten fazla sayıyor (beklenen −3.9°, ölçülen −1.4°;
  fark gürültünün ~2σ'sı).

## Açık / sıradaki

- Tekerlek: taban tepki modeli 24.09 teleop verisiyle; yeni kayıtlardaki `/cmd_vel_smoothed` → `/odom_duz` ile
  yeniden uydurulabilir (Nav2 komutlarıyla daha geniş aralık).
- Lidar z ve kamera x fiziksel olarak ölçülmeli (bkz. olcum_protokolu.md 1.1).
- Renk görüntüsünün pozlama/bulanıklığı (D435 dönüşte bulanıklaşıyor, ≤ 0.2 rad/s) simde yok.
- Sim–gerçek karşılaştırma: aynı komut dizisi (kayıttaki `cmd_vel`) simde oynatılıp (`scripts/sim_ikiz.py`) odom/IMU/
  derinlik istatistikleri karşılaştırılmalı.
