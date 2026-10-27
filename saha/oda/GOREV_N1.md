# Görev N1: sim_kabul.py — simin gerçek robotla aynı arayüzü verdiğini tek komutla doğrula (YENİ dosya)

Dosya: saha/oda/sim_kabul.py (yeni; başka dosyaya dokunma). ROS 2 Humble, rclpy, tf2_ros. Sim zaten çalışıyorken
(saha/oda/sim_ac.sh <dünya>) çalıştırılır; simi kendisi BAŞLATMAZ.

CLI: python3 sim_kabul.py [--sure 15] [--cikti rapor.yaml] [--gorsel kare.png]

Kontroller (süre boyunca dinle, sonra değerlendir):
1. Konu hızları (QoS: görüntü/scan/imu için sensor_data, odom için varsayılan güvenilir) — beklenen aralık:
   /scan (LaserScan) 8–15 Hz, /imu (Imu) 70–100 Hz, /odom (Odometry) 40–60 Hz,
   /camera/color/image_raw (Image) 13–16 Hz, /camera/color/camera_info 13–16 Hz,
   /camera/depth/image_raw (Image, 16UC1) 11–16 Hz, /camera/depth/camera_info 11–16 Hz.
   Her biri için ölçülen Hz, mesaj sayısı, GEÇTİ/KALDI.
2. İçerik: renk görüntüsü 1280x720 rgb8/bgr8; derinlik 848x480 16UC1 ve geçerli (>0) piksel oranı 0.3–0.95;
   /scan sonlu ölçüm oranı 0.85–0.995 (gerçek %96.7); /imu ivme z ortalaması 9.6–10.3 (gerçek 10.04);
   camera_info fx: renk 900–918, derinlik 425–435.
3. TF: map yoksa odom kökü kabul; şu dönüşümler bulunmalı (tf2 buffer, 2 s zaman aşımı):
   odom->base_footprint, base_footprint->base_link (ya da base_link doğrudan), base_link->laser (ya da laser_link;
   hangi ad varsa — ikisini de dene, bulunanı yaz), base_link->camera_link, camera_link->camera_color_optical_frame,
   camera_link->camera_depth_optical_frame, base_link->imu_link (bulunamazsa UYARI, KALDI değil).
   base_link->camera_link ötelemesi (0.084, 0.0175, 0.028) ±1 cm olmalı (gerçek montaj).
4. Zaman: use_sim_time'lı mesajların header.stamp'i /clock ile tutarlı (fark < 0.5 s).
5. Çıktı: terminalde hizalı tablo + en sonda "SONUÇ: GEÇTİ (n/n)" ya da "SONUÇ: KALDI (k başarısız)"; çıkış kodu
   0/1. --cikti ile YAML rapor. --gorsel verilirse son renk karesi ve derinliğin renkli haritası yan yana PNG.
Düğüm use_sim_time=True ile oluşturulsun. Kodu çalıştıramazsın; Claude simde test edecek. Kısa özetle bitir.
