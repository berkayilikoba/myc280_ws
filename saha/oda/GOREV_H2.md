# Görev H2: oda_dogrula.py'ye lidar karşılaştırması ekle (saha/oda/oda_dogrula.py)

Mevcut betik her seçilen karede robotu /gazebo/set_entity_state ile ışınlayıp kamera görüntülerini alıyor
(teleport_and_capture). Buna /scan karşılaştırması ekle; mevcut kamera ölçütleri ve çıktıları değişmesin.

1. Abonelik: /scan (sensor_msgs/LaserScan, QoS sensor_data / best effort). Işınlamadan ve mevcut beklemeden sonra
   gelen İLK yeni taramayı al (ışınlamadan önceki tarama kullanılmamalı: mesaj başlık zamanı >= ışınlama sonrası
   sim zamanı olsun).
2. Referans: 2B doluluk haritası (--harita, varsayılan ~/limo_robot/limo_ws/src/limo_bringup/maps/lab_oda.yaml;
   YAML'de image, resolution, origin [x,y,yaw], negate, occupied_thresh; pgm'i cv2 ile oku; doluluk = piksel
   değeri (negate'e dikkat) occupied_thresh'i geçen). Haritada ışın izleme (adım = resolution/2, max range =
   scan.range_max) ile her lidar ışını için beklenen menzil.
3. Lidar pozu: ışınlanan gövde pozu (harita çerçevesi; dünya = harita) ∘ base_link->laser sabiti:
   (0.137, 0.015), yaw -0.1686 rad. CLI ile değiştirilebilir: --lidar_x --lidar_y --lidar_yaw.
4. Kare başına ölçütler (yalnız iki tarafın da sonlu ve range_min..range_max içinde olduğu ışınlar):
   lidar_medyan_fark_m (sim - harita, işaretli), lidar_mutlak_medyan_m, lidar_5cm_ici_oran, lidar_20cm_disi_oran,
   lidar_gecerli_oran (sim taramasında sonlu ışın oranı). ozet.yaml'da kare kayıtlarına ve ortalamaya "lidar"
   anahtarıyla ekle.
5. Görsel: her kare için ayrıca lidar_<i>.png: açıya göre sim menzili ve harita menzili (matplotlib Agg) ya da
   üstten görünüm. Basit tut.
6. --lidar_yok bayrağı ile bu bölüm tamamen kapatılabilsin. /scan hiç gelmezse uyarı verip kamera ölçütlerine
   devam et (çökme yok).

Kısıt: yeni bağımlılık yok (rclpy, numpy, cv2, yaml, matplotlib). Kodu çalıştıramazsın; Claude test edecek.
Kısa özetle bitir.
