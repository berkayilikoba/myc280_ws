# Gece görevi F3: simde hareketli engel (yürüyen insan) + Nav2 tepki ölçümü
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude simde koşturacak.
Dosyalar (yeni): src/limo_bringup/scripts/dinamik_engel.py, src/limo_bringup/scripts/dinamik_engel_senaryolar.yaml,
src/limo_car/models/insan_silindir/{model.config,model.sdf}. Önce OKU: src/limo_bringup/scripts/nav_bench.py
(ölçüm kalıbı, /odom_gercek kullanımı, açıklık hesabı), src/limo_bringup/param/nav2_diff.yaml (collision_monitor,
local_costmap voxel_layer + obstacle_layer).
1. insan_silindir modeli: statik OLMAYAN ama yerçekimsiz/kinematik hareket için `<static>false</static>` +
   `<gravity>false</gravity>` link, silindir r 0.20 h 1.70 (lidar 8 cm'de ve D435'te görünsün), malzeme ambient/diffuse
   0.2 0.3 0.8 (ışıksız dünyada ambient önemli).
2. dinamik_engel.py (rclpy, use_sim_time): parametre/argüman --senaryo AD --senaryolar YAML.
   Engeli /spawn_entity ile (gazebo_msgs/srv/SpawnEntity, xml model.sdf'den) doğurur, /gazebo/set_entity_state
   (gazebo_msgs/srv/SetEntityState) ile 20 Hz'de yol noktaları arasında sabit hızla doğrusal hareket ettirir (yaw
   yürüyüş yönüne), bitince ya da Ctrl-C'de /delete_entity ile siler.
   Aynı anda robota NavigateToPose hedefi gönderir (senaryodaki robot_hedef) ve ölçer: hedefe ulaştı mı, süre,
   robot–engel en küçük merkez mesafesi ve en küçük ayak izi–silindir yüzeyi açıklığı (robot ayak izi ±0.161 x
   ±0.11, /odom_gercek'ten; engel konumu kendi bildiği), collision_monitor durdurma sayısı (state konusu
   `collision_monitor_state`, nav2_msgs/msg/CollisionMonitorState varsa; yoksa cmd_vel_smoothed≠0 iken cmd_vel=0
   anlarını say), çarpışma (açıklık < 0) var mı, Nav2 kurtarma sayısı (bt_navigator geri bildirimi
   number_of_recoveries).
   Çıktı: --cikti YAML (senaryo başına ölçütler) + terminal tablosu. Senaryo bitiş ölçütü: Nav2 sonucu ya da
   --zaman_asimi (vars. 120 s).
3. dinamik_engel_senaryolar.yaml (lab_oda haritası, harita=dünya; robot (0,0)'dan başlar): 
   - karsidan: robot_hedef (3.0, 0.3, 0°); engel yol [(4.5, 0.3), (0.5, 0.3)] 0.6 m/s (robota doğru yürür).
   - yandan_kesen: robot_hedef (3.0, 0.3, 0°); engel [(1.5, 1.5), (1.5, -0.6)] 0.8 m/s, başlama gecikmesi 2 s.
   - duran_engel: robot_hedef (3.0, 0.3, 0°); engel [(1.6, 0.3)] sabit (koridoru kapatır; etrafından dolanmalı).
   - takip: robot_hedef (4.6, 0.8, 0°); engel robotun 1 m önünde aynı yönde 0.25 m/s yürür [(1.0, 0.0),(4.0, 0.0)].
4. Kod üslubu nav_bench.py ile uyumlu; ROS'suz saf yardımcılar (doğrusal yörünge enterpolasyonu, ayak izi–daire
   açıklığı, durdurma sayacı) ve testleri src/limo_bringup/scripts/test_dinamik_engel.py (pytest, ROS'suz).
