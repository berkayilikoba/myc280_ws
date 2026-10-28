# Gece görevi F6: hızla yaklaşan engelde çarpışma — ölçümde kusur ayrımı + collision_monitor/costmap iyileştirmesi
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude simde tekrar koşturacak.
Ölçüm (Claude, lab_oda_3b, dinamik_engel.py): karsidan (insan 0.6 m/s robota doğru) SUCCEEDED ama min açıklık
-0.015 m ÇARPIŞMA, durdurma 1, kurtarma 3; yandan_kesen (0.8 m/s) -0.068 m ÇARPIŞMA, durdurma 9, kurtarma 15;
duran_engel ve takip sorunsuz (0.18 / 0.46 m). Not: insan set_entity_state ile kinematik itiliyor, yol vermiyor.
Dosyalar: src/limo_bringup/scripts/dinamik_engel.py (+ test_dinamik_engel.py'ye ekle),
src/limo_bringup/param/nav2_diff.yaml. Önce ikisini de TAMAMEN OKU.
A) dinamik_engel.py kusur ayrımı: açıklık < 0.05 m olduğu HER anda robot gerçek hızının (odom_gercek twist,
   dünya çerçevesine çevrilmiş) robot→engel doğrultusundaki bileşeni v_yaklasma ve robot hızının büyüklüğü
   kaydedilsin. Çarpışma sınıfı: 'robot_carpti' (temas anında v_yaklasma > 0.05 m/s), 'engel_carpti' (robot
   |v| < 0.03 ve v_yaklasma ≤ 0.05), 'karisik'. Ayrıca ilk temastan önceki 2 s'de robot durmuş muydu (durma_suresi_s).
   Tabloya 'Sınıf' ve 'Yaklaşma v (m/s)' sütunları, yaml'a alanlar.
B) nav2_diff.yaml collision_monitor (Humble alanları: polygons listesi; her poligon type: polygon|circle,
   points ya da radius, action_type: stop|slowdown|approach, slowdown_ratio (slowdown için), time_before_collision
   (approach için), simulation_time_step, max_points, visualize, polygon_pub_topic, enabled; ayrıca üst düzey
   base_frame_id, odom_frame_id, cmd_vel_in_topic, cmd_vel_out_topic, transform_tolerance, source_timeout,
   stop_pub_timeout, observation_sources). Mevcut FootprintApproach'u KORU; ekle:
   - "YavasBolge": circle, radius 0.55, action_type slowdown, slowdown_ratio 0.4, max_points 3
   - "DurBolge": polygon, points ayak izi + 0.06 m ([[0.221,0.17],[0.221,-0.17],[-0.221,-0.17],[-0.221,0.17]]),
     action_type stop, max_points 3
   polygons sırası: ["DurBolge", "YavasBolge", "FootprintApproach"]. visualize true, polygon_pub_topic adları
   poligon adıyla. Humble'da bu alanlar yoksa UYDURMA; emin olmadığını yorumla belirt.
C) local_costmap: update_frequency ve publish_frequency mevcut değerleri 10.0'dan küçükse 10.0 yap (hızlı
   engel), obstacle_layer ve voxel_layer'da observation_persistence 0.0 (yoksa ekleme), raytrace (clearing) açık
   olduğunu doğrula. Hareketli engelin arkasında iz kalmaması için gerekçeyi yorumla.
D) Dosya sonundaki değişiklik bloğuna F6 bölümü.
