# İnceleme R1: hareketli_kalkan.py bağımsız kod incelemesi (yalnız oku, KOD DEĞİŞTİRME)
ÖNEMLİ: terminal iznin YOK. Yalnız saha/gece/R1_RAPOR.md dosyasını yaz.
Dosyalar: src/limo_autonomy/limo_autonomy/hareketli_kalkan.py, src/limo_autonomy/test/test_hareketli_kalkan.py,
src/limo_bringup/launch/limo_nav2.launch.py (kalkan bağlantısı), src/limo_bringup/param/nav2_diff.yaml (collision_monitor).
Bağlam: hız zinciri velocity_smoother → cmd_vel_smoothed → hareketli_kalkan → cmd_vel_kalkan → collision_monitor →
cmd_vel. ROS 2 Humble, use_sim_time simde true. Tepkisiz insan simde 0.6–0.8 m/s; durağan mobilya ASLA tetiklememeli.
Ara: (1) gerçek hatalar (zaman birimi/saat karışıklığı sim vs duvar, çerçeve dönüşümü, yaw wrap, eşleştirme
mantığı, iz silme, bölme sıfır, NaN/inf lidar değerleri), (2) güvenlik açıkları (kalkan çökerse/taraması
eskirse cmd_vel ne olur — zincir kopar mı, robot durur mu?), (3) sahte tetikleme kaynakları, (4) testlerin
kaçırdığı durumlar. Her bulgu için: dosya:satır, kanıt (ilgili satırı alıntıla), somut senaryo, önerilen düzeltme,
önem (yüksek/orta/düşük). Emin olmadığını "doğrulanmalı" diye işaretle; uydurma API/parametre yazma.
