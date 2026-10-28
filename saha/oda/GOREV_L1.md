# Görev L1: renk_bozu'yu simdeki sensör modeli düğümüne bağla

Dosyalar (yalnız bunlar):
- src/limo_kinematics/limo_kinematics/sensor_modeli_dugumu.py
- src/limo_description/urdf/limo_d435.xacro
renk_bozu.py'yi (aynı paket: limo_kinematics.renk_bozu) OKU, DEĞİŞTİRME.

1. limo_d435.xacro, renk sensörü (d435_renk): eklentideki <camera_name>color</camera_name> -> color_sim
   (Gazebo artık /camera/color_sim/image_raw ve /camera/color_sim/camera_info yayınlar). Sensördeki Gazebo
   gaussian <noise> bloğunu kaldır (gürültüyü artık renk_bozu veriyor). Dosya başındaki yorumda konu
   açıklamasını güncelle (renk de derinlik gibi sensor_modeli'nden geçer). Başka bir şeyi değiştirme.
2. sensor_modeli_dugumu.py:
   - Yeni bolum değeri 'renk' (ve 'hepsi' de kapsasın): /camera/color_sim/image_raw (Image, rgb8 veya bgr8) ->
     renk_bozu.renk_boz(...) -> /camera/color/image_raw (aynı encoding, aynı header) ; /camera/color_sim/camera_info
     -> /camera/color/camera_info aynen (header korunur). QoS: qos_profile_sensor_data.
   - Hız: renk_boz w_z ve v_x ister. Bu süreçte /odom_gercek'e (Odometry) abone ol, son twist'i kullan.
   - Parametreler: 'renk_bozu' (bool, vars. True; False ise görüntü aynen geçer), 'renk_param_dosyasi' (str, vars.
     ''; boşsa renk_bozu.VARSAYILAN_PARAMETRELER ya da modüldeki varsayılan sözlük — adını renk_bozu.py'den oku;
     doluysa YAML yüklenip varsayılanın üstüne yazılır), 'renk_tohum' (int, vars. 2).
   - renk_boz çağrısı RGB/BGR'den bağımsız olmalı: beyaz denge kazancı BGR sırasında; encoding 'rgb8' ise kazancı
     ters çevir (ya da görüntüyü BGR'ye çevirip geri çevir). Çıkış uint8, aynı boyut.
   - Kare atlamamak için iş tek abonelikte; performans: 1280x720 @ 15 Hz hedef. np.random.Generator'ı bir kez
     oluştur. Gürültü için float32 kullan.
   - Modül docstring'ine 'renk' satırı ekle; başlangıç log satırı (diğer bölümler gibi) parametreleri yazsın.
   - Mevcut odom_imu ve derinlik davranışına DOKUNMA.
3. Saf yardımcı: `renk_mesaj_boz(veri: np.ndarray, encoding: str, param, w_z, v_x, rng) -> np.ndarray` (ROS'suz)
   düğüm içinde kullanılsın; testi Claude yazacak.
Kodu çalıştıramazsın; Claude derleyip simde test edecek. Kısa özetle bitir.
