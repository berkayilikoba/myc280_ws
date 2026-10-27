# Görev N4: sahte_algi.py — simde nesne algısı yer tutucusu (YENİ ROS düğümü)
Dosya: src/limo_autonomy/limo_autonomy/sahte_algi.py (yeni) + setup.py'ye DOKUNMA (Claude ekleyecek).
Simde gerçek algı yokken `/kavrama/nesne_algi`'yı üretir (gerçek algı ekibinin arayüzüyle aynı).
- Parametreler: nesne_x, nesne_y, nesne_z (dünya = harita çerçevesi, simde Gazebo dünyası ile aynı), hz=5.0,
  sigma=0.01 (m, xy gürültüsü), menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0 (derece, yatay toplam),
  kayip_olasiligi=0.1, cerceve='base_footprint', tohum=0.
- Robotun GERÇEK pozu `/odom_gercek` (nav_msgs/Odometry; Gazebo planar_move, dünya çerçevesinde, odom_gercek
  başlangıcı dünya orijini ile aynı varsayımı — docstring'e yaz) aboneliğinden. Kamera robot merkezinin 0.084 m
  önünde: görüş testi kamera noktasından yapılır.
- Nesne görüşteyse ve menzildeyse, kayıp olasılığıyla atlayarak, nesnenin base_footprint'teki konumunu (+ gürültü)
  PointStamped olarak yayınla (header.stamp = /odom_gercek mesajının stamp'i). z = nesne_z.
- `/kavrama/nesne_algi_durum` (std_msgs/String) ile 'GORUYOR' / 'GORMUYOR' yayınla (1 Hz).
- Saf yardımcı `gorus_testi(robot_pose, nesne_xy, kamera_ofset, menzil_min, menzil_max, gorus_acisi) -> (bool,
  (x_r, y_r))` ve testi src/limo_autonomy/test/test_sahte_algi.py (önde 0.5 m görünür, arkada görünmez, 2 m uzakta
  görünmez, 40° yanda görünmez (35° yarım açı), çok yakın 0.1 m görünmez).
