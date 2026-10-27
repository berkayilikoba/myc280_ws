# Gece görevi G2 (uzun): kavrama/navigasyon görselleştirme düğümü (RViz MarkerArray) + RViz yapılandırması
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude sabah derleyip test edecek.
kavrama_dugumu.py ve kavrama_yaklasim.py'yi OKU ama DEĞİŞTİRME (başka görev düzenliyor).
YENİ dosyalar: src/limo_autonomy/limo_autonomy/kavrama_gorsel.py (düğüm + saf yardımcılar),
src/limo_autonomy/test/test_kavrama_gorsel.py, src/limo_autonomy/config/kavrama.rviz.
1. Düğüm abonelikleri: /kavrama/plan (PoseArray map: [Q, P] — kavrama_dugumu yayınlıyor; kontrol et),
   /kavrama/durum (String), /kavrama/nesne_algi (PointStamped), /kavrama/hedef (PointStamped map),
   /kavrama/nesne_robotta (PointStamped base_footprint), /scan (yalnız son yaklaşma güvenlik konisini çizmek için).
2. /kavrama/isaretler (visualization_msgs/MarkerArray) yayını, 5 Hz, her işaretin ns'i ayrı:
   'nesne' (silindir, r=0.03, hedefteki z yüksekliğinde; algı varsa algı konumu yeşil, yoksa hedef turuncu),
   'Q' ve 'P' (ok, yön θ; Q mavi, P yeşil), 'koridor' (Q→P arası, ayak izi genişliği 0.22 + 2*0.05 şerit, yarı
   saydam), 'ayak_izi_P' (P'de robot ayak izi dikdörtgeni ±0.161 x ±0.11), 'kavrama_noktasi' (P'de robot önünde
   d_kavrama=0.22'deki nokta, küçük küre — nesneyle çakışmalı), 'durum' (TEXT_VIEW_FACING, robotun 0.5 m
   üstünde son durum metni; HATA kırmızı, HAZIR yeşil), 'guvenlik_konisi' (base_footprint'te ±25° koni, 0.5 m,
   SON_YAKLASMA sırasında görünür). Durum PLAN'a dönünce eski işaretleri sil (DELETEALL).
3. Saf yardımcılar (marker üretimini ROS mesajı olmadan sözlük/numpy düzeyinde test edilebilir yap ya da mesaj
   nesnelerini oluşturup alanlarını test et): koridor köşeleri, ayak izi köşeleri, kavrama noktası, renk seçimi.
4. kavrama.rviz: Fixed Frame map; Map (/map), LaserScan (/scan), PointCloud2 (/camera/depth/points, voxel), local
   costmap, global plan (/plan), MarkerArray (/kavrama/isaretler), RobotModel, TF (yalnız base_footprint, map,
   odom), Image (/camera/color/image_raw) paneli.
5. src/limo_autonomy/launch/kavrama.launch.py'yi OKU; `gorsel:=true` argümanı ile kavrama_gorsel düğümünü ve
   `rviz:=false` (vars.) ile rviz2 -d kavrama.rviz başlatan değişikliği YAPMA — bunun yerine yanıtında Claude'un
   ekleyeceği tam launch kod parçasını ver (dosya başka görevin değil ama setup.py giriş noktası da Claude'da).
Yanıtın sonunda ajan rapor JSON bloğunu unutma.
