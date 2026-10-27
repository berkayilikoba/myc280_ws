# Görev N7: kavrama_dugumu.py — ICP'siz, nesneye göreli son yaklaşmalı ve kurtarmalı kavrama durum makinesi
Dosyalar: src/limo_autonomy/limo_autonomy/kavrama_dugumu.py (yeniden düzenle),
src/limo_bringup/behavior_trees/navigate_to_pose_hassas.xml (YENİ). Önce şunları OKU: kavrama_dugumu.py (mevcut),
son_yaklasma.py (N2, yeni denetleyici), kavrama_yaklasim.py (N3), behavior_trees/navigate_to_pose_diff.xml.

1. BT navigate_to_pose_hassas.xml: navigate_to_pose_diff.xml'in kopyası, FollowPath düğümüne
   goal_checker_id="hassas_hedef" (nav2_diff.yaml'da tanımlı: xy 0.08, yaw 0.10). Kurtarma ağacında yerel costmap
   temizleme kalsın; Spin kurtarmasını KALDIR (nesne yanında dönmek kolu çarptırabilir), BackUp 0.15 m kalsın.
2. Düğüm akışı (durumlar `/kavrama/durum` String ile, her biri 'DURUM ayrıntı' biçiminde):
   PLAN → NAV_Q → SON_YAKLASMA → HAZIR  |  hata → KURTARMA → (sonraki aday) NAV_Q ... | HATA
   - PLAN: yaklasim_bul(harita, nesne_map, ayar, robot, hepsi=True) → en iyi 3 aday.
   - NAV_Q: NavigateToPose(Q, behavior_tree = navigate_to_pose_hassas.xml mutlak yolu; parametre `bt_hassas`,
     varsayılan get_package_share_directory('limo_bringup')/behavior_trees/navigate_to_pose_hassas.xml).
   - SON_YAKLASMA: N2 SonYaklasma. Yaklaşma yönü (odom): adayın θ'sı map->odom TF ile odom'a çevrilir. Nesne
     ölçümü: `/kavrama/nesne_algi` (PointStamped, TF ile odom'a). Ölçüm yoksa başlangıçta map'teki nesne konumu
     map->odom TF ile odom'a çevrilip TEK ölçüm olarak verilir (algısız yedek; durum mesajında 'algisiz' yaz).
     Güvenlik mesafesi: `/scan`'den robotun önündeki ±25° koni içinde, nesnenin tahmini konumunun 0.08 m
     çevresi hariç en yakın nokta, lidar çerçevesi TF ile base_footprint'e çevrilerek, ön tampon (0.161 m)
     çıkarılarak. 50 Hz denetim döngüsü (timer, MultiThreadedExecutor / ReentrantCallbackGroup; mevcut kod
     thread+sleep kullanıyorsa temiz biçimde dönüştür). cmd: `/cmd_vel` (parametre `cmd_konu`).
   - HAZIR: `/kavrama/nesne_robotta` (PointStamped base_footprint) = N2 nesne_robotta_son. Mesajda P hatası
     (tahmini) ve son yaklaşma süresi.
   - KURTARMA: son yaklaşma HATA verirse: BackUp benzeri — odomla düz 0.20 m geri (v -0.08), sonra sonraki adayla
     NAV_Q. En çok 2 kurtarma; sonra 'HATA tüm adaylar denendi'.
   - İptal: `/kavrama/iptal` (std_msgs/Empty) → Nav2 hedefini iptal et, dur, 'HATA iptal'.
3. ICP kodunu (icp_poz) ve lidar-harita bağımlılığını kaldır; harita yalnız yaklasim_bul için (statik map yaml,
   mevcut parametre).
4. Saf yardımcıları (guvenlik_mesafesi(scan_noktalari_robotta, nesne_robotta, koni, haric_r, tampon) vb.) modül
   düzeyinde yaz ve src/limo_autonomy/test/test_kavrama_dugumu.py'de test et (ROS gerektirmeyen kısımlar).
