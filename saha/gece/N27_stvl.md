# N27 — Costmap'te VoxelLayer yerine STVL (spatio_temporal_voxel_layer)

## Bağlam (teşhis Claude'da, ölçülmüş)
bati_dar hedefinde (0.27 m dar geçit) robot ~40–50 s takılıyor: yerel costmap'teki VoxelLayer (D435 derinlik) mobilyanın
üst kısmını (oturak/tabla) görüyor, küresel costmap yalnız lidar (ayaklar) görüyor → küresel planlayıcı yolu mobilyanın
yanından çiziyor, yerel takılıyor. Küresel costmap'e VoxelLayer eklemek kalıcı izlerle turu bozdu (4/7, REDDEDİLDİ).
Çözüm: zamanla sönen 3B katman STVL, hem yerel hem küresel costmap'te. Paket kurulu: ros-humble-spatio-temporal-voxel-layer 2.3.4.
Örnek yapılandırma (OKU): /opt/ros/humble/share/spatio_temporal_voxel_layer/example/standard_indoor_environment_config.yaml
Eklenti sınıf adı: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer".

## Caveman araçları (ZORUNLU, bol kullan)
Bu oturumda `caveman` MCP sunucusu bağlı. Şu araçları kullan:
- Büyük dosya içeriğini (nav2_diff.yaml ~560 satır, örnek yapılandırma) bağlama almadan önce `caveman_compress` ile sıkıştır;
  bir ayrıntı kaybolursa `caveman_retrieve` ile geri al. Düzenleyeceğin satırları her zaman özgün dosyadan birebir oku.
- İş sonunda `caveman_stats` çağır ve çıktısını raporuna ekle (kaç çağrı, token önce/sonra).

## Yapılacak — YALNIZ src/limo_bringup/param/nav2_diff.yaml
1. local_costmap: `voxel_layer` girdisini (plugins listesinde aynı ad ve sıra kalsın) STVL'ye çevir:
   plugin "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"; enabled true; voxel_decay 3.0; decay_model 0;
   voxel_size 0.05; track_unknown_space false; mark_threshold 0; update_footprint_enabled true; combination_method 1;
   origin_z 0.0; publish_voxel_map false; transform_tolerance 0.5; mapping_mode false;
   observation_sources "depth_mark depth_clear".
   depth_mark: data_type PointCloud2, topic /camera/depth/points, marking true, clearing false, obstacle_range 2.0,
     min_obstacle_height 0.05, max_obstacle_height 0.50, expected_update_rate 0.0, observation_persistence 0.0,
     inf_is_valid false, filter "voxel", voxel_min_points 0, clear_after_reading true.
   depth_clear: data_type PointCloud2, topic /camera/depth/points, marking false, clearing true, max_z 2.5, min_z 0.1,
     vertical_fov_angle 1.0, horizontal_fov_angle 1.52, decay_acceleration 5.0, model_type 0.
   Eski VoxelLayer'a özgü anahtarları (z_resolution, z_voxels, raytrace_*, obstacle_max_range, publish_voxel_map tekrarı vb.) kaldır.
   Mevcut Türkçe gerekçe yorumlarını (zemin gürültüsü 0.05, masa altı 0.50) koru, STVL'ye uyarla.
2. global_costmap: plugins listesini ["static_layer", "obstacle_layer", "stvl_layer", "inflation_layer"] yap ve
   `stvl_layer` ekle: yereldekiyle aynı ama voxel_decay 15.0, track_unknown_space true. inflation_layer en sonda kalmalı.
   "VoxelLayer global costmap'e eklenmemiştir" yorumunu güncelle: STVL zamanla söndüğü için kalıcı iz sorunu yok (gerekçe: bati_dar).
3. Dosya sonundaki değişiklik notları bölümüne (yorum satırları) kısa bir "N27 (28.10): VoxelLayer → STVL" girdisi ekle;
   eski notları silme. SAYI UYDURMA: ölçüm sonucu yazma, sonuçları Claude simde ölçecek.
4. collision_monitor'daki depth kaynağına DOKUNMA. Başka dosyaya DOKUNMA.

## Kurallar
- Komut çalıştırma iznin yok (terminal kapalı). Derleme/sim/test Claude'da.
- YAML geçerli kalmalı (girinti 2 boşluk, mevcut düzen).
- Bitince raporda: değiştirdiğin bölümler, caveman_stats çıktısı, açık konular.
