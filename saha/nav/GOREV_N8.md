# Görev N8: nav2_diff.yaml — derinlik kamerasıyla 3B engel katmanı (VoxelLayer) + collision_monitor kaynağı
Dosya: src/limo_bringup/param/nav2_diff.yaml (yalnız bu; önce TAMAMINI oku, N5 değişiklikleri yeni eklendi —
koru). Launch tarafını Claude yapacak: depth_image_proc/point_cloud_xyz düğümü `/camera/depth/image_raw` +
`/camera/depth/camera_info` → `/camera/depth/points` (sensor_msgs/PointCloud2, çerçeve camera_depth_optical_frame).
Sorun: 2B lidar ~8 cm yükseklikte kesit görüyor; sandalye oturağı, alçak raf çıkıntısı, masa altı çapraz ayakları
costmap'te yok. Robot + kol yüksekliği ~0.45 m: 0.50 m üstü (masa tablası 0.75) engel SAYILMAMALI (robot kavrama
için masa kenarının altına girebilmeli).
1. local_costmap: plugins listesine "voxel_layer"ı obstacle_layer'dan ÖNCE ekle:
   voxel_layer: plugin nav2_costmap_2d::VoxelLayer, enabled true, publish_voxel_map false, origin_z 0.0,
   z_resolution 0.05, z_voxels 10, max_obstacle_height 0.50, mark_threshold 1 (yoksa 0), observation_sources depth,
   depth: { topic /camera/depth/points, data_type PointCloud2, min_obstacle_height 0.05, max_obstacle_height 0.50,
   marking true, clearing true, obstacle_max_range 2.0, obstacle_min_range 0.25, raytrace_max_range 2.5,
   raytrace_min_range 0.25, expected_update_rate 0.0, inf_is_valid false }. combination_method 1 (max).
   Neden yorumla: zemin gürültüsü (D435 σ = 0.00176·z²; 2 m'de 7 mm) min 0.05 ile süzülür.
2. global_costmap'te obstacle_layer varsa voxel_layer'ı EKLEME (yalnız yerel; global statik + lidar kalsın) —
   yorumla gerekçelendir (kamera dar görüşlü, global'de iz bırakır).
3. collision_monitor: observation_sources listesine "depth" ekle: type pointcloud, topic /camera/depth/points,
   min_height 0.05, max_height 0.50, enabled true (Humble collision_monitor pointcloud kaynağı alanları: type,
   topic, min_height, max_height — başka alan uydurma).
4. Dosya sonundaki N5 değişiklik bloğunun altına N8 bloğu ekle. YAML geçerli kalsın.
