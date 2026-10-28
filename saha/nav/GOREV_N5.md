# Görev N5: nav2_diff.yaml — hassas varış için ikinci hedef denetleyicisi ve yaklaşma ayarları
Dosya: src/limo_bringup/param/nav2_diff.yaml (yalnız bu). Önce TAMAMINI oku; mevcut değerleri gerekmedikçe değiştirme.
1. controller_server: `goal_checker_plugins: ["general_goal_checker", "hassas_hedef"]` (mevcut ad neyse onu koru +
   yenisini ekle). `hassas_hedef`: nav2_controller::SimpleGoalChecker, xy_goal_tolerance 0.08, yaw_goal_tolerance
   0.10, stateful true. (Kavrama düğümü Q'ya giderken NavigateToPose'u bu denetleyiciyle çağıracak — BT'de
   goal_checker_id kullanımı için navigate_to_pose_diff.xml'e DOKUNMA; Claude ayrı BT yazacak.)
2. MPPI: hedefe yaklaşırken yavaşlama ve yön tutma için — GoalCritic threshold_to_consider 1.0→ mevcut neyse
   koru; GoalAngleCritic threshold_to_consider 0.5 (yoksa ekle), PathAlignCritic/PathFollowCritic'in
   threshold_to_consider değerleri GoalAngle'dan büyük kalsın. Yalnız mevcut critic listesinde olanları ayarla,
   yeni critic ekleme. Yaptığın her değişikliği dosyada satır sonu yorumla gerekçelendir.
3. velocity_smoother: max_accel/max_decel'i taban modeliyle tutarlı yap: doğrusal ivme 1.0 m/s² (gerçek tepki
   3.0 ama güvenli pay), açısal 3.0 rad/s²; max_velocity [0.5, 0.0, 1.2] MPPI vx_max/wz_max ile aynı olsun.
4. collision_monitor: mevcut FootprintApproach poligonunun time_before_collision değeri 1.2 s değilse 1.2 yap;
   yoksa dokunma. use_sim_time'ı launch belirliyorsa elle değiştirme.
5. Dosya sonuna, değişikliklerin listesi ve gerekçesi olan kısa yorum bloğu ekle. YAML geçerli kalmalı
   (girinti!). Değiştirmediğin bölümlere dokunma.
