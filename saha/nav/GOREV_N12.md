# Görev N12: nav_bench.py — YAML hedef turu + odom kayması altında gezinme sağlamlığı ölçütleri
Dosyalar: src/limo_bringup/scripts/nav_bench.py (geriye uyumlu genişlet), src/limo_bringup/scripts/
nav_tur_lab_oda.yaml (yeni). Önce nav_bench.py'yi TAMAMEN oku (mevcut sabit 6 hedefli tur, metrikler, /odom_gercek
kullanımı). Mevcut varsayılan davranış ve çıktı biçimi korunmalı (bench_run.sh / bench_summary.py ona bağlı).
1. `--hedefler <yaml>` seçeneği: verilirse sabit tur yerine yaml'deki liste: [{ad, x, y, yaw_derece}], sırayla.
2. Yeni ölçütler (her hedef için, yaml/json çıktısına ek alan olarak; mevcut alanları değiştirme):
   varis_hatasi_gercek_m ve varis_yon_hatasi_gercek_deg (hedef ↔ /odom_gercek, Nav2 SUCCEEDED anında),
   lokalizasyon_hatasi_m (o anda map->base_footprint TF ile /odom_gercek arasındaki fark; dünya = harita varsayımı),
   min_aciklik_m (mevcut açıklık ölçümü neyse onu kullan), kurtarma_sayisi (mevcut), sure_s.
3. Tur sonu özet: başarı oranı, ortalama/maks varış hatası (gerçek), ortalama lokalizasyon hatası, toplam süre.
4. nav_tur_lab_oda.yaml (Claude haritada açıklığı ölçtü, parantezde): baslangic (0,0);
   hedefler: oda_orta (3.0, 0.3, 0°) [0.94 m], dogu (5.2, 0.9, 90°) [0.56], masa_onu (2.0, 1.2, 180°) [0.52],
   kuzeydogu (4.2, 2.3, 0°) [0.34, dar], bati_dar (-1.5, 1.0, 90°) [0.27, çok dar — zor hedef], orta (1.0, 1.0,
   -90°) [0.75], ev (0.0, 0.0, 0°) [0.66].
