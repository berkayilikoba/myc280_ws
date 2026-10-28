# Ortak bağlam (tüm N görevleri)
Proje: AgileX LIMO (diff sürüş, ayak izi ±0.161 x ±0.11 m) + üstünde myCobot 280 kol. ROS 2 Humble, Nav2 (MPPI + Smac2D),
Python düğümleri `src/limo_autonomy/limo_autonomy/`. Testler pytest, ROS'suz saf fonksiyonlar tercih edilir.
MİMARİ KARARI: lidar odometrisi / lidar-harita ICP KULLANILMAYACAK. Odom = görsel + teker + IMU füzyonu (başka ekip),
kayar (~%1-2 mesafe, birkaç derece). map->odom bir lokalizasyon katmanından gelir (simde AMCL yer tutucu). Lidar
yalnız engel algısı (costmap, collision_monitor, güvenlik durdurması) için. Kavrama doğruluğu küresel pozdan değil,
NESNEYE GÖRELİ kapalı çevrim son yaklaşmadan gelmeli.
Kol erişimi: kavrama noktası robot merkezinin d_kavrama = 0.22 m önünde (0.18–0.30 aralığı), yanal ofset 0 (±0.03
tolerans), robotun nesneye yönelimi kavrama doğruluğu için kritik (±3°).
Nesne algısı arayüzü: `/kavrama/nesne_algi` (geometry_msgs/PointStamped, herhangi bir çerçeve, tipik base_footprint
veya camera_color_optical_frame), 2–10 Hz, nesne kamera görüşünde ve ~1.5 m içindeyken gelir; arada gelmeyebilir.
Kod Türkçe adlandırma ve yorumlarla, mevcut dosyaların üslubunda yazılsın. Kodu çalıştıramazsın; Claude test edecek.
Sonunda kısa özet yaz.

## Caveman araçları (her görevde kullan)
`caveman` MCP sunucusu bağlı (izin: `mcp(caveman/*)`).
- Büyük dosya/çıktıyı (≳200 satır) bağlama almadan önce `caveman_compress`; kaybolan ayrıntı için `caveman_retrieve`.
  Düzenleyeceğin satırları her zaman özgün dosyadan birebir oku (sıkıştırılmış metinden düzenleme yapma).
- Tablo biçimli JSON'u alıntılamadan önce `caveman_toon_encode`.
- Görev sonunda `caveman_stats` çağır, çıktısını raporuna ekle.
