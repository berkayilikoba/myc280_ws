# Gece görevi G1 (uzun): kavrama için ROS 2 EYLEM (action) arayüzü — limo_interfaces paketi + sunucu + istemci
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude sabah derleyip test edecek. Acele etme,
dosyaları baştan sona dikkatle yaz; her dosyayı yazdıktan sonra tekrar oku ve tutarlılığı kontrol et.
Önce OKU: src/limo_autonomy/limo_autonomy/kavrama_dugumu.py (durum makinesi, konular), gorev_dugumu.py,
gorev_yoneticisi.py, src/limo_autonomy/package.xml, setup.py, src/limo_autonomy/launch/kavrama.launch.py.

1. YENİ paket src/limo_interfaces (ament_cmake, rosidl): package.xml (rosidl_default_generators build,
   rosidl_default_runtime exec, member_of_group rosidl_interface_packages, geometry_msgs bağımlılığı),
   CMakeLists.txt (rosidl_generate_interfaces), action/Kavra.action:
     # Hedef
     geometry_msgs/PointStamped nesne      # herhangi bir çerçeve (tipik map)
     float32 yon_derece                    # istenen yaklaşma yönü; NaN = serbest
     bool algisiz_izinli                   # algı yoksa haritadaki konumla son yaklaşma yapılabilir mi
     ---
     # Sonuç
     bool basarili
     string neden                          # 'HAZIR' | 'HATA <ayrıntı>'
     geometry_msgs/PointStamped nesne_robotta   # base_footprint
     float32 tahmini_hata_cm
     float32 tahmini_yon_hatasi_derece
     float32 sure_s
     ---
     # Geri bildirim
     string durum                          # PLAN | NAV_Q | SON_YAKLASMA | KURTARMA
     string ayrinti
     int32 aday_no
     int32 aday_sayisi
     float32 kalan_mesafe_m
2. kavrama_dugumu.py: mevcut konu arayüzünü (/kavrama/hedef, /kavrama/durum, /kavrama/iptal) AYNEN koru; ek
   olarak `kavra` eylem sunucusu (rclpy.action.ActionServer, ReentrantCallbackGroup): hedef geldiğinde mevcut
   akışı başlat; geri bildirimi durum değişimlerinde ve SON_YAKLASMA'da 5 Hz yayınla (kalan_mesafe_m = rho);
   iptal isteğinde mevcut iptal yolunu çağır ve eylemi canceled bitir; bitişte sonucu doldur. Aynı anda tek görev:
   meşgulse yeni hedefi REDDET (goal_callback). Yaklaşma yönü verilirse (NaN değilse) Ayar.yon'a radyan olarak ver.
   limo_interfaces importu başarısızsa (paket derlenmemiş) eylem sunucusunu atla ve uyar; konu arayüzü çalışsın.
3. gorev_dugumu.py: kavramayı `kavra` eylem istemcisiyle çağır (geri bildirimi /gorev/durum'a aktar); eylem
   sunucusu 5 s içinde yoksa eski konu arayüzüne düş (parametre `kavrama_arayuzu`: auto|eylem|konu, vars. auto).
   gorev_yoneticisi.py saf kalsın (yalnız olay/eylem türleri gerekirse genişlet, testleri bozma).
4. limo_autonomy/package.xml'e limo_interfaces exec_depend ekle (setup.py'ye dokunmaya gerek yok).
5. Saf yardımcılar + testler: src/limo_autonomy/test/test_kavra_eylem.py — eylem hedefinden iç parametrelere dönüşüm
   (yon_derece NaN → None, derece → radyan), sonuç doldurma yardımcısı (HAZIR/HATA metninden basarili/neden),
   geri bildirim kısıtlayıcı (5 Hz) — ROS gerektirmeyen saf fonksiyonlar.
6. docs için kısa bölüm: src/limo_interfaces/README.md (eylem tanımı, örnek
   `ros2 action send_goal /kavra limo_interfaces/action/Kavra "{nesne: {header: {frame_id: map}, point: {x: 3.73, y: 2.18, z: 0.3}}, yon_derece: .nan, algisiz_izinli: true}" --feedback`).
Yanıtın sonunda ajan rapor JSON bloğunu unutma.
