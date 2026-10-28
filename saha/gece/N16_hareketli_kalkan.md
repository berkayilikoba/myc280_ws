# Gece görevi N16: hareketli engel kalkanı (bağıl hız + çarpışmaya kalan süre ile durdurma)
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude derleyip test edecek ve simde ölçecek.

## Sorun (Claude'un simde ölçtüğü)
lab_oda_3b, `dinamik_engel.py`: tepkisiz insan silindiri (r≈0.2 m) 0.6 m/s karşıdan ve 0.8 m/s yandan geliyor.
F6 sonrası (DurBolge stop + YavasBolge slowdown 0.4 + FootprintApproach 1.2 s) iki senaryoda da temas var:
karşıdan −2.9 cm, robot temasta 0.094 m/s ileri; yandan −2.0 cm, robot 0.129 m/s. Humble collision_monitor
"approach" engeli DURAĞAN varsayıyor (yalnız robot hızıyla öteler); hareketli engelin kendi hızını bilmiyor.
DurBolge önünü büyütmek kavramayı bozar (son yaklaşmada nesne gövde önüne ~6 cm yaklaşıyor).

## İstenen: `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py` + `src/limo_autonomy/test/test_hareketli_kalkan.py`
Hız zincirine giren küçük filtre düğümü: `cmd_vel_giris` (vars. `cmd_vel_smoothed`) → `cmd_vel_cikis` (vars.
`cmd_vel_kalkan`); collision_monitor bundan sonra gelir (launch bağlantısını Claude yapacak, sen yapma).

1) Saf fonksiyonlar (ROS'suz, test edilebilir; dosyanın üstünde):
   - `tarama_noktalari(ranges, angle_min, angle_increment, range_min, range_max, max_menzil)` → base_link'te (x, y)
     numpy dizisi (lidar base_link'te varsay; ofset parametresi `lidar_x` vars. 0.0 ekle).
   - `kumele(noktalar, esik=0.10, min_nokta=3)` → küme merkezleri listesi (ardışık nokta mesafesi eşiği; tarama
     sıralı olduğu için tek geçiş yeter) ve her kümenin yarıçapı (merkeze en uzak nokta).
   - `IzTakipci` sınıfı: odom çerçevesindeki küme merkezlerini en yakın komşu ile eşler (kapı 0.35 m), her iz için
     üstel yumuşatılmış hız (alfa 0.4), yaş ve görülme sayısı tutar; 0.5 s görülmeyen izi siler. Robot hareketinden
     bağımsız olsun diye merkezler önce odom çerçevesine çevrilir (robot pozu argüman olarak verilir: x, y, yaw).
     Yöntem: `guncelle(merkezler_odom, yaricaplar, t)` ve `izler()`.
   - `carpismaya_kalan_sure(p_rel, v_rel, r_toplam)` → robot merkezli bağıl konum/hızla, |p + v t| = r_toplam olan
     en küçük t ≥ 0 (yoksa math.inf). r_toplam = iz yarıçapı + robot yarıçapı (`robot_yaricap` vars. 0.20).
   - `kalkan_karari(izler, robot_v_odom, ...)` → ('serbest'|'yavas'|'dur', ölçek 0..1, gerekçe metni):
     yalnız HAREKETLİ izler (|v_iz| ≥ `min_engel_hizi` vars. 0.20 m/s, görülme ≥ 3) dikkate alınır. Durağan nesneler
     (kavrama hedefi, mobilya) ASLA tetiklemez — bu en önemli koşul. TTC < `ttc_dur` (vars. 1.5 s) → 'dur' ölçek 0;
     TTC < `ttc_yavas` (vars. 3.0 s) → 'yavas', ölçek doğrusal (ttc_dur'da 0, ttc_yavas'ta 1). Bağıl hız robot
     hızını içerir (v_rel = v_iz − v_robot).
   - `komutu_olcekle(vx, wz, karar, olcek)`: 'dur' iken ileri bileşeni engele doğruysa 0; GERİ (engelden uzaklaşan)
     hızı ve yerinde dönmeyi kısma (wz korunur). Engelin yönü argüman olarak verilir; vx·yön_x > 0 ise kısılır.
2) Düğüm `HareketliKalkan(Node)`: /scan (qos_profile_sensor_data), /odom (nav_msgs/Odometry; poz ve twist),
   cmd_vel_giris aboneliği; her komutta son karara göre ölçekleyip yayınlar. Tarama 0.5 s'den eskiyse komutu
   olduğu gibi geçir (kalkan devre dışı, uyar — güvenlik zaten collision_monitor'da). Parametreler: yukarıdakiler +
   `etkin` (bool, vars. true; false iken saf geçiş). `/hareketli_kalkan/durum` (std_msgs/String) karar değiştikçe:
   "DUR ttc=0.9s v=0.62" gibi. İsteğe bağlı `/hareketli_kalkan/izler` MarkerArray (hızlı iş değilse atla).
   use_sim_time launch'tan gelir. `main()` giriş noktası; setup.py console_scripts'e
   `'hareketli_kalkan = limo_autonomy.hareketli_kalkan:main',` ekle.
3) Testler (pytest, ROS'suz): kümeleme (iki ayrı nesne, gürültülü tek nesne); izleyici hız kestirimi (0.6 m/s
   doğrusal hareket 10 adımda ±0.1); TTC analitik durumlar (kafa kafaya, paralel geçen → inf, zaten temas → 0);
   DURAĞAN nesne robot ona 0.3 m/s giderken kararı 'serbest' (kavrama koruması); karşıdan gelen 0.6 m/s → 'dur';
   'dur' iken geri komutu kısılmıyor; robot da odom'da hareket ederken durağan nesnenin hızı ≈ 0 kestiriliyor.

Mevcut üslup için önce `kavrama_dugumu.py` (parametre/abonelik kalıpları) ve `test_son_yaklasma.py`'yi OKU.
Dosya başında modül belgesi: neden var, zincirdeki yeri, Humble collision_monitor'dan farkı.
