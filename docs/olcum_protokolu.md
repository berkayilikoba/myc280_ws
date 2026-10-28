# Ölçüm doğruluğunu artırma protokolü (sonraki robot oturumları için)

13–21.10 kamera testlerinde doğruluğu sınırlayanlar ve her biri için yöntem. Öncelik sırasıyla.

## 0. Gerekenler

- Lazer mesafe ölçer (±2 mm) ya da 5 m şerit metre
- Düz, sert hedef levha (≥ 60×40 cm, mat, açık renk; köpük levha/MDF)
- Basılı ArUco/ChArUco tahtası (A3, düz yüzeye yapıştırılmış; kare boyu ölçülerek not edilir)
- Kâğıt bant (zemin işaretleri), su terazisi/gönye

## 1. Referansı kesinleştir (en önemli)

Sorun: tüm ölçütler lidara ve 5 cm haritaya göre; lidar konumu (x 0.137 m) bile tahmin. Kalibrasyon araması iki
kamerada da 7–9 cm ortak kayma buldu; kamera konumu ile derinlik ölçek hatası ayrılamıyor.

1. **Fiziksel ölçüm** (robot kapalıyken, 10 dk): base_link = gövde merkezi (dönme ekseni). Metreyle ölç ve not et:
   lidar dönme merkezinin base_link'e göre x, y, z'si; kamera ön camının (D435: sol IR sensörü, ön yüzden 4.2 mm geride
   — üretici belgesi, doğrulanmalı) x, y, z'si. `config/kamera_kalibrasyon*.yaml` ve `ortam.sh lidar_tf_yayinla`.
2. **Bilinen mesafede düz hedef** (lidardan bağımsız mutlak referans): levhayı robota dik, lazer metreyle ölçülen
   0.5 / 1.0 / 1.5 / 2.0 / 2.5 / 3.0 m'ye koy (robot hareket etmez, levha taşınır; yere bant işareti). Her mesafede 5 s
   kayıt. Çıktı: her kamera **ve lidar** için sapma = ölçülen − gerçek, mesafeye göre doğru uydurma → sabit kısım
   (konum ofseti) ile eğim (ölçek hatası) **ayrılır**. Levha robotun tam önünde, gövde x eksenine dik; lazer metrenin
   sıfır noktası (ör. ön tampon) base_link'e göre ölçülüp `--ref_x` verilir.
   ```bash
   SURE=200 bash kayit_baslat.sh mesafe          # robot sabit, levha 6 konumda >= 5 s
   python3 analiz/hedef_mesafe.py ~/limo_veri/robot_kayit/mesafe_<tarih> --gercek 0.5 1.0 1.5 2.0 2.5 3.0 --ref_x <m>
   ```
   Çıktı `hedef_mesafe.yaml/.png`: lidar, kamera ve kamera−lidar için `hata = sabit + ölçek × mesafe`.
3. **Kamera–lidar dış kalibrasyonu, tahta ile**: ChArUco tahtası kameradan tam 6B poz verir (OpenCV), aynı anda lidar
   tahtanın kenar çizgisini görür → kamera↔lidar dönüşümü mm/0.1° düzeyinde. 6–8 farklı konum, her biri 3 s.
   Tahta: `python3 analiz/charuco_kalib.py --uret tahta.png` (7×5, kare 55 mm, DICT_5X5_100, A3 %100 ölçek; basılı
   kareyi ölç). Lidar düzlemi (yerden ~17 cm) tahtayı kesmeli → tahtanın alt kenarı ~10 cm'de; konumlar dik + öne/arkaya
   ±20–30° + yana dönük (yalnız dik tahta z/roll/pitch'i belirlemez; o durumda `--serbest x y yaw`).
   ```bash
   SURE=180 bash kayit_baslat.sh charuco
   python3 analiz/charuco_kalib.py ~/limo_veri/robot_kayit/charuco_<tarih> --kare <ölçülen m> --isaret <m>
   ```
   Çıktı `charuco_kalib.yaml`: yeni base_link→camera_link (lidar referanslı), eski ile fark, konum başına artık.
4. **Daha yoğun referans harita**: 5 cm SLAM haritası yerine, lidar_poz ile hizalanmış tüm taramaları biriktirip 2 cm
   nokta haritası (birden çok kayıttan) → harita ölçütündeki 5 cm taban kalkar (PC'de yapılabilir, robot gerekmez).
   ```bash
   python3 analiz/yogun_harita.py harita.yaml <cikti> --kayit <sensor_bag>,<lidar_poz.csv> [...] --incelt 2
   ```
   Çıktı `yogun.yaml/pgm` (2 cm, map_server biçimi) doğrudan `vlmaps_dogrula.py --harita`, `vlmaps_karsilastir.py`'ye
   verilir.
5. **Lidar "ön" yönü (27.10'da bulunan hata):** `/scan_sabit` `laser_duz` çerçevesinde ve bu çerçeve gövdeye göre
   −9.66° dönük; `duvar_testi.py`/`duvar_analiz.py` lidar açısı 0±3°'yi "ön" alıyordu = gövdenin 9.66° sağı → dik
   duvarda menzil %1.4 uzun. Düzeltildi (gövde çerçevesinde |y| < 5 cm noktaların x medyanı). 14.10 D435 duvar
   sonucundaki kamera−lidar farkının mesafeyle büyüyen kısmının ~%1.4'ü bu hataydı; eski duvar kayıtları Drive'dan
   alınıp yeniden analiz edilmeli.

## 2. Tekrarlanabilirlik ve istatistik

- Her test **en az 3 tekrar**; sonuçlar ortalama ± std (ya da medyan ve IQR). Tek tur farkı (ör. 0.42 vs 0.24)
  tekrarlar arası yayılımla birlikte raporlanmalı.
- Başlangıç: zemine bant ile artı işareti + yön çizgisi; her turdan önce `poz_ver.sh` ve lidar ile kontrol
  (sol/sağ duvar mesafesi bugün 1.84–1.86 / 0.70–0.72 m; ±1 cm dışındaysa düzelt). Turdan sonra robotu işarete geri koy
  (kapsama turu odomla ~6° kayıyor; 27.10 tur sonu x +8 cm, yaw −4.6°): `python3 baslangica_don.py harita.yaml`
  (lidar ICP ile ölç, odomla düzelt; Nav2 açık; 27.10: 1 adımda 0.6 cm / 0.47°). `--sadece_olc` yalnız ölçer.
- Kameralar **aynı gün, aynı ışıkta, arka arkaya** sınanmalı (bugün Dabai 13.10, D435 14.10 ve 21.10).

## 3. Kamera koşullarını sabitle

- Işık: oturum başında ve sonunda lüks ölçümü (telefon uygulaması yeterli), perdeler/lambalar aynı.
- **D435:** renk 1280×720 (640×480'de keskinlik Dabai'nin ~1/3'ü — LSeg etiketleri zayıf olabilir, doğrulanacak),
  derinlik 848×480, High Accuracy ön ayarı + uzamsal/zamansal süzgeç: `D435_AYAR=vlmaps kamera_baslat.sh`
  (`depth_module.visual_preset` 3 = High Accuracy, 27.10 robotta doğrulandı; sürücü varsayılanı 0 = Custom).
  1280×720 renk + 848×480 derinlik 30 Hz'de ~107 MB/s (tur başına ~25 GB) robot diskine sığmıyor → 15 Hz profili
  (`PROFIL=1280x720x15 DPROFIL=848x480x15`, kayıtta 13.9 Hz, 233 s = 10.8 GB). Sahne: `bag2vlmaps.py --olcek 0.5`
  (LSeg zaten uzun kenarı 520 px'e indiriyor). Otomatik pozlama açık, ama **dönüş ≤ 0.2 rad/s**.
- Kısa pozlama denenmeyecek (karanlık/gürültülü). Dabai: varsayılan.
- Her kayıtta `/camera/*/camera_info` ve sürücü parametre dökümü (`ros2 param dump`) sakla.

## 4. Hareket ve poz

- Tur hızı 0.15 m/s, dönüş ≤ 0.2 rad/s (her iki kamera için aynı rota; kapsama turu `--spin_w 0.2`).
- Poz kaynağı her zaman `lidar_poz.py` (odom yalnız başlangıç tahmini).
- Kayıt sırasında PC'de Nav2 dışında ROS düğümü açık olmasın; aktarımdan önce Nav2'yi kapat (Wi-Fi tıkanıyor).

## 5. VLMaps için gerçek etiket (ground truth)

Bugün yalnız "duvar" lidar haritasıyla ölçülebildi. Oda bir kez elle etiketlenmeli:
- Harita çerçevesinde nesne çokgenleri (kapı, masa, sandalye, dolap, raf, pencere, robotun göreceği yükseklikteki
  diğerleri) — harita PNG'si üzerine koordinat ızgarasıyla çizilir ya da her nesnenin köşeleri metreyle ölçülür.
  Şablon: `config/oda_etiket_sablon.png` (0.25 m / 1 m ızgara, `analiz/etiket_sablonu.py`), biçim:
  `config/oda_etiket.yaml` (örnek iki nesne silinip doldurulacak). Değerlendirme:
  `python3 analiz/etiket_degerlendir.py <vlmaps_cikti> config/oda_etiket.yaml [--hedef vlmaps_hedef_cikti.txt]`.
- Sonra her kategori için kesinlik/geri çağırma ve dil sorgusu başarısı (`vlmaps_hedef.py` hedefi doğru nesneye mi
  gösteriyor). Fotoğraflarla birlikte `config/oda_etiket.yaml` olarak saklanır.

## 6. Oturum kontrol listesi

1. Robot: `lsusb` (kamera USB 3 mü), disk ≥ 20 GB, `whoopsie` kapalı mı, başka kullanıcı var mı.
2. PC: disk ≥ 30 GB, Robot_Lab ağı (Koray'a kaymasın), Drive yüklemeleri durdurulmuş.
3. Fiziksel ölçümler (1.1), ışık ölçümü.
4. Testler (her biri 3 tekrar), her testten sonra gzip ile çek (`robot2pc_gz.sh`), MD5 doğrulaması.
5. Oturum sonu: Nav2 kapalı, analiz, Drive.
