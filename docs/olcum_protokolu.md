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
   (konum ofseti) ile eğim (ölçek hatası) **ayrılır**. Betik: `duvar_analiz.py` mantığı + `--gercek` mesafe listesi
   (yazılacak).
3. **Kamera–lidar dış kalibrasyonu, tahta ile**: ChArUco tahtası kameradan tam 6B poz verir (OpenCV), aynı anda lidar
   tahtanın kenar çizgisini görür → kamera↔lidar dönüşümü mm/0.1° düzeyinde. 6–8 farklı konum, her biri 3 s.
4. **Daha yoğun referans harita**: 5 cm SLAM haritası yerine, lidar_poz ile hizalanmış tüm taramaları biriktirip 2 cm
   nokta haritası (birden çok kayıttan) → harita ölçütündeki 5 cm taban kalkar (PC'de yapılabilir, robot gerekmez).

## 2. Tekrarlanabilirlik ve istatistik

- Her test **en az 3 tekrar**; sonuçlar ortalama ± std (ya da medyan ve IQR). Tek tur farkı (ör. 0.42 vs 0.24)
  tekrarlar arası yayılımla birlikte raporlanmalı.
- Başlangıç: zemine bant ile artı işareti + yön çizgisi; her turdan önce `poz_ver.sh` ve lidar ile kontrol
  (sol/sağ duvar mesafesi bugün 1.84–1.86 / 0.70–0.72 m; ±1 cm dışındaysa düzelt). Turdan sonra robotu işarete geri koy
  (kapsama turu odomla ~6° kayıyor).
- Kameralar **aynı gün, aynı ışıkta, arka arkaya** sınanmalı (bugün Dabai 13.10, D435 14.10 ve 21.10).

## 3. Kamera koşullarını sabitle

- Işık: oturum başında ve sonunda lüks ölçümü (telefon uygulaması yeterli), perdeler/lambalar aynı.
- **D435:** renk 1280×720 (640×480'de keskinlik Dabai'nin ~1/3'ü — LSeg etiketleri zayıf olabilir, doğrulanacak),
  derinlik 848×480, High Accuracy ön ayarı + uzamsal/zamansal süzgeç: `D435_AYAR=vlmaps kamera_baslat.sh`
  (visual_preset parametre adını robotta doğrula). Otomatik pozlama açık, ama **dönüş ≤ 0.2 rad/s**.
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
- Sonra her kategori için kesinlik/geri çağırma ve dil sorgusu başarısı (`vlmaps_hedef.py` hedefi doğru nesneye mi
  gösteriyor). Fotoğraflarla birlikte `config/oda_etiket.yaml` olarak saklanır.

## 6. Oturum kontrol listesi

1. Robot: `lsusb` (kamera USB 3 mü), disk ≥ 20 GB, `whoopsie` kapalı mı, başka kullanıcı var mı.
2. PC: disk ≥ 30 GB, Robot_Lab ağı (Koray'a kaymasın), Drive yüklemeleri durdurulmuş.
3. Fiziksel ölçümler (1.1), ışık ölçümü.
4. Testler (her biri 3 tekrar), her testten sonra gzip ile çek (`robot2pc_gz.sh`), MD5 doğrulaması.
5. Oturum sonu: Nav2 kapalı, analiz, Drive.
