# Görsel odometri — dikdörtgen tur kayıtları üzerinde denemeler (28.10.2026)

Amaç: kamera kayıtlarından EKF'ten (`/odom` vx + `/imu` yönü, `ekf_robot.yaml`) daha doğru bir odometri üretmek.
Kod `saha/analiz/vo/`, ara çıktılar `bags/vo_calisma/<koşu>/` (sensör önbelleği, referans, yörüngeler).

## Veri

| koşu | kayıt | kamera | not |
|---|---|---|---|
| `dabai_0910a` | `LIMO_2026-10-09_dikdortgen/bag/kamera_tur_20261009_122158` | Orbbec Dabai, renk ~20 Hz | |
| `dabai_0910b` | `LIMO_2026-10-09_dikdortgen_lidarduz/bag/dikdortgen_20261009_162849` | Dabai, renk 27 Hz | |
| `d435_1015` | `LIMO_2026-10-15_dikdortgen_realsense/bag/dikdortgen_rs_20261015_133037` | RealSense D435, renk ~12 Hz | EKF ile sürüldü |

İki Dabai kaydının kamera `.db3` dosyaları `bags/kamera_0-001.db3` (→ 0910b) ve `bags/kamera_0-002.db3` (→ 0910a)
olarak klasör dışındaydı (zaman damgalarıyla eşlendi); kayıt klasörlerine `kamera_0.db3` sembolik bağı eklendi.

## Referans ve ölçütler

`vo/referans.py`: her lidar taraması `LIMO_2026-10-09_dikdortgen_lidarduz/harita` haritasına 2B nokta-doğru ICP ile
oturtulur (lidar dış kalibrasyonu 0.137, 0.015 m, −9.66°). Üç koşuda da reddedilen tarama yok, eşleşme medyanı
0.92–0.97, tarama başına rms 1.3–1.4 cm. d435_1015 bitişi `donus_hatasi` ICP sonucuyla tutarlı. Referansın gürültü
tabanı ~1–1.5 cm olduğundan aşağıdaki farkların 1 cm altı anlamlı değil.

`vo/degerlendir.py` ölçütleri (cm, derece):
- **ATE-SE2:** yörünge referansa SE(2) en iyi uyumla (ölçeksiz) oturtulduktan sonraki RMS konum hatası. Başlangıç pozu
  yanlılığından bağımsız; ana ölçüt.
- **ATE:** yalnız başlangıç pozunda hizalanmış RMS konum hatası. Bu ölçütte bitiş pozunun hatası da birikir.
- **maks / bitiş:** başlangıç hizasıyla en büyük konum hatası ve bitişteki konum hatası.
- **RPE/1 m:** referansta 1 m'lik parçalar arasındaki göreli hata (medyan).

## Denenen yaklaşımlar

1. **Özellik tabanlı RGB-D VO** (`vo_ozellik.py`): SIFT, oran testi ve karşılıklı kontrolle eşleme. Anahtar karedeki
   derinlikten 3B nokta alınır, aykırılar `solvePnPRansac` ile ayıklanır. Anahtar kareye göre gövde pozu üç varyantla
   çözülür:
   - `ozellik`: düzlem kısıtlı (x, y, yaw).
   - `ozellik_jiro`: yaw IMU'dan.
   - `ozellik_pnp6`: 6 SD çözüm, sonra düzleme izdüşüm.
2. **Yoğun RGB-D VO** (`vo_yogun.py`, Open3D CUDA, hibrit fotometrik + geometrik): yerinde dönüşlerde dağılıyor
   (d435_1015'te bitiş yaw'ı −87°, yaw'ı IMU'dan alınca da ATE 32 cm). Dokusuz duvarlar ve derinlik gürültüsü yüzünden
   bırakıldı.
3. **VO + IMU + tekerlek füzyonu** (`fuzyon.py`, nedensel):
   - `vo_hiz`: ileri mesafe VO'dan, yön IMU'dan.
   - `olcek`: tekerlek ölçeği VO ile çevrimiçi kalibre edilir.
   - `votam`: VO ötelemesi ile IMU yönü birleşir.
4. **Kamera düzlem konumunun öz kalibrasyonu** (`oz_kalibrasyon.py`, lidarsız): yerinde dönüşte gövde ötelenmez
   varsayımına dayanır. VO'nun dönüşlerdeki sahte ötelemesini sıfırlayan (dx, dy) kayması en küçük kareler ile bulunur.
   Düzlem çözümünde kapalı biçim olduğu için VO yeniden koşturulmaz.

## Bulunan sorunlar ve düzeltmeler

- **D435 kaydında `aligned_depth_to_color` karelerinin %58'i hizalanmamış ham derinlik.** Bu karelerde zemin 0.61 m
  yerine 0.35 m'de görünüyor: derinlik kamerasının fx değeri 389, renk kamerasınınki 606. RealSense düğümü yük altında
  (README'de %95 CPU) bu kareleri hizalamadan yayınlamış. Böyle bir kare anahtar kare olursa VO ölçeği 0.55'e düşüyor.
  Bu, 15.10'daki RTAB-Map denemesinin dönüşlerde kaybolmasının da olası nedeni.
  - **Düzeltme:** `ortak.zemin_orani` bilinen kamera yüksekliğinden zemin derinliğini kontrol eder. Tutarsız kare
    `/camera/depth/camera_info` ve TF ile yeniden hizalanır; hâlâ tutarsızsa anahtar kare yapılmaz.
  - **Sonraki kayıtlar için:** `align_depth` kapalı, ham derinlik + extrinsics kaydı (`kayit_realsense.sh`). Bu, hem
    CPU yükünü hem bu hatayı ortadan kaldırır.
- **Dabai renk ve derinlik akışları eşzamansız (±15 ms).** Derinlik artık renk anına, aradaki gövde hareketiyle (öteleme
  + dönüş) taşınıyor.
- **Dabai VO'su mesafeyi %3–7 kısa ölçüyor**; D435 VO'su 0.993–1.000 ölçüyor (düz gidiş, lidar referansına göre). 0.5 m
  zeminde Dabai derinliği doğru (oran 1.00), yani sapma uzak mesafede. Dabai için derinlik ölçek kalibrasyonu yapılmadı.
  Dabai artık robotta değil.
- **Tekerlek mesafesi her koşuda %1–1.6 kısa** (düz gidişte 0.984–0.990). EKF konum hatasının ana kaynağı bu ve
  dönüş başına 0.3–0.8°'lik jiroskop yön hatası.

## Sonuçlar

| koşu | yöntem | ATE-SE2 | ATE | maks | bitiş | yaw rms ° | RPE/1 m |
|---|---|---|---|---|---|---|---|
| d435_1015 | EKF (kayıttaki `/odometry/filtered`) | 4.2 | 6.5 | 11.1 | 7.6 | 0.62 | 3.45 |
| d435_1015 | EKF-eşdeğeri (çevrimdışı) | 3.5 | 5.0 | 8.4 | 5.1 | 0.98 | 3.33 |
| d435_1015 | ozellik | 3.6 | 5.9 | 13.3 | 5.3 | 1.27 | 3.13 |
| d435_1015 | ozellik_pnp6 | 4.0 | 6.7 | 13.5 | 7.5 | 1.51 | 2.96 |
| d435_1015 | **ozellik + öz kalibrasyon** | **3.0** | **3.9** | **7.3** | **4.4** | 1.27 | 2.99 |
| d435_1015 | füzyon vo_hiz (pnp6) | 3.6 | 6.3 | 10.3 | 6.9 | 1.22 | **2.90** |
| d435_1015 | yoğun (hibrit) | 132.7 | | | | 54 | |
| dabai_0910a | EKF-eşdeğeri | **2.3** | 3.6 | 7.3 | 5.2 | 1.99 | 2.75 |
| dabai_0910a | ozellik_pnp6 + öz kalibrasyon (0910b'den) | 6.6 | 12.7 | 21.6 | 11.5 | 1.80 | 5.23 |
| dabai_0910a | füzyon vo_hiz (pnp6 + öz kal.) | 4.2 | 7.6 | 15.4 | 6.2 | 2.00 | 4.58 |
| dabai_0910b | EKF-eşdeğeri | **4.0** | 7.0 | 12.8 | 6.3 | 1.09 | 3.69 |
| dabai_0910b | ozellik_pnp6 + öz kalibrasyon (0910a'dan) | 4.9 | 8.1 | 16.7 | 4.5 | 1.54 | 5.21 |
| dabai_0910b | füzyon votam (pnp6 + öz kal.) | 4.3 | 9.9 | 20.7 | 1.7 | 1.09 | 5.12 |

Tüm satırlar: `python3 saha/analiz/vo/degerlendir.py`.

**D435.** Öz kalibrasyonlu özellik VO'su (`ozellik_ozkal`) tüm konum ölçütlerinde EKF'ten iyi:
- ATE-SE2: 3.0 cm (EKF 3.5, kayıttaki EKF 4.2).
- Başlangıç hizalı ATE: 3.9 cm (5.0).
- Bitiş hatası: 4.4 cm (5.1).

Kazancın kaynağı mesafe ölçeği: VO 0.99–1.00 ölçüyor, tekerlek 0.984. Öz kalibrasyon dönüşlerdeki 3–15 cm'lik sahte
ötelemeyi giderdi. Bulunan kayma kameranın varsayılandan 4.9 cm önde ve 3.6 cm solda olduğunu söylüyor; bu değer
robotun gerçek dönme merkezinin base_link'te olmamasını da içeriyor olabilir. Yaw için VO, IMU'dan iyi değil:
- yaw rms: VO 1.27°, EKF 0.98°.
- Dönüş başına sapma: VO +0.3 ile +2° arası, jiroskop ±0.5°.

Ancak yaw'ı IMU'dan alan varyantlar ATE'yi iyileştirmedi.

**Dabai.** Hiçbir VO varyantı EKF'i geçmiyor. Bunun nedeni Dabai'nin uzak mesafe derinlik ölçek sapması.

## Sınırlar

- **Tek D435 koşusu:** D435 sonucu tek bir koşudan (3 dönüş) geliyor. Öz kalibrasyon referans kullanmıyor ama aynı
  koşudan kestirildi. Dabai'de çapraz koşu doğrulaması (bir koşunun kalibrasyonunu diğerine uygulamak) aynı yönde
  sonuç verdi: 0910b 5.1 → 4.9 cm.
- **Fark küçük:** D435'teki EKF farkı (0.5–1.1 cm ATE) referansın tarama gürültüsüyle (1.3 cm) aynı mertebede.
- **Gerçek zamanlı değil:** Tüm yöntemler çevrimdışı. `vo_ozellik.py` D435'te ~8 kare/s (CPU, SIFT).

## Sonraki adımlar

1. **Yeni D435 kayıtları:** `align_depth` kapalı, renk 30 Hz, yine 2 m × 1 m dikdörtgen turu; en az 3 koşu. Öz
   kalibrasyonu bir koşuda kestirip diğerlerinde doğrulamak için gerekli.
2. **Dönme merkezi ölçümü:** Yerinde dönüş testinde (lidar ICP) dönme merkezi doğrudan ölçülmeli. Böylece öz
   kalibrasyonla bulunan kaymanın kamera konumu mu, dönme merkezi mi olduğu ayrılır.
3. **EKF'e VO hızı girdisi:** VO ileri hızı EKF'e ikinci bir hız ölçümü olarak verilebilir (`robot_localization`
   `twist0`, tekerlek ölçeği hatasını düzeltir). Yön IMU'da kalır.
