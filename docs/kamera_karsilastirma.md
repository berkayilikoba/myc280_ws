# Derinlik kamerası karşılaştırması: Orbbec Dabai DC1 ↔ Intel RealSense D435 (LIMO, VLMaps için)

Aynı robot (LIMO, Orin Nano), aynı oda, aynı başlangıç işareti (lidar sol 1.854 / 1.859 m, sağ 0.705 / 0.718 m),
aynı betikler (`docs/kamera_testleri.md`). Dabai: 13.10.2026, D435: 14.10.2026 (USB 3.2, doğrudan Jetson'a).
Kamera aynı yuvada: optik merkez yerden ikisinde de 17.8 cm.

## 1. Kurulum ve akış

| | Dabai DC1 | D435 |
|---|---|---|
| Bağlantı | USB 2 (yeterli) | **USB 3 şart** (hub üzerinden USB 2'de 480 Mbps'ye düştü; Jetson'ın mavi portuna taşındı) |
| Sürücü (Foxy) | `orbbec_camera` dabai.launch.py | `realsense2_camera` 4.51.1 (`realsense2_camera_node`) |
| Derinlik | 640×400 16UC1 mm, yapılandırılmış ışık | 640×480 16UC1 mm (848×480 önerilen), aktif stereo |
| Renk | 640×480 | 640×480 |
| Kayıtta hız (tur) | derinlik 29.1–30.0, renk 25.8–27.3 Hz | derinlik 29.1–29.7, **renk 29.5–29.8 Hz** |
| Renk-derinlik damgası | ayrı (fark 7–12 ms) | **aynı damga** (fark 0) |
| Donanım hizası | kamerada (`depth_registration`), 30 Hz kalır | **sürücüde, Jetson CPU'sunda: 20 Hz'e düşer, σ bozulur → kullanılmamalı** |
| Kalibrasyon (zemin) | pitch −1.42°, roll +0.43°, RMS 3 mm | pitch −1.09°, roll +0.09°, RMS 2.1 mm |

## 2. Sabit sahne (başlangıç işareti, 8 s) — `derinlik_karakter.py`

| ölçüt | Dabai varsayılan | D435 varsayılan | D435 + uzamsal/zamansal süzgeç |
|---|---|---|---|
| doluluk (tüm görüntü) | 0.796 | **0.893** | 0.895 |
| σ 0.3–1 m | 2.51 mm | 1.53 mm | **0.64 mm** |
| σ 1–2 m | **3.77 mm** | 8.53 mm | 6.19 mm |
| σ 2–4 m | **28.4 mm** | 37.4 mm | 37.0 mm |
| σ 4–8 m | **89.5 mm** | 114.7 mm | 111.7 mm |
| kırpışma (titreşen delik) | 0.166 | **0.088** | 0.083 |
| uçan piksel | **0.016%** | 0.194% | 0.154% |

D435 ek ayarlar: 848×480 doluluk 0.912, σ 1–2 m 9.8 mm (640×480'den iyi değil); lazer kapalı: doluluk 0.693, σ her
aralıkta ~1.5× kötü; sürücü hizası (`align_depth`): 20 Hz, σ 186–640 mm (Jetson'da bozuk).

Not: Görüş alanları farklı (D435 derinlik ~87°×58°, Dabai daha dar), mesafe aralıklarındaki pikseller aynı nesneler
değil; σ mesafe aralığına göre karşılaştırılmalı.

## 3. VLMaps sahnesi (0.15 m/s dikdörtgen tur, her 2. kare) — `bag2vlmaps.py` + `vlmaps_dogrula.py`

| ölçüt | Dabai (`dikdortgen_20261013_144019`) | D435 (`dikdortgen_20261014_154842`) |
|---|---|---|
| yazılan kare | 1100 | 1183 |
| renk görüntüsünde derinlik oranı | ~0.70 | 0.40 → **0.89** (izdüşüm deliği doldurma ile) |
| derinlik-renk hizası (düz/yavaş) | 1.9 px, ≤3 px %76 | 1.9 px, ≤3 px %69 (8 kare) |
| zemin yüksekliği | −0.1 cm (−0.6..+1.1) | −0.5 cm (−1.0..+0.4) |
| kamera-lidar <5 cm | %88 (medyan 0.5 cm) | %88 (medyan 0.5 cm) |
| en iyi kamera yaw | −0.5° | +0.5° |
| başlangıca dönüş (lidar ICP) | 0.8 cm / −1.2° | 1.2 cm / −1.9° |

**İzdüşüm deliği:** D435'in derinlik görüş alanı renkten geniş (87° vs 69°, ikisi de 640 px), derinlik pikselleri
renk ızgarasına seyrek düşüyor: deliksiz doldurma olmadan renk görüntüsünün yalnız %40'ında derinlik vardı.
`bag2vlmaps.py` artık 3×3'te ≥4 geçerli komşusu olan boş pikseli en yakın komşuyla dolduruyor (varsayılan; `--doldurma`
ile kapatılır) → %89.

## 4. Kapsama turu (dikdörtgen + her köşede 360° @0.5 rad/s) ve dönüşte hiza — `donus_analiz.py`

| ölçüt | Dabai (`kapsama_20261013_160700`) | D435 (`kapsama_20261014_155422`) |
|---|---|---|
| yazılan kare (her 2.) | 1677 | 1948 |
| kamera-lidar <5 cm | %92 (medyan 0.5 cm) | **%94** (medyan 0.4 cm) |
| zemin yüksekliği | −0.1 cm (−0.6..+2.9) | −0.6 cm (−1.6..+0.3) |
| başlangıca dönüş lidar ICP / odom | 9.7 cm, −5.7° / 6.6 cm, −0.7° | 5.8 cm, −6.6° / 1.5 cm, 0.0° |

Açısal hıza göre hiza (kare başına; derinlik kenarı → renk kenarı, px) ve renk keskinliği (Laplace varyansı):

| \|w\| (rad/s) | Dabai kare / hiza / keskinlik | D435 kare / hiza / keskinlik | D435 0.15 m/s tur |
|---|---|---|---|
| <0.05 | 413 / 1.91 px / 378 | 84 / 1.91 px / 134 | 188 / 1.91 px |
| 0.05–0.20 | 36 / 1.91 / 340 | 9 / 2.87 / 105 | — |
| 0.20–0.45 | 128 / 1.91 / 303 | 6 / 11.05 / 72 | 27 / 10.19 |
| 0.45–0.70 | 151 / 2.87 / 290 | 25 / **13.37** / 55 | 6 / 22.05 |

- **D435 dönerken hiza 6–7× bozuluyor, Dabai neredeyse hiç.** D435 renk görüntüsü dönüşte %60 bulanıklaşıyor
  (keskinlik 134 → 55); Dabai'de %23 (378 → 290). D435 RGB sensörü rolling shutter + iç mekânda uzun otomatik
  pozlama. Renk ve derinlik aynı damgayı taşıyor (fark 0 ms), yani bag2vlmaps'in hareket telafisi zaman farkı
  görmüyor.
- D435 kayıtlarında dönüşte ölçülebilir kare az (derinlik kenarı + keskin renk kenarı gerekiyor): örnek küçük ama iki
  ayrı kayıtta (tur, kapsama) aynı yön.
- **VLMaps için:** D435 ile `bag2vlmaps.py --max_w 0.2` (dönüş karelerini at) ya da yavaş dönüş; Dabai'de gerek yok.
- Odom her iki kamera gününde de 4×360° dönüşte ~6° yön kaydırıyor (kameradan bağımsız) → kapsama turunda lidar/SLAM pozu şart.

## 5. Duvar testi (odanın uzun düz duvarı, 0.35–2.0 m) — `duvar_analiz.py`

Fark = kamera görüntü merkezi (+0.084 m) − lidar tam ön (+0.137 m), base_link'ten. Düzlem RMS: merkez 120×120 piksel,
tek kare. Zamansal σ: merkez piksellerin kareler boyunca std'si.

| lidar (m) | fark Dabai / D435 | düzlem RMS Dabai / D435 | zamansal σ Dabai / D435 | doluluk Dabai / D435 |
|---|---|---|---|---|
| 0.38 | −4.5 / −9.2 cm | 0.4 / 6.0 mm | <0.5 / 0.9 mm | 1.00 / 0.91 |
| 0.50 / 0.44 | −4.6 / −9.7 cm | 0.6 / 9.9 mm | <0.5 / 1.1 mm | 1.00 / 0.90 |
| 0.70 | −6.0 / −10.9 cm | 1.3 / 12.2 mm | <0.5 / 3.9 mm | 1.00 / 0.97 |
| 1.00 | −5.9 / −11.7 cm | 9.4 / 13.9 mm | <0.5 / 7.7 mm | 1.00 / 0.98 |
| 1.30 | −4.6 / −13.2 cm | 11.5 / 22.1 mm | 0.5 / 11.3 mm | 1.00 / 0.95 |
| 1.60 | −10.1 / — | 11.1 / — | 1.5 / — | 1.00 / — |
| 2.00 | −11.1 / −15.0 cm | 12.0 / 56.4 mm | 2.6 / 29.7 mm | 1.00 / 0.97 |

- **Duvarda D435 belirgin şekilde gürültülü:** 2 m'de zamansal σ 30 mm (Dabai 2.6), düzlem RMS 56 mm (Dabai 12).
  Dabai'nin zamansal σ'sının bu kadar düşük olması yerleşik süzgecinden olabilir (sabit sahnede zamansal ortalama).
- **Fark iki kamerada da negatif ve mesafeyle büyüyor (eğim ikisinde ~−4 cm/m)**: ortak eğim kamerada değil ortak
  referansta (lidar ölçeği/yön ya da duvar geometrisi) olabilir — doğrulanmadı. D435'teki ek ~−4.5 cm sabit kısım:
  `camera_link` orijini (D435: sol IR sensörü) Dabai'ninkinden farklı; x ofseti ayrı ölçülmeli (metre + düz hedef).
- D435 testinde durak süresi 3 s'ye indirildi; 1.6 m durağı analizde ayrılamadı (durak bulma eşiği 2.5 s).

## 6. Lidar ile düzeltilmiş poz (21.10) — `lidar_poz.py`, `vlmaps_dogrula.py --harita`

Her `/scan_sabit` taraması 09.10 haritasına 2B ICP ile oturtulur (odom yalnız başlangıç tahmini); `bag2vlmaps.py
--poz_dosyasi` ile sahne harita çerçevesinde üretilir. Doğrulama için **bağımsız referans**: derinlik diliminin
haritadaki en yakın duvara uzaklığı (eski kamera-lidar ölçütü aynı pozu kullandığından poz kaymasını görmüyordu).

| kayıt | poz | harita: medyan uzaklık | <10 cm | ilk / ikinci yarı |
|---|---|---|---|---|
| D435 kısa pozlama kapsama (odom 21 cm kaymış) | odom | 14.7 cm | %38 | 23.2 / 9.4 cm |
| ″ | **lidar** | **5.0 cm** | **%76** | 5.0 / 5.1 cm |
| Dabai kapsama | lidar | 5.6 cm | %74 | 6.0 / 5.4 cm |
| D435 kapsama | lidar | 4.9 cm | %80 | 5.2 / 4.7 cm |
| D435 yavaş dönüş (yerinde) | lidar | 3.5 cm | %94 | 4.1 / 3.0 cm |

5 cm ≈ harita çözünürlüğü. ICP uyumu medyan 0.91–0.96, hiç başarısız tarama yok; 0.30 m/s turdaki başlangıç
10° yön hatasını ve kapsama turlarındaki ~6–9° odom kaymasını yakalıyor. **VLMaps için poz kaynağı: lidar_poz.**

## 7. D435 dönüş hızı ve pozlama (21.10)

| D435 | kare | hiza (CLAHE'li ölçüt) |
|---|---|---|
| durağan (kapsama) | 84 | 1.9 px |
| **yavaş dönüş 0.2 rad/s** (`donus_yavas_20261021_142831`) | 212 | **2.9 px** |
| 0.25–0.45 rad/s | 6 | 9.1 px |
| 0.45–0.70 rad/s (kapsama) | 25 | 10.5 px |

- **D435 ile kayıtta dönüş ≤ 0.2 rad/s** → hiza Dabai seviyesinde kalıyor.
- Kısa pozlama (rgb 5 ms, kazanç 128): görüntü karanlık (ortalama parlaklık 56/255) ve gürültülü; hiza ölçütü
  tutarsız (CLAHE'li bile durağan 32 px). **Önerilmez**; yavaş dönüş daha iyi çözüm.

## 8. Gerçek VLMaps haritası (LSeg + CLIP) — `vlmaps_calistir.py`, `vlmaps_karsilastir.py`

Kapsama turları (her 2. kare, lidar pozu, derinlik ≤ 3 m), 5 cm hücre, 11 kategori (duvar, zemin, kapı, sandalye,
masa, dolap, raf, kutu, pencere, çöp kutusu, robot) + diğer.

| ölçüt | Dabai | D435 |
|---|---|---|
| 3B hücre | 76 681 | 88 740 |
| kuşbakışı kapsama | 24.4 m² | **28.0 m²** |
| "duvar" kesinliği (<10 cm'de harita duvarı) | **0.42** | 0.24 |
| harita duvarlarını bulma (geri çağırma) | **0.53** | 0.49 |
| zemin / duvar oranı | **%37 / %16.5** | %13 / %27 |
| ortak 23.5 m²'de etiket uyumu | %43 | |

- Dabai haritası odayı temiz veriyor (ortada zemin, sağda uzun duvar, solda masa/sandalye sırası, altta kapı).
- **D435'in duvar hücreleri gerçek duvarın iki yanına 0.5–1 m taşıyor** (duvarın arkası ve önündeki zemin): uzun
  duvarı geniş görüş alanıyla yatık açıdan görürken stereo derinlik hatası büyüyor (duvar testi: 2 m'de σ 30 mm,
  düzlem RMS 56 mm). Zemin oranının düşüklüğü de bu taşmadan.

## Özet (VLMaps açısından)

| konu | daha iyi | not |
|---|---|---|
| Kare hızı / renk | **D435** | renk 29.7 Hz (Dabai 26–27 Hz) |
| Derinlik doluluğu | **D435** (sahnede) / Dabai (duvarda) | sabit sahne 0.89 vs 0.80; düz duvarda Dabai 1.00 |
| Yakın (<1 m) gürültü | **D435** (sabit sahne) / Dabai (duvar) | ölçüme göre değişiyor |
| Uzak (>1 m) gürültü | **Dabai** | 2 m duvarda σ 2.6 vs 30 mm; sahnede 2–4 m 28 vs 37 mm |
| Uçan piksel | **Dabai** (12× az) | D435 stereo kenar gürültüsü |
| VLMaps sahne ölçütleri (düz tur) | eşit | hiza 1.9 px, örtüşme %88, zemin ±1 cm |
| Kapsama turunda örtüşme | **D435** (%94 vs %92) | geniş görüş alanı |
| Dönüşte hiza | **Dabai** | D435 0.5 rad/s'de 13 px (bulanıklık), Dabai 2.9 px |
| Jetson yükü / donanım hizası | **Dabai** | D435 sürücü hizası Jetson'da 20 Hz, kullanılamaz |
| Kurulum | **Dabai** | D435 USB 3 şart; izdüşüm deliği doldurma gerekli (bag2vlmaps artık yapıyor) |

**Öneri:** VLMaps için iki kamera da kullanılabilir; D435 ile (1) `bag2vlmaps.py --max_w 0.2` ya da yavaş dönüş,
(2) derinlik ~2 m ile sınırla (`--zmax 2.0`), (3) uzamsal+zamansal süzgeç açık. Kısa pozlama denemesi
(`donus_kisapoz_*`, `kapsama_kisapoz_*`: rgb exposure 5 ms, gain 128) kaydedildi, analizi bekliyor.
Her iki günde de kapsama turundaki 4×360° dönüş odom yönünü ~6° kaydırıyor → VLMaps için lidar/SLAM pozu gerekli.

## Kayıtlar (Drive: Robot Folder / LIMO_2026-10-13_orbbec, LIMO_2026-10-14_d435)

| test | Dabai (13.10) | D435 (14.10) |
|---|---|---|
| kalibrasyon | 09.10 tur | `d435_kalib_20261014_154443` |
| ayar | `ayar_20261013_152500` | `ayar_20261014_155033` |
| 0.15 m/s tur | `dikdortgen_20261013_144019` | `dikdortgen_20261014_154842` |
| 0.30 m/s tur | `dikdortgen_20261013_153935` | `dikdortgen_20261014_162237` |
| kapsama | `kapsama_20261013_160700` | `kapsama_20261014_155422`, `kapsama_kisapoz_20261014_164153` |
| duvar | `duvar_20261013_153501` | `duvar_20261014_160522` |
| dönüş | `donus_20261013_160413` | `donus_20261014_162358`, `donus_kisapoz_20261014_164036` |
