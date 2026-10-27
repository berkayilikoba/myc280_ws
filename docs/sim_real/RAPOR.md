# Sim–gerçek farkı (27.10.2026)

Yöntem: gerçek bir kapsama turunun (`kapsama_20261014_155422`, D435) tabana giden komutları (`/cmd_vel`, robotta
kaydedilmiş) aynı başlangıç noktasından simde (gerçek odanın dünyası `lab_oda.world`) **açık çevrim** oynatıldı;
sim tepkisi gerçek kayıtla karşılaştırıldı. Gerçek referans konum: lidar–harita ICP (`lidar_poz.py`). Derinlik için
bugünkü vlmaps2 turunun (`v2a`) başlangıç karesi. Araç: `src/limo_bringup/scripts/sim_real_fark.py`
(`oynat` simde, `karsilastir` sonra); ham sonuç `sim_real_fark.yaml`.

## Bulunan ve düzeltilen farklar

| | Önce | Neden | Düzeltme | Sonra |
|---|---|---|---|---|
| Düz sürüş yolu | sim, gerçeğin ~%70'i (dikdörtgen 1.54×0.84 m, gerçek 2.2×1.1) | `planar_move` gövdeyi taşıyor, tekerlekler tahrik edilmiyor; serbest tekerlekler zeminde fren yapıyor (0.15 m/s × 10 s: komut 1.425 m → 1.136 m) | `limo_four_diff.xacro` tekerlek mu1 = mu2 = 0 | 1.425 m (birebir) |
| İleri hız tepkisi | sim 0.74 s gecikme, kazanç 0.73 | taban modeli 24.09'da **PC'de** kaydedilmiş teleop verisiyle uydurulmuştu: 0.3 s gecikmenin çoğu Wi-Fi | robotta kaydedilmiş 4 bag ile yeniden uyduruldu (`saha/analiz/taban_uydur.py`): v gecikme 0.05 s, τ 0.05 s, ivme 3.0 m/s², kazanç 1.0 — çapraz doğrulama R² 0.98–0.99 (eski 0.85–0.88) | kazanç 0.996 (gerçek 0.996) |
| Dönüş miktarı | 5 yerinde turda sim +19° fazla (1811° / 1792°) | w kazancı 1.05 | 1.039 | 1792.5° / 1792.2° |

Sonuç (aynı komutlar, açık çevrim, 127 s): hız farkı RMSE v 0.035 → **0.022 m/s**, w 0.066 rad/s; tur sonunda sim
ile gerçek konum arasında **6.8 cm** (başlangıçtaki 4.8 cm / 3.2° fark dahil). `yorunge.png`.

## Kalan farklar (ölçülen)

| | Gerçek | Sim | Yorum |
|---|---|---|---|
| Tepki gecikmesi (v / w) | 0.00 / 0.04 s | 0.20 / 0.36 s | ~0.15 s'si oynatma zinciri (cmd_vel → taban_modeli → planar_move 20–50 Hz); w modelinin τ 0.2 s'si gerçekten yavaş — yeni w uydurması kararsızdı (iki bölmede farklı), değiştirilmedi |
| Odom ölçek hatası k_w / k_v | 0.9921 / 1.0162 | 0.9956 / 1.0188 | sensor_modeli 5 kaydın ortalamasını uyguluyor; tur sonu odom yön kayması gerçek −7.8°, sim −8.1° |
| IMU durağan gürültü (jiro σ, ivme σ) | 0.0008–0.0017 rad/s, 0.025 m/s² | aynı | ✓ |
| IMU durağan ivme ortalaması | (−0.15, −0.10, **10.04**) | (0.00, 0.00, 9.80) | gerçek kartta %2.4 ölçek + ~1° eğim/sapma; modellenmedi |
| IMU hareketteki titreşim | ivme σ (0.25, 0.27, 0.08), jiro artığı 0.047 rad/s | (0.08, 0.03, 0.03), 0.011 | gerçek gövde sürüşte titriyor (4 teker kaymalı dönüş); simde yok → VIO için önemli |
| IMU hızı | 93 Hz | 74–83 Hz | kayıt tarafında kayıp olabilir |
| Lidar, başlangıç pozunda (yön farkı −3.2° düzeltildi, 4.8 cm konum farkı düzeltilmedi) | — | — | açı başına medyan fark −2.4 cm, mutlak medyan 3.2 cm, %79'u 5 cm içinde, %8'i 20 cm dışında (kapı/sandalye gibi haritadan beri değişenler, kenarlar) |
| Lidar geçerli ölçüm oranı | %96.7 | %100 | gerçekte yüzeyde %3–10 kayıp; Gazebo ray sensörü vermiyor |
| Derinlik geçerli oranı (başlangıç karesi) | 0.59 | 0.66 | sim gölge/boşluk modeli kabaca tutuyor |
| Derinlik farkı (ikisinin de geçerli olduğu %33 piksel) | — | — | **sim gerçekten 6–14 cm yakın** (0.3–1 m −6.5, 1–2 m −13.8, 2–4 m −11.6 cm); 4–8 m +11 cm |
| Görüntü | renkli, dokulu | gri kutular | renk tabanlı algı / VIO için sim dünyasına doku gerekli |
| Yüksek nesneler | masa üstü, raf, sandalye sırtı | yok (2B haritadan 0.6 m yükseltilmiş kesit) | kamera simde farklı sahne görüyor |

### Derinlikteki sistematik 6–14 cm

Sim dünyası lidar haritasından üretildi; sim kamerası duvarları gerçek D435'ten 6–14 cm **yakın** görüyor. Aynı fark
daha önce iki kamerada da (Dabai, D435) kalibrasyon aramasında "+7..+9 cm dx" olarak çıkmıştı: kamera derinliği ile
lidar haritası arasında ~10 cm'lik ortak bir uyumsuzluk var. Olası nedenler: lidarın x konumu (0.137, ICP ile
tahmin), kameranın x konumu (0.084, ölçülmedi) ya da D435 derinlik ölçeği. Ayırmak için bilinen mesafe testi
(`saha/analiz/hedef_mesafe.py`) ve fiziksel ölçüm gerekli (olcum_protokolu.md 1.1–1.2). Simin değil, **gerçek
kalibrasyonun** sorunu; sim haritayla tutarlı.

## Görseller

- `yorunge.png` — gerçek (kırmızı) ve sim (mavi) yörünge, aynı komutlar, açık çevrim
- `lidar_baslangic.png` — başlangıç pozunda açı başına menzil ve fark
- `derinlik_baslangic.png` — gerçek renk/derinlik, sim derinlik, fark

## Sıradaki

1. Bilinen mesafe testi + lidar/kamera fiziksel konumu → derinlik–harita 10 cm farkını çözmek (gerçek tarafı).
2. IMU: hareket titreşimi (gövde ivme/jiro gürültüsünün hıza bağlı modeli) ve ivme ölçeği/sapması sensor_modeli'ne.
3. Sim dünyasına doku ve yüksek nesneler (masa, raf, sandalye) — kamera tabanlı işler (VIO, nesne tespiti) için.
4. Lidar: rastgele ölçüm kaybı (%3–10) sensor_modeli'ne (simde /scan da düğümden geçirilerek).

## İyileştirme turu 2 (27.10, sim-iyilestirme)

Eklenenler (`sensor_modeli_dugumu.py`, gerçek kayıtlardan ölçülen değerler): IMU ivme ölçeği 1.024 + x/y sapması ve
hıza/dönüşe bağlı titreşim gürültüsü; lidar %3.3 ölçüm kaybı (`scan_ham` → `scan`); w tepkisi gecikme 0, τ 0.16 s,
ivme 12 rad/s² (4 kayıt, R² ort 0.937, önceki 0.925); derinlikte zemin geçerliliği %20–40 ve lekeli boşluk.

| | Gerçek | Önce | Sonra |
|---|---|---|---|
| Hareketteki ivme σ (x, y, z) | 0.25, 0.27, 0.08 | 0.08, 0.03, 0.03 | 0.27, 0.20, 0.085 |
| Jiroskop artığı (hareket) | 0.047 | 0.011 | 0.047 |
| Durağan ivme ortalaması | (−0.15, −0.10, 10.04) | (0, 0, 9.80) | (−0.13, −0.09, 10.04) |
| Lidar geçerli oran | 0.967 | 1.00 | 0.97 |
| w tepki gecikmesi | 0.04 s | 0.36 s | 0.26 s |
| Toplam dönüş | 1792.2° | 1792.5° | 1792.3° |

Derinlik: boşluk önce geliş açısına bağlıydı → sim geçerli oranı 0.25'e düştü (gerçek 0.59; sıyırma açısındaki yan
duvar gerçekte geçerli). Yalnız zemine uygulanacak şekilde düzeltildi, birim testleri geçiyor; **simde
doğrulanmadı** (son oynatma bitmeden kapatıldı). Sıradaki ilk iş: oynatmayı tekrarlayıp ölçmek.
