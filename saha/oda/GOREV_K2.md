# Görev K2: renk kamerası (D435) bozulma modeli — ölçüm betiği + saf fonksiyonlar

Amaç: simdeki renk görüntüsü (Gazebo, kusursuz) gerçek D435 renk görüntüsüne istatistik olarak benzesin (görsel
odometri çalışan ekip arkadaşı için). İki YENİ dosya:

## A) saha/oda/renk_karakter.py — gerçek veriden parametre ölçümü
Veri: sahne dizinleri (ör. ~/limo_veri/vlmaps_sahne/v2a_lidar, v2b_lidar, rs720_kapsama_lidar):
rgb/NNNNNN.png (640x360 BGR; orijinal 1280x720'den küçültülmüş — bunu raporda belirt), poses.txt (gövde
"x y z qx qy qz qw"), zaman.txt ("i t t"). CLI: renk_karakter.py <cikti_dizini> <sahne>... [--her 3]
Ölç:
1. Gürültü-parlaklık eğrisi: her karede düşük dokulu yamalar (8x8, yerel gradyan küçük) seç; yama içi std'yi
   (yüksek geçiren: yama - Gauss(σ=1.5) ile) yama ortalama parlaklığına karşı topla; kanal başına
   σ²(I) = a·I + b (Poisson-Gauss) uydur. a, b raporla.
2. Vinyet: tüm karelerin parlaklık ortalaması (kare başına kendi medyanına normalize) → merkezden yarıçapa göre
   radyal profil; I(r)/I(0) = 1 + k1 r² + k2 r⁴ (r normalize, köşe=1) uydur.
3. Hareket bulanıklığı: kare başına Laplacian varyansı vs gövde açısal hızı |w| (poses+zaman'dan) ve doğrusal
   hız. Bulanıklık uzunluğu modeli: piksel cinsinden L ≈ |w|·fx·t_poz (fx=909 1280 genişlikte, 640'ta 454.5).
   Laplacian oranını (sabit sahneye göre) en iyi açıklayan efektif pozlama süresi t_poz'u bul (ızgara arama:
   kendi bulanık kernelini keskin karelere uygulayıp Laplacian düşüşünü eşleştirerek). t_poz raporla.
4. Renk istatistikleri: kanal ortalamaları/oranları (beyaz denge), doygunluk dağılımı (HSV S medyanı),
   parlaklık histogramı yüzdelikleri (p1, p50, p99).
Çıktı: <cikti>/renk_param.yaml + grafikler (gurultu.png, vinyet.png, bulaniklik.png; matplotlib Agg).

## B) src/limo_kinematics/limo_kinematics/renk_bozu.py — saf fonksiyonlar (ROS'suz, numpy+cv2)
- `vinyet_uygula(img, k1, k2)`, `gurultu_uygula(img, a, b, rng)` (Poisson-Gauss, kanal başına),
- `hareket_bulanikligi(img, w_z, v_x, t_poz, fx, derinlik_ortalama=2.0)`: yaw dönüşü yatay çizgi kernel
  (uzunluk |w_z|·fx·t_poz px), ileri hareket ihmal edilebilir; uzunluk < 1 px ise dokunma,
- `beyaz_denge_uygula(img, kazanc_bgr)`,
- `renk_boz(img, param: dict, w_z, v_x, rng)`: hepsini sırayla uygular (bulanıklık → vinyet → beyaz denge →
  gürültü), uint8 döndürür. param anahtarları renk_param.yaml ile aynı adlarda.
- Dosyanın sonuna kısa docstring ile parametre anlamları. Mevcut dosyalara (sensor_modeli_dugumu.py vb.)
  DOKUNMA; entegrasyonu Claude yapacak.
- src/limo_kinematics/test/ dizinine yazma iznin olmayabilir; testleri saha/oda/test_renk_bozu.py olarak yaz
  (importlib ile src/limo_kinematics/limo_kinematics/renk_bozu.py'yi yükle): sabit görüntüde vinyet köşeyi
  karartır, gürültü std'si a·I+b ile uyumlu (büyük görüntüde %10 tolerans), w_z=0'da bulanıklık kimlik,
  çıktı dtype uint8 ve boyut korunur.
Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
