# İnceleme R2: renk_algi uzak menzilde sistematik ileri sapmanın kaynağı (yalnız oku, KOD DEĞİŞTİRME)
ÖNEMLİ: terminal iznin YOK. Yalnız saha/gece/R2_RAPOR.md yaz.
## Ölçüm (zemindeki elma r=0.035 m, merkez z=0.04; robot gövde çerçevesi, kaydedici ölçüm anındaki gerçek pozla)
Ölçülen − gerçek (x ileri): 0.83 m'de +2.4 cm, 1.0 m +3.0, 1.5 m +3.1, 2.0 m +3.6, 2.5 m +4.0, 3.0 m +4 cm;
yanal (y) ~+1 cm sabit; ölçülen z +0.05…+0.07 (gerçek 0.04) — z hatası menzille artıyor. 0.6 m içinde < 1 cm.
Sensör modeli (src/limo_kinematics/limo_kinematics/sensor_modeli_dugumu.py derinlik_bozu) yalnız σ=a·z² gürültü
ekliyor; ölçek/sapma yok (doğrula).
## İncele
1. URDF/xacro kamera çerçeveleri (src/limo_description/urdf/ — camera_link, camera_color_optical_frame,
   camera_depth_optical_frame vb. joint origin'leri) ile Gazebo kamera eklentilerinin (d435_renk_eklenti,
   d435_derinlik_eklenti; <pose>, <frame_name>, hack_baseline, görüş alanı/çözünürlük) gerçek sensör pozlarını
   karşılaştır. Sensörün görüntü ürettiği poz ile TF'te yayınlanan çerçeve farklı mı? (özellikle x ofseti ve pitch)
2. renk_algi.py: renk->derinlik dış parametreleri (T_RENK_DERINLIK_VARSAYILAN, TF okuma), iç parametreler
   (camera_info'dan mı, sabit mi), derinlik birimi (16UC1 mm), yüzey->merkez ötelemesi (isin_derinlik_3b, oklid_mi).
3. Gözlenen deseni (sabit ~+2…3 cm + menzille artan küçük bileşen, z'nin menzille artması) üreten aday hataları
   sayısal olarak kestir: ör. kamera pitch farkı δ → z hatası ≈ d·δ; x ofset farkı → sabit x hatası; derinlik/renk
   çerçeve karışıklığı; Öklid vs Z derinlik karışıklığı (merkezden uzak piksellerde). Her aday için beklenen
   hata eğrisini ölçümle karşılaştır, en olasıyı seç.
Her bulgu: dosya:satır + ilgili satır alıntısı, sayısal kestirim, önerilen düzeltme, güven (yüksek/orta/düşük).
Emin olmadığını "doğrulanmalı" yaz; uydurma yok.
