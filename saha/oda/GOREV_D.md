# Görev D: oda_doku.py — kamera görüntülerinden duvar ve zemin dokuları (Gazebo Classic malzemeleri)

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_doku.py (yalnız bu ve çıktı dizinine yazdığı dosyalar). Python 3.10,
numpy, cv2, yaml, argparse. Türkçe yorum, kısa kod.

## Girdiler
- --yapi oda_yapi.yaml (duvarlar [{p1: [x,y], p2: [x,y], uzunluk, kalinlik}], oda_siniri [[x,y],...])
- sahne dizinleri (biçim Görev A ile aynı: rgb/*.png BGR 640x360, depth/*.npy metre renge hizalı, poses.txt gövde,
  poses_kamera.txt renk optik kamera, x y z qx qy qz qw harita çerçevesi, vlmaps_config.yaml cam_calib_mat)
- poz düzeltmesi --dx 0.09 --dy 0.01 --dyaw -1.0 (Görev A'daki gibi: T_wc_duz = T_wb @ dT @ inv(T_wb) @ T_wc)
- --tavan 2.7, --cozunurluk 0.01 (m/piksel), --her 3, --max_w 0.25 (açısal hız süzgeci, Görev A gibi)

## Duvar dokusu (her duvar i)
- Duvar düzlemi: p1->p2 doğrusu boyunca, duvarın ODA İÇİNE bakan yüzü (oda_siniri çokgeninin içine doğru normal; yüzey
  duvar çizgisinden kalinlik/2 içeride). Doku koordinatı: s = p1'den uzaklık (0..uzunluk), t = yükseklik (0..tavan).
  Doku boyutu W = ceil(uzunluk/coz), H = ceil(tavan/coz).
- Her kare için: dokunun her tekseline karşılık gelen 3B noktayı kameraya izdüşür (K, T_wc_duz), görüntü içinde mi ve
  kameranın önünde mi kontrol et; ÖRTÜŞME: o pikseldeki ölçülen derinlik (depth npy) teksel derinliğinden 0.15 m'den
  fazla küçükse (önünde nesne var) kullanma; derinlik 0 ise ve teksel 3 m'den uzaksa kullanma. Görüş açısı ağırlığı
  w = cos(ışın ile duvar normali açısı) * (1 / mesafe); renkleri ağırlıklı ortalama ile biriktir (float toplam + ağırlık).
  Hızlı olmalı: tekselleri vektörel izdüşür (numpy), kare başına yalnız görüntü çerçevesine düşenler.
- Hiç görülmeyen teksel: aynı sütunda görülen en üst tekselin rengi yerine duvarın görülen tekselleri medyan rengi
  (tavan tarafı düz duvar rengi), ardından 15 px Gauss ile görülen/görülmeyen geçişi yumuşat (yalnız görülmeyenlere).
- Kayıt: <cikti>/doku/duvar_<i>.png (BGR), maske duvar_<i>_gorulen.png (görülen teksel 255).

## Zemin dokusu
- Düzlem z = 0, oda_siniri sınırlayıcı kutusu; doku çözünürlüğü --zemin_coz (0.01). Aynı izdüşüm/örtüşme yöntemi (normal
  +z). Görülmeyen yerler zemin görülen medyan rengi. <cikti>/doku/zemin.png, zemin_gorulen.png; kutu sınırları yaml'a.

## Gazebo malzemeleri
- <cikti>/materials/scripts/oda.material: her doku için OGRE malzemesi:
  material Oda/duvar_<i> { technique { pass { ambient 0.8 0.8 0.8 1.0 diffuse 1 1 1 1 texture_unit {
    texture duvar_<i>.png filtering trilinear } } } } (zemin için Oda/zemin)
- <cikti>/materials/textures/: PNG'lerin kopyası (Gazebo dosya adıyla bulur).
- <cikti>/doku.yaml: duvarlar: [{i, p1, p2, yuz_normali [nx, ny], doku, malzeme, gorulen_oran}], zemin: {kutu: [xmin,
  ymin, xmax, ymax], doku, malzeme, gorulen_oran}, tavan, cozunurluk.

## CLI
oda_doku.py <cikti_dizini> --yapi Y <sahne> [<sahne> ...] [--dx --dy --dyaw --tavan --cozunurluk --zemin_coz --her --max_w]

## Kabul
- v2a_lidar sahnesi + gerçek oda_yapi.yaml ile hatasız; her duvarın gorulen_oran'ı yazılır; PNG'ler duvar boyutunda.
- Bellek: 10 m x 2.7 m duvar 1 cm'de 1000x270 teksel; tüm duvarlar < 2 GB RAM.
