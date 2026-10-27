# Görev B: oda_yapi.py — 2B doluluk haritasından duvarlar ve mobilya ayak izleri

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_yapi.py (yalnız bu dosya). Python 3.10, numpy, cv2, yaml, argparse.
Türkçe yorum, kısa kod.

## Girdi
map_server biçimi harita: <harita.yaml> (anahtarlar image, resolution, origin [x, y, yaw], negate, occupied_thresh,
free_thresh) + pgm (0 = dolu, 254 = boş, 205 = bilinmiyor). pgm satır 0 = en büyük y. Hücre (satır v, sütun u) merkezi:
x = ox + (u + 0.5)*r, y = oy + (H - v - 0.5)*r. Örnek: ~/limo_robot/limo_ws/src/limo_bringup/maps/lab_oda.yaml (r 0.02).

## Algoritma
1. dolu = p > occupied_thresh (p = (255 - piksel)/255); serbest = piksel > 250.
2. ODA SINIRI: serbest alanın en büyük bağlı bileşeni (8-komşu) -> küçük boşlukları kapat (morph close 5 hücre) ->
   dış konturu (cv2.findContours RETR_EXTERNAL) -> cv2.approxPolyDP (epsilon --eps 0.05 m) -> çokgen (harita koordinatı).
3. DUVAR: dolu hücrelerden, oda sınırı çokgeninin kenarına --duvar_pay (0.10 m) içinde olanlar duvar adayı. Duvar
   adaylarında doğru parçaları: cv2.HoughLinesP (rho 1 hücre, theta 1°, threshold 15, minLineLength --min_duvar (0.4 m),
   maxLineGap 0.15 m) -> aynı doğrultudaki (açı farkı < 5°, doğru uzaklığı < 0.05 m) ve üst üste/ardışık (boşluk < 0.3 m)
   parçaları birleştir -> her duvar: [x1, y1, x2, y2] (m), kalinlik = --kalinlik (0.10). Kapı açıklıkları birleştirmede
   0.6 m'den büyük boşluk varsa ayrı parça kalsın.
4. MOBİLYA: dolu hücrelerden duvar parçalarına (doğru parçası uzaklığı) > 0.06 m olanlar; bağlı bileşenler (8-komşu,
   önce 1 hücre dilate) -> alanı < --min_alan (0.0012 m², 3 hücre) olanları at -> her bileşen: dışbükey zarf çokgeni (m),
   merkez, alan, hücre sayısı, ve "ince" bayrağı (en küçük dikdörtgen kısa kenarı < 0.06 m: masa/sandalye ayağı gibi).
5. Çıktı YAML (<cikti>.yaml):
   kaynak: harita yolu; cozunurluk; oda_siniri: [[x,y],...]; duvarlar: [{p1: [x,y], p2: [x,y], uzunluk, kalinlik}];
   mobilya: [{id, merkez: [x,y], alan_m2, cokgen: [[x,y],...], ince: bool}]
6. Hata ayıklama PNG (<cikti>.png): harita gri, duvarlar kırmızı çizgi (2 px, 4x büyütülmüş), mobilya yeşil çokgen,
   oda sınırı mavi.

## CLI
oda_yapi.py <harita.yaml> <cikti_oneki> [--eps 0.05 --duvar_pay 0.10 --min_duvar 0.4 --kalinlik 0.10 --min_alan 0.0012]

## Kabul
- lab_oda.yaml ile hatasız; oda sınırı yaklaşık x -2.5..8.3, y -1.4..3.1 içinde; en az 6 duvar parçası; toplam duvar
  uzunluğu > 15 m; mobilya listesi boş değil.
