# Görev G1: iki küçük düzeltme (oda_doku.py ve oda_yapi.py; ikisini de DÜZENLE, diğer davranışları koru)

## 1. oda_doku.py — zemin dokusunda mobilya altı
Sorun: masa/dolap gibi zemin üstündeki nesnelerin kameradan görülen yüzeyleri, örtüşme denetimine rağmen zemin
dokusuna "yayılıyor" (zemin kenarlarında uzun çizgisel lekeler). Düzeltme:
- --yapi YAML'ındaki mobilya çokgenlerini (mobilya[*].cokgen, harita metre) zemin doku ızgarasına cv2.fillPoly ile çiz,
  0.05 m genişlet (dilate) -> "mobilya_alti" maskesi. Ayrıca duvar parçalarının (duvarlar[*].p1, p2, kalinlik) 0.05 m
  çevresi.
- Bu maskedeki tekseller biriktirmeye katılmasın; sonunda görülmeyen gibi zemin medyan rengiyle dolsun.
- EK süzgeç: bir teksel için gerçek derinlik ile teksel derinliği farkı 0.15 m yerine zemin için 0.05 m olsun
  (zemin sıyırma açısında görülüyor, önündeki nesne çok yakın düşer).
- doku.yaml zemin kaydına "maskeli_oran" ekle.

## 2. oda_yapi.py — çapraz sahte duvarlar
Sorun: dolap kümeleri gibi yoğun dolu bölgelerde, doluluk ölçütünü geçen çapraz parçalar kalıyor.
- Odanın baskın iki ekseni: duvar parçalarının uzunluk ağırlıklı açı histogramı (0-180°, 2° kutular) -> en büyük tepe
  θ1, θ2 = θ1 + 90°.
- Bir parça θ1 ya da θ2'ye --eksen_tol (10°) içinde değilse VE doluluk < --capraz_doluluk (0.9) ise at.
- Özette "eksen dışı atılan" sayısı, YAML'a eksenler: [θ1, θ2] (derece).
CLI'ye --eksen_tol, --capraz_doluluk ekle.

Kabul: python3 -m py_compile geçer; gerçek veride (sen çalıştıramazsın) davranış yukarıdaki gibi.
