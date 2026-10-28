# Görev N3: kavrama_yaklasim.py — yaklaşma seçimini odom kaymasına ve düz son yaklaşmaya dayanıklı yap
Dosyalar: src/limo_autonomy/limo_autonomy/kavrama_yaklasim.py, src/limo_autonomy/test/test_kavrama_yaklasim.py
(mevcut testleri bozma; yeni test ekle). Mevcut API (`Ayar`, `Yaklasim`, `yaklasim_bul`, `Harita`) geriye uyumlu
kalsın; `icp2d` fonksiyonu dokunulmadan kalabilir (artık kullanılmayacak, silme).

Önce dosyayı ve testini OKU. İstenen eklemeler (Ayar'a alan + puanlamaya katkı):
1. Düz koridor koşulu: Q→P doğrusu boyunca (d_on uzunluğunda) ayak izi (pay + `konum_belirsizligi`, vars. 0.06 m)
   ile engelsiz olmalı; değilse aday geçersiz. Bu, son yaklaşma denetleyicisinin düz/yumuşak gidebilmesi için.
2. Q açıklığı: Q'da ayak izinin en yakın engele uzaklığı da hesaplanıp puana katılsın (Nav2 Q'ya kayık varabilir;
   açıklık doyumu acik_doyum).
3. Yüzey normali tercihi: nesnenin çevresindeki (r = 0.30 m) dolu hücrelerden (nesne dairesi r_nesne hariç) baskın
   kenar doğrultusunu (PCA) bul; nesne bir masa/duvar kenarındaysa yaklaşma yönünü kenara DİK ve dış taraftan
   tercih et (`yuzey_agirlik`, vars. 1.0, puana cos(açı farkı) katkısı). Kenar yoksa katkı 0.
4. `d_on` artık aday başına uyarlanabilir: [d_on_min 0.30, d_on_max 0.60] aralığında 0.05 adımla, koridor koşulunu
   sağlayan EN BÜYÜK d_on seçilsin (kamera nesneyi 0.3 m'den uzakta daha iyi görür, son yaklaşma daha uzun düz yol).
   Yaklasim'a `d_on` alanı ekle.
5. `yaklasim_bul(..., hepsi=True)` en iyi K'yı puan sırasıyla döndürsün (zaten varsa koru) — kurtarma için 2./3.
   aday kullanılacak; birbirine çok yakın açıları (< 20°) tekrar etme.
Testler: (a) duvar dibindeki nesnede seçilen yaklaşma duvara dik (±10°); (b) dar koridorda koridor koşulu
sağlanmayan açı seçilmez; (c) d_on engel yokken 0.60, önünde 0.45 m'de engel varken daha küçük; (d) hepsi=True
sonuçlarında açılar en az 20° ayrık.
