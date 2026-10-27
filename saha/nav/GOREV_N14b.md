# Görev N14b: kavrama_yaklasim.py — N14 sonrası düzeltmeler (Claude ölçtü, lab_oda haritası, yaklasim_tani ile)
Dosyalar: src/limo_autonomy/limo_autonomy/kavrama_yaklasim.py, src/limo_autonomy/test/test_kavrama_yaklasim.py.
1. HATA: uyarlanabilir d_on yalnız koridoru denetliyor; Q çarpışırsa aday eleniyor (kuzey_duvar2 (1.2, 2.58):
   en iyi 3 aday 'Q_carpisma', θ 115–125°, açıklık 0.10). d_on taraması [d_on_max → d_on_min] sırasında koridor
   VE Q ayak izi (pay + konum_belirsizligi) birlikte sağlanan EN BÜYÜK d_on seçilsin; hiçbiri sağlanmazsa neden
   'Q_carpisma' (Q tümünde çarpışıyorsa) ya da 'koridor'.
2. P'deki güvenlik payı ayrı ayar: `pay_p` (vars. 0.03 m). Gerekçe yorumu: P'ye son 3 cm 3 cm/s sürünmeyle,
   nesneye göreli kapalı çevrimle ve lidar güvenlik durdurmasıyla varılıyor. P geçerlilik eşiği = pay_p +
   belirsizlik_p; Q ve koridor (Q tarafı) pay + konum_belirsizligi, koridor boyunca doğrusal geçiş.
3. test_dar_koridorda_koridor_kosulu_saglanmayan_aci_secilmez N14'ten sonra kalıyor (yeni modelde sentetik
   koridor geçerli sayılıyor, açıklık 0.11). Testin AMACINI koru: koridor genişliğini ayak izi + Q tarafı payının
   altına indir (ör. toplam genişlik 2*(0.11+0.05+0.06) - 0.02) ki bu açı reddedilsin; ve bu genişlikte
   yaklasim_tani nedeninin 'koridor' olduğunu doğrula.
4. Yeni test: duvara dik yaklaşmada Q çarpışırken daha kısa d_on ile aday BULUNUR (d_on < d_on_max).
Kodu çalıştıramazsın; Claude test edecek ve lab_oda'da şu hedefleri deneyecek: kuzey_duvar2 (1.2, 2.58) bulunmalı;
guney_duvar (4.0, -0.55) P açıklığı 0.038 < 0.045 olduğundan bulunmaması doğru.
