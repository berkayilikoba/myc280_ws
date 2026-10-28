# Gece görevi N19: hareketli_kalkan yanlış alarmları (durağan nesneler hareketli görünüyor)
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. Claude test edip simde ölçecek.

## Ölçüm (Claude, lab_oda_3b, hareketli_kalkan:=true)
- İyi: karşıdan insan → temasta robot DURUYOR (engel_carpti, v 0.000); yandan geçen → temas yok (+0.13 m);
  duran_engel ve takip senaryolarında 0 tetikleme.
- KÖTÜ: HİÇ hareketli engel olmayan 7 hedefli gezinme turunda 9 DUR + 7 YAVAS. Örnek durum metinleri:
  "DUR ttc=1.17s v=1.02", "DUR ttc=0.74s v=0.60", "DUR ttc=0.94s v=0.30", "YAVAS ttc=2.90s v=0.49".
  Tur süresi 120 s → 150 s. Odada masa/sandalye ayakları, dolap, duvar köşeleri var.

## Muhtemel nedenler (hepsini ele al)
1. Zaman uyuşmazlığı: `_scan_cb` taramayı EN SON odom pozuyla odom çerçevesine çeviriyor. Robot ~1 rad/s
   dönerken 50–100 ms fark 1 m'deki noktayı 5–10 cm kaydırır → sahte 0.5–1 m/s hız.
   ÇÖZÜM: odom geçmişi tamponu (son 1 s, (t, x, y, yaw)); taramanın header.stamp anındaki pozu doğrusal
   aradeğerle (yaw wrap'e dikkat) bul; tampon aralığı dışındaysa en yakını kullan, 0.2 s'den uzaksa o taramayı
   izleyiciye VERME. Saf fonksiyon: `poz_aradeger(gecmis, t)`.
2. Belirsiz eşleşme: yakın küçük kümeler (sandalye/masa ayakları ~0.3–0.5 m arayla) en yakın komşuda
   karışıyor. ÇÖZÜM: IzTakipci'de bir iz için kapı içinde ≥2 aday küme varsa ya da bir kümeye ≥2 iz
   adaysa o izin hız kestirimini SIFIRLA ve görülme sayısını 0'a çek (yeniden kanıtlamalı). Eşleştirmede
   izin öngörülen konumunu (x + vx·dt) kullan.
3. Kısa ömürlü/tutarsız izler: kalkan_karari'nda hareketli sayılmak için görülme ≥ `min_gorulme` (vars. 3 → 5)
   VE iz yaşı ≥ `min_iz_yasi_s` (vars. 0.4 s) VE hız yön tutarlılığı: izin son N (vars. 5) ham hız ölçümünün
   ortalama birim vektörünün uzunluğu ≥ `yon_tutarlilik` (vars. 0.7). Ham hızları iz içinde tut.
4. Dönüş sırasında güvenilmezlik: robot |wz| > `maks_donus_hizi` (vars. 0.8 rad/s) iken izlerin hız
   güncellemesini atla (konumu güncelle, hızı koru), çünkü küçük zaman hataları büyür.
Mevcut testleri kırma, güncellemen gerekirse gerekçesini yaz. Yeni testler (ROS'suz): (a) robot yerinde 1 rad/s
dönerken, tarama 80 ms gecikmeli ve DOĞRU aradeğerli pozla verilince durağan noktaların hızı < 0.1 m/s;
(b) 0.4 m arayla iki durağan küme, gürültülü ölçümle 30 adımda 'serbest' kararı; (c) karşıdan 0.6 m/s gelen
insan hâlâ ≤ 1.0 s içinde 'dur' tetikler (gerçek tehditi kaçırma); (d) poz_aradeger yaw wrap (179° → −179°).
Dosyalar: src/limo_autonomy/limo_autonomy/hareketli_kalkan.py, src/limo_autonomy/test/test_hareketli_kalkan.py.
Önce ikisini TAMAMEN OKU (Claude son düzenlemeleri yaptı: max_iz_yaricap filtresi, örtüşmede uzaklaşırken inf).
