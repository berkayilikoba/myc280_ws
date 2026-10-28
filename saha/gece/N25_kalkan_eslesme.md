# Gece görevi N25: hareketli_kalkan eşleşme — dağınık ortamda insan izi sürekli sıfırlanıyor (kalkan kör)
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. Claude çevrimdışı oynatma + simde ölçecek.
## Ölçüm (Claude; saha/gece/kalkan_oynat.py ile kaydedilmiş /scan+/odom'un IzTakipci'den geçirilmesi)
Senaryo: robot doğu duvarındaki nesneye son yaklaşmada (~(4.95,0.73) odom), insan 0.5 m/s kuzeyden güneye x=5.35'te
geçiyor, en yakın merkez 0.36 m (temas). Kalkan HİÇ tetiklenmedi. İz dökümü (x, y, |v|, görülme, yaş):
46.17 (5.32, 0.57, 0.52, 1, 0.83) → sonraki taramalarda iz yok/yeni iz;
47.51 (5.20, 0.04, 0.46, 1, 1.0) → 47.84 görülme 2 → 48.01 görülme 3 (insan çoktan geçmiş).
Aynı anda yakında DURAĞAN parçalar sürekli görülme 0'da: (4.85,0.08) (4.57,0.12) (4.66,0.10) (4.31,0.04) —
duvar/mobilya parçaları birbirine < eslesme_kapi (0.35 m). N19'daki "kapıda ≥2 aday ya da kümeye ≥2 iz → hız ve
görülme SIFIRLA" kuralı bu dağınık ortamda sürekli tetikleniyor; insan izi de bu yüzden onaylanmıyor (min_gorulme 3,
min_iz_yasi_s 0.35, yön tutarlılığı).
## İstenen (src/limo_autonomy/limo_autonomy/hareketli_kalkan.py IzTakipci + test)
1. Eşleştirmeyi GLOBAL yap: tüm (iz, küme) çiftleri için öngörülen konuma (x + vx·dt) uzaklık; kapı içindekileri
   artan uzaklıkla açgözlü (greedy) ata (her iz ve her küme en fazla bir kez). Saf fonksiyon `eslestir(...)`.
2. Belirsizlik sıfırlamasını DARALT: yalnız bir izin en iyi iki adayı birbirine çok yakın ve ikisi de yakınsa
   (d2 - d1 < `belirsizlik_payi`, vars. 0.05 m, VE d1 > 0.10 m) o izin HIZINI sıfırla; görülme sayısını SIFIRLAMA
   (yarıya indir). Durağan yakın parçalar için sıfırlama izi zaten durağan bırakır; hareketli iz onayını yok etmesin.
3. İz kimliği kopmasın: eşleşmeyen iz 0.5 s (iz_silme_suresi) yaşar; tekrar görülünce öngörülen konum (sabit hız)
   ile yakalanabilsin — kapı, iz hızına göre büyüsün: kapı = eslesme_kapi + |v|·dt (dt son görülmeden beri).
4. Testler: (a) 0.4 m arayla 3 durağan küme + aralarından 0.5 m/s geçen insan (6 Hz, gürültü σ 1 cm): insan izi en
   geç 0.7 s'de onaylanır (görülme ≥3, |v|≈0.5) ve TTC kararı üretilir; (b) durağan kümeler hareketli SAYILMAZ (N19
   yanlış alarm testleri geçmeye devam etsin); (c) 1 tarama kaçırılan insan izi aynı kimlikle devam eder.
   Mevcut testleri kırma (değiştirirsen gerekçe yaz).
Önce hareketli_kalkan.py (IzTakipci tamamı) ve test_hareketli_kalkan.py OKU. Yalnız bu iki dosya.
