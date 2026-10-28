# Gece görevi N26: hareketli_kalkan — statik harita çıkarımı (duvar/mobilya noktalarını kümelemeden önce at)
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. Claude çevrimdışı oynatma (saha/gece/kalkan_oynat.py) + simde ölçecek.
## Ölçümler (Claude)
1. Kavrama son yaklaşmasında duvar boyunca (x=5.35, duvar ~5.7) yürüyen insan 44–46 s boyunca doğu duvarı kümesine
   karışıyor: küme merkezi (5.56, 0.9), yarıçap ~1.0 m → max_iz_yaricap (0.35) ile eleniyor; insan yalnız birkaç
   taramada ayrı görünüyor (r 0.26). kumele (ardışık nokta eşiği 0.10) doğru çalışıyor; sıyırma açısında insan-duvar
   noktaları zincirleniyor.
2. N25 (global eşleşme) sonrası engelsiz 7 hedefli turda 10 DUR + 6 YAVAS yanlış alarm; hızlar çoğunlukla 0.20–0.30 m/s
   (min_engel_hizi 0.20 hemen üstü) — duran dağınık parçaların (duvar parçaları, mobilya ayakları) gürültü hızları.
## İstenen (src/limo_autonomy/limo_autonomy/hareketli_kalkan.py + test)
1. Parametre `harita_cikar` (bool, vars. true), `harita_konu` (vars. '/map', QoS: transient_local, reliable — map_server
   latched yayın), `harita_pay_m` (vars. 0.10), `harita_esik` (vars. 65; OccupancyGrid değeri ≥ eşik dolu).
2. Haritadan dolu hücre maskesini `harita_pay_m` kadar genişlet (saf numpy ikili genişletme; scipy kullanma — kare
   çekirdekle kaydırma/OR yeterli) ve sakla.
3. Her taramada: tarama noktalarını (base_link) map çerçevesine çevir (TF map<-base_link, taramanın damgasında;
   bulunamazsa en son; o da yoksa çıkarımı o tarama için atla ve bir kez uyar), genişletilmiş dolu hücreye düşen
   noktaları AT, kalan noktaları (sıra korunarak) kumele'ye ver. Saf fonksiyonlar: `harita_maskesi_genislet(grid, pay_hucre)`,
   `haritada_dolu_mu(noktalar_map, maske, cozunurluk, orijin) -> bool dizisi`.
4. Harita yoksa (konu yayınlanmıyor) mevcut davranış aynen (geri uyum). /hareketli_kalkan/durum'a etkisi yok.
5. Lokalizasyon kayması: map->odom yanlışsa duvarlar çıkarılamaz, davranış eskisine döner (kötüleşmez) — modül
   belgesine yaz. Kayma saf odom modunda (map->odom statik) büyüyebilir.
6. Testler (ROS'suz): genişletme; dolu hücre sorgusu (sınır/orijin/çözünürlük, harita dışı nokta → dolu DEĞİL);
   duvar + 0.25 m yanında insan sentetik taramasında çıkarım sonrası insan ayrı küme (r ≤ 0.3); çıkarım kapalıyken
   eski davranış. Mevcut testleri kırma.
Önce hareketli_kalkan.py (düğüm + saf fonksiyonlar) ve test_hareketli_kalkan.py OKU. Yalnız bu iki dosya.
