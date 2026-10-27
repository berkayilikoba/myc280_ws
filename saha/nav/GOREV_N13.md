# Görev N13: ikinci uçtan uca denemenin düzeltmeleri (Claude teşhisi, cikti/k2)
Sonuç: dolap KAVRANABILIR (0.5 cm / 0.5°, 7.5 s son yaklaşma) — mimari doğru. Masa başarısız; üç kusur:

A) kavrama_dugumu.py KURTARMA yarışı: KURTARMA bitince 50 Hz döngü aynı dalı iki kez çalıştırıyor (günlükte
   "KURTARMA tamamlandı; sonraki aday 2/3" ve hemen "3/3"): kurtarma_sayisi ve aktif_aday_idx iki kez artıyor,
   aday 2 atlanıyor. Düzelt: durumu (ör. self.durum_adi) aday artırma/NAV_Q başlatmadan ÖNCE geçiş durumuna al;
   tüm durum geçişlerinde aynı korumayı uygula (bir dal yalnız bir kez). Test: saf durum geçiş yardımcısı ya da
   sahte zamanlayıcıyla iki ardışık tikte tek artış.
B) son_yaklasma.py görüş koruma: denetleyici w_max'a doyup robotu nesneden çevirdi; nesne kamera görüşünden
   (yatay ±35°) çıktı, ölçüm kesildi, rho=0.16'da 'HATA:zaman_asimi'. İstenen:
   - Ayar `gorus_yari_aci` (vars. 35°) ve `gorus_payi` (vars. 10°): robottan nesneye kerteriz |b| >
     (gorus_yari_aci - gorus_payi) olacaksa (bir sonraki adımın tahmini dahil) w'yi kerterizi azaltacak yöne
     kırp; v'yi kerteriz büyüdükçe azalt (b → sınır iken v → v_min). Son bantta (aşama A/B) bu kısıt uygulanmaz.
   - Ölçüm zaman aşımı yalnız rho > `olcumsuz_mesafe` (yeni, vars. 0.25 m; eski sabit 0.10 yerine) iken hata;
     daha yakında son tahminle (odom çerçevesinde) devam et (25 cm'de odom kayması ~mm).
   - Testler: başlangıçta nesne 30° yandayken kerteriz hiçbir adımda 25°'yi aşmaz ve TAMAM; ölçüm 0.2 m'de
     kesilirse TAMAM, 0.4 m'de kesilirse HATA. Mevcut testleri GEVŞETME.
C) Kol erişimi (myCobot 280: taban yerden ~0.19 m, erişim ~0.28 m → kavranabilir z ≈ 0.0–0.45 m):
   kavrama_dugumu.py PLAN'da nesne z'si [z_min (param, vars. 0.0), z_max (param, vars. 0.45)] dışındaysa
   'HATA erisilemez yukseklik z=..' ve dur. sahte_algi.py: dikey görüş testi ekle — kamera yüksekliği
   `kamera_z` (vars. 0.18 m), dikey yarım açı `dikey_yari_aci` (vars. 21°): nesneye yükselme açısı bu aralık
   dışındaysa görünmez; gorus_testi'ne isteğe bağlı nesne_z/kamera_z parametreleri (geriye uyumlu) + test.
D) src/limo_bringup/scripts/kavrama_hedefler_lab_oda.yaml: masa hedefini (z 0.75) `beklenen: HATA` alanıyla
   işaretle (olumsuz test); kavrama_bench.py 'beklenen: HATA' olan hedefte HATA sonucunu BAŞARILI say (kabul
   sütununda 'BEKLENEN_HATA'). Yeni erişilebilir hedef ekleme (Claude haritadan seçecek).
