# Gece görevi N21: hareketli_kalkan sertleştirme (R1 incelemesinin kalan bulguları)
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. Claude test edip simde ölçecek.
Önce TAMAMEN OKU: src/limo_autonomy/limo_autonomy/hareketli_kalkan.py, test/test_hareketli_kalkan.py,
saha/gece/R1_RAPOR.md (B1.3, B1.4, B2.2, B2.3 bölümleri). Claude zaten düzeltti: B1.1 (lidar TF), B1.6 (iz yaşı),
B2.1 (kapanışta sıfır + launch respawn). B1.5 GEREKSİZ: LIMO lidarı 360° değil (-123°…+141°), sarma yok.

## Son sim ölçümü (lidar 6 Hz, sim ve gerçek LIMO)
Engelsiz tur 0 yanlış alarm (iyi, KORU). Yandan 0.8 m/s geçen insan: ilk DUR ttc=0.76 s'de geldi (ttc_dur 1.5 s) —
min_gorulme=5 + min_iz_yasi_s=0.4 s 6 Hz'de ~0.8 s tanıma gecikmesi. Karşıdan 0.6 m/s: ilk YAVAS ttc 2.6 s (iyi).

## İstenen
1. B1.3: tanıma ölçütünü tarama frekansından bağımsız yap: `min_gorulme` yerine/yanında süre temelli
   `min_iz_yasi_s` (vars. 0.4 → 0.35) ve `min_gorulme` vars. 5 → 3, AMA yön tutarlılığı (yon_tutarlilik 0.7,
   son N=3 ham hız) korunur. Yanlış alarm korumasını zayıflatmadığını gerekçelendir. Ham hızlar dt ile hesaplanıyor mu
   kontrol et (6 Hz'de dt≈0.167 s); sabit dt varsayımı varsa düzelt.
2. B1.4: |wz| > maks_donus_hizi iken hız güncellemesi atlanıyorsa görülme sayısı da artmasın (yoksa dönüş biter
   bitmez eski/tutarsız hızla tetiklenebilir) — R1'deki senaryoyu teste dök.
3. B2.2: odom geçmişi boşaldı/eskidi (poz_aradeger None, > 0.5 s) ise son karar KİLİTLİ kalmasın: izleri temizle,
   karar 'serbest' (güvenlik collision_monitor'da), /hareketli_kalkan/durum'a "ODOM_YOK" bir kez yayınla, uyar.
4. B2.3: tarama eskidiğinde (tarama_zaman_asimi) 'dur'dan anında 'serbest'e geçme: son karar 'dur' ise
   `eski_tarama_tutma_s` (vars. 0.5) boyunca dur'u tut, sonra serbest + "TARAMA_YOK" durumu.
5. Testler (ROS'suz): her madde için en az bir test; mevcut 185 testi kırma (değiştirmen gerekirse gerekçe yaz).
Dosyalar: yalnız hareketli_kalkan.py ve test_hareketli_kalkan.py.
