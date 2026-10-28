# Gece görevi N22: son yaklaşmada uzakta tahmin sıçraması → iptal yerine yeniden hedefle
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. Claude test edip simde ölçecek.
## Ölçüm (Claude, kamera algılı uçtan uca "elma" görevi, ALGI=renk)
Görev TAMAM (P hatası 1.1 cm, 0.8°) ama ilk adayda: "SON_YAKLASMA algı aktif" → "KURTARMA HATA:sapma" → geri
çekilme → aday 2/3 → yakın kabul → HAZIR (27 s; sapma olmasa ~10 s). İlk ölçüm(ler) hatalı, ardından 3 tutarlı
doğru ölçüm NesneTahmini'nde aykırı sayacını doldurup tahmini sıçratıyor (son_yaklasma.py guncelle, aykiri_kabul_sayisi)
ve `sicrama > sapma_esik` → HATA:sapma.
## İstenen (src/limo_autonomy/limo_autonomy/son_yaklasma.py + test/test_son_yaklasma.py)
1. Yeni Ayar alanı `sapma_iptal_mesafe` (vars. 0.30 m): tahmin sıçraması rho > sapma_iptal_mesafe iken olursa
   İPTAL ETME — yeniden hedefle: yaklaşma yönünü (yaklasma_yaw_odom) yalnız dışarıdan VERİLMEMİŞSE yeni tahmine
   göre robot->nesne doğrultusundan yeniden hesapla, onceki_hedef'i sıfırla, sayaç `yeniden_hedef_sayisi` artır,
   bilgi sözlüğüne 'uyari': 'yeniden_hedef' koy. `maks_yeniden_hedef` (vars. 2) aşılırsa HATA:sapma (sonsuz döngü yok).
2. rho ≤ sapma_iptal_mesafe iken mevcut davranış (HATA:sapma) KORUNUR; rho < olcumsuz_mesafe'de dondurma (N17) aynen.
3. İki yerde sapma denetimi var (tahmin sıçraması ~satır 242 ve hedef sıçraması ~satır 266): ikisi de aynı kurala uysun;
   yeniden hedefleme sonrası hedef sıçraması denetimi o adımda tetiklenmesin.
4. Testler: (a) rho 0.45'te 3 tutarlı 0.25 m kaymış ölçüm → HATA yok, komut yeni hedefe yönelir, uyari 'yeniden_hedef';
   (b) rho 0.20'de aynı → HATA:sapma (mevcut); (c) maks_yeniden_hedef aşımı → HATA:sapma; (d) yön dışarıdan verilmişse
   yaklasma_yaw_odom değişmez. Mevcut testleri kırma.
Önce son_yaklasma.py ve test_son_yaklasma.py'yi TAMAMEN OKU. Yalnız bu iki dosya.
