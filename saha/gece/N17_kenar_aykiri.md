# Gece görevi N17: son yaklaşmada kısmi görünen nesnenin yanlış ölçümü kavramayı iptal ediyor
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude test edip simde koşturacak.

## Ölçüm (Claude, saha/gece/algi_sim_test.sh, renk_algi + Gazebo elması r=0.035 m, zeminde)
Son yaklaşmada algı hatası (cm, Gazebo gerçeğine göre) sırayla:
0.8 0.3 0.1 … 1.5 1.7 1.7 **49.4 49.0 49.6** 3.2 4.0 — robot nesneye rho≈0.17 m iken elma görüntünün ALT
kenarına giriyor, leke kesiliyor, derinlik penceresi zemine/arka plana düşüyor. `NesneTahmini.guncelle`
(son_yaklasma.py:129) 3 ardışık aykırıyı "nesne gerçekten yer değiştirdi" sayıp kabul ediyor → `HATA:sapma`
(son_yaklasma.py:242) → kurtarma → aday 2/3 → … → "HATA tüm adaylar denendi". Algı geri kalan sürede 2 cm.

## İstenen (iki katman, ikisi de)
A) `src/limo_autonomy/limo_autonomy/renk_algi.py`: lekenin sınırlayıcı kutusu (connectedComponentsWithStats
   stats, satır ~134) görüntü kenarına `kenar_payi_px` (parametre, vars. 4) kadar yakınsa ölçüm YAYINLANMASIN;
   durum metni "KISMI" (mevcut durum yayınlama kalıbına uy; `/kavrama/nesne_algi_durum` değerlerini oku, yeni
   değer eklediğini modül belgesinde belirt). Ayrıca derinlik penceresindeki geçerli piksellerin medyanı ile leke
   merkezinin beklenen boyutu tutarsızsa (görünen yarıçap_px · derinlik / fx ile kestirilen fiziksel yarıçap,
   `beklenen_yaricap_m` vars. 0.035 ±%60 dışında) reddet — parametreyle kapatılabilir (`boyut_denetimi` vars. true).
   Mevcut saf fonksiyon yapısına uy; yeni mantık saf fonksiyon olsun ve test edilsin (test_renk_algi.py'ye ekle).
B) `src/limo_autonomy/limo_autonomy/son_yaklasma.py`: robot→hedef mesafesi rho `olcumsuz_mesafe` (0.25) altına
   indikten sonra nesne tahmini DONDURULSUN: ölçümler tahmini güncellemez, sapma denetimi yapılmaz (kamera kör
   noktası; odomla tamamlanır — zaten ölçüm kaybı bu bölgede tolere ediliyor, satır ~286). Bu geçişi bilgi
   sözlüğüne (`_bos_bilgi`/bilgi alanları neyse) `tahmin_donduruldu` olarak ekle. Ayrıca sapma kararı için
   NesneTahmini'ne ardışık aykırı sayısı eşiği parametresi (`aykiri_kabul_sayisi`, vars. 3 kalsın) ekle, Ayar'dan
   geçirilebilsin.
   Testler (test_son_yaklasma.py): (1) rho<0.25'te gelen 3 adet 0.5 m sıçramalı ölçüm HATA:sapma ÜRETMEZ ve
   komut hedefe doğru sürer; (2) rho>0.25'te aynı dizi hâlâ HATA:sapma üretir (mevcut davranış korunur);
   (3) renk_algi kenar ve boyut reddi.
Mevcut testleri kırma; davranış değişikliği yalnız yukarıdakiler. Önce iki dosyayı ve testlerini TAMAMEN OKU.
