# Görev J1: oda_doku.py — zemin/duvar dokularını keskin ve temiz yap

Dosya: saha/oda/oda_doku.py (yalnız bu). CLI ve çıktı biçimi (doku/*.png, materials/..., doku.yaml) korunmalı;
yeni seçenekler CLI bayrağıyla.

Teşhis (Claude): zemin dokusunda duvar/mobilya pikselleri ışınsal çizgiler halinde zemine yayılıyor, orta kısım
bulanık. Neden kare_izdusur'daki örtüşme satırı:
    gorunur = np.where(d_olc > 0.0, (z_g - d_olc) <= tol_derinlik, z_g <= 3.0)
D435 derinliği zeminde yalnız %20-40 geçerli; derinlik 0 olan her piksel 3 m'ye kadar kabul ediliyor, oysa o
pikseller çoğu zaman zemin değil (duvar, mobilya, siyah nesne). Ayrıca ağırlıklı ortalama birden çok kareyi
(küçük poz hatalarıyla) karıştırıp bulanıklaştırıyor; kareler arası pozlama farkı lekeler yaratıyor.

İstenen:
1. Örtüşme: derinlik geçerliyse |z_g - d_olc| <= tol (çift taraflı; zemin 0.04, duvar 0.08 m). Derinlik
   geçersizse: zeminde yalnız z_g <= --zemin_gecersiz_menzil (vars. 1.2 m) VE pikselin 7x7 komşuluğunda geçerli
   derinlik varsa onların medyanı düzleme |fark| <= tol uymalı (komşuluk desteği); komşulukta hiç geçerli yoksa
   reddet. Duvarda geçersiz derinlik -> reddet (duvar derinliği zaten çoğunlukla geçerli).
   Ayrıca zemin için: düzleme göre yükseklik testi — geçerli derinlikten hesaplanan 3B noktanın z'si > 0.03 m ise
   reddet (zemin üstündeki nesne).
2. Keskinlik — "en iyi görünüm" birleştirme `--birlestir {ortalama,enIyi}` (vars. enIyi): her teksel için yalnız
   ağırlığı en yüksek K (=3) gözlemin ağırlıklı ortalaması (veya en iyi tek gözlem). Bellek için: teksel başına
   en iyi 3 (ağırlık, renk) tutan sabit boyutlu diziler (H,W,3) ve (H,W,3,3); her karede vektörel güncelleme
   (yeni ağırlık en küçükten büyükse yerine yaz). Ağırlık = cos(açı)/mesafe² (mevcut cos/mesafe yerine) ve
   bulanık kareler için Laplacian varyansı ile çarp (kare başına bir kez, normalize: var/medyan_var, 0.2..1.5'e kırp).
3. Pozlama dengeleme `--pozlama_dengele` (vars. açık): her kare için bir kazanç (BGR ortak, ölçek). Basit yöntem:
   iki geçiş. 1. geçişte enIyi doku üretilir; 2. geçişte her karenin örtüşen tekselleri için medyan(doku/kare
   pikseli) oranı kazanç olarak bulunur (0.6..1.6'ya kırp), kare pikselleri kazançla çarpılıp doku yeniden
   üretilir. Zaman sınırlıysa 2. geçişi yalnız zemin için yap.
4. Görülmeyen bölgeler: zemin için `--zemin_doldur {medyan,yama}` (vars. yama): görülen zemin dokusunun en iyi
   (en yüksek ağırlıklı, en keskin) 64x64 tekselli bölgesini bul ve görülmeyen alanları bu yamanın döşenmesiyle
   doldur (yama kenarlarını cv2.seamlessClone ya da basit yumuşak kenarla karıştır). Duvarlarda mevcut medyan +
   blur kalsın.
5. Mobilya altı maskesi (G1'de eklenen) davranışı korunsun.
6. doku.yaml'a: her yüzey için görülen oran, kazanç istatistikleri (min/medyan/max), reddedilen piksel oranı.
Kodu çalıştıramazsın; Claude test edecek:
  python3 oda_doku.py cikti/doku_j1 --yapi cikti/kur/oda_yapi.yaml <3 sahne> --her 6 --tavan 2.7
Kısa özetle bitir.
