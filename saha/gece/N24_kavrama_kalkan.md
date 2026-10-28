# Gece görevi N24: son yaklaşma hareketli engel kalkanına uysun (kavrama /cmd_vel'e doğrudan yayınlıyor)
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. Claude test edip simde ölçecek.
## Sorun
kavrama_dugumu.py son yaklaşmada (SON_YAKLASMA, hizala/sürün, KURTARMA geri çekilme) hız komutunu doğrudan `cmd_konu`
(vars. 'cmd_vel') konusuna yayınlıyor (satır ~751/768) → Nav2 hız zinciri (velocity_smoother → hareketli_kalkan →
collision_monitor) devre dışı. Yaklaşma sırasında araya insan girerse durmaz. collision_monitor'dan geçirmek çözüm
değil: nesne gövde önüne ~6 cm yaklaşıyor, DurBolge (ayak izi + 0.06 m) kavramayı keser.
hareketli_kalkan.py durağan nesneleri yok sayar, yalnız hareketli engelde `/hareketli_kalkan/durum` (std_msgs/String)
yayınlar: "DUR ttc=..s v=..", "YAVAS ttc=..s v=..", "serbest", "ODOM_YOK", "TARAMA_YOK" (karar DEĞİŞTİKÇE yayın).
## İstenen (src/limo_autonomy/limo_autonomy/kavrama_dugumu.py + test)
1. `/hareketli_kalkan/durum` aboneliği (parametre `kalkan_konu`, vars. '/hareketli_kalkan/durum'; `kalkan_uy` bool
   vars. true). Son durum + alındığı an saklanır. Karar yalnız değişince geldiği için "son durum" kalıcıdır; ancak
   kalkan düğümü yoksa hiç mesaj gelmez → davranış değişmez.
2. Kavramanın KENDİ yayınladığı her hız komutu (son yaklaşma, hizala/sürün, kurtarma geri çekilme) yayından önce saf
   bir fonksiyondan geçsin: `kalkan_uygula(vx, wz, durum_metni) -> (vx, wz)`: 'DUR' → vx=0 (wz korunur, yerinde
   dönüş serbest); 'YAVAS' → vx*0.5; diğer/None → değişmez. Geri çekilmede (vx<0) de DUR'da vx=0 (engel arkada olabilir;
   kalkan yönü bilinmiyor — bunu yorumla belirt).
3. DUR sürerken son yaklaşmanın zaman aşımları (son_yaklasma max_sure, ölçüm zaman aşımı) haksız yere dolmasın:
   bekleme süresini SonYaklasma'ya `duraklat(dt)` benzeri bir yolla ilet ya da kavrama_dugumu'nda t_baslangic'ı
   kaydır — mevcut yapıya en az müdahale edeni seç ve gerekçelendir. DUR 15 s'den uzun sürerse ('kalkan_maks_bekleme',
   vars. 15.0) KURTARMA yerine HATA:kalkan_bekleme ile iptal (insan önünde bekliyor).
4. /kavrama/durum geri bildirimine "KALKAN_BEKLE" ayrıntısı (durum makinesi kodunu değiştirmeden, ayrıntı metni).
5. Testler (ROS'suz): kalkan_uygula tablosu; DUR sırasında zaman aşımının dolmadığı (saf fonksiyon/sınıf seviyesinde
   test edilebilir hale getir); kalkan_maks_bekleme aşımı. Mevcut testleri kırma.
Önce kavrama_dugumu.py (tamamı), son_yaklasma.py (SonYaklasma.adim, zaman aşımı alanları), test_kavrama_dugumu.py OKU.
Yalnız kavrama_dugumu.py, gerekirse son_yaklasma.py ve ilgili test dosyaları.
