# Görev N14: kavrama_yaklasim.py — konum belirsizliği payını Q→P boyunca azalt (duvar dibi nesneler)
Dosyalar: src/limo_autonomy/limo_autonomy/kavrama_yaklasim.py, src/limo_autonomy/test/test_kavrama_yaklasim.py
(mevcut testleri bozma, yeni ekle). kavrama_dugumu.py'ye DOKUNMA (başka görev).
Claude teşhisi (lab_oda haritası): duvara 2–8 cm uzaklıktaki nesnelerde (gerçekte en sık durum) hiç yaklaşma
bulunmuyor: guney_duvar (4.0, -0.55), kuzey_duvar2 (1.2, 2.58), bati (-2.08, 1.2). konum_belirsizligi=0 yapınca
kuzey_duvar2 bulunuyor (açıklık 0.07). Neden: düz koridor koşulu (N3) `pay + konum_belirsizligi` (0.05+0.06)
payını Q'dan P'ye SABİT uyguluyor. Oysa Q'da belirsizlik Nav2+lokalizasyon kadar (~6 cm), P'ye yaklaşırken robot
NESNEYE GÖRELİ kapalı çevrimle gidiyor (göreli hata ~1–2 cm).
İstenen:
1. Ayar: `belirsizlik_p` (P'deki belirsizlik, vars. 0.015 m). Koridorda Q'dan P'ye ilerledikçe ek pay
   konum_belirsizligi → belirsizlik_p doğrusal azalır. P'deki ayak izi denetimi de pay + belirsizlik_p ile.
2. P'de en yakın engele açıklık (aciklik) puanı aynı kalsın; ama geçerlilik eşiği pay + belirsizlik_p.
3. Bulunamama nedenini raporlayan hata ayıklama yardımcısı: `yaklasim_tani(h, nesne, a, robot) -> dict`
   (her aday açı için hangi koşulda elendi: 'P_carpisma', 'koridor', 'Q_carpisma', 'erisim', 'yuzey' ... sayıları
   ve en iyi 3 elenen adayın en küçük açıklıkları). yaklasim_bul None döndürdüğünde kavrama düğümü bunu
   günlüğe yazabilsin diye.
4. Testler: sentetik harita — duvardan 5 cm uzaktaki nesne için duvara dik yaklaşma BULUNUR; 2 cm'den yakın
   (nesne duvara gömülü) için robot ön tamponu duvara pay+belirsizlik_p'den yaklaşacaksa bulunmaz ve
   yaklasim_tani 'P_carpisma' raporlar; dar koridor testi (mevcut) hâlâ geçer.
