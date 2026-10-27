# Görev G4: oda_yapi.py — iç içe paralel çift duvarları ele

Dosya: saha/oda/oda_yapi.py (yalnız bu). G3'te eklenen eksene hizalama + birleştirme adımından SONRA çalışacak
küçük bir adım ekle.

Sorun: aynı eksende, birbirine 0.06–0.20 m dik uzaklıkta iki paralel duvar kalıyor (kalın duvarın iki yüzü /
çift tarama). Örnek (YAML'den): (-2.28,2.24)->(-2.25,0.07) L 2.18 ve (-2.14,0.40)->(-2.16,1.58) L 1.18;
kısa olan, uzunun eksen yönündeki aralığının içinde.

İstenen:
- Yeni CLI `--cift_mesafe` (m, varsayılan 0.20; 0 = kapalı).
- Aynı baskın eksene hizalı iki duvar için: dik uzaklık <= cift_mesafe VE kısa olanın eksen yönündeki izdüşüm
  aralığının >= %80'i uzun olanınkiyle örtüşüyorsa kısa olanı at (uzun olanın konumunu değiştirme).
- Bu kontrolü atılacak bir şey kalmayana kadar tekrarla.
- Terminal özetine "çift duvar atılan: K" ekle.
Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
