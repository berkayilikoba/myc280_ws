# Görev N6: kavrama_bench.py — kavrama yaklaşmasının simde ölçümü (YENİ betik)
Dosya: src/limo_bringup/scripts/kavrama_bench.py (yeni). Sim + Nav2 + kavrama düğümü çalışırken koşar; onları
başlatmaz.
- Girdi: --hedefler YAML (liste: ad, x, y, z, isteğe bağlı yon_derece), --cikti YAML, --zaman_asimi 180.
- Her hedef için: `/kavrama/hedef` (PointStamped, frame map) yayınla; `/kavrama/durum` (std_msgs/String)
  mesajlarını topla; 'HAZIR' ile başlayan ya da 'HATA' ile başlayan mesajı ya da zaman aşımını bekle.
- Ölçüm (Gazebo gerçeği): `/odom_gercek` (dünya = harita varsayımı) ile robotun gerçek pozu; nesnenin GERÇEK
  robot çerçevesindeki konumu (x_r, y_r) ve bunun hedefi (d_kavrama 0.22, yanal 0) ile farkı; robot yönünün
  robot→nesne doğrultusuna göre hatası (derece); `/kavrama/nesne_robotta` (PointStamped base_footprint) bildirilen
  ↔ gerçek fark; toplam süre; durum mesajı geçmişi; son yaklaşma süresi ('SON_YAKLASMA' ile başlayan mesajdan
  'HAZIR'a).
- Hedefler arasında robotu başlangıca döndürme yok; sırayla koşar. Her hedef sonrası 2 s bekle.
- Özet: başarı oranı, kavrama hatası (cm) medyan/maks, yön hatası (°) medyan/maks, bildirilen-gerçek fark (mm).
  Kabul ölçütü bayrağı: |Δx|,|Δy| < 2 cm ve |yön| < 3° → 'KAVRANABILIR'.
- Ayrıca örnek hedef dosyası: src/limo_bringup/scripts/kavrama_hedefler_lab_oda.yaml — lab_oda haritasında iki
  hedef: dolap (3.73, 2.18, 0.30), masa (0.23, 1.40, 0.75) (önceki ölçümlerde kullanılan noktalar).
