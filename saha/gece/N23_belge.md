# Gece görevi N23: docs/nav_kavrama.md'yi N19–N22 ve son ölçümlerle güncelle
ÖNEMLİ: terminal iznin YOK; yalnız docs/nav_kavrama.md'yi değiştir. UYDURMA YOK: parametre/konu adları kaynak
kodda geçmeli; SAYILAR yalnız aşağıdan ve saha/gece/DEVIR.md'den. Tablo satırı uydurma (önceki görevde g6
satırına kaynakta olmayan süreler yazılmıştı — tekrarlama).
Kaynaklar: src/limo_autonomy/limo_autonomy/{hareketli_kalkan,son_yaklasma,renk_algi,kavrama_dugumu}.py,
src/limo_bringup/launch/limo_nav2.launch.py, saha/gece/{gorev_sim_test.sh,shm_temizle.sh,bekle_aktif.py,R1_RAPOR.md}.
Eklenecekler:
1. Kalkan (hız zinciri bölümü): lidar montajı TF'ten, belirsiz eşleşme/poz aradeğeri/yön tutarlılığı, ODOM_YOK ve
   TARAMA_YOK davranışı, respawn, varsayılan KAPALI (gerçek robotta denenmedi). Son ölçüm: engelsiz tur 7/7 126 s
   1 kurtarma 0 yanlış alarm (N19 öncesi 9 DUR + 7 YAVAS); yandan 0.8 m/s temas yok (+0.09 m, ilk DUR ttc 0.86 s);
   karşıdan 0.6 m/s tepkisiz insan robota çarpıyor ama robot temasta duruyor (engel_carpti, v 0); duran/takip 0 tetikleme.
2. Son yaklaşma: kayip_tolerans_mesafe 0.40 (kamera zemindeki elmayı robot merkezine ~0.53 m'den yakında kaybediyor),
   sapma_iptal_mesafe / maks_yeniden_hedef (N22), algı ölçümünün damga anındaki TF ile odom'a çevrilmesi.
3. Algı (renk_algi): leke içi düşük yüzdelik derinlik (derinlik_yuzdelik, min_derinlik_piksel, derinlik_bant_m,
   maks_menzil_m). Ölçüm: tüm algı ölçümleri ortalama hatası 6.7–9.5 cm → 1.84 cm; kamera algısıyla kavrama
   −0.65 cm / −0.12°; uzak menzilde sistematik ileri sapma (kaynağı doğrulanmadı).
4. Kamera algılı uçtan uca görev (ALGI=renk gorev_sim_test.sh): 2/2 TAMAM, P hatası 0.9 cm / 0.1–0.3°, kavrama ~10 s.
5. Simde çalıştırma bölümüne "Sorun giderme": 'AMCL/Nav2 açılmadı' → zombi FastDDS SHM (shm_temizle.sh) ve
   CLI keşfi (bekle_aktif.py); kill -9 sonrası /dev/shm artıkları.
6. Bilinen kısıtlar: tepkisiz insan kafa kafaya gelirse temas kaçınılmaz (robot durur); saf odom vio_kotu dar geçit.
Sonunda eklediklerini madde madde özetle.
