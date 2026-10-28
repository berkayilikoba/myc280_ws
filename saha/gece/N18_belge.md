# Gece görevi N18: docs/nav_kavrama.md'yi gece eklenen özelliklerle güncelle
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. UYDURMA YOK: her parametre adı, varsayılan değer, konu adı
ve launch argümanı kaynak dosyada geçmeli; emin olmadığını yazma. Sayısal ölçümleri yalnız aşağıdaki ve
saha/gece/DEVIR.md'deki "Gece sonuçları" bölümünden al.

Önce TAMAMEN OKU: docs/nav_kavrama.md, saha/gece/DEVIR.md, ve aşağıdaki kaynaklar.
Güncellenecek/eklenecek konular (mevcut bölüm yapısına yerleştir, yeni bölüm gerekiyorsa numaralandırmayı koru):
1. Erken kabul ve NAV_Q zaman aşımı (kavrama_dugumu.py: q_erken_* parametreleri, 90 s zaman aşımı) — 3.3'e.
2. Görüş kaybı/kör nokta: son_yaklasma.py olcumsuz_mesafe ve YENİ tahmin dondurma (tahmin_donduruldu,
   aykiri_kabul_sayisi) — 3.4'e; neden: kısmi görünen elma 49 cm hatalı ölçüm → HATA:sapma (sim ölçümü).
3. Algı: renk_algi.py (kenar_payi_px, KISMI durumu, boyut_denetimi, beklenen_yaricap_m), kavrama.launch.py
   algi:=sahte|renk|yok ve hata_ayiklama argümanları — 2.2 ve 5.1'e; saha/gece/algi_sim_test.sh 5.2 yanına.
4. Pil/şarj görev davranışı (gorev_yoneticisi.py / gorev_dugumu.py: pil eşikleri, /gorev/sahte_pil) — yeni alt bölüm.
5. Dinamik engel güvenliği: nav2_diff.yaml collision_monitor DurBolge/YavasBolge/FootprintApproach ve
   hareketli_kalkan.py (limo_nav2.launch.py hareketli_kalkan argümanı, zincir cmd_vel_smoothed →
   cmd_vel_kalkan → collision_monitor, ttc_dur/ttc_yavas/min_engel_hizi/max_iz_yaricap, /hareketli_kalkan/durum)
   — yeni alt bölüm "Hız zinciri ve güvenlik"; dinamik_engel.py ile ölçüm (src/limo_bringup/scripts).
6. Saf odom (limo_sim_nav2.launch.py localization:=yok) ölçümleri ve 2.4 VO ekibi gereksinimi: dar geçitte
   kayma < ~0.15 m ya da map->odom düzeltmesi (DEVIR.md sayıları).
7. Bölüm 7 "Bilinen kısıtlar": güncelle (kalkan sim sonuçları Claude tarafından eklenecek — yer tutucu satır bırak:
   "Hareketli kalkan sim ölçümü: (Claude dolduracak)").
Kaynaklar: src/limo_autonomy/limo_autonomy/{kavrama_dugumu,son_yaklasma,renk_algi,hareketli_kalkan,
gorev_yoneticisi,gorev_dugumu}.py, src/limo_autonomy/launch/kavrama.launch.py,
src/limo_bringup/launch/{limo_nav2,limo_sim_nav2}.launch.py, src/limo_bringup/param/nav2_diff.yaml.
Yalnız docs/nav_kavrama.md'yi değiştir. Sonunda neyi eklediğini madde madde özetle.
