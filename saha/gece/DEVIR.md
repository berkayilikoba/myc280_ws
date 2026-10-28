# Gece geliştirme — devir notu (28.10.2026, ~01:45)

Dal: `gece-gelistirme` (main'den, PR #7 birleştikten sonra). Kural: her iş paketi bu dalda (ya da alt dalda),
bitince push → PR → merge → dal sil (Güneş'in kalıcı tercihi).

## Çalışan Gemini görevleri (`ajan durum`)
| etiket | model | görev dosyası | yazdığı yer | Claude'un yapacağı |
|---|---|---|---|---|
| dogrula | flash | — | BİTTİ: 20/20 test, commit edildi | `ajan dogrula` gerçek görevde dene |
| G1 | argon | saha/gece/G1_eylem.md | src/limo_interfaces (yeni paket), limo_autonomy | colcon build limo_interfaces limo_autonomy; pytest; simde `ros2 action send_goal /kavra ...` |
| G2 | flash | saha/gece/G2_gorsel.md | limo_autonomy (kavrama_gorsel.py, test, config/kavrama.rviz) | setup.py giriş noktası `kavrama_gorsel`; launch'a `gorsel:=`/`rviz:=` (yanıttaki parça); simde işaretleri gör |
| G3 | flash | saha/gece/G3_belge.md | docs/nav_kavrama.md | gerçeklerle karşılaştır (uydurma parametre/konu var mı), düzelt |

Bitince: `ajan rapor <etiket>` → diff oku → testleri KENDİN çalıştır → kalan varsa `ajan devam <etiket> -m "..."`.

## Durum (ölçülmüş)
- Kavrama simde (3B oda, odom hatası açık, AMCL yer tutucu): dolap 0.8 cm/1.6°, duvar dibi (5.62, 0.8) 0.7 cm/1.7°
  KAVRANABILIR; masa 0.75 m beklenen hata (erişilemez). `saha/nav/kavrama_sim_test.sh "" cikti/kN`.
- Testler: limo_autonomy 91/91, ajan 15/15.

## Sıradaki özellikler (öncelik sırasıyla) — planla, Gemini'ye `ajan` ile ver, denetle
1. G1–G3 + dogrula'yı entegre et, test et, PR.
2. **Gezinme sağlamlık turu** (N12 hazır): `nav_bench.py --hedefler scripts/nav_tur_lab_oda.yaml` 3B odada;
   bati_dar (0.27 m) zor hedef. Sonuca göre MPPI/inflation ayarı (bench_many ile varyant karşılaştırma).
3. **VO odom kayması stres testi**: sensor_modeli odom hatasını büyüt (k_v ±%3, k_w ±%2, ara sıra sıçrama /
   ölçek kayması) → kavrama ve tur başarı oranı; son yaklaşmanın dayanıklılığını belgele (VO ekibine gereksinim).
4. **Görev uçtan uca**: `kavrama.launch.py use_sim_time:=true sahte_algi:=true` + `/gorev/istek elma`
   (kol_sahte) → kavra → taşı → bırak → eve dön; görev yöneticisi testine sim koşusu.
5. **Kameradan gerçek algı prototipi** (sahte_algi yerine): simde renkli nesne + D435 renk/derinlik → nesne
   konumu (renk eşik ya da YOLO) → /kavrama/nesne_algi; algı ekibine örnek uygulama.
6. Dinamik engel: simde hareket eden kutu/insan; collision_monitor + MPPI tepkisi ölçümü.
7. `ajan` iyileştirmeleri: dogrula sonrası; projeden bağımsız depo hazırlığı (README, örnekler) — paylaşım kararı Güneş'te.

## Dikkat
- Robot şarjda, HAREKET ETTİRME (gerçek robot işleri yok).
- Sim temizliği: iş bitince gzserver/joint_state_publisher kalmasın (`ps -eo args | awk '$1=="gzserver"'`).
- agy görev bitişi: yalnız `ajan bekle` (arka planda). Günlük boyutuna bakma.

## Gece sonuçları (28.10, ~10:15) — ölçülmüş
- Görev g6 (N15 sonrası): /kavra 0.9 cm SUCCEEDED; "elma" görevi TAMAM, `NAV_Q erken_kabul (18.4 cm, 4.2°)` tetiklendi.
- Saf odom (`localization:=yok`, map->odom statik): teker 7/7, 124 s, ort. varış 0.20 m, min açıklık 0.17.
  vio_kotu 6/7: bati_dar (0.27 m) TIMEOUT, kurtarma 18, min açıklık 0.0 (temas), lok. hatası 0.31–0.44 m.
  → VO ekibine gereksinim: dar geçitlerde kümülatif kayma < ~0.15 m, ya da map->odom düzeltmesi (yer tutucu AMCL ile vio_kotu 7/7).
- Dinamik engel F6 sonrası karşıdan: SUCCEEDED, temas −2.9 cm, robot temasta 0.094 m/s (robot_carpti sınıfı).
  Tepkisiz insan 0.6 m/s ile kafa kafaya geliyor; robot yavaşlıyor ama tam durmuyor. DurBolge önünü büyütmek kavrama
  yaklaşmasını (nesne gövdeye 6 cm) bozar → ayrı ele alınmalı (kavrama sırasında bölge değiştirme ya da hıza bağlı bölge).
- ilk_poz.py: duvar saati damgası sim saatinde "gelecek" sayılıp AMCL tarafından reddediliyordu → stamp 0 (düzeltildi).
- N16 hareketli_kalkan (hareketli_kalkan:=true, vars. kapalı): karşıdan temasta robot DURUYOR (engel_carpti, v 0),
  yandan temas yok (+0.13 m), duran/takip 0 tetikleme. AMA engelsiz turda 9 DUR + 7 YAVAS yanlış alarm (120→150 s)
  → N19 (zaman aradeğeri, belirsiz eşleşme, yön tutarlılığı). N19 doğrulanmadan varsayılan açılmamalı.
- N17 sonrası kamera algısıyla (algi:=renk) uçtan uca kavrama: −0.7 cm / −1.3°, son yaklaşma 8.3 s.
  Sahte algı gerileme: dolap 0.21 cm/0.4°, duvar 0.38 cm/0.9° (öncekinden iyi), masa beklenen hata.
  Not: renk_algi uzak ölçüm ortalaması 9.5 cm, mesaj sayısı 579→54 (boyut denetimi) — incelenecek.
- "Nav2 açılmadı" yanlış başarısızlıkları: ros2 lifecycle CLI keşfi; betikler bekle_aktif.py'ye geçti.
- Damga düzeltmesi (kavrama_dugumu algı TF'i ölçüm anında): kamera algısıyla −0.46 cm / 0.16°. Uzak algı hataları iki kümeli (~4 cm / ~19 cm) — incelenecek (derinlik penceresi arka plan? ikinci kırmızı leke?).
- Altyapı: zombi FastDDS SHM kilitleri yeni düğümleri asılı bıraktı (≈10:53–11:30 tüm sim testleri 'AMCL açılmadı'); saha/gece/shm_temizle.sh betiklerin başında. N20 sonrası algı: tüm ölçümler ort. 1.84 cm (önce 6.7–9.5), kavrama −0.65 cm/−0.12°.
- Kamera algılı uçtan uca görev (ALGI=renk gorev_sim_test.sh): elma TAMAM, P 1.1 cm/0.8°; ilk adayda HATA:sapma kurtarması (→ N22). Kalkan son: engelsiz tur 0 yanlış alarm, yandan temas yok, karşıdan temasta durmuş (engel_carpti).
- N21/N22 sonrası (12:1x): kamera algılı görev 2/2 TAMAM (0.9 cm, ~10 s, kurtarma yok); kalkanlı tur 7/7 126 s 0 yanlış alarm; yandan temas yok; karşıdan robot temasta durmuş; duran/takip 0 tetikleme. hareketli_kalkan varsayılan KAPALI (gerçek robotta denenmedi).
- R2 (argon, algı sapması incelemesi) GÜVENİLMEZ: var olmayan fonksiyon/parametre alıntıladı (`_derinlik_oku_iki_asamali`, `golge_band=6`, `zemin_kayip=0.80`); gölge yönü yanlış. Doğru kalan: 0.6 m ilk aşama varsayımı var.
- N20 sonrası menzile göre algı sapması (algi9): ≤1 m dx +0.5…+1.3 cm; ≥1.5 m ~+3.5 cm (≈ r; uzakta az pikselde derinlik merkeze yakın, +r fazla). dy ~0. z +1.5 cm sabit (gerçek z kaynağı doğrulanmalı). Kavramaya etkisi yok (son yaklaşma <0.6 m); Q planına ~3.5 cm.
- Gerileme (12:15): normal tur 7/7 123 s; vio_kotu+AMCL 7/7 146 s; kavrama dolap 0.76 cm/1.4°, duvar 0.28 cm/0.6°.
