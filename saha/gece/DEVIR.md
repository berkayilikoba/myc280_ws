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
