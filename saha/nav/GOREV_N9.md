# Görev N9: görev yöneticisi — "nesneyi al, getir" karar katmanı (YENİ)
Dosyalar (yeni): src/limo_autonomy/limo_autonomy/gorev_yoneticisi.py (saf durum makinesi, ROS'suz),
src/limo_autonomy/limo_autonomy/gorev_dugumu.py (ROS düğümü), src/limo_autonomy/config/nesneler_lab_oda.yaml,
src/limo_autonomy/test/test_gorev_yoneticisi.py. setup.py'ye DOKUNMA. kavrama_dugumu.py'ye DOKUNMA (başka görev).
Önce mission.py'yi oku (üslup, yaml yükleme kalıbı).

Arayüzler:
- Kavrama katmanı (mevcut): `/kavrama/hedef` (PointStamped map) gönder; `/kavrama/durum` (String; 'HAZIR ...',
  'HATA ...', diğerleri ara durum) dinle; `/kavrama/iptal` (Empty).
- Kol (ekip arkadaşı; henüz yok): `/kol/kavra` ve `/kol/birak` (std_srvs/Trigger). Parametre `kol_sahte` (vars.
  True): servis yoksa 3 s bekleyip başarılı say (simde uçtan uca test için). `kol_zaman_asimi` 30 s.
- Gezinme: NavigateToPose eylemi (`navigate_to_pose`), ev pozu parametre/yaml'den.
- Görev girişi: `/gorev/istek` (String: nesne adı, ör. "elma"; "iptal" → tümünü iptal), durum `/gorev/durum`
  (String, 'DURUM ayrıntı').
- nesneler yaml: `ev: {x, y, yaw_derece}`, `birakma: {x, y, yaw_derece}` (getirilecek yer; yoksa ev),
  `nesneler: {elma: {x, y, z}, ...}` (lab_oda: elma masa üstünde (0.23, 1.40, 0.75), kutu dolapta (3.73, 2.18, 0.30)).

Durum makinesi (gorev_yoneticisi.py, olay güdümlü, saat dışarıdan verilir):
BOSTA → KAVRAMAYA_GIT (kavrama hedefi gönder) → [HAZIR] KOL_KAVRA → [başarılı] TASI (birakma pozuna
NavigateToPose) → [başarılı] KOL_BIRAK → [başarılı] EVE_DON → BOSTA (sonuç 'TAMAM').
Hatalar: kavrama HATA → 1 kez yeniden dene (yeniden hedef gönder), sonra 'BASARISIZ kavrama'; kol kavra başarısız →
geri çekil yok (kavrama düğümü bilir), kavramayı 1 kez yeniden dene; TASI başarısız → 2 kez yeniden dene, sonra
'BASARISIZ tasima' ve olduğun yerde KOL_BIRAK YAPMA (nesneyi tutmaya devam), eve dönmeyi dene. İptal her durumda
→ aktif eylemi iptal et, BOSTA. Her geçiş `gecmis` listesine (zaman, eski, yeni, neden).
API: `GorevYoneticisi(nesneler, ev, birakma, ayar)`, `istek(ad, t) -> list[Eylem]`, `olay(tur, veri, t) ->
list[Eylem]` (tur: 'kavrama_durum', 'kol_sonuc', 'nav_sonuc', 'iptal', 'zaman'), Eylem dataclass (tur:
'kavrama_hedef'|'kavrama_iptal'|'kol_kavra'|'kol_birak'|'nav_git'|'nav_iptal'|'durum', veri). Düğüm yalnız
eylemleri uygular ve olayları besler (iş mantığı düğümde değil). Görev zaman aşımı: aşama başına (kavrama 240 s,
taşıma 180 s).
Testler: mutlu yol; kavrama ilk HATA sonra HAZIR; taşıma 3 kez başarısız → BASARISIZ ve KOL_BIRAK yok; iptal
ortada; bilinmeyen nesne adı → 'HATA bilinmeyen nesne'; zaman aşımı.
