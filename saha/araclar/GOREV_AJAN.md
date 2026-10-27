# Görev: `ajan` — Gemini (agy CLI) üzerinde kendi alt ajan aracımız (YENİ, Python 3.10, yalnız standart kütüphane)
Dosyalar: saha/araclar/ajan.py (yeni), saha/araclar/test_ajan.py (pytest; GERÇEK agy ÇAĞIRMADAN, sahte agy
betiğiyle), saha/araclar/AJAN.md (kısa kullanım kılavuzu). agy-delegate eklentisinden/agy-bridge'den BAĞIMSIZ:
doğrudan `agy` ikilisini çağırır (yol: $AJAN_AGY, vars. `agy` PATH'te).

## agy CLI gerçekleri (Claude ölçtü, 28.10.2026)
- Çalıştırma: `agy -p "<istem>" --output-format stream-json --model <id> [--effort low|medium|high|xhigh|max]
  [--add-dir DIZIN]... [--conversation <id>] [--print-timeout <süre>] [--sandbox]`. Çalışma dizini (cwd)
  içindeki GEMINI.md'yi bağlayıcı talimat olarak okur. Uzun istemi GEMINI.md'ye yaz, -p'ye kısa işaretçi ver
  ("Complete the TASK described in your GEMINI.md." benzeri) — istem ps/cmdline'da görünmez, boyut sınırı yok.
- agy SIGTERM'i yok sayabiliyor: iptal = süreç grubuna SIGTERM, 5 s sonra SIGKILL (start_new_session ile başlat).
- stream-json: satır başına bir JSON nesnesi:
  {"event":"init","conversation_id":"<uuid>","init":{"model":..,"cwd":..,"tools":[...]}}
  {"event":"step_update","step_update":{"conversation_id":..,"step_index":N,"state":"ACTIVE"|"DONE",
     "step_type":"user_input"|"agent_response"|"tool", "tool_name":"read_file"|"write_file"|"run_command"|...,
     "tool_info":{"name":..,"parameters":{...,"Path"|"CommandLine"...},"output":"..."},
     "text_delta":"...", "duration_seconds":..., "usage":{"input_tokens":..,"output_tokens":..,"total_tokens":..}}}
  {"event":"result","result":{"conversation_id":..,"status":"SUCCESS"|<başka>,"response":"<son metin>",
     "duration_seconds":..,"num_turns":..,"usage":{...}}}
  Parametre anahtar adları araçtan araca değişebilir: yazılan dosya yolunu parameters içindeki "Path",
  "TargetFile", "AbsolutePath", "path", "file_path" anahtarlarından hangisi varsa al (yoksa tüm string
  değerlerde "/" ile başlayanları dene).
- `--conversation <id>` aynı konuşmayı bağlamıyla sürdürür (ölçüldü: ikinci turda önceki sonucu hatırladı).
- İzinler: ~/.gemini/antigravity-cli/settings.json → "permissions": {"allow": [...]}. Biçimler:
  "write_file(<mutlak dizin>)" (dizin alt yolları kapsar), "command(<komut öneki>)" (ör. "command(ls)",
  "command(colcon)"). İzni olmayan araç çağrısı headless modda REDDEDİLİR ve tur hata/boş sonuçla biter.
- Modeller (`agy models`): gemini-3.8-flash-{low,medium,high}, gemini-4-argon-{low,medium,high},
  claude-opus-5-5-*, claude-sonnet-5-5-*, gpt-oss-120b-medium.

## Komutlar
Durum kökü: $AJAN_DIZIN (vars. ~/.local/state/ajan). Her görev: <kök>/<etiket>/ altında meta.json, GEMINI.md
(çalışma dizini = bu klasörün içindeki calisma/ alt dizini), olaylar.jsonl (ham akış), yanit.md (son metin),
rapor.json (yapılandırılmış rapor, aşağıda), hata.log.

1. `ajan baslat ETIKET (-g DOSYA... | -m "metin") [--model flash|argon|<tam id>] [--effort E] [--yaz DIZIN]...
   [--oku DIZIN]... [--komut "önek"]... [--sure 30m] [--sandbox] [--izin-ekle] [--bekle]`
   - Model takma adları: flash → gemini-3.8-flash-high, flash-orta → gemini-3.8-flash-medium, argon →
     gemini-4-argon-high, argon-orta → gemini-4-argon-medium; varsayılan flash.
   - ETIKET canlı çalışıyorsa REDDET (çıkış 3). Bitmiş etiket varsa <etiket>.<zaman> olarak arşivle.
   - Ön kontrol: her --yaz dizini için kapsayan write_file kuralı, her --komut için tam "command(<önek>)"
     kuralı settings.json'da olmalı; eksikse eklenecek kuralları yazdır ve çık (2). --izin-ekle ile
     settings.json'u settings.json.bak-<zaman> yedeğiyle güncelle. --yaz ve --oku dizinleri --add-dir olur.
     --sandbox verilmezse ve --komut yoksa yine de --sandbox kullan (komut izni verilmediyse terminali kapat).
   - GEMINI.md = sabit ajan kuralları (aşağıda) + görev metni.
   - Arka planda başlat; bir bekçi süreç (`ajan _bekci ETIKET`, çift fork / start_new_session) akışı okuyup
     olaylar.jsonl'a yazar, meta.json'u canlı günceller: durum ("calisiyor"), son_adim (adım no, tür, araç adı,
     yol/komut özeti), yazilan_dosyalar (set), calistirilan_komutlar, token toplamı. Bittiğinde: durum
     "bitti" (result.status == SUCCESS ve yanıt boş değil) / "hata" (aksi; hata.log son ERROR satırı ya da
     result.status), sure_s, conversation_id, yanit.md, rapor.json.
   - --bekle verilirse bitene kadar bekleyip özet basar (çıkış 0 bitti, 1 aksi).
2. `ajan devam ETIKET -m "takip mesajı" [--bekle]`: aynı conversation_id ile yeni tur (meta'da tur sayısı ve
   tur geçmişi), aynı izinler/dizinler. Bitmemiş görevde reddet.
3. `ajan durum [ETIKET] [--json]`: tablo: etiket, model, durum (calisiyor/bitti/hata/kayip/iptal), tur, süre,
   son adım (ör. "write_file son_yaklasma.py"), yazılan dosya sayısı, token. pid ölü + "calisiyor" → "kayip".
4. `ajan bekle [ETIKET...] [--zaman-asimi S]`: verilenler (yoksa tüm çalışanlar) bitene kadar; her biri için tek
   satır özet; hepsi "bitti" → 0, değilse 1.
5. `ajan rapor ETIKET [--ham]`: rapor.json'u okunur biçimde (yazılan dosyalar, komutlar, ajanın özeti, açık
   konular); --ham ile yanit.md.
6. `ajan izle ETIKET`: olaylar.jsonl'u canlı takip et (tail -f benzeri, araç çağrılarını tek satır özetle).
7. `ajan iptal ETIKET`, `ajan temizle [--gun 7]`, `ajan izin kontrol --yaz D --komut K` (ön kontrol tek başına).

## Sabit ajan kuralları (GEMINI.md başı, Türkçe)
- Yalnız şu dizinlere yazabilirsin: <--yaz listesi>. Başka yere yazma girişimi reddedilir ve görev başarısız olur.
- Çalıştırabileceğin komutlar: <--komut listesi ya da "yok (terminal kapalı)">. Komutun önek eşleşmesi gerekir.
- Görevde istenmeyen dosyaları değiştirme, silme. Var olan testleri gevşetme.
- Bitirince yanıtının SONUNA şu biçimde bir JSON bloğu koy (```json ... ```): {"ozet": "...",
  "degisen_dosyalar": ["..."], "calistirilan_testler": ["komut: sonuç"], "acik_konular": ["..."],
  "basarili": true|false}. ajan bu bloğu rapor.json'a çıkarır (bulamazsa yazilan_dosyalar'dan ve yanıttan
  rapor üretir, "basarili": null).

## Testler (sahte agy: AJAN_AGY ile verilen, argümanları kaydedip örnek stream-json basan küçük Python betiği)
baslat→bekle→bitti ve rapor.json çıkarımı; aynı etiket çalışırken ret; eksik izin → çıkış 2 ve önerilen kural;
--izin-ekle yedekleyip ekler (geçici settings, $AJAN_AYAR ile yol); devam ikinci turda --conversation geçer;
hata sonucu → "hata"; iptal süreç grubunu öldürür; pid ölü → kayip; model takma adları; yazılan dosya yolu
çıkarımı farklı parametre anahtarlarıyla.
Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
