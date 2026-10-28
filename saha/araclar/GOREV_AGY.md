# Görev: agy_gorev.py — agy-bridge için görev yöneticisi sarmalayıcı (YENİ, Python 3.10, yalnız standart kütüphane)
Dosya: saha/araclar/agy_gorev.py (yeni) + saha/araclar/test_agy_gorev.py (pytest, gerçek agy çağırmadan; bridge
yerine sahte bir betik/komut kullan). Bridge: `agy-bridge` (PATH'te; seçenekler: --type --model --timeout
--add-dir --json --log-file, istem `--` sonrası ya da stdin). Bridge çıktıyı YALNIZ iş bitince stdout'a yazar;
başta stderr'e "agy: Fetching available models..." yazabilir; süreç `exec -a agy-bridge` ile adını gizler.
agy'nin kendi yazma izinleri: ~/.gemini/antigravity-cli/settings.json → permissions.allow listesinde
"write_file(<mutlak dizin>)" kuralları (dizin kuralı alt dosyaları kapsar).

Çözülecek sorunlar: (1) bitiş tespiti güvenilmez, (2) hangi süreç hangi görev bilinmiyor, aynı görev yanlışlıkla
iki kez başlatılabiliyor, (3) yazma izni eksikliği iş bittikten sonra fark ediliyor.

Alt komutlar (argparse):
- `baslat ETIKET --model M --gorev DOSYA [DOSYA...] [--oku DIZIN]... [--yaz DIZIN]... [--timeout N] [--type implement]`
  * Durum dizini: $AGY_GOREV_DIZIN (vars. ~/.local/state/agy-gorev)/ETIKET/. ETIKET zaten "calisiyor" durumdaysa
    (pid canlı) REDDET (çıkış 3). Biten etiket yeniden başlatılabilir (eskiyi ETIKET.<zaman> olarak arşivle).
  * Ön kontrol: her --yaz dizini için settings.json'da onu (ya da bir üst dizinini) kapsayan write_file kuralı
    yoksa hata ver ve eklenecek tam kural satırını yazdır (otomatik EKLEME; `--izin-ekle` bayrağı verilirse
    settings.json'u .bak-<zaman> yedeğiyle güncelle). --yaz dizinleri otomatik olarak --add-dir'e de eklenir.
  * İstem = --gorev dosyalarının birleşimi (sırayla) + sona eklenen sabit not: "Yalnız şu dizinlere yazabilirsin:
    <liste>. Başka yere yazma (izin reddedilir ve görev hata olur)."
  * Bridge'i ayrı süreç grubunda (start_new_session) başlat; stdout → cikti.md, stderr → hata.log; meta.json:
    {etiket, model, gorevler, yaz, oku, pid, pgid, baslangic, durum: "calisiyor"}. Arka planda bekleyen bir
    "bekçi" süreç (çift fork veya `python3 agy_gorev.py _bekci ETIKET`) bridge bitince meta.json'a bitis, sure_s,
    cikis_kodu, durum ("bitti" çıkış 0 ve cikti.md boş değil; aksi "hata") yazar ve hata.log'daki son "ERROR:"
    satırını meta'ya `hata` olarak koyar. baslat komutu hemen döner ve etiketi yazdırır.
- `durum [--json]`: tüm etiketler tablosu: etiket, model, durum, süre (çalışıyorsa geçen), çıkış, hata özeti
  (60 karakter). pid öldüğü hâlde meta "calisiyor" ise "kayip" göster.
- `bekle ETIKET [ETIKET...] [--zaman-asimi S]`: hepsi "calisiyor" dışına çıkana kadar bekle (2 s aralık); sonra her
  biri için tek satır özet yazdır; hepsi "bitti" ise çıkış 0, değilse 1. Etiket verilmezse çalışan tümü.
- `cikti ETIKET [--son N]`: cikti.md (ya da son N satırı).
- `iptal ETIKET`: süreç grubuna SIGTERM, 5 s sonra SIGKILL; durum "iptal".
- `temizle [--gun 7]`: eski bitmiş/hatalı kayıtları sil.
Testler: sahte bridge (ör. `sleep 1; echo tamam` yapan küçük betik, ortam değişkeni AGY_GOREV_BRIDGE ile bridge
komutu değiştirilebilsin) ile: baslat→bekle→bitti; aynı etiket çalışırken ikinci baslat reddedilir; çıkış 3 + boş
çıktı → hata ve hata özeti; eksik write_file kuralı → ön kontrol hatası (geçici settings.json, ortam değişkeni
AGY_GOREV_AYAR ile yolu değiştirilebilsin); iptal; pid ölü + meta calisiyor → kayip.
Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
