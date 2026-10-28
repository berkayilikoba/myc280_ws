# Görev F4: `ajan`'ı projeden bağımsız, başkalarının da kurabileceği hale getir (paylaşım HAZIRLIĞI; yayınlama yok)
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude test edecek.
Önce OKU: saha/araclar/ajan.py, AJAN.md, test_ajan.py.
1. ajan.py içinde projeye özgü (limo, /home/gunes, Türkçe olmayan) sabit yol/varsayım kalmasın; hepsi ortam
   değişkeni ya da varsayılan XDG yolları (~/.local/state/ajan, ~/.gemini/antigravity-cli/settings.json) olsun.
   Kullanıcıya görünen metinler Türkçe kalsın; `AJAN_DIL=en` ile İngilizce iletiler (basit sözlük; tüm iletileri
   çevirmen gerekmez, en azından durum/bekle/rapor/baslat hata iletileri).
2. saha/araclar/ajan_paket/ altında bağımsız paket iskeleti: pyproject.toml (setuptools, console_scripts ajan =
   ajan:main, Python ≥3.10, bağımlılık yok), README.md (İngilizce: ne işe yarar — Claude Code / herhangi bir
   orkestratör için Gemini (agy CLI) alt ajanı; kurulum; komutlar; izin modeli; stream-json/conversation; tipik
   orkestratör döngüsü baslat→bekle→rapor→test→devam/dogrula; sınırlamalar), README.tr.md (Türkçe aynısı),
   LICENSE dosyası OLUŞTURMA (karar sahibinde; README'de "lisans: belirlenecek" yaz), examples/ (örnek görev
   dosyası, örnek Claude Code skill SKILL.md — ~/.claude/skills/ajan/SKILL.md içeriğini oku ve projeden
   bağımsızlaştır), ajan.py'ye sembolik bağ YERİNE kopya koyma — README'de "ajan.py tek dosyadır" de ve
   pyproject'te py-modules = ["ajan"] ile saha/araclar/ajan.py'yi paket köküne kopyalanacak dosya olarak belirt
   (kopyalamayı Claude yapacak).
3. test_ajan.py ortam değişkenleriyle izole çalışıyor mu kontrol et (gerçek ~/.gemini'ye dokunmamalı); dokunuyorsa düzelt.
