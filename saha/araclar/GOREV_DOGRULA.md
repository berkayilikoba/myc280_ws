# Görev: ajan.py'ye `dogrula` komutu — otomatik test → düzelt döngüsü
Dosyalar: saha/araclar/ajan.py, saha/araclar/test_ajan.py (mevcut 15 testi bozma, yeni test ekle),
saha/araclar/AJAN.md (kılavuza bölüm ekle). ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz.
Önce ajan.py'yi oku (cmd_devam, wait_for_tasks, gorev_canli, meta yapısı).

`ajan dogrula ETIKET --test "KOMUT" [--tur 3] [--cwd DIZIN] [--zaman-asimi S]`:
1. ETIKET "bitti" ya da "hata" durumunda olmalı (çalışıyorsa önce bekle).
2. KOMUT'u (shell=True, cwd verilmişse orada, zaman aşımıyla) Claude'un ortamında çalıştır (ajan'ın kendisi
   çalıştırır, Gemini değil). Çıkış 0 → "DOGRULANDI" yaz, meta'ya dogrulama geçmişi ekle, çıkış 0.
3. Başarısızsa: çıktının son 60 satırını (stdout+stderr) al, `devam` ile aynı konuşmaya gönder:
   "Doğrulama komutu başarısız: <KOMUT>\n```\n<çıktı>\n```\nKök nedeni bul ve düzelt. Testleri gevşetme."
   bitmesini bekle, tekrar test et. En çok --tur kadar. Her tur meta["dogrulama"] listesine {tur, komut, cikis,
   ozet(ilk FAILED/Error satırları), zaman}.
4. Tur bitince başarısızsa "DOGRULANAMADI (n tur)" ve çıkış 1.
5. `ajan durum`'a DOGRULAMA sütunu: son dogrulama sonucu (✓/✗/-).
Testler (sahte agy + sahte test komutu): ilk denemede geçen; bir turda düzelen (sahte test bir dosya varlığına
bakar, sahte agy ikinci turda o dosyayı yazar); tur sınırında başarısız; çalışan görevde ret.
