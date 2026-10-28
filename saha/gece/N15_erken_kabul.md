# Görev N15: kavrama_dugumu.py — NAV_Q erken kabul ve zaman aşımı (Claude teşhisi, cikti/g5)
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude test edecek.
Dosyalar: src/limo_autonomy/limo_autonomy/kavrama_dugumu.py, src/limo_autonomy/test/test_kavrama_dugumu.py
(mevcutları bozma, ekle). Önce dosyayı OKU (NAV_Q akışı, q_yakin_mi, 50 Hz döngü, eylem sunucusu).
Bulgu: aynı dolap hedefinde bir koşuda Nav2 Q'ya 8 s'de vardı, diğerinde MPPI Q'nun yakınında 14 kez "Failed to
make progress" verip ~5 dk sonra ABORTED döndü; kavrama düğümü bu sürede bekledi (yakın kabul yalnız Nav2 SONUCU
gelince değerlendiriliyor). Q'ya hassas varış gerekmiyor (son yaklaşma kapalı çevrim düzeltir).
İstenen:
1. Erken kabul: NAV_Q sürerken 50 Hz döngüde robotun map pozunu Q ile karşılaştır; `q_erken_xy` (param, vars.
   0.20 m) ve `q_erken_yaw` (vars. 20°) içinde `q_erken_sure` (vars. 1.0 s) boyunca kalırsa Nav2 hedefini İPTAL et
   (cancel_goal_async), iptal sonucunu bekleme (yarış: iptal sonucu geldiğinde durum zaten SON_YAKLASMA ise yok say)
   ve SON_YAKLASMA'ya geç; durumda 'NAV_Q erken_kabul (d cm, a°)'.
2. Zaman aşımı: NAV_Q `nav_q_zaman_asimi` (vars. 90 s) içinde bitmezse Nav2'yi iptal et; robot q_kabul_xy/yaw
   (mevcut 0.35 m/35°) içindeyse SON_YAKLASMA, değilse sonraki aday (mevcut başarısızlık yolu).
3. Nav2 sonucu ABORTED geldiğinde mevcut yakın kabul aynen kalsın. Tüm yolların tek bir geçiş fonksiyonundan
   geçtiğinden ve aynı tikte iki kez tetiklenmediğinden emin ol (N13'teki geçiş koruması).
4. Eylem geri bildiriminde NAV_Q sırasında `kalan_mesafe_m` = robot–Q mesafesi.
5. Saf yardımcı `erken_kabul_durumu(gecmis: list[(t, d, a)], xy, yaw, sure) -> bool` + testler (eşik içinde
   yeterince uzun → True, kısa süre → False, eşik dışına çıkınca sayaç sıfırlanır).
