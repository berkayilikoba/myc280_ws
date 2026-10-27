# Görev N11: ilk uçtan uca sim denemesinin düzeltmeleri (Claude teşhisi, cikti/k1)
Bulgular: (1) Nav2 Q'ya 13–30 cm yaklaştı ama `hassas_hedef` (xy 0.08 / yaw 0.10) MPPI ile tutturulamadı →
"Failed to make progress" ×21 → NavigateToPose ABORTED (6). Q'dan sonrasını kapalı çevrim son yaklaşma düzeltiyor
(N2 testleri 8 cm/15° başlangıç hatasını kapatıyor), Q'da 8 cm gereksiz. (2) kavrama_bench.py zaman aşımında
kavramayı iptal etmiyor → sonraki hedef 'HATA önceki görev sürüyor'. (3) bench 'toplam_sure_s' hep 0.0.

Dosyalar ve istenenler:
A) src/limo_bringup/param/nav2_diff.yaml: hassas_hedef xy_goal_tolerance 0.08→0.15, yaw_goal_tolerance
   0.10→0.25 (satır sonu yorumu güncelle: "son yaklaşma kapalı çevrim düzeltir; MPPI 0.08'i tutturamadı, k1").
   Başka değişiklik yok.
B) src/limo_autonomy/limo_autonomy/kavrama_dugumu.py: NAV_Q sonucu başarısız (ABORTED/CANCELED dışı her
   başarısızlık, iptal hariç) olsa da robotun map pozu Q'ya `q_kabul_xy` (param, vars. 0.35 m) ve `q_kabul_yaw`
   (param, vars. 35°) içindeyse SON_YAKLASMA'ya geç ve durumda 'NAV_Q yakin_kabul (d cm, a°)' yaz. Değilse
   mevcut davranış (sonraki aday). Saf yardımcı `q_yakin_mi(robot, q, xy, yaw) -> (bool, d, a)` + test
   (test_kavrama_dugumu.py'ye EKLE, mevcutları değiştirme).
C) src/limo_bringup/scripts/kavrama_bench.py: zaman aşımında `/kavrama/iptal` (std_msgs/Empty) yayınla ve
   kavrama düğümünden 'HATA' durumunu en çok 10 s bekle; toplam_sure_s'yi hedef gönderiminden sonuca kadar
   doğru ölç (sim zamanı ya da duvar saati, hangisi tutarlıysa — ikisini de yaz: sure_sim_s, sure_duvar_s).
