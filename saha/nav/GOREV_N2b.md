# Görev N2b: son_yaklasma.py — son bantta salınım (DONUYOR'da takılma) düzeltmesi
Dosyalar: src/limo_autonomy/limo_autonomy/son_yaklasma.py, src/limo_autonomy/test/test_son_yaklasma.py.
Claude teşhisi: 3 test 'DONUYOR'da kalıyor. `rho < son_donus_mesafe` bandında robot ya yerinde dönüyor (rho
azalmaz) ya da P'ye `w = k_alpha*alpha` ile sürüyor (P yanda kaldığında alpha büyük → yön bozulur → tekrar dönüş):
diff sürüş yanal hatayı yerinde dönerek kapatamaz (holonom değil) → sınır çevrimi.

Yeni son bant mantığı (rho < son_donus_mesafe ya da bir kez girildiyse histerezisle rho < 2*son_donus_mesafe):
- Hataları HEDEF çerçevesinde hesapla: e_ileri = (robot→P) · [cos th_g, sin th_g], e_yanal = (robot→P) ·
  [-sin th_g, cos th_g], e_yaw = wrap(th_g - ryaw).
- Aşama A (HIZALA): |e_yaw| > tol_yaw ise v=0, w = clamp(k_yaw*e_yaw) (k_yaw yeni ayar, vars. 1.5; |w| en az
  w_min_donus 0.05 — ölü bant), durum 'DONUYOR'.
- Aşama B (SURUN): v = clamp(k_ileri*e_ileri, ±v_surunme) (k_ileri 1.0, v_surunme 0.03, |v| en az v_min yalnız
  |e_ileri|>tol_xy iken), w = k_yaw*e_yaw (yönü tut), durum 'YAKLASIYOR'. Geri gerekebilir (e_ileri<0).
- TAMAM: |e_ileri| < tol_xy VE |e_yaw| < tol_yaw VE |e_yanal| < tol_yanal (yeni, vars. 0.015).
- |e_yanal| ≥ yanal_hata_esik (yeni, vars. 0.03) ise 'HATA:yanal' (çağıran geri çekilip yeniden dener).
  tol_yanal ≤ |e_yanal| < yanal_hata_esik iken e_ileri ve e_yaw sağlandıysa da TAMAM say ve bilgi['uyari'] =
  'yanal' yaz (kol küçük yanal hatayı tolere eder) — bunu ayar `yanal_kabul_genis` (vars. True) ile denetle.
- Bant dışında Astolfi aynen kalır (asimptotik kararlı, banda küçük yanal hatayla girer).
Testler: mevcut 3 başarısız test geçmeli (ölçütleri GEVŞETME); yeni test: banda 2 cm yanal hatayla girilirse
'HATA:yanal' değil TAMAM+uyarı (yanal_kabul_genis) ya da yanal 4 cm → 'HATA:yanal'.
