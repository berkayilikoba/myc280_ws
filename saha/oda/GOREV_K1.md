# Görev K1: oda_dunya.py — mesh'teki duvar yüzeylerini kes (dokulu duvar kutularıyla çakışmasın)

Dosya: saha/oda/oda_dunya.py (yalnız bu). Mevcut CLI/çıktılar korunmalı.

Sorun: dünyada TSDF mesh'i (mobilya_mesh modeli; duvarlar dahil tüm oda) ve yapı YAML'inden üretilen dokulu
duvar kutuları aynı yerde; duvar iki kez çiziliyor (z-fighting, yamalı görünüm, mesh'teki 2 m üstü delikler).

İstenen:
1. Yeni CLI `--mesh_duvar_kes` (m, varsayılan 0.08; 0 = kapalı). Açıksa dünya yazılmadan önce görsel mesh OBJ'si
   (oda_mesh.obj, `usemtl` grupları olan OBJ + MTL) okunup, TÜM köşeleri herhangi bir duvar parçasına
   (yapı YAML `duvarlar`: p1, p2, kalinlik) yatay düzlemde (x,y) dik uzaklığı <= kalinlik/2 + mesh_duvar_kes OLAN ve
   parçanın uzunluğu boyunca (izdüşüm t ∈ [-0.05 m, L+0.05 m]) kalan üçgenler atılır. Zemin seviyesindeki
   (z < 0.05) üçgenlere ayrıca dokunma (zaten zemin kesilmiş olmalı).
2. Kesilmiş mesh `<mesh_dizin>/oda_mesh_kesik.obj` olarak yazılır (aynı MTL'e `mtllib` ile başvurur, `v`/`vn`/`vt`
   satırları aynen korunabilir; yalnız `f` satırları süzülür; kullanılmayan köşeler kalabilir). Dünyada görsel
   mesh bu dosyayı kullanır; çarpışma mesh'i değişmez.
3. Atılan üçgen sayısı / oranını terminale yaz.
4. Saf yardımcı fonksiyon: `ucgen_duvara_yapisik(ucgen_koseleri (3,3), duvarlar, pay) -> bool` ve vektörel sürümü;
   saha/oda/test_oda.py'ye bu fonksiyon için 2 küçük test ekle (duvara yapışık üçgen atılır, 0.5 m içerideki
   kalır). test_oda.py'deki mevcut testleri DEĞİŞTİRME.
Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
