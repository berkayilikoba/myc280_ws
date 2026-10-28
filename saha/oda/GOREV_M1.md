# Görev M1: oda_dunya.py — duvarları açık UV'li OBJ yap, dokulu yüzeyleri ışıksız (unlit) çiz

Dosya: saha/oda/oda_dunya.py (yalnız bu; test_oda.py'ye test EKLEYEBİLİRSİN, mevcutları değiştirme).
Mevcut CLI korunmalı; yeni davranış varsayılan açık, `--isikli` bayrağıyla eskiye dönülebilsin.

## Teşhis (Claude ölçtü, simde başlangıç karesi, Lab L kanalı)
- Gerçek zemin L=121, doku dosyası L=130, sim zemin L=214 (1.8x parlak). Gerçek duvar L=158, sim duvar L=95.
  Dokular gerçek kameradan, yani IŞIĞI ZATEN İÇERİYOR; Gazebo üstüne ortam (0.25) + tavan nokta ışıkları ekliyor:
  yatay zemin aşırı, dikey duvar az aydınlanıyor.
- Gazebo Classic'te <box> yüzlerine doku eşlemesi öngörülemez (zemin dokusu simde dönük/ölçeksiz çıktı; zemin bu
  yüzden açık UV'li OBJ dörtgenine çevrildi, bkz. bu dosyadaki `zemin_doku.obj` üretimi — aynı kalıbı kullan).
  Duvarlar hâlâ <box> + material script: bazı kısa duvarlar simde düz gri, bir duvarda sandalye rengi yanlış yerde.

## İstenen
1. Duvar görseli = OBJ dörtgeni (çarpışma kutusu AYNEN kalsın). doku_veri["duvarlar"][i] (oda_doku.py çıktısı
   doku.yaml): p1, p2, yuz_normali (iç normal, 2B), malzeme "Oda/duvar_i", doku "doku/duvar_i.png". Yapı YAML'deki
   duvarla eşleşme mevcut koddaki gibi (indis).
   oda_doku.py kuralı (duvar_duzlemi_olustur): yüz düzlemi p1_yuz = p1 + 0.5*kalinlik*n_ic; sütun s: p1'den p2
   yönüne 0..uzunluk (W = ceil(uzunluk/coz) teksel); satır 0 = z=tavan (üst), son satır z=0.
   OBJ: köşeler p1_yuz(z=0), p2_yuz(z=0), p2_yuz(z=tavan), p1_yuz(z=tavan); UV (0,0),(1,0),(1,1),(0,1)
   (OBJ v=1 = görüntünün üst satırı). p2_yuz = p1_yuz + uzunluk*u. Ön yüz iç normale baksın (üçgen sarımını
   normal ile çapraz çarpım işaretine göre seç), vn = iç normal. Ayrıca ARKA yüz (dış taraf, kalinlik kadar
   geride, normal ters, UV u ters) aynı dokuyla — duvarın arkasından bakılınca da gri kalmasın.
   Dosyalar: <mesh_dizin>/duvar_doku_<i>.obj (+ ortak veya ayrı .mtl; map_Kd duvar_<i>.png, png'yi mesh_dizin'e
   kopyala — zemin_doku.obj ile aynı yöntem). Visual pozu: duvar modelinin/link'inin pozunu telafi et ya da
   duvar görsellerini ayrı bir statik modelde (pose 0) topla — hangisi kodda daha temizse; köşeler dünya
   koordinatında yazılabilir.
2. Işıksız çizim (dokulu tüm yüzeyler: zemin OBJ, duvar OBJ'leri): her visual'a <material><script> ver ve
   materials/scripts altına (doku_dizin/materials/scripts) yeni bir dosya `oda_isiksiz.material` yaz:
     material Oda/isiksiz/zemin { technique { pass { lighting off  texture_unit { texture zemin.png  filtering trilinear } } } }
     material Oda/isiksiz/duvar_<i> { ... texture duvar_<i>.png ... }
   Doku png'leri doku_dizin/materials/textures içinde zaten var (oda_doku çıktısı). Script, OBJ'nin UV'lerini
   kullanır (mesh visual'ına script materyali Gazebo'da uygulanır). OBJ'deki MTL yedek kalır.
3. Mobilya mesh'i (oda_mesh_kesik.obj, köşe renkleri k-means gruplu MTL: Ka/Kd) da gerçek ışığı içeriyor:
   paketlenen MTL'in ışıksız eşdeğeri için mesh_dizin'e `oda_mesh_isiksiz.mtl` yaz (her malzemede Ka = Kd =
   orijinal Kd, Ks 0) ve kesik OBJ'nin `mtllib` satırını buna çevir; ayrıca dünyada scene <ambient> 1 1 1 1 yap ve
   tavan nokta ışıklarının diffuse'unu 0.0 yap (ışıklar kalsın ama etkisiz; robot modeli de ortam ışığıyla
   görünür). Bu madde Gazebo'nun Ka'yı ortam rengi olarak kullanmasına dayanıyor; `--isikli` ile eski ayar.
4. Tavan: yapı varsa tavan kutusu kalsın, rengi gerçek duvar üst rengine yakın (doku.yaml'da tavan varsa onun
   rengi), ışıksız ise ambient = o renk.
5. Terminale: yazılan duvar OBJ sayısı, ışıksız malzeme sayısı.
6. test_oda.py'ye: sentetik 1 duvarlı yapı + doku YAML ile dünya üret, duvar_doku_0.obj var, 4 köşe + UV satırları
   var, ön yüz normali iç normal yönünde, oda_isiksiz.material içinde "lighting off" geçiyor.
Kodu çalıştıramazsın; Claude derleyip simde ölçecek. Kısa özetle bitir.
