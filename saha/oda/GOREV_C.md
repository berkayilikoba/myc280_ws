# Görev C: oda_dunya.py — duvar/zemin/tavan/mobilya mesh'inden Gazebo Classic SDF dünyası

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_dunya.py (yalnız bu). Python 3.10, yaml, json, argparse, numpy.
Türkçe yorum, kısa kod. Gazebo Classic 11, SDF 1.6.

## Girdiler
- --yapi <oda_yapi.yaml> (Görev B çıktısı): oda_siniri [[x,y],...], duvarlar [{p1, p2, uzunluk, kalinlik}],
  mobilya [{id, merkez, alan_m2, cokgen, ince}]
- --mesh_dizin <dizin> (Görev A çıktısı): oda_mesh.obj (+ .mtl), oda_mesh.json (anahtarlar: sinirlar,
  duvar_rengi_tahmini [r,g,b 0-1], zemin_rengi_tahmini [r,g,b 0-1])
- --tavan (2.7 m), --mobilya_yuk (0.6 m: mesh'te görünmeyen mobilya için kutu yüksekliği), --cikti <x.world>

## Dünya içeriği (koordinatlar = harita çerçevesi, robot (0,0,0)'da başlar)
1. <include> model://sun; fizik ODE max_step_size 0.001 real_time_update_rate 1000; scene ambient 0.5 0.5 0.5,
   shadows false. ground_plane EKLEME (kendi zeminimiz var).
2. "zemin" statik model: oda_siniri çokgeninin sınırlayıcı kutusu + 0.5 m pay, kalınlık 0.02 m kutu, üst yüzü z=0
   (pose z = -0.01); renk zemin_rengi_tahmini (ambient = 0.6*diffuse); sürtünme mu 1.0.
3. "tavan" statik model: aynı alan, z = tavan + 0.01, kalınlık 0.02; renk 0.92 0.92 0.92. Yalnız visual DEĞİL
   collision da olsun.
4. "duvarlar" statik model: her duvar için box link-elemanı: uzunluk = |p2-p1| + kalinlik, genişlik = kalinlik,
   yükseklik = tavan; merkez ((p1+p2)/2, tavan/2), yaw = atan2. Renk duvar_rengi_tahmini. Visual + collision.
5. "mobilya_mesh" statik model: tek link, visual ve collision mesh uri = file://<mutlak yol>/oda_mesh.obj, ölçek 1,
   pose 0. (Gazebo OBJ+MTL renklerini okur.)
6. Mesh'in kapsamadığı mobilya için yedek kutular ("mobilya_kutu" statik model): yapi.mobilya içinde, çokgen merkezi
   mesh sınırlayıcı kutusu (json sinirlar) dışında olanlar -> çokgenin en küçük dönük dikdörtgeni (numpy ile:
   kenar yönlerinde döndürüp alan en küçük) boyutunda, yükseklik --mobilya_yuk kutu; ince=true olanlar silindir
   (yarıçap 0.02, yükseklik 0.7).
7. Dosya başına yorum: kaynak dosyalar, tavan yüksekliği varsayım olduğu notu.

## CLI
oda_dunya.py --yapi Y --mesh_dizin D --cikti W [--tavan 2.7 --mobilya_yuk 0.6]

## Kabul
- Üretilen dosya geçerli XML (xml.etree ile okunur), <world name="lab_oda_3b">.
- Sahte girdilerle (2 duvar, 1 mobilya, json) çalışır; mesh yoksa (dosya bulunamazsa) uyarı verip mobilya_mesh'i atlar.
