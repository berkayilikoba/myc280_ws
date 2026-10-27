# Görev I1: oda_tsdf.py — füzyondaki kaymaları (çift kenar, lekeli renk) gider

Dosya: saha/oda/oda_tsdf.py (oku; mevcut CLI, çıktı dosyaları ve poz_matrisi/duzeltme_matrisi fonksiyon imzaları
korunmalı — başka betikler ve testler bunları kullanıyor). Open3D 0.19 (ScalableTSDFVolume) kullanılıyor.

Sorun: 3 sahneden (v2a_lidar, v2b_lidar, rs720_kapsama_lidar; ~2000 kare) birleşik mesh'te kapı kasası, dolap
kenarları, sandalye çift/kaymış; renkler lekeli. Pozlar lidar ICP'den (2B, ~1–3 cm / ~0.5–1° hata), kamera–lidar
düzeltmesi tek sabit (dx/dy/dyaw). Sahneler farklı günlerde: aralarında birkaç cm/derece kayma ve yeri değişen
nesneler (kapı açık/kapalı, sandalye, insan) var. D435 dönüşte bulanık.

İstenen (her biri CLI bayrağıyla açılıp kapanabilir; varsayılan AÇIK):
1. Bulanıklık süzgeci `--bulanik_yuzde 20`: her sahnede seçilen karelerin gri görüntüsünde Laplacian varyansı;
   sahne içi en düşük %N at. (Mevcut --max_w süzgeci kalsın.)
2. Sahneler arası hizalama `--sahne_hizala`: ilk sahne referans. Her sahneyi önce ayrı ayrı kaba TSDF'e (voksel
   0.04) birleştir, nokta bulutu çıkar (voxel_down 0.03, normaller), diğer sahneleri referansa
   registration_icp point-to-plane (max_corr 0.10 -> 0.05 iki aşama) ile hizala; düzeltmeyi YALNIZ yaw + x,y +
   (z küçük) olarak kısıtla: sonuç dönüşümünden yaw'ı çıkar, roll/pitch'i at (zemin düzlemi zaten doğru).
   Düzeltme > 0.15 m veya > 5° ise uygulama, uyar. Her sahnenin düzeltmesini ve fitness/rmse'yi yazdır ve
   hizalama.yaml'a kaydet.
3. Kare başına ince hizalama `--ince` (varsayılan AÇIK, --ince_her 5): sahne birleştirilirken her 5. karede kare
   nokta bulutunu (derinlik <= 2.0 m, voxel 0.02) o ana kadarki TSDF'ten extract_point_cloud'un (her 50 karede
   bir yenilenen önbellek) ilgili kısmına colored ICP (registration_colored_icp, ölçekler 0.04/0.02) ile hizala;
   kabul koşulu fitness > 0.5, düzeltme < 0.05 m ve < 2°. Kabul edilen düzeltme bir sonraki ince karede yenilenene
   kadar o sahnenin sonraki karelerine de uygulanır (sürüklenme düzeltme; birikimli değil, mutlak ofset olarak
   sakla). Kabul oranını ve ortalama düzeltmeyi yazdır. İlk 30 kare (model boşken) atlanır.
4. Tutarsızlık temizliği `--tutarlilik`: sahne başına ayrı TSDF'lerden (adım 2'deki hizalanmış) her final mesh
   köşesi için kaç sahnenin o noktada (1.5 voksel içinde) yüzey gördüğünü, ve kaç sahnenin o noktayı GÖRÜP
   (kamera konisi içinde, derinlik ölçümü noktadan belirgin uzakta = noktanın içinden geçmiş) boş bulduğunu say.
   Boş bulan sahne sayısı > yüzey gören sahne sayısı olan köşeleri (yeri değişmiş nesne) ve bağlı üçgenleri at.
   Basit tutmak için boş-görme testini sahne başına en çok 200 kare örnekle yap. Atılan üçgen oranını yazdır.
5. Renk: TSDF renk entegrasyonunda kare ağırlığı Open3D'de ayarlanamıyorsa, renk için ayrı bir son geçiş öner
   (zorunlu değil): mesh köşe rengini, köşeyi en yakından ve en dik açıyla gören karelerden medyan ile yeniden
   hesapla (`--renk_yeniden`, varsayılan KAPALI; açılınca her köşe için en iyi 5 kare). Zamanın yetmezse bunu
   atla ve özetinde belirt.

Çıktıya ek: hizalama.yaml (sahne düzeltmeleri, ince hizalama istatistikleri, bulanık atılan, tutarlılık atılan).
Bellek/zaman: ~2000 kare, 32 GB RAM, CPU; toplam süre 20 dk'yı geçmemeli (gerekirse ince hizalamayı seyrelt).
Kodu çalıştıramazsın; Claude test edecek:
  python3 oda_tsdf.py cikti/tsdf_i1 <3 sahne> --her 2 --ucgen 150000
Kısa özetle bitir: hangi maddeler yapıldı, hangileri atlandı.
