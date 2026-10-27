# Görev H1: oda_tavan.py — tavan yüksekliğini gerçek D435 derinliğinden ölç (YENİ dosya: saha/oda/oda_tavan.py)

Veri (her sahne dizini, ör. ~/limo_veri/vlmaps_sahne/v2a_lidar, v2b_lidar, rs720_kapsama_lidar):
- depth/NNNNNN.npy: float32 metre, renk görüntüsüne hizalı 640x360, 0 = geçersiz.
- poses_kamera.txt: her satır "x y z qx qy qz qw" = renk OPTİK çerçevesinin (z ileri, x sağ, y aşağı) harita
  çerçevesindeki pozu; satır i <-> depth/i. Kamera yerden ~0.18 m yüksekte, yatay bakıyor; dikey görüş ±21.6°.
- vlmaps_config.yaml: cam_calib_mat (3x3 satır düzeninde 9 sayı): fx 454.49, fy 454.21, cx 324.67, cy 189.46.
- Kamera pozu lidar haritasına göre düzeltilmeli (oda_tsdf.py ile aynı): gövde çerçevesinde dx, dy, dyaw.
  oda_tsdf.py'deki düzeltme fonksiyonunu import ETME, KOPYALA (oda_tsdf.py'yi oku: --dx 0.09 --dy 0.01 --dyaw -1.0,
  T_wc_duz = T_wb @ dT @ inv(T_wb) @ T_wc; poses.txt gövde pozu). Aynı parametreleri CLI'da sun.

CLI: python3 oda_tavan.py <cikti_dizini> <sahne>... [--her 5 --zmin 1.8 --zmax_m 9.0 --dx 0.09 --dy 0.01 --dyaw -1.0]

Yöntem:
1. Her --her'inci karede derinliği 3B noktalara (harita çerçevesi) çevir; yalnız görüntünün üst yarısı (v < cy)
   ve derinlik <= --zmax_m. Dünya z > --zmin noktaları biriktir.
2. Yatay yüzey seçimi: görüntüde 5x5 komşulukla yerel normal (derinlik gradyanından veya komşu 3B noktalarla
   çapraz çarpım) hesapla; |n_z| > 0.9 olan noktalar "yatay". Yatay noktaların z histogramı (2 cm kutu).
3. Tavan = yatay noktaların histogramında en yüksek belirgin tepe (en az toplam yatay noktanın %5'i). Ayrıca
   tüm z>zmin noktaların 99. yüzdeliği. Derinlik gürültüsü σ = 0.00176·d² — uzak noktalar gürültülü, tepeyi
   d ile ağırlıklandır (ağırlık 1/σ²) veya d<6 m ile sınırlı ikinci tahmin ver.
4. Tavan tepkisi yoksa (kamera tavanı hiç görmüyorsa) bunu açıkça yaz: tavan_z: null, ve duvarlarda görülen
   en yüksek z (alt sınır) ver.
5. Çıktılar: <cikti>/tavan.yaml {tavan_z, tavan_z_yakin (d<6m), z99, alt_sinir, yatay_nokta, toplam_nokta,
   sahne_basina: {...}, histogram: [[z, sayi, agirlikli], ...]} ve tavan_hist.png (matplotlib, Agg backend):
   tüm noktalar ve yatay noktalar histogramı, tahmin çizgisi. Ayrıca tavan_ustten.png: tavan bandındaki
   (tavan_z ± 0.1) yatay noktaların x-y dağılımı (tavanın odanın neresinde görüldüğü).
6. Bellek: noktaları kare kare seyrelt (her karede en çok 20k rastgele nokta).

Bağımlılık: numpy, yaml, opencv, matplotlib. Open3D gerekmez. Kodu çalıştıramazsın; Claude test edecek.
Sonunda kısa özet.
