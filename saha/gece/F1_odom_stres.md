# Gece görevi F1: sensör modeline görsel odometri (VIO) hata profili — stres testi için
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude derleyip test edecek.
Dosyalar: src/limo_kinematics/limo_kinematics/sensor_modeli_dugumu.py (yalnız OdomHatasi ve parametre okuma kısmı;
derinlik/renk/IMU/lidar kısmına DOKUNMA), src/limo_kinematics/test/test_sensor_modeli.py'ye test EKLEME izni yoksa
saha/gece/test_odom_stres.py (importlib ile modülü yükle). Önce dosyayı OKU (OdomHatasi sınıfı: k_w, k_v, sigma_w).
Amaç: gerçek odom ileride görsel+teker+IMU füzyonundan gelecek (başka ekip). Nav2/kavrama katmanının buna
dayanıklılığını simde ölçmek için /odom'a VIO'ya özgü hatalar eklenebilsin. Varsayılan davranış DEĞİŞMESİN.
Yeni parametreler (hepsi varsayılan kapalı/0):
- odom_profil: 'teker' (vars., mevcut davranış) | 'vio' | 'vio_kotu'. 'vio' ön ayarı: olcek_kayma_hizi
  0.002 1/m (ölçek hatası mesafeyle rastgele yürür, σ), yaw_kayma 0.3 °/m (rastgele yürüyüş σ), konum
  rastgele yürüyüş 0.005 m/√m, sicrama_olasiligi 0.0 ; 'vio_kotu': 0.005, 0.8 °/m, 0.01, sıçrama 0.02 /s
  (sıçrama: konumda N(0, 0.05 m), yaw'da N(0, 2°) tek seferlik kayma — VIO yeniden başlatma/izleme kaybı).
- Her biri ayrıca tek tek parametreyle ayarlanabilsin (profil değerlerinin üstüne yazar): vio_olcek_kayma,
  vio_yaw_kayma_deg_m, vio_konum_ry, vio_sicrama_hz, vio_sicrama_konum, vio_sicrama_yaw_deg, vio_tohum.
- Uygulama: OdomHatasi'na (ya da yeni VioHatasi sınıfına, OdomHatasi'nı sarmalayarak) her adımda kat edilen
  mesafe ds ve dönüş dθ ile hata durumunu güncelle; /odom pozu = gerçek artışların hatalı tümlevi. TF
  odom->base_footprint aynı pozla. Twist (hız) alanına ölçek hatası uygula, sıçramayı hıza YANSITMA.
- Başlangıç log satırına profil ve değerler.
- Saf testler: 'teker' profilinde eski çıktı birebir aynı (aynı tohum); 'vio' ile 10 m düz yolda konum hatası
  dağılımının σ'sı beklenen büyüklükte (birkaç cm; ±%50); sıçrama hz 0'da sıçrama yok; 'vio_kotu'da 100 s'de en az
  bir sıçrama (tohum sabit).
