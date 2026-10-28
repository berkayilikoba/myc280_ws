# Görev N10: kavrama + görev katmanı launch dosyası (YENİ)
Dosya: src/limo_autonomy/launch/kavrama.launch.py (yeni). Mevcut launch dosyalarını (src/limo_autonomy/launch/*)
OKU, üslubu izle. setup.py launch/*.launch.py'yi zaten kuruyor.
Başlatılacaklar (hepsi use_sim_time argümanıyla):
- limo_autonomy/kavrama (parametre harita: argüman `harita`, varsayılan
  get_package_share_directory('limo_bringup')/maps/lab_oda.yaml),
- limo_autonomy/gorev (parametre nesneler: argüman `nesneler`, varsayılan
  get_package_share_directory('limo_autonomy')/config/nesneler_lab_oda.yaml; kol_sahte: argüman, vars. true),
  — gorev_dugumu.py'deki gerçek parametre adlarını OKU ve onları kullan,
- limo_autonomy/sahte_algi yalnız `sahte_algi:=true` iken (vars. false); nesne_x/y/z argümanları (vars. elma:
  0.23, 1.40, 0.75).
Argümanlar: use_sim_time (vars. false), harita, nesneler, kol_sahte, sahte_algi, nesne_x, nesne_y, nesne_z.
Dosya başına kullanım örneği yorumu: sim için
  ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true sahte_algi:=true
ve görev isteği: ros2 topic pub --once /gorev/istek std_msgs/String "{data: elma}".
