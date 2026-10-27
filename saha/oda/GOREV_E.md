# Görev E: oda_dogrula.py — sim odası ile gerçek odanın kamera karşılaştırması (ROS 2 Humble + Gazebo Classic)

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_dogrula.py (yalnız bu). Python 3.10, rclpy, numpy, cv2, yaml.

Sim çalışırken (robot modeli adı "limo_mycobot", use_sim_time), gerçek sahnedeki kare pozlarına robotu ışınlayıp sim
renk ve derinlik görüntüsünü alır, gerçek kareyle karşılaştırır.

## Girdiler
- <sahne> (Görev A biçimi; poses.txt gövde pozu = base_footprint, harita = Gazebo dünyası), --kareler 0,100,200,... ya da
  --her 100; poz düzeltmesi --dx 0.09 --dy 0.01 --dyaw -1.0 (sim kamerasını gerçek kameranın DÜZELTİLMİŞ pozuna koymak için:
  gövde pozunu T_wb @ dT olarak ışınla; sim kamerası URDF'teki yerde).
- Konular: /camera/color/image_raw (rgb8 1280x720), /camera/depth/image_raw (16UC1 mm, 848x480),
  /camera/depth/camera_info, /camera/color/camera_info. Gazebo servisi /gazebo/set_entity_state
  (gazebo_msgs/srv/SetEntityState; state.name "limo_mycobot", pose z 0.15'ten değil 0.0 ver? -> modelin doğduğu z: mevcut
  /gazebo/get_entity_state ile oku ve z'yi koru), reference_frame "world".

## Her kare
1. Işınla, 1.0 s sim zamanı bekle (görüntüler yenilensin), sonra en son renk ve derinlik karesini al.
2. Sim derinliğini renk kamerasına hizala: derinlik optik -> renk optik dönüşümü y ekseninde 0.0148 m (renk, derinliğin
   +y'sinde = görüntüde x'te -0.0148 m: P_renk = P_derinlik + [-0.0148, 0, 0]); renk K'sını gerçek sahnenin 640x360
   K'sına (vlmaps_config cam_calib_mat) göre kullan: sim renk görüntüsünü 640x360'a küçült, derinliği bu K ile izdüşür
   (z-tamponu).
3. Karşılaştır (gerçek: rgb/<i>.png, depth/<i>.npy):
   - derinlik: ikisinin de geçerli olduğu piksellerde |sim - gerçek| medyanı, <10 cm oranı; geçerli oranları;
     satır bantlarına göre (0-120, 120-200, 200-260, 260-360) geçerli oran
   - renk: iki görüntüyü Lab'a çevir; ortalama ΔE (yalnız ikisinin de derinliği geçerli olan pikseller), HSV histogram
     korelasyonu (cv2.compareHist CORREL), kenar haritası (Canny) örtüşmesi: sim kenarlarının 5 px içinde gerçek kenar oranı
4. Yan yana görsel: <cikti>/kare_<i>.png = [gerçek renk | sim renk] üstte, [gerçek derinlik | sim derinlik] altta
   (renk haritası JET, 0-5 m).
5. Özet <cikti>/ozet.yaml: kare başına ve ortalama ölçütler.

## CLI
oda_dogrula.py <sahne> <cikti_dizini> [--her 150 --dx 0.09 --dy 0.01 --dyaw -1.0]

## Kabul
- Kod python3 -m py_compile ile derlenir; servis yoksa anlaşılır hata verir; her kare için yaml'a satır yazar.
