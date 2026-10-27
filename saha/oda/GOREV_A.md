# Görev A: oda_tsdf.py — kamera sahnelerinden renkli 3B oda mesh'i (TSDF füzyonu)

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_tsdf.py (yalnız bu dosyayı yaz; başka dosyaya dokunma).
Python 3.10, numpy, opencv (cv2), open3d 0.19, yaml, argparse. Kodu Türkçe yorumla (mevcut depo üslubu), kısa tut.

## Girdi: bir ya da birden çok "sahne" dizini (ör. ~/limo_veri/vlmaps_sahne/v2a_lidar)
- rgb/000000.png ... : BGR 640x360 (cv2.imread)
- depth/000000.npy ... : float32 (360, 640), metre, renk görüntüsüne hizalı, 0 = geçersiz
- poses.txt : satır i = kare i'nin GÖVDE (base_footprint) pozu, harita çerçevesi, "x y z qx qy qz qw"
- poses_kamera.txt : satır i = kare i'nin RENK OPTİK kamera pozu (x sağ, y aşağı, z ileri), harita çerçevesi, aynı biçim
- zaman.txt : satır i = "i t_renk t_derinlik" (saniye)
- vlmaps_config.yaml : anahtar cam_calib_mat = 9 sayı (3x3 K, satır öncelikli)

## Kalibrasyon düzeltmesi (zorunlu, CLI ile)
Kamera derinliği lidar haritasına göre kaymış; gövde çerçevesinde düzeltme --dx --dy (m) --dyaw (derece)
(varsayılan dx 0.09, dy 0.01, dyaw -1.0). Her kare için:
  T_wb = pose(poses.txt[i]), T_wc = pose(poses_kamera.txt[i]), T_bc = inv(T_wb) @ T_wc
  dT = 4x4 (yaw dyaw etrafında z dönüşü, öteleme (dx, dy, 0))
  T_wc_duz = T_wb @ dT @ T_bc        # düzeltilmiş kamera pozu (harita çerçevesi)

## Kare seçimi
- --her N (varsayılan 2): her N. kare
- gövde açısal hızı |dyaw/dt| > --max_w (varsayılan 0.25 rad/s) olan kareleri atla (yaw poses.txt'ten, t zaman.txt 2. sütun;
  np.unwrap + np.gradient)

## Füzyon (open3d.pipelines.integration.ScalableTSDFVolume)
- voxel_length --voxel (0.02), sdf_trunc 4*voxel, color_type RGB8
- her kare: derinlik --zmax (2.5 m) üstü 0 yapılsın; RGBDImage.create_from_color_and_depth(depth_scale=1.0,
  depth_trunc=zmax, convert_rgb_to_intensity=False); renk BGR->RGB çevrilmeli
- extrinsic = inv(T_wc_duz) (open3d dünyadan kameraya ister); intrinsic PinholeCameraIntrinsic(640, 360, fx, fy, cx, cy)
- birden çok sahne aynı hacme eklenir; ilerleme yazdır (her 100 karede)

## Mesh sonrası
1. extract_triangle_mesh(); compute_vertex_normals()
2. küçük bileşenleri at: cluster_connected_triangles, üçgen sayısı < --min_bilesen (500) olan kümeleri sil
3. --zemin_kes (0.02): z < bu olan üçgenleri sil (zemin ayrı düzlemle modellenecek); z > --tavan_kes (2.0) olanları da sil
4. simplify_quadric_decimation(target_number_of_triangles=--ucgen (300000)); remove_degenerate/duplicated/unreferenced
5. kaydet: <cikti>/oda_mesh.ply (vertex renkli)

## Gazebo için renk gruplu OBJ (Gazebo Classic vertex rengini göstermez)
- üçgen rengi = 3 köşe renginin ortalaması; k-means (cv2.kmeans, K = --renk (32)) ile renklere küme
- <cikti>/oda_mesh.obj + oda_mesh.mtl: her küme bir malzeme ("m00".."m31"), Kd = küme rengi (0-1), Ka = 0.6*Kd;
  OBJ'de "mtllib oda_mesh.mtl", "v", "vn" (köşe normali), her küme için "usemtl mXX" + o kümenin "f a//a b//b c//c" yüzleri
  (1 tabanlı indeks). Koordinatlar harita çerçevesinde metre (Gazebo dünyası = harita çerçevesi, ölçek 1).
- <cikti>/oda_mesh.json: {"ucgen": n, "kose": n, "sinirlar": [[xmin,ymin,zmin],[xmax,ymax,zmax]], "kare": kullanılan kare,
  "renkler": [[r,g,b]...], "duvar_rengi_tahmini": dikey yüzeylerin (|normal z| < 0.3) medyan rengi 0-1,
  "zemin_rengi_tahmini": zemin_kes altında atılan üçgenlerin medyan rengi 0-1, "duzeltme": {dx,dy,dyaw}}

## CLI
oda_tsdf.py <cikti_dizini> <sahne> [<sahne> ...] [--dx 0.09 --dy 0.01 --dyaw -1.0 --her 2 --max_w 0.25 --voxel 0.02
  --zmax 2.5 --zemin_kes 0.02 --tavan_kes 2.0 --min_bilesen 500 --ucgen 300000 --renk 32]

## Kabul
- `python3 oda_tsdf.py /tmp/x ~/limo_veri/vlmaps_sahne/v2a_lidar --her 20` hatasız çalışır, dört çıktı dosyası oluşur.
- OBJ geçerli (open3d.io.read_triangle_mesh ile okunur), yüz sayısı PLY ile aynı.
- Harita çerçevesi korunur: mesh sınırları x yaklaşık -2..9, y -1.4..3.1 aralığında.
