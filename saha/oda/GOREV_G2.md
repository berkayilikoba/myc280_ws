# Görev G2: oda_dogrula.py — sim derinliğini kaplamalı (splat) hizalama ile renk kamerasına izdüşür

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_dogrula.py (DÜZENLE).
Sorun: sim derinliği renk görüntüsüne nokta nokta izdüşürülüyor; derinlik pikselleri renk ızgarasına seyrek düştüğü
için sim derinlik görselinde ve ölçütlerde düzenli ızgara boşlukları oluşuyor.
Düzeltme: depodaki hazır fonksiyonu kullan:
  import sys; sys.path.insert(0, '/home/gunes/limo_robot/limo_ws/saha/analiz')
  from bag2vlmaps import derinlik_hizala
  imza: derinlik_hizala(dmm, Kd, Kc, T_c_d, boyut, zmin=0.2, zmax=8.0, doldur=True, kapla=True) -> float32 (H, W) metre
  dmm: uint16 mm derinlik (H_d, W_d); Kd derinlik K (3x3), Kc hedef renk K (3x3, 640x360 ölçekli), T_c_d 4x4 derinlik
  optik -> renk optik (öteleme [-0.0148, 0, 0]), boyut (360, 640). bag2vlmaps içe aktarılırken rosbag2_py vb. ROS
  modülleri de yüklenir — ortam ROS ile source edilmiş olacak, sorun değil.
- Mevcut kendi izdüşüm kodunu bu çağrıyla değiştir (kapla=True); sim derinliğine sensor_modeli boşlukları zaten
  uygulanmış, ek doldurma yapma.
Kabul: python3 -m py_compile geçer; ölçüt ve görsel çıktı biçimi değişmez.
