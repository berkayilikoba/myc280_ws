# Görev J2: mesh_karsilastir.py'ye sayısal ölçüt ekle

Dosya: saha/oda/mesh_karsilastir.py (oku; yalnız bu). Şu an mesh'ten yoğun nokta örnekleyip gerçek kamera
pozundan z-tamponlu renk görüntüsü üretiyor ve gerçek kareyle yan yana koyuyor. Bunu ölçülebilir yap:

1. ciz() renkle birlikte derinlik görüntüsünü de döndürsün (aynı z-tampon; float32 metre, 0 = boş).
2. Gerçek derinlik: <sahne>/depth/NNNNNN.npy (float32 metre, renge hizalı 640x360, 0 = geçersiz).
3. Kare ve mesh başına ölçütler (yalnız iki derinliğin de geçerli olduğu ve gerçek derinlik <= 3.0 m pikseller):
   derinlik mutlak medyan farkı (cm), |fark|<3 cm oranı, kapsama (gerçek geçerli piksellerin mesh'te dolu oranı),
   renk farkı: Lab uzayında ΔE medyanı (cv2.cvtColor ile), keskinlik: mesh görüntüsünün Laplacian varyansı /
   gerçek görüntününki, ve kenar uyumu: Canny kenarları (gerçek ve mesh) arasında 3 px tolerans ile F1.
4. Çıktı: <cikti>.yaml (kare ve mesh başına + mesh başına ortalama) ve terminale mesh başına özet tablosu.
5. Varsayılan kare listesi "0,300,700,1100" yerine --kareler verilmezse --aralik 100 ile tüm sahneden örnek al
   (görsel PNG yalnız ilk 4 kare için üret, ölçütler tüm örnek kareler için).
6. Mevcut CLI ve PNG çıktısı korunmalı.
Bağımlılık: numpy, cv2, open3d, yaml. Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
