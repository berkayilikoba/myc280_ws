# Gece görevi F2: kameradan nesne algısı prototipi (renk + derinlik) ve simde kırmızı elma modeli
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude derleyip simde test edecek.
Bağlam: kavrama katmanı nesne konumunu `/kavrama/nesne_algi` (PointStamped) bekliyor; şimdilik sahte_algi.py
(Gazebo gerçeğinden) üretiyor. İlk gerçek algı: renkle nesne bulma. sahte_algi.py'yi OKU (arayüz, durum konusu).
Dosyalar (yeni): src/limo_autonomy/limo_autonomy/renk_algi.py, src/limo_autonomy/test/test_renk_algi.py,
src/limo_car/models/elma/{model.config,model.sdf}. setup.py'ye DOKUNMA (Claude ekleyecek).
1. renk_algi.py düğümü: /camera/color/image_raw (rgb8/bgr8, 1280x720), /camera/color/camera_info,
   /camera/depth/image_raw (16UC1 mm, 848x480, KENDİ camera_info'su /camera/depth/camera_info; renk ve derinlik
   farklı çözünürlük ve çerçevede: camera_color_optical_frame / camera_depth_optical_frame). Yaklaşım: derinliği
   kullanmak için renk pikselindeki nesneyi bul → nesne maskesinin merkezinin renk ışını; derinlik görüntüsünde
   TF (color_optical → depth_optical, tf2) ile o ışını izdüşür ve çevresindeki (7x7) geçerli derinliklerin
   medyanını al; basit yol: renk ve derinlik kameraları neredeyse aynı yerde (y 1.5 cm fark) — önce saf
   fonksiyonla piksel eşlemesi (K_renk, K_derinlik, T_renk→derinlik) yap.
   - Renk eşiği: HSV, parametreler hsv_alt [0, 120, 70], hsv_ust [10, 255, 255] ve ikinci aralık [170..180] (kırmızı
     sarması), min_alan_px 150, en büyük bileşen; morfolojik açma.
   - Çıktı: nesne merkezinin 3B konumu (renk optik çerçevesinde) → PointStamped, frame camera_color_optical_frame,
     stamp = renk görüntüsü stamp'i; /kavrama/nesne_algi_durum 'GORUYOR'/'GORMUYOR'. Hız sınırı hz (vars. 5).
   - Derinlik nesnenin yüzeyidir; merkez için yarıçap kadar (parametre nesne_yaricap 0.035) ışın boyunca ilerlet.
   - Hata ayıklama görüntüsü (parametre hata_ayiklama, vars. false): /kavrama/algi_goruntu (maske + işaret).
2. Saf fonksiyonlar + testler (sentetik görüntü: siyah zemin, bilinen yerde kırmızı daire; sentetik derinlik):
   maske bulma, merkez, piksel→ışın, ışın+derinlik→3B, renk↔derinlik piksel eşlemesi (bilinen dış kalibrasyon),
   kırmızı sarması (H 175 de bulunur), küçük leke reddi.
3. Elma modeli (SDF 1.6): statik değil, kütle 0.1 kg, küre r 0.035, malzeme kırmızı (ambient/diffuse 0.8 0.05 0.05),
   çarpışma küresi; model.config. (Işıksız dünyada ambient önemli: ambient 0.8 0.05 0.05 1 olsun.)
Yanıtta Claude'un setup.py'ye ekleyeceği satırı ve simde elmayı (5.62, 0.8, 0.035) konumuna koyacak
`ros2 run gazebo_ros spawn_entity.py -entity elma -file ... -x .. -y .. -z ..` komutunu yaz.
