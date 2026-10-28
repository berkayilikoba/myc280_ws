# Görev I2: oda_tsdf.py — insanları füzyondan çıkar (YOLO segmentasyon maskesi)

Dosya: saha/oda/oda_tsdf.py (yalnız bu). Mevcut CLI ve diğer özellikler (bulanık, sahne_hizala, ince, tutarlilik,
renk_yeniden) korunmalı.

Sorun: kayıtlarda bir kişi uzun süre aynı yerde durmuş; TSDF'te kalıcı "insan hayaleti" oluşuyor. Sahneler arası
tutarlılık testi bunu yakalamıyor (aynı sahne içinde).

İstenen:
1. CLI: `--insan_maske` / `--no-insan_maske` (varsayılan AÇIK), `--yolo_model` (varsayılan
   /home/gunes/limo_robot/robot_ws_ref/koray_agile_ws/src/ultralytics_ros/models/yolov8m-seg.pt),
   `--maske_genislet` (piksel, varsayılan 15), `--yolo_guven` (varsayılan 0.35),
   `--maske_siniflar` (virgüllü COCO sınıf no, varsayılan "0" = person).
2. ultralytics (8.3, kurulu) `YOLO(model)` ile, renk görüntüsünde (640x360 BGR) segmentasyon; GPU varsa
   device=0, yoksa cpu. Seçili sınıfların maskelerini görüntü boyutuna getir, birleştir, cv2.dilate ile genişlet.
   Toplu (batch 16) çıkarım yap; yalnız füzyona girecek kareler için çalıştır.
3. Maskeli piksellerde derinliği 0 yap (TSDF'e girmesin). Ayrıca renk yeniden hesaplama (renk_yeniden) ve
   ince hizalama da aynı maskeli derinliği kullanmalı. Sahne hizalama kaba TSDF'i de maskeli olmalı.
4. Sahne başına maskelenen kare sayısı ve ortalama maskeli piksel oranını yazdır; hizalama.yaml'a "insan_maske"
   bölümü ekle.
5. ultralytics import edilemezse veya model dosyası yoksa uyarı verip maskesiz devam et.
Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
