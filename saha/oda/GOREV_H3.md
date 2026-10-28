# Görev H3: saha/oda için birim testleri (YENİ dosya: saha/oda/test_oda.py, pytest)

Test edilecek betikler (oku; yalnız test dosyasını yaz, betikleri DEĞİŞTİRME): oda_dunya.py, oda_doku.py,
oda_tsdf.py. (oda_yapi.py şu an başka görevde değişiyor; ona dokunma, test de yazma.)

Hedef: ROS/Gazebo/gerçek veri olmadan, sentetik küçük girdilerle saniyeler içinde çalışan testler.
Betikleri importlib ile dosya yolundan modül olarak yükle (sys.path'e saha/oda ekle). Open3D gerektiren
testleri `pytest.importorskip("open3d")` ile koru.

Önerilen testler (betiklerdeki gerçek fonksiyon adlarını OKUYARAK kullan, uydurma):
1. oda_dunya: küçük sentetik yapı YAML'i (tmp_path: oda_siniri, 2 duvar, 1 mobilya) + boş/küçük mesh dizini ile
   dünya üret (main veya fonksiyon); çıktı xml.etree ile parse edilmeli; zemin, tavan, duvar modelleri var;
   gazebo_ros_state plugin'i var; ışık sayısı > 0; tavan yüksekliği parametresi duvar boyuna yansıyor.
2. oda_dunya --sadelestir: open3d ile küçük küre mesh'i yaz, sadeleştir, üçgen sayısı <= hedef.
3. oda_doku: poz düzeltme / izdüşüm gibi saf fonksiyonlar: bilinen bir 3B nokta bilinen K ile beklenen piksele
   düşüyor; mobilya altı maskesi çokgen içini True yapıyor.
4. oda_tsdf: poz düzeltme fonksiyonu dx=dy=dyaw=0 iken kimlik; dx=0.1 iken kamera konumu gövde x ekseninde
   0.1 kayıyor.
Her test bağımsız, tmp_path kullanmalı, ağ/ROS yok. Kodu çalıştıramazsın; Claude `python3 -m pytest saha/oda/test_oda.py`
ile çalıştıracak. Kısa özetle bitir.
