# Görev G3: oda_yapi.py — duvarları baskın eksene hizala (snap) ve yeniden birleştir

Dosya: saha/oda/oda_yapi.py (yalnız bu dosyayı değiştir). Mevcut CLI ve YAML biçimi korunmalı.

Sorun: eksen süzgecinden (adım 2, theta1/theta2) geçen ama eksene göre 2–10° eğik kısa duvar parçaları kalıyor;
yan yana iki eğik parça birbirini kesiyor (sol duvar, kapı kenarları). Gerçek oda dikdörtgen eksenli.

İstenen (adım 2'den sonra, adım 3 kesişim temizliğinden ÖNCE):
1. Yeni CLI: `--snap_tol` (derece, varsayılan 10.0; 0 ise kapalı).
2. theta1 veya theta2'ye açı farkı <= snap_tol olan her duvarı o eksene hizala: duvarın doluluk ağırlıklı değil,
   basitçe orta noktası etrafında, yönü tam eksen açısı olacak şekilde; uç noktaları eski uçların yeni eksen
   üzerine izdüşümü (uzunluk eksen yönündeki izdüşüm aralığı). Daha iyisi: orta nokta yerine o duvarın yakınındaki
   (dist_to_segment <= kalinlik) dolu hücrelerin eksene dik koordinatının MEDYANI ile hat konumunu belirle.
3. Hizalamadan sonra aynı eksende, dik uzaklığı <= 0.06 m ve uçlar arası boşluğu <= 0.3 m olan parçaları tek
   parçada birleştir (mevcut can_merge / merge_two_lines kullanılabilir; açı eşiği zaten 0 olur).
4. uzunluk ve doluluk alanlarını yeni uçlarla yeniden hesapla (hesapla_doluluk).
5. Terminal özetine "eksene hizalanan: N, birleşen: M" ekle.
6. Hizalanan duvar sayısı 0 değilken PNG çıktısı yeni duvarları göstermeli (mevcut çizim zaten gecerli listeyi
   kullanıyorsa ek iş yok).

Kısıt: numpy/opencv/yaml dışında yeni bağımlılık yok. Kodu çalıştıramazsın; Claude test edecek
(`python3 oda_yapi.py ~/limo_robot/limo_ws/src/limo_bringup/maps/lab_oda.yaml cikti/kur/oda_yapi`).
Değişikliği kısa özetle bitir.
