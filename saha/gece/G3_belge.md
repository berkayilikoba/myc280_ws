# Gece görevi G3 (uzun): Nav2 + kavrama mimari belgesi ve ekip için kullanım rehberi
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Kaynak kodu DEĞİŞTİRME (yalnız docs/ altına yaz).
Hedef okuyucu: ekip arkadaşları (biri görsel odometri, biri robot kolu üzerinde çalışıyor) ve gelecekteki biz.
Önce OKU (hepsini, dikkatle): src/limo_autonomy/limo_autonomy/*.py (kavrama_dugumu, son_yaklasma,
kavrama_yaklasim, gorev_yoneticisi, gorev_dugumu, sahte_algi), src/limo_autonomy/launch/kavrama.launch.py,
src/limo_bringup/param/nav2_diff.yaml (özellikle hassas_hedef, MPPI, voxel_layer, collision_monitor),
src/limo_bringup/behavior_trees/navigate_to_pose_hassas.xml, src/limo_bringup/launch/limo_nav2.launch.py,
src/limo_bringup/scripts/kavrama_bench.py, saha/nav/kavrama_sim_test.sh, saha/nav/GOREV_N*.md (tasarım
gerekçeleri), docs/oda_3b.md, docs/sim_gercek.md.
YENİ docs/nav_kavrama.md (Türkçe, ~400–600 satır, kod/komut örnekleri gerçek dosyalardan):
1. Mimari özet ve karar: lidar odometrisi yok; odom görsel+teker+IMU (ekip arkadaşı); map->odom lokalizasyon
   katmanından (simde AMCL yer tutucu); kavrama doğruluğu nesneye göreli kapalı çevrim son yaklaşmadan.
   Katman diyagramı (mermaid): görev yöneticisi → kavrama durum makinesi → Nav2 (hassas BT) → son yaklaşma
   denetleyicisi → cmd_vel; algı, kol, lokalizasyon arayüzleri.
2. Arayüz sözleşmeleri tablosu (konu/servis/eylem adı, tip, çerçeve, hız, kim yayınlar/dinler): özellikle
   /kavrama/nesne_algi (algı ekibi bunu yayınlamalı: çerçeve, hız, kayıp toleransı), /kol/kavra ve /kol/birak
   (kol ekibi bunları sağlamalı), /odom ve TF beklentileri (VO ekibi: odom->base_footprint, hız, kayma
   toleransı — son yaklaşmanın odom kaymasına dayanıklılığı, test (c)).
3. Kavrama akışı ayrıntısı: yaklaşma seçimi (aday açılar, koridor, Q→P azalan belirsizlik, d_on, yüzeye dik,
   yaklasim_tani nedenleri), NAV_Q (hassas_hedef 0.15/0.25, yakın kabul 0.35 m/35°), SON_YAKLASMA (Astolfi
   kazançları, son bant hizala/sürün, görüş koruma, ölçümsüz 0.25 m, güvenlik konisi), KURTARMA, HATA türleri.
4. Parametre referansı (her önemli parametre: anlamı, varsayılan, ne zaman değiştirilir).
5. Simde çalıştırma ve ölçüm: tek komutlar (sim + Nav2 + kavrama + sahte algı; kavrama_sim_test.sh;
   kavrama_bench.py; kabul ölçütleri) ve son ölçüm sonuçları (dolap 0.8 cm/1.6°, duvar dibi 0.7 cm/1.7°, masa
   beklenen hata).
6. Gerçek robota geçiş kontrol listesi: algı konusunun gelmesi, kol servisleri, odom kalitesi, hız sınırları,
   güvenlik (collision_monitor, durdurma), ilk denemede yapılacaklar.
7. Bilinen kısıtlar ve sıradaki işler.
Yanıtın sonunda ajan rapor JSON bloğunu unutma.
