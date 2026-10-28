# Gece görevi F7: kameradan algıyla uçtan uca kavrama (sahte algı yerine renk_algi)
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude simde koşturacak.
Önce OKU: src/limo_autonomy/launch/kavrama.launch.py, src/limo_autonomy/limo_autonomy/renk_algi.py (parametreler,
konular), sahte_algi.py, src/limo_car/models/elma/model.sdf, saha/gece/gorev_sim_test.sh (betik kalıbı: sim +
Nav2/AMCL + ilk poz + kavrama.launch + eylem/görev), saha/nav/kavrama_sim_test.sh.
1. kavrama.launch.py: `algi` argümanı (sahte|renk|yok, vars. sahte). Mevcut `sahte_algi` argümanı geriye uyumlu
   kalsın: sahte_algi:=true → algi sahte ile aynı. algi:=renk → renk_algi düğümü (use_sim_time, hz 5,
   hata_ayiklama argümanı vars. false). İkisi aynı anda çalışmasın.
2. saha/gece/algi_sim_test.sh (YENİ, gorev_sim_test.sh kalıbında, ilk poz için python3 $WS/saha/gece/ilk_poz.py,
   bekleme yalnız `ros2 lifecycle get --no-daemon` + PID, pgrep YOK): Nav2 hazır olduktan sonra elma modelini
   gazebo'ya doğur: `ros2 run gazebo_ros spawn_entity.py -entity elma -file
   $WS/install/limo_car/share/limo_car/models/elma/model.sdf -x 5.62 -y 0.8 -z 0.04` (zemin üstü, r 0.035);
   kavrama.launch use_sim_time:=true algi:=renk hata_ayiklama:=true; 10 s içinde /kavrama/nesne_algi_durum
   'GORUYOR' gelmezse bunu raporla ama devam et (robot başlangıçta elmayı görmeyebilir — algısız yedekle Q'ya
   gider, son yaklaşmada görür). Sonra `ros2 action send_goal /kavra limo_interfaces/action/Kavra
   "{nesne: {header: {frame_id: map}, point: {x: 5.62, y: 0.8, z: 0.035}}, yon_derece: .nan, algisiz_izinli: true}"
   --feedback`. Sonunda: eylem sonucu + Gazebo gerçeğine göre nesnenin robot çerçevesindeki konumu (python ile
   /odom_gercek'ten; d_kavrama 0.22 hedefine fark cm, yön hatası) + kaç algı ölçümü geldi (son yaklaşma
   sırasında /kavrama/nesne_algi mesaj sayısı) ve algının Gazebo gerçeğine göre hatası (ortalama, cm).
   Temizlik: trap ile kendi süreç grupları + gzserver.
3. renk_algi.py'de yalnız sim testinde ortaya çıkabilecek açık hata görürsen düzelt (ör. konu adı uyuşmazlığı);
   davranışı değiştirme.
