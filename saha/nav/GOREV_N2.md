# Görev N2: son_yaklasma.py — nesneye göreli kapalı çevrim son yaklaşma (YENİ, saf Python, ROS'suz)
Dosyalar: src/limo_autonomy/limo_autonomy/son_yaklasma.py (yeni), src/limo_autonomy/test/test_son_yaklasma.py (yeni).

Amaç: Robot kavrama öncesi poz Q'dayken (nesneye ~0.35–0.6 m), nesnenin robot çerçevesindeki ölçümüne göre P'ye
(nesne robotun (d_kavrama, yanal)'ında, robot yönü istenen yaklaşma yönünde) hassas varmak.

API (dataclass + fonksiyonlar):
- `@dataclass SonYaklasmaAyar`: d_kavrama=0.22, yanal=0.0, v_max=0.10, v_min=0.015, w_max=0.6, a_max=0.3 (m/s²),
  k_rho=0.6, k_alpha=1.8, k_beta=-0.5 (Astolfi kutupsal denetleyici kazançları; kararlılık koşulu k_rho>0,
  k_beta<0, k_alpha-k_rho>0 — __post_init__'te doğrula), tol_xy=0.01, tol_yaw=math.radians(2.0),
  son_donus_mesafe=0.03 (bu mesafe içinde yalnız yerinde dönüş), olcum_zaman_asimi=1.0 s, sapma_esik=0.15 m
  (tahmin edilen hedefin ani sıçraması → hata), geri_izin=True (P'yi geçerse kısa geri), max_sure=40 s.
- `hedef_pozu(nesne_xy_odom, yaklasma_yaw_odom, ayar) -> (x, y, yaw)`: P = nesne - (d_kavrama*[cos,sin]) +
  yanal*[-sin, cos] (yanal + sol), yaw = yaklasma_yaw.
- `nesne_robottan_odoma(nesne_robot_xy, robot_odom_pose) -> nesne_xy_odom`.
- `class NesneTahmini`: odom çerçevesinde nesne konumu; `guncelle(olcum_xy_odom, t)` (üstel ortalama, alfa 0.4;
  ilk ölçüm doğrudan; aykırı ölçüm (>0.10 m sıçrama) 3 ardışık gelmedikçe reddedilir), `yas(t)`, `gecerli(t)`.
- `class SonYaklasma`: `__init__(ayar, yaklasma_yaw_odom=None)` (None ise yaklaşma yönü ilk ölçümde robot→nesne
  doğrultusu olarak sabitlenir), `adim(robot_odom_pose, t, olcum_robot_xy=None, guvenlik_mesafe=None) ->
  (v, w, durum: str, bilgi: dict)`. durum ∈ {'YAKLASIYOR','DONUYOR','TAMAM','HATA:<neden>'}.
  Akış: ölçüm geldiyse tahmini güncelle → hedef P (odom) → kutupsal hata (rho, alpha, beta) → Astolfi yasası; rho
  < son_donus_mesafe ise v=0, w ile yaw hatasını kapat; |e_xy|<tol_xy ve |e_yaw|<tol_yaw → TAMAM (v=w=0).
  İvme sınırı (önceki v'den a_max*dt), hız sınırları, hedef arkada kalırsa (alpha > 90°) geri_izin ile ters yön
  formülasyonu. Hata koşulları: ölçüm hiç yok ya da olcum_zaman_asimi'nden uzun süredir yok VE rho > 0.10
  (son 10 cm'de ölçüm kaybı tolere edilir: kamera yakında nesneyi görmeyebilir, odomla tamamla); max_sure aşımı;
  guvenlik_mesafe (öndeki engelin m cinsinden uzaklığı, nesne hariç; çağıran hesaplar) < 0.05 → 'HATA:engel'.
  bilgi: rho, alpha, beta, e_yaw, hedef, nesne_odom, olcum_yasi.
- `nesne_robotta_son(robot_odom_pose, nesne_odom) -> (x, y)`: TAMAM anında nesnenin base_footprint'teki konumu
  (kola bildirilecek).

Testler (test_son_yaklasma.py): basit tekerlek kinematiği ile kapalı çevrim simülasyonu (dt 0.02):
 (a) başlangıç Q = nesneye 0.45 m, yanal 0.08 m kayık, 15° yön hatası → TAMAM, son |e_xy|<1.5 cm, |e_yaw|<2.5°;
 (b) ölçümlere σ=1 cm gürültü + 5 Hz → yine TAMAM, hatalar < 2 cm / 3°;
 (c) odom kayması: robotun "gerçek" hareketi komutun %2 fazlası (ölçüm gerçek göreli konumdan) → kapalı çevrim
     telafi eder, TAMAM ve gerçek göreli hata < 2 cm;
 (d) son 8 cm'de ölçüm kesilir → yine TAMAM; 30 cm'de kesilir → 'HATA' (zaman aşımı);
 (e) guvenlik_mesafe 0.03 → 'HATA:engel' ve v=w=0;
 (f) Astolfi kazanç doğrulaması geçersiz değerde ValueError.
