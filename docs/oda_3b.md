# Gerçek oda, 3B sim dünyası (27.10.2026)

`limo_car/worlds/lab_oda_3b.world` + `limo_car/models/lab_oda_3b`: robotun bulunduğu laboratuvarın tavana kadar
duvarları, dokulu zemini ve renkli 3B mobilyası. Önceki `lab_oda.world` (2B haritadan 0.6 m kutular, gri) yerine kamera
tabanlı işler (görsel odometri, nesne tespiti, kavrama) için. Dünya = harita çerçevesi (`lab_oda.yaml`), robot (0, 0, 0).

```bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/limo_bringup/config/fastdds_sim_shm.xml
ros2 launch limo_description gazebo_limo_mycobot.launch.py headless:=true use_rviz:=False gui:=False \
  world:=$PWD/install/limo_car/share/limo_car/worlds/lab_oda_3b.world
# Nav2 ile: limo_sim_nav2.launch.py ... world:=<aynı yol> map:=.../maps/lab_oda.yaml
# kısa yol: saha/oda/sim_ac.sh <dünya>
```

## Üretim (`saha/oda/oda_kur.sh`, tek komut, ~5 dk)

Girdi: 2 cm lidar haritası `lab_oda.yaml` + üç D435 kaydı (`~/limo_veri/vlmaps_sahne/{v2a,v2b,rs720_kapsama}_lidar`,
lidar ICP pozlu, toplam ~4500 kare). Kamera pozları lidar haritasına göre düzeltilir (`kalib_harita.py`: gövdede
dx +9 cm, dy +1 cm, dyaw −1.0°).

| adım | betik | ne yapar |
|---|---|---|
| yapı | `oda_yapi.py` | haritadan duvar/mobilya ayrımı: RANSAC doğrular, baskın eksen (1°/91°), eksen dışı sahte duvarları atma, eksene hizalama + birleştirme, iç içe çift duvarları atma → 13 duvar, 33 mobilya |
| mesh | `oda_tsdf.py` | renkli TSDF (2 cm, ≤2.5 m): bulanık kare süzgeci (Laplacian, en düşük %20), dönüş süzgeci (>0.25 rad/s), **insan maskesi** (YOLOv8-seg, karelerin %29'unda kişi vardı), sahne arası ICP (sahneler arası kayma yalnız 3 mm), sahneler arası tutarlılık temizliği, köşe rengi en iyi 5 kareden |
| doku | `oda_doku.py` | duvar ve zemin dokuları (1 cm): yalnız derinliği yüzeye uyan pikseller, en iyi 3 görünüm, pozlama dengeleme, görülmeyen zemine parke yaması |
| dünya | `oda_dunya.py` | zemin ve duvar görselleri açık UV'li OBJ dörtgenleri (Gazebo kutu yüzüne dokuyu dönük/ölçeksiz koyuyordu; duvarlar iki yüzlü) — **ışıksız** çizilir (`lighting off`, ortam 1.0, mesh Ka = Kd): dokular gerçek ışığı zaten içeriyor, `--isikli` ile eski ayar — + tavan (2.7 m) + dokulu duvar kutuları + mobilya mesh'i (duvara yapışık üçgenler kesilmiş: dokulu duvarla çakışmasın) + sade çarpışma mesh'i (20k üçgen) + tavan lambaları + `gazebo_ros_state` |

Kabul: `sim_kabul.py` (sim açıkken; konu hızları, görüntü biçimi, derinlik/lidar geçerli oranı, IMU, kamera fx, TF
ağacı ve kamera montajı, /clock) — 27.10'da **23/23 GEÇTİ**.

Yardımcılar: `oda_dogrula.py` (simde robotu gerçek kare pozlarına ışınlayıp kamera + lidar karşılaştırması),
`mesh_karsilastir.py` (mesh'i gerçek kamera pozlarından görüntüleyip ölçer), `oda_tavan.py`, `renk_karakter.py`,
`test_oda.py` / `test_renk_bozu.py` (pytest).

### Ölçümle verilen kararlar (`mesh_karsilastir.py`, v2a / rs720, 15 kare)

| mesh | derinlik medyan fark | \|Δd\| < 3 cm | kenar F1 |
|---|---|---|---|
| ilk sürüm | 3.6 / 2.5 cm | %67 / %63 | 0.14 / 0.17 |
| + kare başına ince ICP | 3.5 / 3.4 | %46 / %49 | 0.17 / 0.19 |
| **+ insan maskesi, ince ICP kapalı (seçilen)** | **2.6 / 2.3** | **%68 / %64** | **0.20 / 0.23** |
| zmax 3.0 m | 4.4 / 2.8 | %65 / %59 | 0.17 / 0.21 |

Kare başına renkli ICP mesh'i lidar çerçevesinden kaydırıyor (simde robot lidar pozuyla gezdiği için zararlı) → kapalı.
2.5 m'den uzak derinlik gürültülü → kesim 2.5 m.

## Doğrulama (simde, v2a kaydının 11 karesi, robot gerçek pozlara ışınlanarak)

| | önceki 3B (ilk) | ışıklı (PR #5) | ışıksız (şimdi) |
|---|---|---|---|
| renk ΔE (Lab) | 35.9 | 24.6 | **14.8** |
| HSV histogram korelasyonu | −0.04 | −0.07 | **0.45** |
| kenar örtüşmesi (Canny, 5 px) | 0.33 | 0.35 | 0.35 |
| derinlik medyan fark / < 10 cm | 9.6 cm / %58 | 9.0 cm / %59 | 8.7 cm / %61 |
| derinlik geçerli oranı (gerçek 0.66) | 0.65 | 0.71 | 0.71 |
| lidar ↔ harita: sapma / mutlak medyan / 5 cm içi / 20 cm dışı | — | −0.6 cm / 3.7 cm / %61 / %11 | −0.6 cm / 3.7 cm / %61 / %11 |

Derinlikteki ~9 cm sistematik fark simin değil, gerçek kamera–lidar kalibrasyonunun (bkz. sim_real/RAPOR.md). Lidarın
20 cm dışındaki %11'i haritadan beri yeri değişen kapı/sandalye.

## Renk kamerası modeli (yeni, `limo_kinematics/renk_bozu.py`, `sensor_modeli` bolum `renk`)

Gazebo renk kamerası artık `/camera/color_sim/*` yayınlar; `sensor_modeli_renk` gerçek D435'ten ölçülen bozulmayı
ekleyip `/camera/color/image_raw` (+ camera_info) yayınlar (15 Hz, kare başına ~16 ms):

- gürültü σ² = b, b = (5.4, 5.0, 5.8) BGR (1280×720; 640×360'ta ölçülen ×4),
- yaw hareket bulanıklığı: L = |w|·fx·t_poz, t_poz = 60 ms (gerçekte 0.2 rad/s'de keskinlik ~%45'e düşüyor),
- vinyet ve beyaz denge kapalı (sahneden ayrılamadı / sim dokuları zaten gerçek karelerden).
`renk_bozu:=false` parametresiyle kapatılır; `renk_param_dosyasi` ile YAML verilebilir.

Yan bulgular: (1) eski dünyalarda zemin doku katmanı zemin kutusunun içinde kalıyordu (link z=−0.01) — zemin hiç dokulu
görünmemişti. (2) rclpy'de `Image.data = bytes` 2.7 MB'ta 146 ms sürüyor (bayt bayt doğrulama); `array.array('B', ...)` 2 ms.
Düzeltmeyle simde derinlik de 12.3 → 15 Hz oldu.

## Bilinen kusurlar / sıradaki

- Tavan yüksekliği ölçülmedi: kamera tavanı görmüyor (`oda_tavan.py`: noktalar ~2.5–2.6 m'de bitiyor, tepe yok);
  2.7 m varsayım. Metreyle bir ölçüm yeterli (`TAVAN=... oda_kur.sh`).
- Kapı kenarlarındaki kısa duvar parçaları kayıtlarda az görülmüş (%25–34): görülmeyen kısım duvar medyan rengiyle
  dolu (düz beyaz). Sandalyenin yanındaki kısa duvara (x 2.82) sandalye rengi sızmış.
- Işıklandırma (ölçülen, başlangıç karesi, Lab L): ışıklı sahnede zemin 214 / duvar 95 (gerçek 121 / 158) idi; ışıksız
  çizimle 141 / 159. Zemin dokusunun kendisi gerçekten biraz açık (130).
- Zemindeki sandalye ayağı lekeleri ve görüntü sınırı izleri (yay biçimli ton geçişi).
- Kayıtlar arası yeri değişen nesneler (kapı açık/kapalı) mesh'te kayıttaki gibi kalır.
