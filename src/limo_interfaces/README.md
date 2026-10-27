# limo_interfaces

LIMO + myCobot otonom kavrama ve görev katmanı için özel ROS 2 arayüz paketi (`ament_cmake` + `rosidl`).

## `action/Kavra.action` Eylem Tanımı

Kavrama yaklaşması düğümü (`kavrama_dugumu.py`) tarafından sunulan `/kavra` eylem sunucusu ve görev düğümü (`gorev_dugumu.py`) tarafından çağrılan eylem istemcisi bu arayüzü kullanır.

```text
# Hedef
geometry_msgs/PointStamped nesne      # herhangi bir çerçeve (tipik map)
float32 yon_derece                    # istenen yaklaşma yönü; NaN = serbest
bool algisiz_izinli                   # algı yoksa haritadaki konumla son yaklaşma yapılabilir mi
---
# Sonuç
bool basarili
string neden                          # 'HAZIR' | 'HATA <ayrıntı>'
geometry_msgs/PointStamped nesne_robotta   # base_footprint
float32 tahmini_hata_cm
float32 tahmini_yon_hatasi_derece
float32 sure_s
---
# Geri bildirim
string durum                          # PLAN | NAV_Q | SON_YAKLASMA | KURTARMA
string ayrinti
int32 aday_no
int32 aday_sayisi
float32 kalan_mesafe_m
```

### Alan Açıklamaları

- **Hedef (Goal)**:
  - `nesne`: Kavranacak nesnenin 3B konumu (`geometry_msgs/PointStamped`). Tipik olarak `map` çerçevesindedir (`z` nesne yüksekliğini belirtir); farklı bir çerçeve verilirse TF ile `map` çerçevesine dönüştürülür.
  - `yon_derece`: İstenen yaklaşma açısı (derece). `.nan` (`NaN`) verildiğinde yaklaşma yönü serbest bırakılır ve en yüksek puanlı engelsiz aday seçilir.
  - `algisiz_izinli`: Canlı `/kavrama/nesne_algi` ölçümü yoksa haritadaki nesne konumuyla son yaklaşma yapılmasına izin verilip verilmeyeceği (`true` / `false`).
- **Sonuç (Result)**:
  - `basarili`: Kavrama yaklaşması `HAZIR` durumuna ulaştıysa `true`, hata veya iptal durumunda `false`.
  - `neden`: Başarıda `'HAZIR'`, başarısızlıkta `'HATA <ayrıntı>'` (ör. `'HATA tüm adaylar denendi'`, `'HATA iptal'`).
  - `nesne_robotta`: `base_footprint` çerçevesinde robot koluna bildirilen son nesne koordinatı (`geometry_msgs/PointStamped`).
  - `tahmini_hata_cm`: P kavrama pozuna göre tahmini konum hatası (cm).
  - `tahmini_yon_hatasi_derece`: P kavrama pozuna göre tahmini yönelim hatası (derece).
  - `sure_s`: Yaklaşma süresi (saniye).
- **Geri Bildirim (Feedback)**:
  - Durum değişimlerinde (`PLAN`, `NAV_Q`, `SON_YAKLASMA`, `KURTARMA`) anında ve `SON_YAKLASMA` sırasında 5 Hz frekansla yayınlanır.
  - `kalan_mesafe_m`: `SON_YAKLASMA` aşamasında hedef P noktasına kalan mesafe (`rho`, metre).

## Derleme

```bash
colcon build --packages-select limo_interfaces
source install/setup.bash
```

## Kullanım Örnekleri

Serbest yaklaşma yönü (`yon_derece: .nan`) ve algısız yedek izni ile `/kavra` eylem hedefi gönderme:

```bash
ros2 action send_goal /kavra limo_interfaces/action/Kavra "{nesne: {header: {frame_id: map}, point: {x: 3.73, y: 2.18, z: 0.3}}, yon_derece: .nan, algisiz_izinli: true}" --feedback
```

Belirli bir yaklaşma yönü (ör. `0.0` derece) belirterek hedef gönderme:

```bash
ros2 action send_goal /kavra limo_interfaces/action/Kavra "{nesne: {header: {frame_id: map}, point: {x: 3.73, y: 2.18, z: 0.3}}, yon_derece: 0.0, algisiz_izinli: true}" --feedback
```
