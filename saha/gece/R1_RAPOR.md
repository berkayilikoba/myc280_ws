# İnceleme R1: `hareketli_kalkan.py` ve Hız Zinciri Bağımsız Kod İncelemesi Raporu

**İncelenen Dosyalar:**
- `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py`
- `src/limo_autonomy/test/test_hareketli_kalkan.py`
- `src/limo_bringup/launch/limo_nav2.launch.py`
- `src/limo_bringup/param/nav2_diff.yaml`
- *(Bağlam doğrulaması için okunanlar: `src/limo_description/urdf/limo_four_diff.xacro`, `docs/sensor_modeli.md`, `src/limo_autonomy/limo_autonomy/kavrama_dugumu.py`, `src/limo_bringup/scripts/dinamik_engel.py`)*

---

## Yönetici Özeti ve Bulgular Tablosu

| Kod | Kategori | Başlık | Dosya:Satır | Önem |
|---|---|---|---|---|
| **B1.1** | (1) Gerçek Hata / (3) Sahte Tetikleme | `laser_link` -> `base_link` TF dönüşümü yok; `lidar_x=0.0` varsayılanı, `lidar_y` (+0.015 m) ve `lidar_yaw` (-9.66°) tamamen eksik | `hareketli_kalkan.py:181-222, 811, 929-937` | **Yüksek** |
| **B1.2** | (1) Gerçek Hata / (2) Güvenlik | `_son_scan_duvar_t` mesaj damgasını değil callback varış anını tutuyor ve `if robot_pose is None: return` öncesinde güncelleniyor; sim reset'te zaman geriye giderse tampon ve izler bozuluyor | `hareketli_kalkan.py:846-847, 887-927, 1035-1054` | **Yüksek** |
| **B1.3** | (1) Gerçek Hata | Gerçek/sim lidar hızı 6 Hz (`dt ≈ 0.167 s`) iken parametrelerin 10 Hz'e göre seçilmesi; ilk adımda `vx=0` yüzünden `0.8 m/s` engelin kapı (`0.35 m`) dışına düşme ve geç tetikleme (`0.67 s`) riski | `hareketli_kalkan.py:444-458, 819-831` | **Yüksek** |
| **B1.4** | (1) Gerçek Hata / (3) Sahte Tetikleme | Robot hızlı dönerken (`\|wz\| > maks_donus_hizi`) hız güncellemesi atlandığı halde `iz.gorulme`'nin artırılması ve `yon_tutarliligi`'nin tek ölçümde `1.0` dönmesi | `hareketli_kalkan.py:151-178, 495-522` | **Yüksek** |
| **B1.5** | (1) Gerçek Hata | `kumele()` içinde 360° tarama başı-sonu sarması (wrap-around) ve yüzey ışın kaybı (%5–10) nedeniyle tek engelin iki kümeye bölünüp `belirsiz_izler` ile sıfırlanması | `hareketli_kalkan.py:225-278, 460-494` | **Yüksek** |
| **B1.6** | (1) Gerçek Hata | `Iz.iz_yasi_s` içinde `t_son == t_ilk` iken yaşın `0.0` yerine `yas * 0.1` dönmesi | `hareketli_kalkan.py:351-356, 699-701` | **Orta** |
| **B1.7** | (1) Gerçek Hata | `poz_aradeger` içinde `min(t_val - t0, t1 - t_val) > maks_dt` kontrolünün geniş odom boşluklarında yanlış enterpolasyon yapması | `hareketli_kalkan.py:127-146` | **Orta** |
| **B1.8** | (1) Gerçek Hata | `_scan_cb` (6 Hz) ve `_cmd_cb` (20 Hz) arasında `robot_pose` / `robot_v_odom` zaman uyumsuzluğu ve çoklu tehditte tek `engel_yon_x` kullanılması | `hareketli_kalkan.py:960-976, 1058-1085` | **Orta** |
| **B2.1** | (2) Güvenlik Açığı | `hareketli_kalkan` çökerse `cmd_vel_kalkan` kesilir; `collision_monitor` yeni mesaj üretmez, düğümde `respawn=True` ve çıkışta sıfır hız yayını yok | `limo_nav2.launch.py:50-51, 238-244`, `hareketli_kalkan.py:1088-1098` | **Yüksek** |
| **B2.2** | (2) Güvenlik Açığı | `/odom` kesilirse veya `>0.2 s` gecikirse `_scan_cb` erken döner ama `_son_scan_duvar_t` güncellendiği için `_son_karar` sonsuza dek kilitli (latched) kalır | `hareketli_kalkan.py:920-927, 1035-1055` | **Yüksek** |
| **B2.3** | (2) Güvenlik Açığı | Tarama zaman aşımında (`dt > 0.5 s`) kalkanın `dur` kararından bile anında `serbest` (fail-open) geçiş yapması ve eski izleri temizlememesi | `hareketli_kalkan.py:1035-1055` | **Orta** |
| **B2.4** | (2) Güvenlik Açığı | `nav2_diff.yaml` `FootprintApproach` içinde hem `max_points: 3` hem `min_points: 6` bulunması ve `kavrama_dugumu`'nun doğrudan `/cmd_vel`'e basması | `nav2_diff.yaml:435-436`, `kavrama_dugumu.py:751` | **Düşük/Orta** |
| **B3.1** | (3) Sahte Tetikleme | Duvar/mobilya kenarlarında kısmi görünüm (occlusion/disocclusion) sırasında `r <= 0.35 m` (70 cm çap) filtresinden geçen kümelerin merkez kayması (sliding centroid) | `hareketli_kalkan.py:817, 948-950` | **Orta/Yüksek** |
| **B3.2** | (3) Sahte Tetikleme | İki yakın durağan kümeden birinin `min_nokta=3` altına düşüp kaybolmasıyla `belirsiz_izler` korumasına girmeden diğer kümeye sıçraması (anlık hız üst sınırı yok) | `hareketli_kalkan.py:252-265, 500-515` | **Orta** |
| **B4.1** | (4) Test Eksikliği | `HareketliKalkan` ROS 2 düğüm sınıfı (`_odom_cb`, `_scan_cb`, `_cmd_cb`) test dosyasında hiç test edilmiyor | `test_hareketli_kalkan.py:1-603` | **Yüksek** |
| **B4.2** | (4) Test Eksikliği | Testlerde 6 Hz lidar, `abs(robot_wz) > maks_donus_hizi`, `belirsiz_izler` sıfırlaması ve `NaN/inf` tarama girdileri hiç test edilmiyor; `Iz` doğrudan oluşturularak yaş ve yön tutarlılığı bypass ediliyor | `test_hareketli_kalkan.py:137-536` | **Orta** |

---

## 1. Gerçek Hatalar (Zaman Birimi/Saat, Çerçeve Dönüşümü, Yaw Wrap, Eşleştirme, İz Silme, Bölme Sıfır, NaN/Inf)

### Bulgu B1.1: `laser_link` -> `base_link` Çerçeve Dönüşümü Yok (`lidar_x=0.0`, `lidar_y` ve `lidar_yaw` Eksik)
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:181-222`, `811`, `929-937` (Karşılaştırma: `src/limo_description/urdf/limo_four_diff.xacro:65-67`, `src/limo_autonomy/limo_autonomy/kavrama_dugumu.py:183-204, 1080-1099`)
- **Önem:** **Yüksek**
- **Kanıt:**
  `hareketli_kalkan.py:218-222`:
  ```python
  acilar = float(angle_min) + indeksler * float(angle_increment)

  x = r_secili * np.cos(acilar) + float(lidar_x)
  y = r_secili * np.sin(acilar)
  return np.column_stack((x, y))
  ```
  `hareketli_kalkan.py:811`:
  ```python
  self.lidar_x = float(p('lidar_x', 0.0).value)
  ```
  `limo_four_diff.xacro:65-67`:
  ```xml
  <!-- Lidar: konum/yön gerçek robotta tarama-harita ICP'si ile ölçüldü (saha/analiz/lidar_kalibrasyon.py, 09.10:
       x 0.137, y 0.015, yaw -9.66° = -0.1686 rad; ortam.sh lidar_tf_yayinla). z ölçülmedi: AgileX değeri 0.08. -->
  <xacro:limo_laser parent_prefix="base_link" frame_prefix="laser">
      <origin xyz="0.137 0.015 0.08" rpy="0 0 -0.1686"/>
  </xacro:limo_laser>
  ```
  ve `limo_nav2.launch.py:238-244` içinde `hareketli_kalkan` düğümüne `lidar_x` parametresi de geçilmemektedir (`parameters=[{'use_sim_time': use_sim_time}]`).
- **Somut Senaryo:**
  1. Hem gerçek LIMO robotunda hem de Gazebo simülasyonunda (`limo_four_diff.xacro`) lidar (`laser_link`) `base_link`'e göre `x = +0.137 m`, `y = +0.015 m` ileride/solda ve `yaw = -0.1686 rad (-9.66°)` dönüktür.
  2. `hareketli_kalkan.py` ise TF ağacını kullanmamakta, `lidar_x`'i `0.0` almakta, `lidar_y` ve `lidar_yaw` dönüşümünü hiç yapmamaktadır.
  3. Robot `wz = 0.7 rad/s` (`< maks_donus_hizi = 0.8 rad/s`) ile dönerken ve/veya `v = 0.4 m/s` ile ilerlerken:
     - `2.0 m` uzaklıktaki **durağan bir mobilyanın** `base_link` konumu `-9.66°` açı hatası yüzünden `2.0 * sin(0.1686) ≈ 0.335 m` yanal sapmayla hesaplanır.
     - Robot düz ilerlerken bile `-9.66°` dönük lidar ekseni yüzünden durağan nesnenin `odom` çerçevesindeki izdüşümü her adımda `v_robot * sin(0.1686) ≈ 0.067 m/s` yanal hızla kayar; robot aynı zamanda dönüyorsa (`wz = 0.7 rad/s`), ihmal edilen `0.137 m` kol uzunluğu ve `-9.66°` açı farkı birleşerek durağan nesne üzerinde `> 0.20 m/s` görünür hız üretir.
- **Önerilen Düzeltme:**
  `kavrama_dugumu.py:1080-1099`'daki gibi `tf2_ros.Buffer` ile `base_link` <- `msg.header.frame_id` (`laser_link`) dönüşümü (`lx, ly, lyaw`) alınmalı veya `tarama_noktalari()` fonksiyonuna `lidar_x=0.137, lidar_y=0.015, lidar_yaw=-0.1686` parametreleri eklenerek:
  ```python
  acilar = float(angle_min) + indeksler * float(angle_increment) + float(lidar_yaw)
  x = r_secili * np.cos(acilar) + float(lidar_x)
  y = r_secili * np.sin(acilar) + float(lidar_y)
  ```
  uygulanmalıdır.

---

### Bulgu B1.2: Zaman Damgası / Saat Karışıklığı (`_son_scan_duvar_t`), Erken Güncelleme ve Zaman Sıfırlanması (Sim Reset)
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:846-847, 887-927, 1035-1054`
- **Önem:** **Yüksek**
- **Kanıt:**
  `hareketli_kalkan.py:915-927`:
  ```python
  def _scan_cb(self, msg: LaserScan) -> None:
      t_msg = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
      now_s = self._simdi_s()
      t_scan = t_msg if t_msg > 0.0 else now_s

      self._son_scan_t = t_scan
      self._son_scan_duvar_t = now_s

      # Taramanın header.stamp anındaki robot pozunu odom geçmişinden doğrusal aradeğerle bul
      robot_pose = poz_aradeger(self._odom_gecmisi, t_scan)
      if robot_pose is None:
          # Tampon aralığı dışındaysa veya 0.2 s'den uzaksa taramayı izleyiciye verme
          return
  ```
  ve `hareketli_kalkan.py:910-913`:
  ```python
  # Odom geçmişi tamponunu güncelle (son 1 saniye)
  self._odom_gecmisi.append((t_odom, float(pos.x), float(pos.y), float(yaw)))
  esik_t = t_odom - 1.0
  self._odom_gecmisi = [kayit for kayit in self._odom_gecmisi if kayit[0] >= esik_t]
  ```
- **Somut Senaryo:**
  1. **`_son_scan_duvar_t` erken ataması ve bayat `header.stamp`:** `self._son_scan_duvar_t = now_s` satırı, `if robot_pose is None: return` kontrolünden **önce** çalışmaktadır ve mesajın `header.stamp` (`t_scan`) yaşını hiç kontrol etmemektedir. Eğer `/scan` mesajları `> 0.2 s` gecikmeli gelirse veya `/odom` kesilirse, `poz_aradeger` `None` döner ve tarama **işlenmez**; fakat `_son_scan_duvar_t = now_s` güncellendiği için `_cmd_cb` taramayı "taze" sanır!
  2. **İsimlendirme/Saat Kaynağı:** `_son_scan_duvar_t` değişken adı "duvar saati" (monotonic/wall clock) izlenimi verse de `self._simdi_s()` üzerinden ROS saatini (`use_sim_time=True` iken sim saatini) kullanmaktadır. Simülasyon duraklatıldığında (`pause`), ROS saati ilerlemediği için `now_s - self._son_scan_duvar_t` sabit kalır.
  3. **Simülasyon Sıfırlaması (Clock Jump Backwards):** Gazebo dünyası sıfırlandığında (`reset_world` / `reset_simulation`) veya bag kaydı başa döndüğünde `t_odom` küçülür (ör. `50.0 s` -> `0.5 s`).
     - `_odom_cb` satır 913'te `kayit[0] >= t_odom - 1.0` (`50.0 >= -0.5`) doğru olduğu için eski `50.0 s` kayıtları `_odom_gecmisi` içinde **kalır!**
     - `poz_aradeger` içinde `t_son = float(gecmis[-1][0])` (`0.5 s`) ve `t_ilk = float(gecmis[0][0])` (`49.0 s`) olduğu için `t_ilk > t_son` olur ve `poz_aradeger` tamamen işlevsiz hale gelir!
     - `IzTakipci.guncelle` satır 559'da `(t_val - iz.t_son) <= self.silme_suresi` (`0.5 - 50.0 = -49.5 <= 0.5`) olduğu için eski izler **asla silinmez** ve `dt = t_val - iz.t_son < 0` yüzünden hızları bir daha güncellenemez!
- **Önerilen Düzeltme:**
  - `_odom_cb` içinde `if self._odom_gecmisi and t_odom < self._odom_gecmisi[-1][0]: self._odom_gecmisi.clear(); self.takipci.sifirla()` kontrolü eklenmelidir.
  - `_scan_cb` içinde `if (now_s - t_scan) > self.tarama_zaman_asimi: return` kontrolü yapılmalı ve `self._son_scan_duvar_t = now_s` yalnızca `robot_pose is not None` doğrulandıktan sonra atanmalıdır.

---

### Bulgu B1.3: Gerçek/Sim Lidar Frekansı (6 Hz) ile `eslesme_kapi`, `min_gorulme` ve `iz_silme_suresi` Parametrelerinin Uyumsuzluğu
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:444-458, 508-515, 819-831` (Kanıt: `src/limo_description/urdf/limo_four_diff.xacro:178-184`, `docs/sensor_modeli.md:35`)
- **Önem:** **Yüksek**
- **Kanıt:**
  `limo_four_diff.xacro:181-184`:
  ```xml
  <sensor name="laser" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>6</update_rate>
  ```
  `hareketli_kalkan.py:444-447`:
  ```python
  for i, iz in enumerate(self._izler):
      dt_ongoru = max(0.0, t_val - iz.t_son)
      pred_x = iz.x + iz.vx * dt_ongoru
      pred_y = iz.y + iz.vy * dt_ongoru
  ```
  `hareketli_kalkan.py:819-827`:
  ```python
  eslesme_kapi = float(p('eslesme_kapi', 0.35).value)
  hiz_alfa = float(p('hiz_alfa', 0.4).value)
  iz_silme_suresi = float(p('iz_silme_suresi', 0.5).value)
  ...
  self.min_gorulme = int(p('min_gorulme', 5).value)
  self.min_iz_yasi_s = float(p('min_iz_yasi_s', 0.4).value)
  ```
- **Somut Senaryo:**
  1. Hem gerçek LIMO'da (YDLIDAR T-mini) hem de Gazebo'da lidar **6 Hz (`dt ≈ 0.167 s`)** çalışmaktadır (`test_hareketli_kalkan.py` ise 10 Hz varsaymaktadır).
  2. **Tepki Süresi Gecikmesi:** 6 Hz'de `min_gorulme = 5` şartının sağlanması için en az 5 tarama = `4 * 0.167 s = 0.67 saniye` gerekir! Eğer tek bir tarama gecikir veya yüzeyde ışın kaybına uğrarsa bu süre `0.83 s` olur. Karşıdan `0.8 m/s` ile gelen insan ve `0.5 m/s` ile giden robot (`v_rel = 1.3 m/s`), kalkan henüz ilk kararını verene kadar `1.3 * 0.67 = 0.87 m` yaklaşır. Gerçek taban modelindeki (`docs/sensor_modeli.md:53`) `0.30 s` saf gecikme + `τ = 0.45 s` eklendiğinde robotun temas öncesi durması tehlikeye girer.
  3. **İlk Adımda (`gorulme = 1`) Kapı (`0.35 m`) Aşımı:** Yeni oluşan bir izin ilk hızı `vx = 0.0, vy = 0.0`'dır. Dolayısıyla 2. taramada `pred_x = iz.x` olur (konum öngörüsü çalışmaz!). Eğer `0.8 m/s` hızla yürüyen insanın 2. taraması Gazebo/DDS jitter'ı veya tek tarama kaybı nedeniyle `0.44 s` sonra gelirse (`< 0.5 s iz_silme_suresi`), insan `0.8 * 0.44 = 0.352 m > 0.35 m (eslesme_kapi)` yer değiştirmiş olur! Kapı dışında kaldığı için eski izle eşleşemez, yeni bir `gorulme = 1` izi açılır ve iz hiçbir zaman `gorulme = 5`'e ulaşamaz!
- **Önerilen Düzeltme:**
  - `IzTakipci.guncelle` içinde `iz.gorulme == 1` (henüz hız kestirimi yokken) eşleşme kapısı `kapi_etkin = max(self.kapi, 1.0 * dt_ongoru)` olarak ölçeklenmeli,
  - 6 Hz lidar için `min_gorulme` varsayılanı `4`'e (`3 * 0.167 s = 0.50 s`) düşürülmeli ve `iz_silme_suresi` `0.6 s` (~3.5 tarama periyodu) yapılmalıdır.

---

### Bulgu B1.4: Dönüş Sırasında (`|wz| > maks_donus_hizi`) `iz.gorulme` Artışı ve `yon_tutarliligi` Tek Ölçüm Hatası
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:151-178, 495-522`
- **Önem:** **Yüksek**
- **Kanıt:**
  `hareketli_kalkan.py:495-522`:
  ```python
  elif hiz_guncelle_atla:
      # Dönüş sırasında güvenilmezlik (|wz| > maks_donus_hizi):
      # Hız güncellemesini atla (konumu güncelle, hızı koru)
      pass
  else:
      dt = t_val - iz.t_son
      if dt > 1e-5:
          ...
  iz.x = cx
  iz.y = cy
  iz.r = cr
  iz.t_son = t_val
  if i not in belirsiz_izler:
      iz.gorulme += 1
  iz.yas += 1
  ```
  ve `hareketli_kalkan.py:165-178`:
  ```python
  olcumler = ham_hizlar[-son_n:]
  if not olcumler:
      return 0.0
  ...
  n = len(olcumler)
  return math.hypot(ux_sum / n, uy_sum / n)
  ```
- **Somut Senaryo:**
  1. Durağan bir nesne ilk kez görüldü (`gorulme = 1`).
  2. İkinci taramada gürültü veya dönüş başlangıcı nedeniyle tek bir kez `v_an = 0.25 m/s` ölçüldü (`gorulme = 2`, `iz.vx = 0.25`, `ham_hizlar = [(0.25, 0.0)]` -> 1 eleman!).
  3. Hemen ardından robot `|wz| = 0.9 rad/s > maks_donus_hizi (0.8)` ile dönmeye başladı.
  4. Sonraki 3 taramada `hiz_guncelle_atla = True` olduğu için `iz.vx = 0.25` **olduğu gibi korunur**, `ham_hizlar` 1 elemanlı kalır, **fakat satır 521'de `iz.gorulme += 1` çalışmaya devam ederek `gorulme = 5` olur!**
  5. `kalkan_karari` çağrıldığında:
     - `v_iz = 0.25 >= 0.20` (Geçer!)
     - `iz.gorulme = 5 >= 5` (Geçer!)
     - `iz_yasi_s >= 0.4` (Geçer!)
     - `yon_tutarliligi(ham_hizlar)` -> `len(olcumler) == 1` olduğu için `math.hypot(ux/1, uy/1) = 1.0 >= 0.7` (**Geçer!**)
     Sonuç: Sadece **tek bir hız ölçümü** yapılmış durağan nesne, robot dönerken **sahte `'dur'` kararı tetikler!**
  6. Tersine, gerçekten hareketli bir nesne ilk kez robot dönerken (`|wz| > 0.8`) görülürse `iz.vx = 0.0` iken `iz.gorulme` 2, 3, 4 diye artar; robot dönüşü bitirdiğinde `iz.gorulme <= 1` (satır 508) koşulu kaçırıldığı için hız `0.0`'dan EMA (`alfa=0.4`) ile yavaşça yükselir (soğuk başlangıç koruması bozulur).
- **Önerilen Düzeltme:**
  1. `hiz_guncelle_atla` (`|wz| > maks_donus_hizi`) veya `dt <= 1e-5` durumunda geçerli bir hız ölçümü yapılmadığı için `iz.gorulme` **artırılmamalıdır**.
  2. `yon_tutarliligi()` fonksiyonunda `if len(olcumler) < 3: return 0.0` kontrolü (veya `len(ham_hizlar) >= min_gorulme - 1`) yapılmalıdır.

---

### Bulgu B1.5: 360° Tarama Başı-Sonu Sarması (Wrap-Around) ve Işın Kaybıyla Küme Bölünmesinin `belirsiz_izler` Üzerinden Gerçek Tehdidi Sıfırlaması
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:225-278, 460-494`
- **Önem:** **Yüksek**
- **Kanıt:**
  `hareketli_kalkan.py:252-265`:
  ```python
  for i in range(1, pts.shape[0]):
      dx = pts[i, 0] - pts[i - 1, 0]
      dy = pts[i, 1] - pts[i - 1, 1]
      dist_sq = dx * dx + dy * dy

      if dist_sq <= esik_kare:
          mevcut_noktalar.append(pts[i])
      else:
          if len(mevcut_noktalar) >= min_nokta:
              kumeler.append(np.array(mevcut_noktalar))
          mevcut_noktalar = [pts[i]]
  ```
  `hareketli_kalkan.py:463-465, 489-494`:
  ```python
  for i, k_list in iz_adaylari.items():
      if len(k_list) >= 2:
          belirsiz_izler.add(i)
  ...
  if i in belirsiz_izler:
      # Belirsiz eşleşme: hız kestirimini sıfırla, görülmeyi sıfırla (yeniden kanıtlamalı)
      iz.vx = 0.0
      iz.vy = 0.0
      iz.gorulme = 0
      iz.ham_hizlar.clear()
  ```
- **Somut Senaryo:**
  1. Gerçek YDLIDAR T-mini'de yüzeyde %5–10 ışın kaybı vardır (`docs/sensor_modeli.md:41`). `2.5 - 3.5 m` uzaktaki `r = 0.20 m` insan silindirinin ortasındaki 1-2 ışın `NaN`/`inf` döndüğünde, iki komşu geçerli nokta arasındaki mesafe `> 0.10 m (kume_esik)` olur.
  2. Aynı şekilde 360° lidarda taramanın başlangıç/bitiş sınırındaki (`angle_min` / `angle_max`) bir nesne, dizinin başı ve sonu birleştirilmediği için iki ayrı kümeye bölünür.
  3. Tek bir insan silindiri aralarında `~0.15 m` olan 2 alt kümeye (`K1`, `K2`) bölündüğü anda, her iki alt küme de mevcut izin `0.35 m` kapısı (`kapi`) içine düşer (`len(iz_adaylari[i]) == 2`)!
  4. Bunun sonucunda `i in belirsiz_izler` True olur ve **robota yaklaşmakta olan gerçek insanın hızı (`vx=0, vy=0`), görülme sayısı (`gorulme=0`) ve geçmişi (`ham_hizlar.clear()`) sıfırlanır!**
- **Önerilen Düzeltme:**
  - `kumele()` fonksiyonunda ilk ve son kümenin uç noktaları arasındaki mesafe `<= esik` ise birleştirilmelidir.
  - Kümeleme sonrasında merkezleri birbirine `< 2 * kume_esik` (`0.20 m`) mesafede olan alt kümeler birleştirilmeli veya `kume_esik` menzile bağlı (`max(0.10, 0.05 * menzil)`) seçilmelidir.

---

### Bulgu B1.6: `Iz.iz_yasi_s` Özelliğinde `t_son == t_ilk` İken Yanlışlıkla `yas * 0.1` Dönmesi
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:351-356, 699-701`
- **Önem:** **Orta**
- **Kanıt:**
  ```python
  @property
  def iz_yasi_s(self) -> float:
      """İzin saniye cinsinden yaşı."""
      if self.t_son > self.t_ilk:
          return self.t_son - self.t_ilk
      return max(0.0, float(self.yas) * 0.1)
  ```
- **Somut Senaryo:**
  Yeni oluşturulan bir izde (`t_ilk = t_val, t_son = t_val, yas = 1`) `self.t_son > self.t_ilk` koşulu `False` olduğu için `iz.iz_yasi_s` `0.0` yerine `0.1` döner. Daha önemlisi, aynı zaman damgalı (`t_son == t_ilk`) birden fazla güncelleme gelirse veya zaman damgası `0.0` ise, gerçekte `0.0 s` geçtiği halde `yas = 5` olduğunda `iz_yasi_s = 0.5 s >= 0.4 s` dönerek `min_iz_yasi_s` filtresini atlar.
- **Önerilen Düzeltme:**
  `t_ilk` ve `t_son` varsayılanları `None` yapılmalı; `t_ilk is not None and t_son is not None` iken doğrudan `max(0.0, self.t_son - self.t_ilk)` döndürülmelidir.

---

### Bulgu B1.7: `poz_aradeger` İçinde Geniş Odom Boşluklarında Hatalı Doğrusal Aradeğer
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:127-146`
- **Önem:** **Orta**
- **Kanıt:**
  ```python
  for i in range(n - 1):
      t0 = float(gecmis[i][0])
      t1 = float(gecmis[i + 1][0])
      if t0 <= t_val <= t1:
          if min(t_val - t0, t1 - t_val) > float(maks_dt):
              return None
  ```
- **Somut Senaryo:**
  `t0 = 10.0 s` ile `t1 = 10.8 s` arasında (ör. Wi-Fi gecikmesiyle, `docs/sensor_modeli.md:85`) 0.8 s odom kesintisi olduğunda:
  - `t_val = 10.05 s` ise `min(0.05, 0.75) = 0.05 <= 0.20` olduğu için `None` dönmez, fakat `10.0 s` ile `10.8 s` arasındaki büyük boşlukta doğrusal enterpolasyon yapar.
  - `t_val = 10.25 s` ise (normalde 0.5 s odom aralığının ortası) `min(0.25, 0.25) > 0.20` olduğu için `None` döner.
- **Önerilen Düzeltme:**
  Eğer `(t1 - t0) > 2.0 * maks_dt` ise iki uzak nokta arasında enterpolasyon yapılmamalı; `t_val - t0 <= maks_dt` ise `gecmis[i]`, `t1 - t_val <= maks_dt` ise `gecmis[i+1]` döndürülmeli, aksi halde `None` dönmelidir.

---

### Bulgu B1.8: `_scan_cb` ve `_cmd_cb` Arasında Robot Pozu/Hızı Uyumsuzluğu ve Çoklu Tehditte Tek `engel_yon_x`
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:960-976, 1058-1085`
- **Önem:** **Orta**
- **Kanıt:**
  `hareketli_kalkan.py:960-964`:
  ```python
  karar_sonuc = kalkan_karari(
      izler=self.takipci.izler(),
      robot_v_odom=self._robot_v_odom,
      robot_p_odom=robot_pose[:2],
  ```
  `hareketli_kalkan.py:1058-1068`:
  ```python
  engel_yon_x = 1.0
  if self._son_kritik_iz is not None and self._robot_pose_odom is not None:
      rx, ry, ryaw = self._robot_pose_odom
      dx = self._son_kritik_iz.x - rx
      dy = self._son_kritik_iz.y - ry
      c = math.cos(ryaw)
      s = math.sin(ryaw)
      # base_link x ekseninde engelin konumu
      x_base = c * dx + s * dy
      engel_yon_x = 1.0 if x_base >= 0.0 else -1.0
  ```
- **Somut Senaryo:**
  1. `_scan_cb` içinde `robot_p_odom` olarak `t_scan` anındaki aradeğerli poz kullanılırken, `robot_v_odom` olarak en son odom mesajındaki (`t_now`) hız kullanılmaktadır.
  2. Hem önde (`ttc = 1.4 s`, `'dur'`) hem arkada (`ttc = 1.1 s`, `'dur'`) iki hareketli engel varsa, `kalkan_karari` yalnızca en küçük TTC'li olan arkadaki engeli `kritik_iz` olarak döndürür (`engel_yon_x = -1.0`). `_cmd_cb` ise yalnızca `vx * (-1.0) > 0` (geri hızı) sıfırlar ve **öndeki `ttc = 1.4 s` engele doğru ileri hızı (`vx > 0`) serbest bırakır!**
  3. Satır 1080'de `out.linear.y = msg.linear.y * (self._son_olcek if self._son_karar != 'serbest' else 1.0)` ile `linear.y` yön kontrolü yapılmaksızın ölçeklenmektedir.
- **Önerilen Düzeltme:**
  `kalkan_karari` veya `komutu_olcekle`, ileri (`x_base >= 0`) ve geri (`x_base < 0`) yarı-uzaylardaki en küçük TTC'leri ayrı ayrı değerlendirmeli; böylece hem önde hem arkada tehdit varken her iki yöndeki hız da uygun şekilde kısıtlanmalıdır.

---

## 2. Güvenlik Açıkları (Kalkan Çökerse / Taraması Eskirse `cmd_vel` Ne Olur?)

### Bulgu B2.1: `hareketli_kalkan` Çökerse veya Donarsa Hız Zinciri Kopar (`respawn` Yok, Çıkışta Sıfır Hız Yayını Yok)
- **Dosya:Satır:** `src/limo_bringup/launch/limo_nav2.launch.py:50-51, 238-244`, `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:1088-1098`, `src/limo_bringup/param/nav2_diff.yaml:396-408`
- **Önem:** **Yüksek**
- **Kanıt:**
  `limo_nav2.launch.py:50-51`:
  ```python
  'cmd_vel_in_topic': PythonExpression(
      ["'cmd_vel_kalkan' if '", hareketli_kalkan, "'.lower() in ('true', '1') else 'cmd_vel_smoothed'"]),
  ```
  `limo_nav2.launch.py:238-244`:
  ```python
  Node(
      condition=IfCondition(hareketli_kalkan),
      package='limo_autonomy',
      executable='hareketli_kalkan',
      name='hareketli_kalkan',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}],
      remappings=remappings),
  ```
  `hareketli_kalkan.py:1095-1098`:
  ```python
  finally:
      node.destroy_node()
      if rclpy.ok():
          rclpy.shutdown()
  ```
- **Somut Senaryo (Zincir Kopar mı, Robot Durur mu?):**
  1. `hareketli_kalkan:=true` iken hız zinciri: `velocity_smoother` -> `cmd_vel_smoothed` -> `hareketli_kalkan` -> `cmd_vel_kalkan` -> `collision_monitor` -> `cmd_vel`.
  2. `hareketli_kalkan` çökerse veya kapanırsa:
     - **Zincir tamamen kopar:** `collision_monitor` yalnızca `cmd_vel_kalkan` konusunu dinlemektedir ve `hareketli_kalkan` düğümünde `respawn=True` tanımlı değildir.
     - **`collision_monitor` robotu DURDURMAZ:** ROS 2 Humble `nav2_collision_monitor` düğümü, hız yayınını kendi timer'ıyla değil `cmd_vel_in` (`cmd_vel_kalkan`) mesaj geri çağrısıyla (`process`) tetikler. `cmd_vel_kalkan` kesildiğinde `collision_monitor` `/cmd_vel` konusuna **sıfır hız dahil hiçbir mesaj yayınlamaz** (`stop_pub_timeout: 2.0` yalnızca poligon ihlali kaynaklı `STOP` durumunda geçerlidir).
     - **Robot ne yapar?**
       * Simülasyonda (`limo_four_diff.xacro:266`, `docs/sensor_modeli.md:59`): `taban_modeli` düğümünün watchdog'u (~0.5–1.0 s) devreye girerek robotu durdurur; ancak `hareketli_kalkan` yeniden başlamadığı için robot bir daha hareket edemez.
       * Gerçek robotta (`docs/sensor_modeli.md:64`: *"Watchdog donanımda ayrıca test edilmedi"*): Alt kontrol kartının watchdog süresi boyunca (veya watchdog yoksa süresiz) robot **son aldığı hız komutuyla kontrolsüz gitmeye devam eder!**
- **Önerilen Düzeltme:**
  1. `limo_nav2.launch.py:238-244`'teki `hareketli_kalkan` `Node` tanımına `respawn=True, respawn_delay=1.0` eklenmelidir.
  2. `hareketli_kalkan.py:1095` `finally:` bloğunda düğüm yok edilmeden önce `node.p_cmd.publish(Twist())` ile `cmd_vel_kalkan`'a sıfır hız gönderilmelidir.

---

### Bulgu B2.2: `/odom` Kesildiğinde veya Geciktiğinde (`poz_aradeger -> None`) Kalkanın Son Kararı Sonsuza Dek Kilitli Kalır (Latched State)
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:920-927, 1035-1055`
- **Önem:** **Yüksek**
- **Kanıt:**
  ```python
  self._son_scan_t = t_scan
  self._son_scan_duvar_t = now_s

  # Taramanın header.stamp anındaki robot pozunu odom geçmişinden doğrusal aradeğerle bul
  robot_pose = poz_aradeger(self._odom_gecmisi, t_scan)
  if robot_pose is None:
      # Tampon aralığı dışındaysa veya 0.2 s'den uzaksa taramayı izleyiciye verme
      return
  ```
- **Somut Senaryo:**
  `/scan` gelmeye devam ederken `/odom` kesilirse veya Wi-Fi gecikmesiyle `> 0.20 s` gecikirse:
  - `_scan_cb` içinde `self._son_scan_duvar_t = now_s` her taramada güncellenir, ardından `robot_pose is None` olduğu için `return` edilir.
  - `_cmd_cb` yalnızca `now_s - self._son_scan_duvar_t > self.tarama_zaman_asimi` kontrolü yaptığı için **zaman aşımını asla fark etmez!**
  - Odom kesilmeden önce `_son_karar == 'dur'` ise, tehdit çoktan geçmiş olsa bile robot **sonsuza kadar durmaya devam eder (kilitlenir)**.
  - Odom kesilmeden önce `_son_karar == 'serbest'` ise, kalkan hiçbir uyarı vermeden **tamamen kör halde** tüm komutları geçirir.
- **Önerilen Düzeltme:**
  `self._son_scan_duvar_t = now_s` satırı `if robot_pose is None: return` kontrolünden **sonraya** taşınmalı; böylece odom aradeğeri bulunamadığında da `tarama_zaman_asimi` (0.5 s) devreye girmelidir.

---

### Bulgu B2.3: Tarama Eskidiğinde (`dt > 0.5 s`) `'dur'` Kararından Anında `'serbest'` (Fail-Open) Geçişi
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:1035-1055`
- **Önem:** **Orta**
- **Kanıt:**
  ```python
  if eski_mi:
      if not self._zaman_asimi_uyarildi:
          self.get_logger().warn(
              f"Tarama verisi eski (>{self.tarama_zaman_asimi}s) veya yok, "
              "hareketli kalkan devre dışı; komut doğrudan aktarılıyor."
          )
          self._zaman_asimi_uyarildi = True
      self.p_cmd.publish(msg)
      return
  ```
- **Somut Senaryo:**
  1. Karşıdan `0.8 m/s` insan yaklaşırken kalkan `'dur'` (`olcek = 0.0`) kararı vermiş ve robotu durdurmuştur.
  2. Tam bu sırada lidar taraması `0.55 s` gecikirse (`tarama_zaman_asimi = 0.5 s`), `eski_mi = True` olur ve kalkan `'dur'` kararını yok sayarak `self.p_cmd.publish(msg)` ile **robotu anında ileri fırlatır!**
  3. Arkadaki `collision_monitor` düğümünün `source_timeout` değeri `1.0 s` (`nav2_diff.yaml:405`) olduğu için `0.5 s - 1.0 s` aralığında `collision_monitor` da taramayı henüz zaman aşımına düşürmemiştir.
  4. Ayrıca zaman aşımına girildiğinde `self.takipci.sifirla()` çağrılmadığı için eski izler bellekte kalır.
- **Önerilen Düzeltme:**
  `eski_mi == True` olduğunda `self.takipci.sifirla()` çağrılmalı; eğer mevcut `_son_karar` `'dur'` veya `'yavas'` ise, tarama kesintisinin ilk `source_timeout` (`1.0 s`) süresince son güvenlik ölçeği korunmalı veya güvenli duruş tercih edilmelidir.

---

### Bulgu B2.4: `nav2_diff.yaml` `FootprintApproach` Parametre Çelişkisi (`max_points: 3` vs `min_points: 6`) ve `kavrama_dugumu` Doğrudan `/cmd_vel` Yayını
- **Dosya:Satır:** `src/limo_bringup/param/nav2_diff.yaml:429-439`, `src/limo_autonomy/limo_autonomy/kavrama_dugumu.py:751, 768`
- **Önem:** **Düşük / Orta** *(doğrulanmalı)*
- **Kanıt:**
  `nav2_diff.yaml:429-439`:
  ```yaml
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.1
      max_points: 3
      min_points: 6
      visualize: True
      polygon_pub_topic: "FootprintApproach"
      enabled: True
  ```
- **Somut Senaryo:**
  1. `DurBolge` ve `YavasBolge` içinde yalnızca `max_points: 3` varken, `FootprintApproach` içinde hem `max_points: 3` hem `min_points: 6` tanımlanmıştır (`nav2_ackermann.yaml:412`'de ise yalnızca `min_points: 6` vardır). ROS 2 Humble `nav2_collision_monitor` sürümüne bağlı olarak `max_points` (3) veya `min_points` (6) okunmaktadır *(hangisinin etkin olduğu sistemdeki Humble paket sürümünde doğrulanmalı)*. Eğer `min_points: 6` etkinse, 6 Hz / 481 noktalı lidarda uzaktaki ince bir engelden (ör. masa ayağı veya insan bacağı) 6 nokta düşmeyebileceği için `FootprintApproach` geç tetiklenebilir.
  2. Ayrıca `hareketli_kalkan.py:18` docstring'inde `[nav2 / smoothed / kavrama] -> hareketli_kalkan` yazmasına karşın, `kavrama_dugumu.py:751` varsayılan olarak doğrudan `/cmd_vel` konusuna yayın yapmakta ve hız zincirini bypass etmektedir.
- **Önerilen Düzeltme:**
  `FootprintApproach` içindeki `max_points` / `min_points` ikiliği diğer poligonlarla (`max_points: 3`) tutarlı hale getirilmeli; `hareketli_kalkan.py` docstring'i güncellenmelidir.

---

## 3. Sahte Tetikleme Kaynakları (Durağan Mobilya / Duvar Neden Tetikleyebilir?)

### Bulgu B3.1: Kısmi Görünüm Kayması (Sliding Centroid) — `max_iz_yaricap = 0.35 m` (70 cm) Filtresinden Geçen Duvar ve Mobilya Kenarları
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:815-817, 948-950`
- **Önem:** **Orta / Yüksek**
- **Kanıt:**
  ```python
  # Duvar/mobilya gibi büyük kümelerin merkezi robot hareket ettikçe kayar (kısmi görünüm) ve sahte hız
  # üretir; insan/kutu boyutundan büyük kümeler izlenmez.
  self.max_iz_yaricap = float(p('max_iz_yaricap', 0.35).value)
  ...
  secili = [i for i, r in enumerate(yaricaplar) if r <= self.max_iz_yaricap]
  ```
- **Somut Senaryo:**
  1. Robot bir kapı aralığından geçerken, köşe dönerken veya bir masanın/dolabın arkasında kalan ikinci bir mobilya/duvar parçası robot ilerledikçe (`v = 0.4 - 0.5 m/s`) kademeli olarak görüş alanına girerken (veya `max_menzil = 5.0 m` çemberine girerken):
  2. Görünen yüzeyin uzunluğu ilk taramada `0.15 m` (`r = 0.075 m`), sonraki taramalarda `0.25 m`, `0.35 m`, `0.45 m`, `0.55 m` (`r = 0.275 m <= 0.35 m`) olarak büyür.
  3. Bu 5 tarama boyunca küme yarıçapı `r <= 0.35 m` (çap `<= 70 cm`) olduğu için **filtreden elenmez!**
  4. Küme tek ucundan uzadığı için küme ağırlık merkezi (`cx, cy`) her taramada uzayan kenar yönünde düzenli olarak `~0.20 - 0.30 m/s` hızla kayar. Kayma yönü sabit olduğu için `yon_tutarliligi >= 0.7` şartını da sağlar ve **durağan mobilya/duvar kenarı sahte `'yavas'` veya `'dur'` tetikler!**
- **Önerilen Düzeltme:**
  - `max_iz_yaricap` varsayılanı `0.25 - 0.30 m`'ye çekilmeli,
  - Bir izin yarıçapı ardışık taramalar arasında hızla büyüyorsa (`abs(cr - iz.r) / dt > 0.15 m/s`), merkez kaymasının gerçek hareketten değil kısmi görünümden (occlusion açılması) kaynaklandığı tespit edilerek o adımda hız güncellemesi yapılmamalıdır.

---

### Bulgu B3.2: Küme Nokta Sayısı Dalgalanmasıyla (`min_nokta = 3`) Tekil Sıçrama ve Anlık Hız Üst Sınırı Eksikliği
- **Dosya:Satır:** `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:228, 260-265, 500-515`
- **Önem:** **Orta**
- **Kanıt:**
  `hareketli_kalkan.py:500-504`:
  ```python
  dt = t_val - iz.t_son
  if dt > 1e-5:
      v_an_x = (cx - iz.x) / dt
      v_an_y = (cy - iz.y) / dt
      iz.ham_hizlar.append((v_an_x, v_an_y))
  ```
- **Somut Senaryo:**
  1. Birbirine `0.25 m` mesafede iki masa/sandalye ayağı (`A` ve `B`) varken:
     - İkisi de `>= 3` nokta aldığında `belirsiz_izler` devreye girer ve izleri sıfırlar.
     - Ancak `A` ayağı 3 noktadan 2 noktaya düştüğü anda (`< min_nokta = 3`), `A` kümesi kaybolur ve ortamda yalnızca `B` kümesi kalır!
     - Bu durumda `A` izi için kapı içinde (`0.25 m <= 0.35 m`) **yalnızca 1 aday küme (`B`)** kalır (`belirsiz_izler` tetiklenmez!).
     - `A` izi `B` kümesine sıçrar ve `v_an = 0.25 / 0.10 = 2.5 m/s` gibi fiziksel olmayan dev bir hız kazanır.
  2. Kodda üst hız sınırı (`maks_engel_hizi`) kontrolü olmadığı için bu `2.5 m/s` hız doğrudan EMA'ya (`alfa = 0.4`) girer.
- **Önerilen Düzeltme:**
  `v_an = math.hypot(v_an_x, v_an_y)` değeri fiziksel üst sınırı (ör. `maks_engel_hizi = 1.5 m/s`) aşıyorsa bu eşleşme sıçrama/gürültü kabul edilmeli ve hız güncellenmemelidir.

*(Not: Sahte tetiklemeye yol açan en büyük iki diğer kaynak **Bulgu B1.1**'deki `lidar_yaw = -9.66°` / `lidar_x = 0.137 m` eksikliği ve **Bulgu B1.4**'teki dönüş sırasında `iz.gorulme` artışıdır.)*

---

## 4. Testlerin Kaçırdığı Durumlar (`test_hareketli_kalkan.py`)

### Bulgu B4.1: `HareketliKalkan` ROS 2 Düğüm Sınıfı Hiç Test Edilmiyor
- **Dosya:Satır:** `src/limo_autonomy/test/test_hareketli_kalkan.py:22-33` ve `src/limo_autonomy/limo_autonomy/hareketli_kalkan.py:793-1086`
- **Önem:** **Yüksek**
- **Kanıt:**
  `test_hareketli_kalkan.py:22-33` yalnızca saf fonksiyonları ve `IzTakipci`'yi içe aktarmakta, `HareketliKalkan` sınıfını hiç test etmemektedir.
- **Somut Senaryo:**
  `HareketliKalkan` düğümündeki `_odom_cb`, `_scan_cb`, `_cmd_cb` metotları hiç çağrılmadığı için:
  - `_son_scan_duvar_t`'nin `robot_pose is None` kontrolünden önce güncellenmesi hatası (B1.2 / B2.2),
  - `max_iz_yaricap` filtresi (satır 948),
  - `_cmd_cb` içindeki `engel_yon_x` dönüşümü (satır 1058-1067) ve `linear.y` ölçeklemesi (satır 1080),
  - `tarama_zaman_asimi` davranışı
  testlerde tamamen görünmez kalmıştır.
- **Önerilen Düzeltme:**
  `HareketliKalkan` sınıfının geri çağrılarını (`_odom_cb`, `_scan_cb`, `_cmd_cb`) test eden birim/entegrasyon testleri `test_hareketli_kalkan.py` dosyasına eklenmelidir.

---

### Bulgu B4.2: Gerçek Sensör Koşullarının (6 Hz Lidar, `NaN/Inf`, `robot_wz > maks_donus_hizi`, `belirsiz_izler`) ve Doğrudan `Iz` İnşasının Kaçırdığı Durumlar
- **Dosya:Satır:** `src/limo_autonomy/test/test_hareketli_kalkan.py:36-86, 137-164, 247-295, 418-536`
- **Önem:** **Orta**
- **Kanıt:**
  1. `test_donus_gecikmeli_tarama_aradeger_hiz_kucuk` (`test_hareketli_kalkan.py:453`):
     ```python
     takipci.guncelle(merkezler_odom, [0.15], t_scan, robot_wz=0.0)
     ```
     Robot `wz = 1.0 rad/s` dönerken `takipci.guncelle` fonksiyonuna `robot_wz=0.0` verilmiş; dolayısıyla `abs(robot_wz) > maks_donus_hizi` (`hiz_guncelle_atla`) dalı hiç test edilmemiştir!
  2. `test_karsidan_gelen_hareketli_engel_dur_ve_yavas` (`test_hareketli_kalkan.py:247-256`):
     ```python
     iz_yakin = Iz(
         id=1, x=1.40, y=0.0, r=0.20, vx=-0.60, vy=0.0, gorulme=5, yas=5,
     )
     ```
     `t_ilk` ve `t_son` verilmediği için `Iz.iz_yasi_s` içindeki `yas * 0.1` yedek dalına ve `Iz.__post_init__` içindeki tek elemanlı `ham_hizlar = [(vx, vy)]` dalına güvenilmiştir.
  3. Tüm zaman adımlı testler (`satır 141, 342, 436, 473, 509`) `dt = 0.10 s` (10 Hz) kullanmaktadır; gerçek/sim 6 Hz (`dt ≈ 0.167 s`) lidar ve `0.8 m/s` engel hızıyla hiçbir test yoktur.
  4. `test_tarama_noktalari_kartezyen_ve_ofset` (`satır 36-86`) içinde `float('nan')`, `float('inf')`, `-float('inf')` ve `<= range_min` değerleri içeren hiçbir test girdisi yoktur.
  5. İki izin veya iki kümenin `< 0.35 m` mesafeye girerek `belirsiz_izler` sıfırlamasını tetiklediği durum test edilmemiştir (`test_yakin_duragan_kumeler_gurultulu_serbest_karari` testinde kümeler arası mesafe `0.40 m > 0.35 m`'dir).
- **Önerilen Düzeltme:**
  `test_hareketli_kalkan.py` içine:
  - 6 Hz (`dt = 1/6 s`) ve `0.8 m/s` hızla yaklaşan engel testi,
  - `robot_wz = 1.2 rad/s > maks_donus_hizi` iken durağan ve hareketli izlerin davranış testi,
  - `belirsiz_izler` (`< 0.35 m` iki küme / iki iz) sıfırlama ve tek kümenin ikiye bölünmesi testi,
  - `NaN`/`inf` içeren `tarama_noktalari` testi,
  - Zamanın geriye gitmesi (sim reset) ve odom kesintisi (`poz_aradeger -> None`) testi
  eklenmelidir.
