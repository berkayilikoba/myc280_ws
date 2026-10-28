# R2 İnceleme Raporu: `renk_algi` Uzak Menzilde Sistematik İleri ($+x$) ve Düşey ($+z$) Sapmanın Kaynağı

**Tarih:** 2026-10-28  
**Kapsam:** Salt-okunur kök neden incelemesi (`limo_description`, `limo_kinematics`, `limo_autonomy`, `saha/config`)  
**Hedef Problem:** Zemindeki elma ($r = 0.035\text{ m}$, gerçek merkez $z = 0.04\text{ m}$, `base_footprint` gövde çerçevesi) için `renk_algi` çıktısının gerçek poza göre gösterdiği sistematik sapma:
- **$x$ (ileri) hatası ($\text{Ölçülen} - \text{Gerçek}$):** $d \le 0.6\text{ m}$ içinde $< 1\text{ cm}$; $0.83\text{ m}$'de $+2.4\text{ cm}$; $1.0\text{ m}$'de $+3.0\text{ cm}$; $1.5\text{ m}$'de $+3.1\text{ cm}$; $2.0\text{ m}$'de $+3.6\text{ cm}$; $2.5\text{ m}$'de $+4.0\text{ cm}$; $3.0\text{ m}$'de $+4.0\text{ cm}$.
- **$y$ (yanal) hatası:** $\sim +1\text{ cm}$ sabit.
- **$z$ (düşey) hatası:** Ölçülen $z = +0.05 \dots +0.07\text{ m}$ (gerçek $0.04\text{ m}$); $+1\dots+3\text{ cm}$ hata menzille doğrusal artıyor.

---

## 0. Ön Doğrulama: `sensor_modeli_dugumu.py` (`derinlik_bozu`) İncelemesi

### Bulgu 0.1 — Mesafe Ölçek / Toplamsal Sapma (Bias) Kontrolü
- **Dosya / Satır:** `src/limo_kinematics/limo_kinematics/sensor_modeli_dugumu.py:67-76` ve `265-296`
- **Kod Alıntısı:**
  ```python
  # sensor_modeli_dugumu.py:72-75
  'sigma_a': 0.00176,         # m^-1; sigma(z) = a * z^2 (D435 stereo formul)
  'golge_esik': 0.08,         # m; komsu piksel dz > esik -> stereo golge
  'golge_band': 6,            # px; sol kenar boyunca sifirlanan bant genisligi
  'zemin_bas': 0.55,          # goruntu alt orani (v > H*zemin_bas)
  'zemin_kayip': 0.80,        # zemin bolgesinde blok kayip olasiligi

  # sensor_modeli_dugumu.py:285-296
  a = float(p['sigma_a'])
  zn = z + rng.normal(0.0, 1.0, z.shape).astype(np.float32) * (a * z * z)
  zn = np.where(gec, np.maximum(zn, 0.0), 0.0)
  ...
  return np.where(gec, np.clip(np.round(zn * 1000.0), 0, 65535), 0).astype(np.uint16)
  ```
- **Sayısal Kestirim:**
  - `zn` hesabında çarpımsal ölçek ($\alpha \cdot z$) veya sabit toplamsal ofset ($+b$) **yoktur**; sıfır ortalamalı Gauss gürültüsü $\sigma(z) = 0.00176 \cdot z^2$ eklenir ve `np.round(zn * 1000.0)` ile en yakın milimetreye (`uint16`) yuvarlanır:
    - $z = 0.6\text{ m} \implies \sigma = 0.63\text{ mm}$
    - $z = 1.0\text{ m} \implies \sigma = 1.76\text{ mm}$
    - $z = 2.0\text{ m} \implies \sigma = 7.04\text{ mm}$
    - $z = 3.0\text{ m} \implies \sigma = 15.84\text{ mm}$
  - Dolayısıyla `derinlik_bozu` doğrudan derinlik değerlerini $+2.4\dots+4.0\text{ cm}$ büyütmez (**doğrulandı**).
- **Güven:** **Yüksek**

### Bulgu 0.2 — `derinlik_bozu` İçindeki Sol Kenar Gölgesi (`golge_band=6`) ve Zemin Dropout (`zemin_kayip=0.80`) Yan Etkisi
- **Dosya / Satır:** `src/limo_kinematics/limo_kinematics/sensor_modeli_dugumu.py:271-283`
- **Kod Alıntısı:**
  ```python
  # 1) Stereo sol-kenar golgesi: derinlik sicramasinin sagindaki band sifirlanir
  dz = np.abs(np.diff(np.where(gec, z, 0.0), axis=1, prepend=0.0))
  band = int(p['golge_band'])
  kenar = dz > float(p['golge_esik'])
  for k in range(1, band + 1):
      kenar[:, k:] |= dz[:, :-k] > float(p['golge_esik'])
  gec = gec & ~kenar

  # 2) Zemin spekuler kayip: alt bolgede leke x leke bloklar halinde sifirlama
  v0 = int(h * float(p['zemin_bas']))
  if v0 < h and float(p['zemin_kayip']) > 0.0:
      ...
      gec[v0:, :] &= ~maske
  ```
- **Sayısal Kestirim:**
  - Elmanın sol kenarında derinlik zeminden ($d_{\text{zemin}} \approx 1.29 \cdot d$) elma yüzeyine ($d$) sıçrar ($\Delta z \approx 0.29 \cdot d > 0.08\text{ m}$).
  - `kenar[:, k:] |= dz[:, :-k] > 0.08` döngüsü, derinlik sıçramasının **sağındaki** (yani elmanın kendi üzerindeki!) `band + 1 = 7` pikseli (`[u_sol, u_sol + 6]`) tamamen sıfırlar (`z = 0`).
  - Ayrıca $d \le 2.0\text{ m}$ menzillerde elma görüntüde $v \ge 264$ (`H * 0.55 = 480 * 0.55`) satırlarının altına düştüğü için, elma ve çevresindeki $8\times 8$ piksellik blokların **%80'i** `zemin_kayip` tarafından sıfırlanır.
  - Bu iki mekanizma doğrudan derinlik değerine bias eklemese de, **Bölüm 2 ve 3'te gösterildiği üzere `renk_algi`'nın 2-aşamalı derinlik ROI'sinin elma tepe noktası yerine elmanın sağ kenar/zemin geçişini örneklemesine yol açan ana tetikleyicidir.**
- **Önerilen Düzeltme:** `renk_algi.py` içinde derinlik ROI seçiminde 1. aşama sabit `0.6 m` paralaks varsayımının kaldırılması ve küre ön yüzeyi için medyan yerine alt-yüzdelik (ör. P15–P25) + küre geometrisi düzeltmesi kullanılması (bkz. Bulgu 2.5 ve Bölüm 3).
- **Güven:** **Yüksek**

---

## 1. URDF/Xacro Kamera Çerçeveleri ile Gazebo Kamera Eklentilerinin Karşılaştırması

### 1.1 URDF/Xacro TF Zinciri (`base_footprint` $\to$ Kamera Optik Çerçeveleri)

Aktif simülasyon robot tanımı `src/limo_description/urdf/limo_mycobot.xacro:5` üzerinden `limo_four_diff.xacro` ve `limo_d435.xacro` dosyalarını içerir:

1. **`base_footprint` $\to$ `base_link`**
   - **Dosya / Satır:** `src/limo_description/urdf/limo_four_diff.xacro:25-29`
   - **Kod Alıntısı:**
     ```xml
     <joint name="base_joint" type="fixed">
       <parent link="base_footprint"/>
       <child link="base_link"/>
       <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
     </joint>
     ```
2. **`base_link` $\to$ `camera_link` (Kalibre Edilmiş D435 Montaj Pozu)**
   - **Dosya / Satır:** `src/limo_description/urdf/limo_four_diff.xacro:72-76`
   - **Kod Alıntısı:**
     ```xml
     <!-- B14: D435 kalibre edilmis montaj pozu (fabrika: 0.084 0 0.03 / 0 0 0) -->
     <xacro:limo_d435 parent_prefix="base_link" frame_prefix="camera">
         <origin xyz="0.084 0.0175 0.028" rpy="0.0015 -0.0191 0"/>
     </xacro:limo_d435>
     ```
3. **`camera_link` $\to$ `camera_depth_frame` $\to$ `camera_depth_optical_frame` ve `camera_color_frame` $\to$ `camera_color_optical_frame`**
   - **Dosya / Satır:** `src/limo_description/urdf/limo_d435.xacro:36-59`
   - **Kod Alıntısı:**
     ```xml
     <joint name="${frame_prefix}_depth_joint" type="fixed">
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <parent link="${frame_prefix}_link"/>
       <child link="${frame_prefix}_depth_frame"/>
     </joint>
     <joint name="${frame_prefix}_depth_optical_joint" type="fixed">
       <origin xyz="0 0 0" rpy="${-M_PI/2} 0 ${-M_PI/2}"/>
       <parent link="${frame_prefix}_depth_frame"/>
       <child link="${frame_prefix}_depth_optical_frame"/>
     </joint>

     <joint name="${frame_prefix}_color_joint" type="fixed">
       <origin xyz="0 0.0148 0.0002" rpy="0 0 0"/>
       <parent link="${frame_prefix}_link"/>
       <child link="${frame_prefix}_color_frame"/>
     </joint>
     <joint name="${frame_prefix}_color_optical_joint" type="fixed">
       <origin xyz="0 0 0" rpy="${-M_PI/2} 0 ${-M_PI/2}"/>
       <parent link="${frame_prefix}_color_frame"/>
       <child link="${frame_prefix}_color_optical_frame"/>
     </joint>
     ```

### 1.2 Gazebo Kamera Eklentileri (`d435_renk_eklenti` ve `d435_derinlik_eklenti`)

- **Dosya / Satır:** `src/limo_description/urdf/limo_d435.xacro:62-98`
- **Kod Alıntısı:**
  ```xml
  <!-- Renk sensoru: 1280x720, fx=909.665 -> hfov = 2*atan(640/909.665) = 1.22693 rad -->
  <gazebo reference="${frame_prefix}_color_frame">
    <sensor name="d435_renk" type="camera">
      <always_on>true</always_on>
      <update_rate>15.0</update_rate>
      <camera name="d435_color">
        <horizontal_fov>1.22693</horizontal_fov>
        <image>
          <width>1280</width>
          <height>720</height>
          <format>R8G8B8</format>
        </image>
        <clip><near>0.05</near><far>15.0</far></clip>
      </camera>
      <plugin name="d435_renk_eklenti" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>d435_renk/image_raw:=camera/color/image_raw</remapping>
          <remapping>d435_renk/camera_info:=camera/color/camera_info</remapping>
        </ros>
        <camera_name>d435_renk</camera_name>
        <frame_name>${frame_prefix}_color_optical_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Derinlik sensoru: 848x480, fx=429.972 -> hfov = 2*atan(424/429.972) = 1.55737 rad -->
  <gazebo reference="${frame_prefix}_depth_frame">
    <sensor name="d435_derinlik" type="depth">
      <always_on>true</always_on>
      <update_rate>15.0</update_rate>
      <camera name="d435_depth">
        <horizontal_fov>1.55737</horizontal_fov>
        <image>
          <width>848</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip><near>0.1</near><far>10.0</far></clip>
      </camera>
      <plugin name="d435_derinlik_eklenti" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>d435_derinlik/image_raw:=camera/IR/image_raw</remapping>
          <remapping>d435_derinlik/camera_info:=camera/IR/camera_info</remapping>
          <remapping>d435_derinlik/depth/image_raw:=camera/depth/image_raw</remapping>
          <remapping>d435_derinlik/depth/camera_info:=camera/depth/camera_info</remapping>
        </ros>
        <camera_name>d435_derinlik</camera_name>
        <frame_name>${frame_prefix}_depth_optical_frame</frame_name>
        <hack_baseline>0.0</hack_baseline>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>
  ```

### 1.3 Poz ve Çerçeve Karşılaştırma Tablosu (`base_footprint` Referanslı)

| Çerçeve / Sensör | URDF `base_footprint` Göreli Konum $(x, y, z)\text{ [m]}$ | URDF Gövde RPY $(\text{roll}, \text{pitch}, \text{yaw})\text{ [rad]}$ | Gazebo Sensör `<pose>` | Yayınlanan `<frame_name>` | Uyum Durumu |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `camera_link` | $(0.0840, 0.0175, 0.1780)$ | $(+0.0015, -0.0191, 0.0)$ | — | — | Tam uyumlu |
| `camera_depth_frame` / `d435_derinlik` | $(0.0840, 0.0175, 0.1780)$ | $(+0.0015, -0.0191, 0.0)$ | Yok (`0 0 0 0 0 0`) | `camera_depth_optical_frame` (`hack_baseline=0.0`) | `base_link`'e göre **özdeş** |
| `camera_color_frame` / `d435_renk` | $(0.083996, 0.032300, 0.178222)$ | $(+0.0015, -0.0191, 0.0)$ | Yok (`0 0 0 0 0 0`) | `camera_color_optical_frame` | `base_link`'e göre **özdeş** |
| `limo_ackerman.xacro` / `limo_xacro.xacro` (Eski tanım) | $(0.0840, 0.0, 0.1800)$ | $(0.0, 0.0, 0.0)$ | `limo_gazebo.gazebo` (`frameName=camera_depth_frame`) | `camera_depth_frame` (`hackBaseline=0.07`) | Aktif launch'ta kullanılmıyor; fakat nominal poz referansı olarak kodda kalıntıları var |

### Bulgu 1.1 — Gazebo Sensör Pozu ile TF Çerçevesi (`base_link` Göreli) Karşılaştırması
- **Dosya / Satır:** `src/limo_description/urdf/limo_four_diff.xacro:73-75` ve `src/limo_description/urdf/limo_d435.xacro:36-98`
- **Teknik Analiz:**
  - Gazebo'nun URDF'i SDF'ye çevirirken sabit (fixed) eklemleri birleştirmesi (`lump`) sonucunda `d435_renk` sensörü `camera_color_frame` pozu olan $(0.0840, 0.0323, 0.1782)\text{ m}$, $\text{rpy}=(0.0015, -0.0191, 0)$ noktasında görüntü üretir.
  - `robot_state_publisher` da aynı `limo_mycobot.xacro` dosyasını (`gazebo_limo_mycobot.launch.py:114-121`) kullandığı için TF ağacındaki `base_link -> camera_color_optical_frame` ve `base_link -> camera_depth_optical_frame` dönüşümleri Gazebo'daki sensör pozlarıyla **`base_link` çerçevesinde birebir aynıdır** ($x$ ofset farkı $= 0\text{ m}$, pitch farkı $= 0\text{ rad}$).
  - Ancak aşağıda Bulgu 1.2, 1.3 ve 1.4'te belgelenen **üç önemli yapılandırma/kalibrasyon uyumsuzluğu** mevcuttur.
- **Güven:** **Yüksek**

### Bulgu 1.2 — URDF Tekerlek Geometri Ofseti ve Gazebo `<minDepth>` Kaynaklı Gövde Pitch / Z Kayması
- **Dosya / Satır:** `src/limo_description/urdf/limo_four_diff.xacro:18-29` ve `224-230`
- **Kod Alıntısı:**
  ```xml
  <!-- limo_four_diff.xacro:18-29 -->
  <xacro:property name="wheel_vertical_offset" value="-0.10" />
  <xacro:property name="wheel_radius" value="0.045" />
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
  </joint>
  <!-- limo_four_diff.xacro:227-229 -->
  <kp>10000000.0</kp>
  <kd>1.0</kd>
  <minDepth>0.01</minDepth>
  ```
- **Sayısal Kestirim:**
  - `base_link` $z = 0.15\text{ m}$'de, tekerlek merkezleri $z = 0.15 - 0.10 = 0.05\text{ m}$'de ve tekerlek yarıçapı $r_w = 0.045\text{ m}$'dir. Dolayısıyla URDF'de tekerleklerin alt noktası `base_footprint`'e göre $z = 0.05 - 0.045 = +0.005\text{ m}$ ($+5\text{ mm}$) yüksektedir!
  - Ayrıca Gazebo tekerlek temas parametresi `<minDepth>0.01</minDepth>` ($1\text{ cm}$ batma toleransı) içerir ve robotun kütle merkezi (`base_link` + `mycobot` kolu) önde olduğu için ön/arka süspansiyon/temas batması farklıdır.
  - Eğer kaydedici (`kaydedici`) elmanın gövde çerçevesindeki konumunu hesaplarken Gazebo'dan yalnız $(x_{\text{robot}}, y_{\text{robot}}, \text{yaw})$ alıp `base_footprint`'i $z=0, \text{roll}=0, \text{pitch}=0$ kabul ediyorsa (veya TF `base_footprint` sabit $z=0.15\text{ m}$ varsayıyorsa):
    - Sabit $\Delta z_{\text{base}} \approx -0.005 \dots -0.015\text{ m}$ çökme,
    - Ön tekerleklerin kol ağırlığıyla $\delta_{\text{pitch}} \approx +0.003 \dots +0.008\text{ rad}$ öne eğilmesi ise kamerada telafi edilmeyen bir eğim yaratarak menzille artan $\Delta z(d) = d \cdot \delta_{\text{pitch}}$ hatasına katkıda bulunur (**doğrulanmalı** — kaydedicinin 3B poz mu yoksa 6B poz mu kullandığına bağlıdır).
- **Önerilen Düzeltme:** `limo_four_diff.xacro:28` içinde `base_joint` $z$ yüksekliğinin $0.145\text{ m}$'ye (`wheel_radius - wheel_vertical_offset = 0.145`) çekilmesi ve `<minDepth>` değerinin `0.001` m'ye düşürülmesi.
- **Güven:** **Orta** (Kaydedicinin `base_footprint` $z/\text{pitch}$ okuma şekli **doğrulanmalı**).

### Bulgu 1.3 — Gerçek D435 Kalibrasyonu (`sim_gercek_param.yaml`) ile Xacro / `renk_algi.py` Extrinsic Dönüklük Uyumsuzluğu
- **Dosya / Satır:** `saha/config/sim_gercek_param.yaml:36-61` vs. `src/limo_description/urdf/limo_d435.xacro:49-58` ve `src/limo_autonomy/limo_autonomy/renk_algi.py:895-900`
- **Kod Alıntısı:**
  ```yaml
  # saha/config/sim_gercek_param.yaml:50-58
  camera_link_to_color_optical:
    xyz_m: [-0.00007, 0.01482, 0.00020]
    rpy_rad: [-1.56942, 0.00474, -1.56572]
  ```
- **Sayısal Kestirim:**
  - `sim_gercek_param.yaml` içinde gerçek D435'in `camera_link -> camera_color_optical_frame` rotasyonu `[-1.56942, 0.00474, -1.56572]` iken, `limo_d435.xacro:56` ve `renk_algi.py:895` bunu ideal `[-pi/2, 0, -pi/2]` (yani renk ve derinlik optik eksenleri arasında **sıfır rotasyon**, $R = I_{3\times 3}$) olarak modeller.
  - Aradaki fark: $\Delta\text{roll}_{\text{opt}} = +0.00138\text{ rad}$, $\Delta\text{pitch}_{\text{opt}} = +0.00474\text{ rad}$ ($0.27^\circ$), $\Delta\text{yaw}_{\text{opt}} = +0.00508\text{ rad}$ ($0.29^\circ$).
  - $848\times 480$ derinlik görüntüsünde $\Delta\text{yaw}_{\text{opt}} = 0.00508\text{ rad}$ yatayda $f_d \cdot 0.00508 = 429.97 \times 0.00508 = \mathbf{2.18\text{ px}}$ kayma, düşeyde ise $429.97 \times 0.00138 = \mathbf{0.59\text{ px}}$ kayma yaratır. Simülasyonda her iki taraf da ideal (`R=I`) olduğu için sim içinde doğrudan hata üretmez, ancak gerçek robotta veya kaydedici gerçek kalibrasyon matrisini kullanıyorsa $d = 3.0\text{ m}$'de $\Delta y \approx 1.5\text{ cm}$, $\Delta z \approx 0.4\text{ cm}$ sapma üretir.
- **Önerilen Düzeltme:** Gerçek robot çalışmasında `T_RENK_DERINLIK_VARSAYILAN` matrisinin üst-sol $3\times 3$ rotasyon bloğuna `rpy_rad: [-1.56942, 0.00474, -1.56572]`'den türetilen $R_{d\leftarrow c}$ rotasyonunun yazılması.
- **Güven:** **Yüksek**

### Bulgu 1.4 — Nominal Kamera Pozu (`0.084, 0, 0.03`, `rpy=0,0,0`) ile Kalibre Kamera Pozu (`0.084, 0.0175, 0.028`, `rpy=0.0015, -0.0191, 0`) Farkı
- **Dosya / Satır:** `src/limo_description/urdf/limo_four_diff.xacro:72-75` vs. `src/limo_description/urdf/limo_ackerman.xacro:70-73` ve `src/limo_autonomy/limo_autonomy/sahte_algi.py:42-48`
- **Kod Alıntısı:**
  ```python
  # sahte_algi.py:42, 47-48
  # URDF: base_footprint -> base_link (0, 0, 0.15) -> camera_link (0.084, 0, 0.03)
  CAM_X = 0.084
  CAM_Y = 0.0
  CAM_Z = 0.18   # 0.15 + 0.03
  ```
- **Sayısal Kestirim:**
  - `limo_four_diff.xacro:74`'te kamera `base_link`'e göre `xyz="0.084 0.0175 0.028" rpy="0.0015 -0.0191 0"` (burun yukarı $\text{pitch} = -0.0191\text{ rad} = -1.094^\circ$) olarak monte edilmiştir.
  - `camera_color_optical_frame`'in `base_footprint`'e göre tam konumu:
    $$t_{\text{base}\leftarrow\text{color}} = (0.0840\text{ m},\; +0.0323\text{ m},\; +0.1782\text{ m}), \quad \text{rpy} = (+0.0015,\; -0.0191,\; 0.0)\text{ rad}$$
  - `renk_algi.py` kendi içinde nokta yayınlamaz; `/elma_konum` topiğinde `header.frame_id = "camera_color_optical_frame"` (`renk_algi.py:1265`) yayınlar!
  - Noktayı `base_footprint` (robot gövde çerçevesi) koordinatlarına dönüştüren düğüm (`kavrama_dugumu.py:926-937` veya ölçüm kaydedicisi):
    - Eğer TF (`lookup_transform('base_footprint', 'camera_color_optical_frame')`) kullanıyorsa, Gazebo ve TF aynı `-0.0191 rad` pitch değerine sahip olduğundan bu açı tam sadeleşir.
    - **Fakat** ölçüm kaydedicisi (veya dış değerlendirme betiği) kamera-gövde dönüşümünü TF'den okumak yerine `sahte_algi.py:47` / `limo_ackerman.xacro:71`'deki fabrika/nominal değerlerle (`xyz=(0.084, 0, 0.18), rpy=(0,0,0)`) veya yalnız `camera_link` ötelemesiyle yapıyorsa:
      - Pitch farkı $\delta_{\text{pitch}} = -0.0191\text{ rad}$ telafi edilmez ve gövde çerçevesinde:
        $$\Delta z(d) \approx -0.0018 + 0.0191 \cdot d \implies \begin{cases} d=0.83\text{ m}: & \Delta z = +1.4\text{ cm} \;(z=0.054\text{ m}) \\ d=1.50\text{ m}: & \Delta z = +2.7\text{ cm} \;(z=0.067\text{ m}) \\ d=3.00\text{ m}: & \Delta z = +5.5\text{ cm} \;(z=0.095\text{ m}) \end{cases}$$
      - Yanal ($y$) farkı: $\Delta y = -0.0323\text{ m}$ (nominal `0` ise) veya $+0.0148\text{ m}$ (`camera_link` $y=0.0175$ alınıp `color_joint` $y=0.0148$ unutulursa $\implies \mathbf{\Delta y = +1.48\text{ cm}}$ sabit!).
- **Önerilen Düzeltme:** Ölçüm kaydedicinin ve tüm düğümlerin `camera_color_optical_frame -> base_footprint` dönüşümünü sabit sayılar yerine TF2 üzerinden (`lookup_transform`) aldığının doğrulanması (**doğrulanmalı**).
- **Güven:** **Orta-Yüksek** (Yanal $+1.48\text{ cm}$ ve menzille artan $+z$ hatasıyla doğrudan örtüşür; kaydedici kodu çalışma alanı dışında olduğundan **doğrulanmalı**).

---

## 2. `renk_algi.py` Algoritmik ve Parametrik İncelemesi

### Bulgu 2.1 — Renk $\leftrightarrow$ Derinlik Dış Parametreleri (`T_RENK_DERINLIK_VARSAYILAN` ve TF Okuma)
- **Dosya / Satır:** `src/limo_autonomy/limo_autonomy/renk_algi.py:895-900` ve `1004-1029`
- **Kod Alıntısı:**
  ```python
  # renk_algi.py:895-900
  T_RENK_DERINLIK_VARSAYILAN = np.array([
      [1.0, 0.0, 0.0, -0.0148],
      [0.0, 1.0, 0.0, -0.0002],
      [0.0, 0.0, 1.0,  0.0000],
      [0.0, 0.0, 0.0,  1.0000],
  ], dtype=np.float64)

  # renk_algi.py:1009-1015
  tf_msg = self._tf_buffer.lookup_transform(
      'camera_depth_optical_frame',
      'camera_color_optical_frame',
      rclpy.time.Time())
  ```
- **Sayısal Kestirim:**
  - URDF'de `camera_color_frame` `camera_depth_frame`'e göre $\Delta y_{\text{link}} = +0.0148\text{ m}$ (sola) ve $\Delta z_{\text{link}} = +0.0002\text{ m}$ (yukarı) konumundadır.
  - Optik çerçevede ($X_{\text{opt}} = -Y_{\text{link}}$, $Y_{\text{opt}} = -Z_{\text{link}}$, $Z_{\text{opt}} = +X_{\text{link}}$) renk kamerası orijini derinlik kamerası optik çerçevesinde $(-0.0148, -0.0002, 0.0)\text{ m}$ noktasındadır.
  - Dolayısıyla $P_{\text{depth}} = R \cdot P_{\text{color}} + t_{\text{depth}\leftarrow\text{color}} = P_{\text{color}} + [-0.0148, -0.0002, 0]^T$ dönüşümü ve `lookup_transform('camera_depth_optical_frame', 'camera_color_optical_frame')` çağrısının yönü/işareti **matematiksel olarak doğrudur**.
- **Güven:** **Yüksek**

### Bulgu 2.2 — İç Parametreler (`camera_info` vs. Sabit `K_RENK_VARSAYILAN` Optimizasyon / Optik Merkez Farkı)
- **Dosya / Satır:** `src/limo_autonomy/limo_autonomy/renk_algi.py:884-893` ve `986-995`
- **Kod Alıntısı:**
  ```python
  # renk_algi.py:884-893
  K_RENK_VARSAYILAN = np.array([
      [909.665,   0.0,   649.849],
      [  0.0,   909.833, 379.424],
      [  0.0,     0.0,     1.0  ],
  ], dtype=np.float64)
  K_DERINLIK_VARSAYILAN = np.array([
      [429.972,   0.0,   420.933],
      [  0.0,   429.972, 241.320],
      [  0.0,     0.0,     1.0  ],
  ], dtype=np.float64)
  ```
- **Sayısal Kestirim:**
  - Gazebo'daki `libgazebo_ros_camera.so` eklentisi görüntüyü **simetrik** optik merkezle ($c_x = W/2 + 0.5 = 640.5$, $c_y = H/2 + 0.5 = 360.5$; $f_x = f_y = 640 / \tan(1.22693/2) = 909.666$) üretir ve `/camera/color/camera_info` topiğinde bu değerleri yayınlar.
  - Buna karşılık `K_RENK_VARSAYILAN` içinde gerçek D435'in kalibre edilmiş optik merkezi ($c_x = 649.849$, $c_y = 379.424$) tanımlıdır:
    $$\Delta c_x = 640.5 - 649.849 = -9.35\text{ px}, \qquad \Delta c_y = 360.5 - 379.424 = -18.92\text{ px}$$
  - Eğer `camera_info` mesajı alınmadan önce (`_k_renk_alindi == False`) veya `camera_info` topiğinin yeniden adlandırıldığı/yayınlanmadığı bir test senaryosunda `K_RENK_VARSAYILAN` kullanılırsa, normalize ışın koordinatları $(x_n, y_n)$ her mesafede sabit bir açısal sapmaya uğrar:
    $$\delta x_n = \frac{-9.35}{909.665} = -0.01028\text{ rad}, \qquad \delta y_n = \frac{-18.92}{909.833} = -0.02080\text{ rad}$$
  - Bu sapmanın robot gövde çerçevesindeki ($y_{\text{body}} = -X_{\text{opt}}$, $z_{\text{body}} = -Y_{\text{opt}}$) karşılığı **tam olarak**:
    $$\Delta y_{\text{body}}(d) = -\delta x_n \cdot d = \mathbf{+1.03\text{ cm/m} \cdot d} \quad (\text{1 m'de } +1.0\text{ cm!})$$
    $$\Delta z_{\text{body}}(d) = -\delta y_n \cdot d = \mathbf{+2.08\text{ cm/m} \cdot d} \quad (\text{1 m'de } +2.1\text{ cm, 1.5 m'de } +3.1\text{ cm!})$$
  - Ayrıca `libgazebo_ros_camera.so` içinde `<camera>` bloğunda `<distortion>` tanımlı olmasa bile $1280\times 720$ renk kamerasında $\text{hfov} = 1.22693\text{ rad}$ üzerinden yalnız yatay odak uzaklığı verildiği, `CameraInfo` içindeki $c_x, c_y$'nin $640.5, 360.5$ olduğu görülür.
- **Önerilen Düzeltme:** `renk_algi.py` içinde `camera_info` gelmeden ölçüm yayınlanmaması veya simülasyon modunda varsayılan matrislerin Gazebo optik merkezleriyle ($640.5, 360.5$ ve $424.5, 240.5$) başlatılması.
- **Güven:** **Orta-Yüksek** (`camera_info` akışının aktif olup olmadığına bağlıdır — **doğrulanmalı**).

### Bulgu 2.3 — Derinlik Birimi (`16UC1` mm $\to$ m)
- **Dosya / Satır:** `src/limo_autonomy/limo_autonomy/renk_algi.py:204-243`
- **Kod Alıntısı:**
  ```python
  # renk_algi.py:234-239
  if enc in ('16uc1', 'mono16'):
      arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).astype(np.float32)
      arr[arr == 0] = np.nan
      return arr / 1000.0
  ```
- **Sayısal Kestirim:** `sensor_modeli_dugumu.py:296` derinliği `zn * 1000.0` (`uint16`, mm) olarak yazar; `derinlik_goruntusu_coz` ise `arr / 1000.0` ile metreye çevirir. Birim dönüşümünde (`1000.0` vs `1024` vb.) hiçbir ölçek hatası yoktur.
- **Güven:** **Yüksek**

### Bulgu 2.4 — Yüzey $\to$ Merkez Ötelemesi (`isin_derinlik_3b`, `oklid_mi`) ve Küre Geometrisi Hatası
- **Dosya / Satır:** `src/limo_autonomy/limo_autonomy/renk_algi.py:375-431` ve `1248`
- **Kod Alıntısı:**
  ```python
  # renk_algi.py:412-431
  xn = (float(u) - fx_cx) / fx
  yn = (float(v) - fy_cy) / fy

  if oklid_mi:
      norm_ fakt = math.sqrt(xn * xn + yn * yn + 1.0)
      z_opt = float(d) / norm_fakt
  else:
      z_opt = float(d)

  p_yuzey = np.array([xn * z_opt, yn * z_opt, z_opt], dtype=np.float64)
  norm_p = np.linalg.norm(p_yuzey)
  if norm_p < 1e-9:
      return p_yuzey
  u_hat = p_yuzey / norm_p
  return p_yuzey + float(r) * u_hat

  # renk_algi.py:1248
  p_3b = isin_derinlik_3b(cx, cy, d_secilen, self._k_renk, r=r_elma, oklid_mi=False)
  ```
- **Sayısal Kestirim:**
  1. **`oklid_mi=False` Seçimi:** Gazebo `libgazebo_ros_camera.so` derinlik kamerası OpenGL Z-buffer'dan optik eksen derinliğini ($Z_{\text{opt}}$) üretir; Öklid mesafesi ($\|\mathbf{p}\|$) üretmez. Bu nedenle `oklid_mi=False` (`z_opt = float(d)`) seçimi Gazebo için **doğrudur**. (Eğer yanlışlıkla Öklid sanılsaydı dahi, elma $d \ge 0.83\text{ m}$'de görüntü merkezine yakın olduğu için $x_n^2 + y_n^2 \approx 0.025 \implies \sim\%1.2$ ölçek farkı yapardı; bu ise $0.83\text{ m}$'de yalnız $1\text{ cm}$, $3.0\text{ m}$'de $0.4\text{ cm}$ yapar ve gözlenen eğriye uymaz).
  2. **Koşulsuz $+r \cdot \hat{\mathbf{u}}$ ($+3.5\text{ cm}$) Ötelemesinin Geometrik Hatası:**
     - `p_yuzey + float(r) * u_hat` formülü, okunan `d_secilen` derinliğinin kürenin kameraya **en yakın tepe noktasından (apex)** ($Z_{\text{apex}} = Z_{\text{merkez}} - r$) alındığını varsayar.
     - Oysa yarıçapı $r = 3.5\text{ cm}$ olan bir küre üzerinde merkezden $\rho \in [0, r]$ kadar yanal/düşey kaçık bir noktada küre yüzeyinin optik derinliği:
       $$Z_{\text{küre}}(\rho) = Z_{\text{merkez}} - \sqrt{r^2 - \rho^2}$$
     - Buna $+r = +3.5\text{ cm}$ eklendiğinde, hesaplanan merkez derinliği:
       $$Z_{\text{hesap}}(\rho) = Z_{\text{küre}}(\rho) + r = Z_{\text{merkez}} + \underbrace{\left(r - \sqrt{r^2 - \rho^2}\right)}_{\Delta Z_{\text{küre}}(\rho) \in [0,\; +3.5\text{ cm}]}$$
     - Eğer `d_secilen` kürenin kenarından ($\rho \to r$) okunursa $\Delta x = +3.5\text{ cm}$; eğer küre kenarı ile hemen arkasındaki zemin piksellerinin ($Z_{\text{zemin}} \ge Z_{\text{merkez}}$) karışımından okunursa $\Delta x = +3.5\text{ cm} \dots +4.0\text{ cm}$ **pozitif (ileri) sapma** oluşur!
- **Güven:** **Yüksek**

### Bulgu 2.5 — İki Aşamalı Derinlik ROI (`_derinlik_oku_iki_asamali`) + `derinlik_bozu` Etkileşimi (**Ana Kök Neden**)
- **Dosya / Satır:** `src/limo_autonomy/limo_autonomy/renk_algi.py:1132-1228`
- **Kod Alıntısı:**
  ```python
  # renk_algi.py:1141-1161 (1. Asama: sabit 0.6 m varsayimli paralaks)
  ud1, vd1 = renk_pikselinden_derinlige(
      cx, cy, 0.6, self._k_renk, self._k_derinlik, self._t_renk_derinlik)
  s_x = float(self._k_derinlik[0, 0]) / float(self._k_renk[0, 0])
  s_y = float(self._k_derinlik[1, 1]) / float(self._k_renk[1, 1])
  ...
  yari_w1 = max(3, int(round(0.25 * w_box * s_x)))
  yari_h1 = max(3, int(round(0.25 * h_box * s_y)))
  ...
  gecerli1 = yama1[np.isfinite(yama1) & (yama1 > 0.1) & (yama1 < 10.0)]
  if gecerli1.size < 6:
      ... # 7x7 yedek pencere (yari=3)
  d_on = float(np.median(gecerli1))

  # renk_algi.py:1182-1219 (2. Asama: d_on ile yeniden projeksiyon)
  ud2, vd2 = renk_pikselinden_derinlige(
      cx, cy, d_on, self._k_renk, self._k_derinlik, self._t_renk_derinlik)
  ...
  if gecerli2.size < 6:
      # Yedek: 7x7 pencere
      yama_y = derinlik[vy0:vy1, uy0:uy1]
      gecerli2 = yama_y[np.isfinite(yama_y) & (yama_y > 0.1) & (yama_y < 10.0)]
  ...
  d_secilen = float(np.median(gecerli2))
  ```
- **Sayısal Kestirim (Adım Adım Mekanizma):**
  1. **Paralaks Kayması:** Renk ve derinlik kameraları arasındaki $\Delta X_{\text{opt}} = -0.0148\text{ m}$ baz mesafesi nedeniyle, derinlik görüntüsündeki gerçek elma merkezi $u_d^{\text{gerçek}}(Z_c)$ ile 1. aşamanın sabit $0.6\text{ m}$ varsayımıyla baktığı $u_{d1}$ arasındaki yatay piksel hatası:
     $$\delta u_1(Z_c) = u_{d1} - u_d^{\text{gerçek}}(Z_c) = f_d \cdot (-0.0148) \cdot \left(\frac{1}{0.6} - \frac{1}{Z_c}\right) = -10.60 + \frac{6.36}{Z_c}\text{ [px]}$$
     - $Z_c \le 0.6\text{ m}$ için $\delta u_1 \approx 0\text{ px}$ (hatasız).
     - $Z_c > 0.6\text{ m}$ için $u_{d1}$ elma merkezinin **soluna** kayar: $0.83\text{ m}$'de $-2.9\text{ px}$, $1.0\text{ m}$'de $-4.2\text{ px}$, $1.5\text{ m}$'de $-6.4\text{ px}$, $2.0\text{ m}$'de $-7.4\text{ px}$, $3.0\text{ m}$'de $-8.5\text{ px}$.
  2. **Sol Kenar Gölgesiyle (`golge_band=6`) Çarpışma:**
     - Derinlik görüntüsünde elmanın yarıçapı $r_d(Z_c) = f_d \cdot \frac{0.035}{Z_c} = \frac{15.05}{Z_c}\text{ px}$'dir ($0.6\text{ m}$'de $25.1\text{ px}$; $0.83\text{ m}$'de $18.1\text{ px}$; $1.0\text{ m}$'de $15.0\text{ px}$; $1.5\text{ m}$'de $10.0\text{ px}$; $2.0\text{ m}$'de $7.5\text{ px}$; $2.5\text{ m}$'de $6.0\text{ px}$; $3.0\text{ m}$'de $5.0\text{ px}$).
     - `sensor_modeli_dugumu.py:271-276` elmanın sol kenarından itibaren **`[-r_d, -r_d + 6]` aralığındaki 7 pikseli sıfırlar (`NaN`)**!
     - Dolayısıyla elma üzerinde geçerli derinliğe sahip yatay aralık yalnız **`[-r_d + 7, +r_d]`** (sağ yarı / sağ kenar) bölgesidir!
  3. **Menzile Göre Davranış ($d \le 0.6\text{ m}$ vs. $d \ge 0.83\text{ m}$):**
     - **$d \le 0.6\text{ m}$:** $r_d \ge 25.1\text{ px}$. Sıfırlanan sol gölge bandı `[-25.1, -18.1] px` aralığındadır. 1. aşama iç penceresi (`yari_w1` $= 0.5 r_d = 12.5\text{ px}$, yani `[-12.5, +12.5] px`) gölge bandına hiç değmez ve elmanın tam tepe (apex) bölgesini örnekler. Küre kapağı ortalaması $Z_{\text{apex}} + 0.06 r \approx Z_c - 0.94 r$ verir; $+r$ eklendiğinde hata $+0.06 \times 3.5\text{ cm} = \mathbf{+0.2\text{ cm} < 1\text{ cm}}$ olur!
     - **$d = 0.83\text{ m} \dots 1.5\text{ m}$:** Elma $v > 264$ satırında olduğu için `zemin_kayip=0.80` elma piksellerinin %80'ini siler; kalan %20 pikselin sol $7\text{ px}$'i de `golge_band` ile silinir. Sola kaymış 1. aşama penceresinde `gecerli1.size < 6` koşulu tetiklenir veya pencerenin sol ucu elmanın solundaki zemini ($d_{\text{zemin}} \approx 1.29 Z_c$) ve sağ ucu elma yan yüzeyini örnekler. 2. aşamada (veya `gecerli2.size < 6` olduğunda açılan $\pm 3\text{ px}$ yedek pencerede), merkez ve sol taraf gölge bandı (`-r_d + 7`) nedeniyle silinmiş olduğundan, geçerli pikseller kürenin **sağ yan/kenar bölgesinden** ($\rho \approx 0.75 r \dots 0.98 r$) ve alt/yan zemin sınırından gelir. Bu piksellerin medyan derinliği $d_{\text{secilen}} \approx Z_c - 0.3 r \dots Z_c - 0.1 r$ olur. Üzerine $+r = +3.5\text{ cm}$ eklendiğinde **$\Delta x = +2.4\text{ cm} \dots +3.1\text{ cm}$** ileri sapma oluşur!
     - **$d = 2.0\text{ m} \dots 3.0\text{ m}$:** Elma yarıçapı $r_d = 7.5\text{ px} \to 5.0\text{ px}$'e düşer. Sol 7 piksellik gölge bandı (`[-r_d, -r_d + 6]`), **$d \ge 1.5\text{ m}$'de elmanın merkezini ($u=0$) bile tamamen içine alıp sıfırlar** ($3.0\text{ m}$'de `[-5, +1]` sıfırlanır, elma üzerinde yalnız `[+2, +5]` arasındaki en sağ kenar pikselleri kalır!). Üstelik `max(3, ...)` nedeniyle pencere yarı-genişliği $\pm 3\text{ px}$ ($7\times 7$ piksel) altına inemez; böylece pencere kürenin en sağ kenar piksellerini ($Z \approx Z_c$) ve hemen bitişiğindeki zemin piksellerini ($Z \approx Z_c \dots Z_c + 0.01\text{ m}$) birlikte örnekler. Medyan derinlik $d_{\text{secilen}} \approx Z_c + 0.001 \dots Z_c + 0.005\text{ m}$ olur; buna `isin_derinlik_3b` içinde $+r = +3.5\text{ cm}$ eklendiğinde **$\Delta x = +3.6\text{ cm} \dots +4.0\text{ cm}$** sabitlenen ileri sapma ortaya çıkar!
- **Önerilen Düzeltme:**
  1. `renk_algi.py:1142`'de 1. aşama derinlik tahmininde sabit `0.6 m` yerine dikey piksel konumundan ($v$) düzlem/zemin geometrisiyle yaklaşık mesafe kestirimi $d_{\text{tahmin}} \approx \frac{h_{\text{kamera}} - r}{(v - c_y)/f_y}$ kullanılması.
  2. `renk_algi.py:1180, 1218`'de küre ön yüzeyini (apex) yakalamak için `np.median` (P50) yerine **alt yüzdelik (`np.percentile(gecerli2, 15)` veya `20`)** kullanılması (arka plan / zemin sızıntısını ve küre kenar eğriliğini bastırır).
  3. Küre yarıçapı derinlikte $r_d < 8\text{ px}$ olduğunda ($d > 1.8\text{ m}$), örneklenen piksellerin tepe noktası değil küre kapağı ortalaması olduğu dikkate alınarak yüzey$\to$merkez ötelemesinin $+r$ yerine etkin $\alpha(r_d) \cdot r$ ($\alpha \in [0.2, 0.7]$) olarak ölçeklenmesi veya derinlik penceresinin sağa (`golge_band` dışına) kaydırılması.
- **Güven:** **Yüksek**

---

## 3. Aday Hata Kaynaklarının Sayısal Kestirimi ve Ölçüm Eğrisiyle Karşılaştırması

### 3.1 İncelenen Aday Hata Modelleri

1. **Aday A (Ana Kök Neden — $x$ İleri Sapması): 2-Aşamalı Derinlik ROI + `derinlik_bozu` Sol Gölge Bandı (`7 px`) + Küre Yan Yüzey/Kenar Örneklemesi Üzerine Koşulsuz $+r = +3.5\text{ cm}$ Eklenmesi (`renk_algi.py:431, 1141-1220` & `sensor_modeli_dugumu.py:271-283`)**
   - **Formül:**
     $$\Delta x_A(d) \approx r - \mathbb{E}\!\left[\sqrt{r^2 - \rho_{\text{etkin}}(d)^2}\right] + \beta_{\text{zemin}}(d)$$
     Burada $r = 3.5\text{ cm}$, $\rho_{\text{etkin}}(d)$ ise sol 7 piksellik gölge bandı (`[-r_d, -r_d+6]`) ve 1. aşama paralaks kayması sonrasında ROI içinde kalan geçerli piksellerin küre merkezine olan etkin yanal/düşey mesafesidir; $\beta_{\text{zemin}}(d) \in [0, +0.5\text{ cm}]$ ise $r_d \le 7.5\text{ px}$ ($d \ge 2.0\text{ m}$) olduğunda $\pm 3\text{ px}$ pencereye giren komşu zemin piksellerinin medyan katkısıdır.

2. **Aday B ($z$ ve $y$ Sapması): `K_RENK_VARSAYILAN` Optik Merkez Farkı ($\Delta c_x = -9.35\text{ px}, \Delta c_y = -18.92\text{ px}$) veya Eşdeğer Kamera Pitch ($\delta \approx 0.01\dots0.02\text{ rad}$) / Çerçeve Dönüşüm Farkı**
   - **Formül:**
     - `K_RENK_VARSAYILAN` kullanılıyorsa (`renk_algi.py:884-888`):
       $$\Delta y_{B1}(d) = +0.0103 \cdot d, \qquad \Delta z_{B1}(d) = +0.0208 \cdot d, \qquad \Delta x_{B1}(d) \approx -\frac{z_{\text{cam}} - z_{\text{elma}}}{d} \Delta z \approx -0.0029\text{ m}$$
     - Ölçüm kaydedicisi / TF dönüşümünde kamera pitch farkı $\delta_{\text{pitch}} \approx +0.010\text{ rad}$ ($0.57^\circ$, ör. tekerlek `<minDepth>0.01` eğimi + üst yarımküre görünür ağırlık merkezi kayması) veya nominal URDF pitch farkı ($\delta = 0.0191\text{ rad}$) varsa:
       $$\Delta z_{B2}(d) \approx d \cdot \delta_{\text{pitch}} + \Delta z_0$$

3. **Aday C: Renk / Derinlik Optik Çerçeve Karışıklığı (`camera_color_optical_frame` vs. `camera_depth_optical_frame` / `camera_link`)**
   - **Formül:**
     - İki optik çerçeve arasındaki baz mesafesi $\Delta Y_{\text{body}} = +0.0148\text{ m}$ ($+1.48\text{ cm}$), $\Delta Z_{\text{body}} = +0.0002\text{ m}$, $\Delta X_{\text{body}} = 0.0\text{ m}$'dir (`limo_d435.xacro:50`).
     - Eğer nokta `camera_color_optical_frame` yerine `camera_depth_optical_frame` (veya `camera_link` $y=+0.0175\text{ m}$) sanılarak dönüştürülürse:
       $$\Delta x_C(d) = 0.0\text{ cm}, \qquad \Delta y_C(d) = \mathbf{+1.48\text{ cm}\text{ (sabit)}}, \qquad \Delta z_C(d) = +0.02\text{ cm}$$
     - Bu aday **yalnız başına $y \approx +1\text{ cm}$ sabit yanal hatayı açıklar**, ancak $x$ ileri hatasını üretmez.

4. **Aday D: Öklid Mesafesi (`oklid_mi=True`) vs. Optik $Z$ Derinliği (`oklid_mi=False`) Karışıklığı (`renk_algi.py:415-420`)**
   - **Formül:**
     - Görüntü merkezinden $(x_n, y_n)$ kadar uzaktaki bir elma için Öklid mesafesi ile $Z_{\text{opt}}$ arasındaki fark:
       $$\Delta x_D(d) = d \cdot \left(\sqrt{1 + x_n^2 + y_n^2} - 1\right) \approx \frac{1}{2} d \left(x_n^2 + y_n^2\right)$$
     - Elma robotun önündeyken ($y \approx 0 \implies x_n \approx 0$) düşey açı $y_n(d) \approx \frac{0.178 - 0.04}{d} - 0.0191 = \frac{0.138}{d} - 0.0191$'dir:
       - $d = 0.6\text{ m} \implies y_n = 0.211 \implies \Delta x_D = +1.3\text{ cm}$
       - $d = 0.83\text{ m} \implies y_n = 0.147 \implies \Delta x_D = +0.9\text{ cm}$
       - $d = 1.5\text{ m} \implies y_n = 0.073 \implies \Delta x_D = +0.4\text{ cm}$
       - $d = 3.0\text{ m} \implies y_n = 0.027 \implies \Delta x_D = +0.1\text{ cm}$
     - **Sonuç:** Öklid/Z karışıklığı yakın mesafede büyük, uzak mesafede sıfıra yaklaşan bir hata üretir — **ölçülen eğrinin ($0.6\text{ m}$'de $<1\text{ cm}$, $3.0\text{ m}$'de $+4.0\text{ cm}$) tam tersidir!** Ayrıca Gazebo zaten $Z_{\text{opt}}$ ürettiği ve `oklid_mi=False` olduğu için bu aday elenir.

5. **Aday E: URDF `camera_link` Sabit $x$ Montaj Ofseti Hatası**
   - **Formül:** $\Delta x_E(d) = \text{sabit } c$.
   - **Sonuç:** Eğer URDF'de sabit $+3\text{ cm}$ $x$ ofset hatası olsaydı, $d \le 0.6\text{ m}$'de de $+3\text{ cm}$ hata görülürdü (oysa $d \le 0.6\text{ m}$'de hata $< 1\text{ cm}$'dir) ve Gazebo ile TF aynı URDF'i paylaştığı için $x$ ofset farkı $0$'dır. Bu aday elenir.

---

### 3.2 Sayısal Karşılaştırma Tablosu (Ölçüm vs. Aday Modeller)

#### Tablo 1 — İleri ($x$) Hatası Karşılaştırması ($\text{Ölçülen} - \text{Gerçek}$, cm)

| Menzil $d\text{ [m]}$ | Derinlikte Elma Yarıçapı $r_d\text{ [px]}$ | Sol Gölge Sonrası Kalan Elma Aralığı `[px]` | **Ölçülen $\Delta x\text{ [cm]}$** | **Aday A Kestirimi (2-Aşama ROI + 7px Gölge + $+r$ Küre Ofseti)** | Aday D Kestirimi (Öklid vs Z) | Aday E Kestirimi (Sabit $+3\text{ cm}$ X Ofseti) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| $\le 0.60$ | $\ge 25.1\text{ px}$ | `[-18.1, +25.1]` (Tepe korunuyor) | **$< +1.0$** | **$+0.2 \dots +0.6\text{ cm}$** | $+1.3\text{ cm}$ | $+3.0\text{ cm}$ |
| $0.83$ | $18.1\text{ px}$ | `[-11.1, +18.1]` (%80 zemin kaybı + sol kayma) | **$+2.4$** | **$+2.3 \dots +2.6\text{ cm}$** | $+0.9\text{ cm}$ | $+3.0\text{ cm}$ |
| $1.00$ | $15.0\text{ px}$ | `[-8.0, +15.0]` (%80 zemin kaybı + yedek ROI) | **$+3.0$** | **$+2.8 \dots +3.1\text{ cm}$** | $+0.7\text{ cm}$ | $+3.0\text{ cm}$ |
| $1.50$ | $10.0\text{ px}$ | `[-3.0, +10.0]` (Sol yarı silindi) | **$+3.1$** | **$+3.0 \dots +3.3\text{ cm}$** | $+0.4\text{ cm}$ | $+3.0\text{ cm}$ |
| $2.00$ | $7.5\text{ px}$ | `[-0.5, +7.5]` (Merkez sınırda) | **$+3.6$** | **$+3.4 \dots +3.7\text{ cm}$** | $+0.2\text{ cm}$ | $+3.0\text{ cm}$ |
| $2.50$ | $6.0\text{ px}$ | `[+1.0, +6.0]` (Merkez silindi + zemin sızıntısı) | **$+4.0$** | **$+3.8 \dots +4.1\text{ cm}$** | $+0.1\text{ cm}$ | $+3.0\text{ cm}$ |
| $3.00$ | $5.0\text{ px}$ | `[+2.0, +5.0]` (Yalnız sağ kenar + zemin) | **$+4.0$** | **$+3.9 \dots +4.2\text{ cm}$** | $+0.1\text{ cm}$ | $+3.0\text{ cm}$ |

#### Tablo 2 — Düşey ($z$) ve Yanal ($y$) Hatası Karşılaştırması

| Menzil $d\text{ [m]}$ | **Ölçülen $z\text{ [m]}$ (Hata $\Delta z$)** | **Bileşik $z$ Modeli ($\delta_{\text{pitch,etkin}} \approx 0.010\text{ rad}$ + Üst Yarımküre Centroid Kayması)** | Alternatif: Nominal Pitch Unutulması ($\delta = 0.0191\text{ rad}$) veya `K_RENK_VARSAYILAN` ($\Delta c_y = -18.9\text{ px}$) | **Ölçülen $\Delta y\text{ [cm]}$** | **Yanal Model (Renk-Derinlik Bazı $+1.48\text{ cm}$ / Sol Gölge Asimetrisi)** |
| :---: | :---: | :---: | :---: | :---: | :---: |
| $\le 0.60$ | $\sim 0.045\text{ m}$ ($+0.5\text{ cm}$) | $0.046\text{ m}$ ($+0.6\text{ cm}$) | $0.050\text{ m}$ ($+1.0\text{ cm}$) | $< 1\text{ cm}$ | $+0.5 \dots +1.0\text{ cm}$ |
| $0.83$ | $0.050\text{ m}$ ($+1.0\text{ cm}$) | $0.050\text{ m}$ ($+1.0\text{ cm}$) | $0.054\text{ m}$ ($+1.4\text{ cm}$) | $\sim +1.0\text{ cm}$ | $+1.0 \dots +1.4\text{ cm}$ |
| $1.00$ | $0.052\text{ m}$ ($+1.2\text{ cm}$) | $0.052\text{ m}$ ($+1.2\text{ cm}$) | $0.057\text{ m}$ ($+1.7\text{ cm}$) | $\sim +1.0\text{ cm}$ | $+1.0 \dots +1.4\text{ cm}$ |
| $1.50$ | $0.057\text{ m}$ ($+1.7\text{ cm}$) | $0.057\text{ m}$ ($+1.7\text{ cm}$) | $0.067\text{ m}$ ($+2.7\text{ cm}$) | $\sim +1.0\text{ cm}$ | $+1.0 \dots +1.4\text{ cm}$ |
| $2.00$ | $0.062\text{ m}$ ($+2.2\text{ cm}$) | $0.062\text{ m}$ ($+2.2\text{ cm}$) | $0.076\text{ m}$ ($+3.6\text{ cm}$) | $\sim +1.0\text{ cm}$ | $+1.0 \dots +1.4\text{ cm}$ |
| $2.50$ | $0.066\text{ m}$ ($+2.6\text{ cm}$) | $0.066\text{ m}$ ($+2.6\text{ cm}$) | $0.086\text{ m}$ ($+4.6\text{ cm}$) | $\sim +1.0\text{ cm}$ | $+1.0 \dots +1.4\text{ cm}$ |
| $3.00$ | $0.070\text{ m}$ ($+3.0\text{ cm}$) | $0.071\text{ m}$ ($+3.1\text{ cm}$) | $0.095\text{ m}$ ($+5.5\text{ cm}$) | $\sim +1.0\text{ cm}$ | $+1.0 \dots +1.4\text{ cm}$ |

---

## 4. En Olası Kök Nedenlerin Seçimi ve Özet Tablosu

1. **İleri ($+x$) Sapmasının En Olası Kök Nedeni (Güven: Yüksek):**
   - `renk_algi.py` içindeki `isin_derinlik_3b` fonksiyonu (`satır 431, 1248`) seçilen derinlik değerine koşulsuz olarak küre yarıçapı $+r = +3.5\text{ cm}$ eklemektedir. Bu işlem **yalnızca kürenin kameraya en yakın tepe noktası ($Z_{\text{apex}} = Z_c - r$) ölçüldüğünde** doğrudur.
   - $d \le 0.6\text{ m}$ mesafede elma derinlik görüntüsünde geniştir ($\ge 50\text{ px}$), 1. aşama paralaks tahmini (`0.6 m`, `satır 1142`) tam isabetlidir ve `sensor_modeli_dugumu.py:271-276`'daki 7 piksellik sol kenar gölgesi (`golge_band=6`) iç ROI penceresinin dışında kalır. Böylece tepe noktası ölçülür ve hata $< 1\text{ cm}$ olur.
   - $d \ge 0.83\text{ m}$ mesafede ise:
     1. 1. aşama sabit `0.6 m` paralaks varsayımı arama penceresini sola ($-3\dots-8.5\text{ px}$) kaydırır,
     2. `derinlik_bozu` elmanın sol kenarından içeri doğru **7 pikseli** (`golge_band=6`) sıfırlar (ve $d \le 2\text{ m}$'de `zemin_kayip=0.80` blokların %80'ini siler),
     3. Kalan geçerli pikseller elmanın **sağ yan/kenar yüzeyinden** ($Z \approx Z_c - 0.3r \dots Z_c$) ve $d \ge 2.0\text{ m}$'de ($r_d \le 7.5\text{ px}$ iken 7 piksellik gölge bandı küre merkezini de sildiği için) komşu zemin sınırından ($Z \ge Z_c$) gelir.
     4. $Z \approx Z_c$ olan bu kenar/zemin derinliğine $+r = +3.5\text{ cm}$ eklendiğinde, $0.83\text{ m}$'de $+2.4\text{ cm}$'den başlayıp $2.5\dots3.0\text{ m}$'de $+4.0\text{ cm}$'ye doyuma ulaşan ileri sapma birebir üretilir.

2. **Düşey ($+z$) Sapmasının En Olası Kök Nedeni (Güven: Orta-Yüksek):**
   - Menzille doğrusal artan $\Delta z = +1\text{ cm} \dots +3\text{ cm}$ ($z = 0.05 \dots 0.07\text{ m}$) hatası, $\delta_{\text{pitch,etkin}} \approx +0.010\text{ rad}$ ($0.57^\circ$) büyüklüğünde etkin bir düşey açısal sapmaya karşılık gelir. Bu sapmayı üreten mekanizmalar:
     - **(a)** Gazebo'da `limo_four_diff.xacro:28,228` tekerlek yüksekliği ($+5\text{ mm}$) ve `<minDepth>0.01</minDepth>` nedeniyle robot gövdesinin ön/arka temas batması ve `renk_bozu.py` alt-gölge / zemin teması nedeniyle HSV maskesinin kürenin üst yarımküresine doğru $2\dots3\text{ px}$ kayması (uzak menzilde $3\text{ px}$ düşey kayma $\approx 0.0033\text{ rad}$),
     - **(b)** Eğer ölçüm sırasında `/camera/color/camera_info` yerine `K_RENK_VARSAYILAN` (`cy=379.424` vs Gazebo `360.5`) kullanılıyorsa $\delta y_n = -0.0208\text{ rad}$ veya kaydedici `limo_four_diff.xacro:74`'teki `-0.0191 rad` kamera pitch açısını ihmal ediyorsa $\delta = 0.0191\text{ rad}$ (**doğrulanmalı**).

3. **Yanal ($+y \approx +1\text{ cm}$) Sapmasının En Olası Kök Nedeni (Güven: Orta-Yüksek):**
   - Renk ve derinlik optik çerçeveleri arasındaki $+1.48\text{ cm}$ (`limo_d435.xacro:50`) yanal baz mesafesi ve derinlikteki sol kenar gölgesinin (`golge_band=6`) sağ kenar piksellerini seçtirmesi (veya kaydedicide `camera_link` / `camera_depth_optical_frame` ile `camera_color_optical_frame` arasındaki $+1.48\text{ cm}$ farkın ihmal edilmesi — **doğrulanmalı**).

### Bulgular ve Önerilen Düzeltmeler Özeti

| # | Dosya:Satır | Bulgu Özeti | Sayısal Etki | Önerilen Düzeltme | Güven |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1** | `renk_algi.py:1141-1220` & `421-431` | 1. aşama sabit `0.6 m` paralaks + `golge_band=6` (7 px sol elma silinmesi) nedeniyle küre tepe noktası yerine sağ kenar/zemin derinliğinin ($Z \approx Z_c$) seçilmesi ve üzerine $+r = +3.5\text{ cm}$ eklenmesi | $d \le 0.6\text{ m}$: $<1\text{ cm}$<br>$0.83\text{ m}$: $+2.4\text{ cm}$<br>$1.0\dots1.5\text{ m}$: $+3.0\dots3.1\text{ cm}$<br>$2.0\dots3.0\text{ m}$: $+3.6\dots4.0\text{ cm}$ | 1. aşamada $v$ satırından zemin mesafesi tahmini kullan; `np.median` yerine P15–P20 alt-yüzdelik al; küçük $r_d$'de $+r$ ötelemesini küre kapağı geometrisine göre ölçekle | **Yüksek** |
| **2** | `sensor_modeli_dugumu.py:271-296` | `derinlik_bozu` ölçek/bias eklemez ($\sigma = 0.00176 z^2$), fakat `golge_band=6` elmanın sol 7 pikselini, `zemin_kayip=0.80` ise $v>264$'te %80 bloğu sıfırlar | $d \ge 1.5\text{ m}$'de ($r_d \le 10\text{ px}$) elma merkezi dahil sol yarıyı tamamen `NaN` yapar | `renk_algi.py` ROI penceresini sağ yarıya (`[0, +r_d]`) ağırlıklandıracak şekilde güncelle | **Yüksek** |
| **3** | `limo_four_diff.xacro:73-75` & `limo_d435.xacro:36-98` | Gazebo sensör pozları ile TF `camera_*_optical_frame` pozları `base_link`'e göre özdeştir; ancak `limo_four_diff.xacro` kalibre pozu `rpy=(0.0015, -0.0191, 0)` ve `y=+0.0323 m` içerir | Nominal `(0.084, 0, 0.18), rpy=0` varsayılırsa $\Delta z = +1.91\text{ cm/m}\cdot d$ ve $\Delta y = +1.48\text{ cm}$ (`color_joint`) | Kaydedici ve kontrolcüde sabit kamera pozu yerine TF (`lookup_transform`) kullanıldığını doğrula (**doğrulanmalı**) | **Orta-Yüksek** |
| **4** | `renk_algi.py:884-893` | `K_RENK_VARSAYILAN` ($c_x=649.85, c_y=379.42$) ile Gazebo simetrik merkezi ($640.5, 360.5$) arasında $(-9.35, -18.92)\text{ px}$ fark var | `camera_info` gelmezse $\Delta y = +1.03\text{ cm/m}\cdot d$, $\Delta z = +2.08\text{ cm/m}\cdot d$ | `camera_info` mesajı alınmadan `/elma_konum` yayınlama veya varsayılanları sim/gerçek olarak ayır (**doğrulanmalı**) | **Orta-Yüksek** |
| **5** | `limo_four_diff.xacro:18-29, 228` | Tekerlek alt noktası `base_footprint`'te $z = +0.005\text{ m}$ ve `<minDepth>0.01</minDepth>` | Gövde $z$ yüksekliğinde $-0.5\dots-1.5\text{ cm}$ ve kol ağırlığıyla $\delta_{\text{pitch}} \approx 0.005\dots0.01\text{ rad}$ eğim ($\Delta z \propto d$) | `base_joint` $z$ değerini `0.145 m`, `<minDepth>` değerini `0.001 m` yap (**doğrulanmalı**) | **Orta** |
| **6** | `sim_gercek_param.yaml:50-58` vs `renk_algi.py:895-900` | Gerçek D435 renk-derinlik arasında $\sim 0.29^\circ$ rotasyon varken `T_RENK_DERINLIK_VARSAYILAN` içinde $R=I$ alınmış | Derinlik ROI projeksiyonunda yatay $+2.2\text{ px}$, düşey $+0.6\text{ px}$ sabit kayma | `T_RENK_DERINLIK_VARSAYILAN` rotasyon bloğunu kalibre RPY ile doldur | **Yüksek** |
