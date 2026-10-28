# Gece Görevi G2: Kavrama / Navigasyon Görselleştirme (RViz MarkerArray + RViz2 Yapılandırması)

## Tamamlanan Dosyalar
1. **Düğüm ve Saf Yardımcılar**:
   - `src/limo_autonomy/limo_autonomy/kavrama_gorsel.py` (ve yedek `saha/gece/kavrama_gorsel.py`)
   - 5 Hz frekansta `/kavrama/isaretler` (`visualization_msgs/MarkerArray`) yayını.
   - 8 ayrı ad alanı (`ns`):
     - `nesne`: Silindir ($r=0.03$ m, $z$ yüksekliğinde; algı varsa yeşil, yoksa turuncu).
     - `Q`: Mavi ok ($Q_\theta$ yönünde).
     - `P`: Yeşil ok ($P_\theta$ yönünde).
     - `koridor`: $Q \to P$ arası, genişlik $0.22 + 2 \times 0.05 = 0.32$ m, yarı saydam açık mavi.
     - `ayak_izi_P`: $P$ pozunda robot ayak izi dikdörtgeni $\pm 0.161 \times \pm 0.11$, sarımsı yeşil kapalı poligon.
     - `kavrama_noktasi`: $P$'de robot önünde $d_{kavrama}=0.22$ m'deki nokta, küçük küre (nesneyle çakışır).
     - `durum`: `TEXT_VIEW_FACING`, robotun 0.5 m üstünde durum metni (HATA kırmızı, HAZIR yeşil, diğer sarı).
     - `guvenlik_konisi`: `base_footprint` çerçevesinde $\pm 25^\circ$ koni, 0.5 m, yalnızca `SON_YAKLASMA` sırasında görünür (ön lidar engelinde kırmızı uyarı).
   - Durum `PLAN`'a dönünce eski işaretleri temizleyen `DELETEALL` mekanizması.

2. **Birim Testleri**:
   - `src/limo_autonomy/test/test_kavrama_gorsel.py` (ve yedek `saha/gece/test_kavrama_gorsel.py`)
   - 12 kapsamlı test fonksiyonu:
     - Açı sarma ve quaternion dönüşümleri (`test_wrap_angle`, `test_quaternion_yaw_donusumu`)
     - Koridor 4 köşesi, genişlik ve uzunluk geometrisi (`test_koridor_koseleri_yatay`, `test_koridor_koseleri_egik`)
     - Ayak izi 4 köşesi ve dönüşümleri (`test_ayak_izi_koseleri_orijin`, `test_ayak_izi_donme_ve_oteleme`)
     - Kavrama noktası ve nesne çakışma doğrulaması (`test_kavrama_noktasi_hesapla`, `test_kavrama_noktasi_nesneyle_cakisir`)
     - Renk kuralları (`test_renk_secimi`)
     - Güvenlik konisi noktaları ve lidar engel denetimi (`test_guvenlik_konisi_noktalari`, `test_guvenlik_konisinde_engel_denetimi`)
     - Marker nesneleri üretimi (`test_marker_nesne_ve_ok`, `test_marker_koridor_ve_ayak_izi`, `test_marker_kavrama_noktasi_ve_durum`, `test_marker_guvenlik_konisi_ve_sil_hepsi`)
     - Düğüm arayüz ve döngü testi (`test_kavrama_gorsel_dugum_arayuz`)

3. **RViz2 Yapılandırması**:
   - `src/limo_autonomy/config/kavrama.rviz` (ve yedek `saha/gece/kavrama.rviz`)
   - Fixed Frame: `map`
   - Map (`/map`)
   - LaserScan (`/scan`)
   - PointCloud2 (`/camera/depth/points`, voxel)
   - Local Costmap (`/local_costmap/costmap`)
   - Global Plan (`/plan`)
   - MarkerArray (`/kavrama/isaretler`)
   - RobotModel (`/robot_description`)
   - TF (yalnızca `base_footprint`, `map`, `odom` açık)
   - Image (`/camera/color/image_raw`) paneli

## Claude İçin Entegrasyon Notları (Sabah Kontrolü)

### 1. `src/limo_autonomy/setup.py` Değişiklikleri:
```python
# Entry points içine ekle:
'console_scripts': [
    'patrol = limo_autonomy.patrol_node:main',
    'kavrama = limo_autonomy.kavrama_dugumu:main',
    'sahte_algi = limo_autonomy.sahte_algi:main',
    'gorev = limo_autonomy.gorev_dugumu:main',
    'kavrama_gorsel = limo_autonomy.kavrama_gorsel:main',  # <-- EKLENECEK
],

# data_files içine *.rviz dosyalarını ekle:
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),  # <-- .rviz EKLENDİ
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
],
```

### 2. `src/limo_autonomy/package.xml` Bağımlılıkları:
```xml
<exec_depend>visualization_msgs</exec_depend>
<exec_depend>rviz2</exec_depend>
```

### 3. `src/limo_autonomy/launch/kavrama.launch.py` Parçası:
```python
    gorsel_arg = DeclareLaunchArgument(
        'gorsel',
        default_value='true',
        description='Kavrama görselleştirme düğümünü (kavrama_gorsel) çalıştır.',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='RViz2 arayüzünü kavrama.rviz yapılandırması ile başlat.',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('limo_autonomy'),
            'config',
            'kavrama.rviz',
        ]),
        description='Kullanılacak RViz yapılandırma dosyası yolu.',
    )

    kavrama_gorsel_node = Node(
        package='limo_autonomy',
        executable='kavrama_gorsel',
        name='kavrama_gorsel',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gorsel')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_kavrama',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
```
