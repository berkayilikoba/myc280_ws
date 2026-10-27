"""kavrama_dugumu saf yardımcı fonksiyon birim testleri (ROS'suz saf Python)."""

import math

import numpy as np
import pytest

from limo_autonomy.kavrama_dugumu import (
    guvenlik_mesafesi,
    kurtarma_durum_gecisi,
    map_noktasi_odoma,
    map_yonu_odoma,
    nokta_donustur_3d,
    poz,
    q_yakin_mi,
    quaternion_nokta_dondur,
    scan_to_base_footprint,
    wrap,
    yaw_q,
)


def test_guvenlik_mesafesi_onunde_engel():
    """Ön konideki engelin tampon mesafesi hesabı."""
    # Engel x=0.30 m, y=0.0 m (tampon=0.161 m) -> mesafe = 0.30 - 0.161 = 0.139 m
    noktalar = [(0.30, 0.0)]
    d = guvenlik_mesafesi(noktalar, tampon=0.161)
    assert d == pytest.approx(0.139, abs=1e-5)

    # Yakın engel x=0.20 m -> mesafe = 0.039 m (< 0.05 m güvenlik durdurma eşiği)
    noktalar_yakin = [(0.20, 0.0)]
    d_yakin = guvenlik_mesafesi(noktalar_yakin, tampon=0.161)
    assert d_yakin == pytest.approx(0.039, abs=1e-5)


def test_guvenlik_mesafesi_koni_sinirlari():
    """±25° koni dışındaki veya robotun arkasındaki engeller yok sayılır."""
    koni_25 = math.radians(25)

    # 1. Koni içindeki nokta (20°): algılanmalı
    x1 = 0.50
    y1 = x1 * math.tan(math.radians(20))
    d1 = guvenlik_mesafesi([(x1, y1)], koni=koni_25, tampon=0.161)
    assert math.isfinite(d1)
    beklenen_d1 = math.hypot(x1, y1) - 0.161
    assert d1 == pytest.approx(beklenen_d1, abs=1e-5)

    # 2. Koni dışındaki nokta (35°): yok sayılmalı
    y2 = x1 * math.tan(math.radians(35))
    d2 = guvenlik_mesafesi([(x1, y2)], koni=koni_25, tampon=0.161)
    assert d2 == float('inf')

    # 3. Robotun arkasındaki nokta (x < 0): yok sayılmalı
    d3 = guvenlik_mesafesi([(-0.30, 0.0)], koni=koni_25, tampon=0.161)
    assert d3 == float('inf')


def test_guvenlik_mesafesi_nesne_haric_tutma():
    """Kavranacak nesnenin haric_r (0.08 m) çevresindeki noktalar güvenlik durdurmasını tetiklemez."""
    nesne_robotta = (0.22, 0.0)

    # Nesne üzerindeki nokta (x=0.24, y=0.01) -> nesneye uzaklık ~0.022 m <= 0.08 m
    nesne_noktasi = (0.24, 0.01)

    # Başka bir engel (x=0.45, y=0.0) -> nesneye uzaklık 0.23 m > 0.08 m
    engel_noktasi = (0.45, 0.0)

    # Yalnız nesne noktası varken: yok sayılmalı -> inf dönmeli
    d_nesne = guvenlik_mesafesi([nesne_noktasi], nesne_robotta=nesne_robotta, haric_r=0.08, tampon=0.161)
    assert d_nesne == float('inf')

    # Hem nesne hem de gerçek engel varken: yalnızca gerçek engel hesaplanmalı
    d_toplam = guvenlik_mesafesi([nesne_noktasi, engel_noktasi], nesne_robotta=nesne_robotta, haric_r=0.08, tampon=0.161)
    assert d_toplam == pytest.approx(0.45 - 0.161, abs=1e-5)


def test_guvenlik_mesafesi_bos_veya_numpy_dizisi():
    """Boş liste veya NumPy dizisi ile çağrıldığında doğru davranış."""
    assert guvenlik_mesafesi([]) == float('inf')

    np_bos = np.empty((0, 2))
    assert guvenlik_mesafesi(np_bos) == float('inf')

    # NumPy 2B dizi testi
    np_noktalar = np.array([
        [0.40, 0.0],
        [0.60, 0.1],
    ])
    d = guvenlik_mesafesi(np_noktalar, tampon=0.161)
    assert d == pytest.approx(0.40 - 0.161, abs=1e-5)


def test_wrap_ve_yaw_q():
    """wrap ve yaw_q açı hesaplamalarının doğruluğu."""
    # wrap testleri
    assert wrap(0.0) == 0.0
    assert abs(wrap(math.pi)) == pytest.approx(math.pi)  # ±π aynı açı (atan2: (-π, π])
    assert abs(wrap(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(wrap(-3 * math.pi)) == pytest.approx(math.pi)
    assert wrap(math.radians(190)) == pytest.approx(math.radians(-170))

    # yaw_q testleri (Pose orientation quaternion mock nesnesi)
    class Quat:

        def __init__(self, x, y, z, w):
            self.x, self.y, self.z, self.w = x, y, z, w

    # 0 rad (kimlik quaternion)
    assert yaw_q(Quat(0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.0)

    # +90° etrafında dönme (z=sin(45°), w=cos(45°))
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    assert yaw_q(Quat(0.0, 0.0, s, c)) == pytest.approx(math.pi / 2.0)

    # -90° etrafında dönme
    assert yaw_q(Quat(0.0, 0.0, -s, c)) == pytest.approx(-math.pi / 2.0)


def test_quaternion_nokta_dondur_ve_3d():
    """quaternion_nokta_dondur ve nokta_donustur_3d geometrisi."""
    # Z ekseni etrafında 90° dönme: (1, 0, 0) -> (0, 1, 0)
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    q_90z = (0.0, 0.0, s, c)

    pt_donen = quaternion_nokta_dondur(q_90z, (1.0, 0.0, 0.0))
    assert pt_donen[0] == pytest.approx(0.0, abs=1e-6)
    assert pt_donen[1] == pytest.approx(1.0, abs=1e-6)
    assert pt_donen[2] == pytest.approx(0.0, abs=1e-6)

    # Öteleme ve dönme ile 3B dönüşüm
    trans = (10.0, 20.0, 30.0)
    pt_out = nokta_donustur_3d(trans, q_90z, (1.0, 0.0, 5.0))
    assert pt_out[0] == pytest.approx(10.0 + 0.0, abs=1e-6)
    assert pt_out[1] == pytest.approx(20.0 + 1.0, abs=1e-6)
    assert pt_out[2] == pytest.approx(30.0 + 5.0, abs=1e-6)


def test_map_noktasi_ve_yonu_odoma():
    """map_noktasi_odoma ve map_yonu_odoma dönüşümleri."""
    # map->odom TF: translation=(1.0, 2.0), yaw=90° (pi/2)
    trans_mo = (1.0, 2.0)
    yaw_mo = math.pi / 2.0

    # (1, 0) map noktası: 90° dönüşle (0, 1) olur, öteleme ile (1.0 + 0, 2.0 + 1) = (1.0, 3.0)
    ox, oy = map_noktasi_odoma((1.0, 0.0), trans_mo, yaw_mo)
    assert ox == pytest.approx(1.0, abs=1e-6)
    assert oy == pytest.approx(3.0, abs=1e-6)

    # Yön açısı: map yönü 45° ise odom yönü 45° + 90° = 135°
    th_odom = map_yonu_odoma(math.radians(45.0), yaw_mo)
    assert th_odom == pytest.approx(math.radians(135.0), abs=1e-6)


def test_scan_to_base_footprint():
    """scan_to_base_footprint lazer taramasını base_footprint noktalarına çevirir."""
    # Lidar link: base_footprint'e göre x=0.10 m ileride, y=0.0, yaw=0.0
    tf_trans = (0.10, 0.0)
    tf_yaw = 0.0

    ranges = [1.0, 2.0, float('nan'), 0.01]  # 0.01 < range_min (yok sayılır)
    angle_min = 0.0
    angle_increment = math.radians(10.0)
    range_min = 0.05
    range_max = 10.0

    noktalar = scan_to_base_footprint(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        tf_trans,
        tf_yaw,
    )

    # 2 geçerli nokta olmalı (1.0 m ve 2.0 m)
    assert len(noktalar) == 2

    # İlk nokta: 0° açıda, mesafe 1.0 m -> lidar linkte (1.0, 0.0), base_footprint'te (1.10, 0.0)
    assert noktalar[0, 0] == pytest.approx(1.10, abs=1e-5)
    assert noktalar[0, 1] == pytest.approx(0.0, abs=1e-5)

    # İkinci nokta: 10° açıda, mesafe 2.0 m
    x_l = 2.0 * math.cos(math.radians(10.0))
    y_l = 2.0 * math.sin(math.radians(10.0))
    assert noktalar[1, 0] == pytest.approx(0.10 + x_l, abs=1e-5)
    assert noktalar[1, 1] == pytest.approx(y_l, abs=1e-5)


def test_poz():
    """poz fonksiyonu doğru Pose nesnesi üretir."""
    p = poz(1.5, -2.5, math.pi / 2.0)
    assert p.position.x == 1.5
    assert p.position.y == -2.5
    assert p.orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    assert p.orientation.w == pytest.approx(math.cos(math.pi / 4.0))


def test_q_yakin_mi_tam_eslesme():
    """Robot ve Q pozları birebir aynı olduğunda (d=0, a=0, True)."""
    robot = (1.5, 2.0, math.radians(45.0))
    q = (1.5, 2.0, math.radians(45.0))
    yakin, d, a = q_yakin_mi(robot, q, xy=0.35, yaw=math.radians(35.0))
    assert yakin is True
    assert d == pytest.approx(0.0, abs=1e-6)
    assert a == pytest.approx(0.0, abs=1e-6)


def test_q_yakin_mi_tolerans_ici():
    """Robot Q'ya toleranslar içindeyken (d=0.20 m < 0.35 m, a=15° < 35°) kabul edilir."""
    robot = (1.0, 2.0, 0.0)
    # 0.20 m mesafe, 15° açısal sapma
    q = (1.20, 2.0, math.radians(15.0))
    yakin, d, a = q_yakin_mi(robot, q, xy=0.35, yaw=math.radians(35.0))
    assert yakin is True
    assert d == pytest.approx(0.20, abs=1e-5)
    assert a == pytest.approx(math.radians(15.0), abs=1e-5)


def test_q_yakin_mi_mesafe_asimi():
    """Mesafe 0.40 m > 0.35 m olduğunda yön uygun olsa dahi reddedilir."""
    robot = (0.0, 0.0, 0.0)
    q = (0.40, 0.0, 0.0)
    yakin, d, a = q_yakin_mi(robot, q, xy=0.35, yaw=math.radians(35.0))
    assert yakin is False
    assert d == pytest.approx(0.40, abs=1e-5)
    assert a == pytest.approx(0.0, abs=1e-5)


def test_q_yakin_mi_yaw_asimi():
    """Mesafe 0.10 m < 0.35 m olsa da açısal fark 45° > 35° ise reddedilir."""
    robot = (0.0, 0.0, 0.0)
    q = (0.10, 0.0, math.radians(45.0))
    yakin, d, a = q_yakin_mi(robot, q, xy=0.35, yaw=math.radians(35.0))
    assert yakin is False
    assert d == pytest.approx(0.10, abs=1e-5)
    assert a == pytest.approx(math.radians(45.0), abs=1e-5)


def test_q_yakin_mi_aci_sarilmasi():
    """±π sınırını aşan açılarda (ör. +175° ile -175° arası fark 10°) wrap doğru hesaplanır."""
    robot = (1.0, 1.0, math.radians(175.0))
    q = (1.0, 1.0, math.radians(-175.0))
    yakin, d, a = q_yakin_mi(robot, q, xy=0.35, yaw=math.radians(35.0))
    assert yakin is True
    assert d == pytest.approx(0.0, abs=1e-6)
    assert a == pytest.approx(math.radians(10.0), abs=1e-5)


def test_q_yakin_mi_derece_veya_radyan():
    """yaw toleransı derece (35.0) veya radyan (math.radians(35.0)) olarak verilebilir."""
    robot = (0.0, 0.0, 0.0)
    q = (0.15, 0.0, math.radians(20.0))
    yakin_rad, d_rad, a_rad = q_yakin_mi(robot, q, xy=0.35, yaw=math.radians(35.0))
    yakin_deg, d_deg, a_deg = q_yakin_mi(robot, q, xy=0.35, yaw=35.0)
    assert yakin_rad is True
    assert yakin_deg is True
    assert d_rad == pytest.approx(d_deg, abs=1e-6)
    assert a_rad == pytest.approx(a_deg, abs=1e-6)


def test_q_yakin_mi_2b_poz():
    """Yalnızca (x, y) 2B poz verildiğinde yaw varsayılan 0.0 kabul edilir."""
    robot = (1.0, 2.0)
    q = (1.10, 2.0)
    yakin, d, a = q_yakin_mi(robot, q, xy=0.35, yaw=35.0)
    assert yakin is True
    assert d == pytest.approx(0.10, abs=1e-5)
    assert a == pytest.approx(0.0, abs=1e-5)


def test_kurtarma_durum_gecisi_ardisik_tik_tek_artis():
    """KURTARMA mesafesi tamamlandığında (>= 0.20 m) 50 Hz döngü art arda iki kez

    tetiklense dahi durum hemen 'GECIS'e alındığı için aday indeksi ve kurtarma
    sayısı yalnızca bir kez artmalı (yarış ve aday atlama önlenmeli).
    """
    durum = 'KURTARMA'
    kurtarma_sayisi = 0
    aktif_aday_idx = 0
    kat_edilen = 0.21  # 0.20 m hedef mesafe aşıldı

    # 1. Tik (ilk çağrı): mesafe tamamlandı -> GECIS durumuna geçer, sayaçlar 1 artar
    yeni_durum, yeni_kurtarma, yeni_idx, bitti = kurtarma_durum_gecisi(
        durum_kodu=durum,
        kat_edilen=kat_edilen,
        kurtarma_sayisi=kurtarma_sayisi,
        aktif_aday_idx=aktif_aday_idx,
        hedef_mesafe=0.20,
    )
    assert bitti is True
    assert yeni_durum == 'GECIS'
    assert yeni_kurtarma == 1
    assert yeni_idx == 1

    # 2. Tik (ardışık ikinci çağrı): durum artık 'GECIS' olduğu için sayaçlar tekrar artmaz!
    durum_2, kurtarma_2, idx_2, bitti_2 = kurtarma_durum_gecisi(
        durum_kodu=yeni_durum,
        kat_edilen=kat_edilen,
        kurtarma_sayisi=yeni_kurtarma,
        aktif_aday_idx=yeni_idx,
        hedef_mesafe=0.20,
    )
    assert bitti_2 is False
    assert durum_2 == 'GECIS'
    assert kurtarma_2 == 1  # Tekrar artmadı!
    assert idx_2 == 1        # Aday 2 atlanmadı!


def test_kurtarma_durum_gecisi_mesafe_tamamlanmadi():
    """Mesafe 0.20 m'den küçükken KURTARMA durumunda kalınır ve sayaçlar değişmez."""
    durum, kurtarma, idx, bitti = kurtarma_durum_gecisi(
        durum_kodu='KURTARMA',
        kat_edilen=0.10,  # henüz 10 cm geri çekildi
        kurtarma_sayisi=0,
        aktif_aday_idx=0,
        hedef_mesafe=0.20,
    )
    assert bitti is False
    assert durum == 'KURTARMA'
    assert kurtarma == 0
    assert idx == 0


def test_sahte_zamanlayici_kurtarma_yarisi():
    """Sahte zamanlayıcı ile simüle edilmiş düğüm durum döngüsünde iki ardışık tik testi."""
    class SahteKavramaState:

        def __init__(self):
            self.durum_kodu = 'KURTARMA'
            self.kurtarma_sayisi = 0
            self.aktif_aday_idx = 0
            self.nav_q_baslatildi = 0

        def dongu_kurtarma_tik(self, kat_edilen: float):
            yeni_durum, yeni_k, yeni_idx, bitti = kurtarma_durum_gecisi(
                self.durum_kodu, kat_edilen, self.kurtarma_sayisi, self.aktif_aday_idx, 0.20
            )
            if bitti:
                self.durum_kodu = yeni_durum  # 'GECIS'
                self.kurtarma_sayisi = yeni_k
                self.aktif_aday_idx = yeni_idx
                self.nav_q_baslatildi += 1
                self.durum_kodu = 'NAV_Q'

    node = SahteKavramaState()
    # 1. Tik: geri çekilme bitti (kat_edilen = 0.205 m)
    node.dongu_kurtarma_tik(0.205)
    assert node.kurtarma_sayisi == 1
    assert node.aktif_aday_idx == 1
    assert node.nav_q_baslatildi == 1
    assert node.durum_kodu == 'NAV_Q'

    # 2. Tik: geciken veya yarışan ikinci tik tetiklendiğinde durum 'NAV_Q' olduğundan tekrar artmamalı
    node.dongu_kurtarma_tik(0.205)
    assert node.kurtarma_sayisi == 1
    assert node.aktif_aday_idx == 1
    assert node.nav_q_baslatildi == 1


def test_kol_erisimi_z_filtresi():
    """Kol erişimi z_min=0.0, z_max=0.45 dışındaki hedeflerin tespiti."""
    z_min, z_max = 0.0, 0.45

    def plan_z_uygun_mu(nz: float) -> tuple[bool, str]:
        if nz < z_min or nz > z_max:
            return False, f'HATA erisilemez yukseklik z={nz:.2f}'
        return True, 'UYGUN'

    # Dolap z=0.30 m: erişilebilir
    uygun, msg = plan_z_uygun_mu(0.30)
    assert uygun is True
    assert msg == 'UYGUN'

    # Masa z=0.75 m: erişilemez (yüksek)
    uygun, msg = plan_z_uygun_mu(0.75)
    assert uygun is False
    assert msg == 'HATA erisilemez yukseklik z=0.75'

    # Taban altı z=-0.10 m: erişilemez (alçak)
    uygun, msg = plan_z_uygun_mu(-0.10)
    assert uygun is False
    assert msg == 'HATA erisilemez yukseklik z=-0.10'
