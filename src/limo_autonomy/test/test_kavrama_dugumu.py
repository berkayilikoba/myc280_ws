"""kavrama_dugumu saf yardımcı fonksiyon birim testleri (ROS'suz saf Python)."""

import math

import numpy as np
import pytest

from limo_autonomy.kavrama_dugumu import (
    durum_metni_ayristir,
    erken_kabul_durumu,
    geri_bildirim_doldur,
    guvenlik_mesafesi,
    kalkan_bekleme_kontrolu,
    kalkan_dur_mu,
    kalkan_uygula,
    kalkan_yavas_mi,
    kurtarma_durum_gecisi,
    map_noktasi_odoma,
    map_yonu_odoma,
    nav_q_durum_gecisi,
    nokta_donustur_3d,
    poz,
    q_yakin_mi,
    quaternion_nokta_dondur,
    scan_to_base_footprint,
    wrap,
    yaw_q,
)
from limo_autonomy.son_yaklasma import SonYaklasma, SonYaklasmaAyar


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


# ---------------------------------------------------------------------------
# N15: NAV_Q Erken Kabul, Zaman Aşımı ve Durum Geçiş Koruması Testleri
# ---------------------------------------------------------------------------


def test_erken_kabul_durumu_yeterli_sure():
    """Robot erken kabul eşiğinde (0.20 m, 20°) en az sure (1.0 s) boyunca kaldığında True döner."""
    # 0.0 saniyeden 1.0 saniyeye kadar 50 Hz gibi örnekler: mesafe 0.15 m, açı 10°
    gecmis = [(t * 0.1, 0.15, math.radians(10.0)) for t in range(11)]  # 0.0 .. 1.0 s
    assert erken_kabul_durumu(gecmis, xy=0.20, yaw=math.radians(20.0), sure=1.0) is True

    # 1.5 saniye kaldığında da True
    gecmis_uzun = [(t * 0.1, 0.18, math.radians(15.0)) for t in range(16)]  # 0.0 .. 1.5 s
    assert erken_kabul_durumu(gecmis_uzun, xy=0.20, yaw=math.radians(20.0), sure=1.0) is True


def test_erken_kabul_durumu_kisa_sure():
    """Robot erken kabul eşiği içinde henüz yeterli süre (1.0 s) kalmamışsa False döner."""
    # 0.0 .. 0.8 s (toplam 0.8 s < 1.0 s)
    gecmis = [(t * 0.1, 0.12, math.radians(5.0)) for t in range(9)]
    assert erken_kabul_durumu(gecmis, xy=0.20, yaw=math.radians(20.0), sure=1.0) is False


def test_erken_kabul_durumu_esik_disina_cikinca_sifirlanir():
    """Robot eşik dışına çıktığında sayaç sıfırlanır; yeniden eşiğe girip yeterince kalmalıdır."""
    # 1. Aşama: 0.0 .. 0.8 s eşik içinde (0.15 m)
    gecmis = [(t * 0.1, 0.15, math.radians(10.0)) for t in range(9)]

    # 2. Aşama: t=0.9 s'de eşik dışına çıkıyor (0.35 m > 0.20 m) -> sayaç sıfırlanır
    gecmis.append((0.9, 0.35, math.radians(10.0)))

    # 3. Aşama: t=1.0 .. 1.5 s tekrar eşik içinde (0.15 m), fakat bu aralık 0.5 s (< 1.0 s)
    for t in range(10, 16):
        gecmis.append((t * 0.1, 0.15, math.radians(10.0)))

    # Toplam geçmiş 1.5 s olsa da son kesintisiz aralık 1.5 - 1.0 = 0.5 s olduğu için False olmalı
    assert erken_kabul_durumu(gecmis, xy=0.20, yaw=math.radians(20.0), sure=1.0) is False

    # 4. Aşama: Eşik içinde kalmaya devam ediyor: t=1.6 .. 2.1 s (kesintisiz 2.1 - 1.0 = 1.1 s >= 1.0 s)
    for t in range(16, 22):
        gecmis.append((t * 0.1, 0.15, math.radians(10.0)))

    # Artık kesintisiz süre >= 1.0 s olduğundan True dönmeli
    assert erken_kabul_durumu(gecmis, xy=0.20, yaw=math.radians(20.0), sure=1.0) is True


def test_erken_kabul_durumu_son_kayit_esik_disinda():
    """Geçmişte uzun süre eşik içinde kalınsa bile son kayıt eşik dışındaysa sayaç sıfırdır (False)."""
    # 0.0 .. 2.0 s boyunca eşik içinde
    gecmis = [(t * 0.1, 0.10, math.radians(5.0)) for t in range(21)]
    # Son an t=2.1'de eşik dışına sapma (0.25 m > 0.20 m)
    gecmis.append((2.1, 0.25, math.radians(5.0)))
    assert erken_kabul_durumu(gecmis, xy=0.20, yaw=math.radians(20.0), sure=1.0) is False


def test_erken_kabul_durumu_yaw_asimi():
    """Mesafe eşik içinde olsa dahi açısal fark 20° üzerindeyse erken kabul verilmez."""
    # d = 0.10 m (uygun), yaw = 25° (20°'den büyük)
    gecmis = [(t * 0.1, 0.10, math.radians(25.0)) for t in range(15)]
    assert erken_kabul_durumu(gecmis, xy=0.20, yaw=math.radians(20.0), sure=1.0) is False


def test_erken_kabul_durumu_bos_ve_tek_kayit():
    """Boş liste veya tek bir kayıt için False dönmeli."""
    assert erken_kabul_durumu([], xy=0.20, yaw=math.radians(20.0), sure=1.0) is False
    # Tek kayıt: dt = 0.0 < 1.0 s
    assert erken_kabul_durumu([(0.0, 0.10, 0.1)], xy=0.20, yaw=math.radians(20.0), sure=1.0) is False


def test_erken_kabul_durumu_tolerans_birimleri():
    """xy ve yaw parametreleri metre/cm ve radyan/derece olarak girilebilir."""
    gecmis = [(t * 0.1, 0.15, math.radians(10.0)) for t in range(11)]  # dt = 1.0 s
    # xy=20.0 (cm olarak algılanıp 0.20 m'ye çevrilir), yaw=20.0 (derece olarak algılanır)
    assert erken_kabul_durumu(gecmis, xy=20.0, yaw=20.0, sure=1.0) is True
    assert erken_kabul_durumu(gecmis, xy=0.20, yaw=math.radians(20.0), sure=1.0) is True


def test_nav_q_durum_gecisi_ardisik_tik_korumasi():
    """NAV_Q çıkışında nav_q_durum_gecisi ilk çağrıda izin verir, ardışık çağrılarda engeller (N13 mantığı)."""
    # 1. Çağrı: durum 'NAV_Q' iken geçiş izni verilir ve durum 'GECIS'e alınır
    yeni_durum, izin = nav_q_durum_gecisi('NAV_Q', 'SON_YAKLASMA')
    assert izin is True
    assert yeni_durum == 'GECIS'

    # 2. Çağrı: aynı tikte veya geciken callback'te durum artık 'GECIS'tir, ikinci geçiş engellenir
    durum_2, izin_2 = nav_q_durum_gecisi(yeni_durum, 'SON_YAKLASMA')
    assert izin_2 is False
    assert durum_2 == 'GECIS'

    # Durum zaten SON_YAKLASMA veya HATA ise de geçiş engellenir
    assert nav_q_durum_gecisi('SON_YAKLASMA')[1] is False
    assert nav_q_durum_gecisi('HATA')[1] is False


def test_sahte_zamanlayici_nav_q_erken_kabul_ve_yarisi():
    """Simüle edilmiş düğüm durum döngüsünde erken kabul tetiklenmesi ve iptal sonucu yarışının önlenmesi."""
    class MockGoalHandle:
        def __init__(self):
            self.iptal_edildi = False

        def cancel_goal_async(self):
            self.iptal_edildi = True

    class SahteDugum:
        def __init__(self):
            self.durum_kodu = 'NAV_Q'
            self.aktif_nav_goal = MockGoalHandle()
            self.nav_q_gecmis = []
            self.q_erken_xy = 0.20
            self.q_erken_yaw = math.radians(20.0)
            self.q_erken_sure = 1.0
            self.son_yaklasma_baslatildi = False
            self.aday_basarisiz_nedeni = None
            self.son_durum_mesaji = ''

        def durum(self, msg: str):
            self.son_durum_mesaji = msg

        def _son_yaklasma_baslat(self):
            self.son_yaklasma_baslatildi = True
            self.durum_kodu = 'SON_YAKLASMA'

        def _aday_basarisiz_isle(self, neden: str):
            self.aday_basarisiz_nedeni = neden
            self.durum_kodu = 'GECIS'

        def nav_q_gecis_yap(self, hedef_durum: str, mesaj: str | None = None, neden: str = ''):
            yeni_durum, izin = nav_q_durum_gecisi(self.durum_kodu, hedef_durum)
            if not izin:
                return False
            self.durum_kodu = yeni_durum
            if self.aktif_nav_goal is not None:
                self.aktif_nav_goal.cancel_goal_async()
                self.aktif_nav_goal = None
            if mesaj:
                self.durum(mesaj)
            if hedef_durum == 'SON_YAKLASMA':
                self._son_yaklasma_baslat()
            elif hedef_durum == 'SONRAKI_ADAY':
                self._aday_basarisiz_isle(neden)
            return True

        def dongu_tik(self, t: float, d: float, a: float):
            if self.durum_kodu != 'NAV_Q':
                return
            self.nav_q_gecmis.append((t, d, a))
            if erken_kabul_durumu(self.nav_q_gecmis, self.q_erken_xy, self.q_erken_yaw, self.q_erken_sure):
                mesaj = f'NAV_Q erken_kabul ({d * 100:.1f} cm, {math.degrees(a):.1f}°)'
                self.nav_q_gecis_yap('SON_YAKLASMA', mesaj=mesaj)

        def nav_result_cb(self, status: int):
            # Nav2 sonucu geldiğinde (örn. status 5: CANCELED veya 6: ABORTED)
            if self.durum_kodu != 'NAV_Q':
                return  # Yarış önleme: durum zaten SON_YAKLASMA ise yok sayılır
            self.nav_q_gecis_yap('SONRAKI_ADAY', neden=f'Nav2 durum {status}')

    node = SahteDugum()
    goal = node.aktif_nav_goal

    # 1. t=0.0'dan t=0.8'e kadar erken kabul eşiğinde adımlar (d=0.15 m, a=10°)
    for step in range(9):
        node.dongu_tik(step * 0.1, 0.15, math.radians(10.0))
    assert node.durum_kodu == 'NAV_Q'
    assert goal.iptal_edildi is False
    assert node.son_yaklasma_baslatildi is False

    # 2. t=1.0 saniyede erken kabul eşiği (1.0 s) dolar
    node.dongu_tik(1.0, 0.15, math.radians(10.0))
    assert goal.iptal_edildi is True
    assert node.durum_kodu == 'SON_YAKLASMA'
    assert node.son_yaklasma_baslatildi is True
    assert 'NAV_Q erken_kabul' in node.son_durum_mesaji

    # 3. YARIŞ KORUMASI: Nav2'den gecikmeli olarak status=5 (CANCELED) sonucu geldiğinde
    # durum zaten SON_YAKLASMA olduğundan yok sayılmalı ve aday başarısız işlemine geçilmemelidir!
    node.nav_result_cb(status=5)
    assert node.durum_kodu == 'SON_YAKLASMA'
    assert node.aday_basarisiz_nedeni is None


def test_sahte_zamanlayici_nav_q_zaman_asimi():
    """Zaman aşımında (90 s) tolerans içindeyse SON_YAKLASMA, dışındaysa sonraki adaya geçiş."""
    class SahteZamanAsimiDugum:
        def __init__(self, d: float, a: float):
            self.durum_kodu = 'NAV_Q'
            self.t_nav_q_baslangic = 0.0
            self.nav_q_zaman_asimi = 90.0
            self.q_kabul_xy = 0.35
            self.q_kabul_yaw = math.radians(35.0)
            self.d = d
            self.a = a
            self.hedef_gecildi = None
            self.iptal_edildi = False

        def nav_q_gecis_yap(self, hedef: str, mesaj: str | None = None, neden: str = ''):
            yeni_durum, izin = nav_q_durum_gecisi(self.durum_kodu, hedef)
            if not izin:
                return False
            self.durum_kodu = yeni_durum
            self.iptal_edildi = True
            self.hedef_gecildi = hedef
            return True

        def dongu_tik(self, simdi: float):
            if self.durum_kodu != 'NAV_Q':
                return
            gecen_sure = simdi - self.t_nav_q_baslangic
            if gecen_sure >= self.nav_q_zaman_asimi:
                yakin = (self.d <= self.q_kabul_xy) and (self.a <= self.q_kabul_yaw)
                if yakin:
                    self.nav_q_gecis_yap('SON_YAKLASMA', mesaj='NAV_Q yakin_kabul')
                else:
                    self.nav_q_gecis_yap('SONRAKI_ADAY', neden='zaman aşımı')

    # Senaryo A: Robot tolerans içinde (d=0.25 m < 0.35 m, a=15° < 35°) -> SON_YAKLASMA
    dugum_a = SahteZamanAsimiDugum(d=0.25, a=math.radians(15.0))
    dugum_a.dongu_tik(89.0)  # Henüz 90 s olmadı
    assert dugum_a.durum_kodu == 'NAV_Q'
    dugum_a.dongu_tik(90.1)  # Zaman aşımı
    assert dugum_a.iptal_edildi is True
    assert dugum_a.hedef_gecildi == 'SON_YAKLASMA'

    # Senaryo B: Robot tolerans dışında (d=0.50 m > 0.35 m) -> SONRAKI_ADAY
    dugum_b = SahteZamanAsimiDugum(d=0.50, a=0.0)
    dugum_b.dongu_tik(90.1)
    assert dugum_b.iptal_edildi is True
    assert dugum_b.hedef_gecildi == 'SONRAKI_ADAY'


def test_eylem_geri_bildirim_kalan_mesafe_nav_q():
    """Eylem geri bildiriminde NAV_Q sırasında kalan_mesafe_m robot-Q mesafesini yansıtır."""
    fb = geri_bildirim_doldur(
        durum='NAV_Q',
        ayrinti='kalan 0.45 m',
        aday_no=1,
        aday_sayisi=3,
        kalan_mesafe_m=0.45,
    )
    assert fb.durum == 'NAV_Q'
    assert fb.kalan_mesafe_m == pytest.approx(0.45, abs=1e-5)
    assert fb.aday_no == 1
    assert fb.aday_sayisi == 3


# ---------------------------------------------------------------------------
# N24: Hareketli Engel Kalkanı Entegrasyonu Birim Testleri
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "vx, wz, durum, beklenen_vx, beklenen_wz",
    [
        # DUR durumları: vx=0 yapılır, wz korunur (yerinde dönüş serbest)
        (0.10, 0.50, 'DUR', 0.0, 0.50),
        (0.10, -0.40, 'DUR ttc=1.25s v=0.55', 0.0, -0.40),
        (0.00, 0.60, 'DUR ttc=0.50s v=0.80', 0.0, 0.60),
        # Geri çekilmede (vx < 0) de DUR'da vx=0 (engel arkada olabilir; kalkan yönü bilinmiyor)
        (-0.08, 0.00, 'DUR', 0.0, 0.00),
        (-0.08, 0.30, 'DUR ttc=0.90s v=0.40', 0.0, 0.30),
        (-0.08, -0.20, 'dur', 0.0, -0.20),  # küçük harf
        (0.10, 0.50, 'DUR:engel', 0.0, 0.50),

        # YAVAS durumları: vx = vx * 0.5 yapılır, wz korunur
        (0.10, 0.50, 'YAVAS', 0.05, 0.50),
        (0.10, -0.20, 'YAVAS ttc=2.30s v=0.40', 0.05, -0.20),
        (-0.08, 0.00, 'YAVAS', -0.04, 0.00),  # geri çekilmede de hızı yarıya indirir
        (-0.08, 0.25, 'YAVAS ttc=2.80s v=0.35', -0.04, 0.25),
        (0.00, 0.40, 'YAVAS', 0.00, 0.40),
        (0.10, 0.50, 'yavas', 0.05, 0.50),  # küçük harf
        (0.10, 0.50, 'YAVAS:hiz', 0.05, 0.50),

        # Diğer/None/boş/bilinmeyen durumlar: hız komutları aynen korunur
        (0.10, 0.50, 'serbest', 0.10, 0.50),
        (-0.08, 0.00, 'serbest', -0.08, 0.00),
        (0.10, 0.50, 'ODOM_YOK', 0.10, 0.50),
        (0.10, 0.50, 'TARAMA_YOK', 0.10, 0.50),
        (0.10, 0.50, None, 0.10, 0.50),
        (0.10, 0.50, '', 0.10, 0.50),
        (0.10, 0.50, 'GECERSIZ_DURUM', 0.10, 0.50),
    ],
)
def test_kalkan_uygula_tablosu(vx, wz, durum, beklenen_vx, beklenen_wz):
    """kalkan_uygula tablosu: DUR'da vx=0 (wz korunur, geri çekilmede de durur); YAVAS'ta vx*0.5; diğer değişmez."""
    vx_out, wz_out = kalkan_uygula(vx, wz, durum)
    assert vx_out == pytest.approx(beklenen_vx, abs=1e-6)
    assert wz_out == pytest.approx(beklenen_wz, abs=1e-6)


def test_kalkan_dur_ve_yavas_yardimcilari():
    """kalkan_dur_mu ve kalkan_yavas_mi yardımcı fonksiyonları."""
    assert kalkan_dur_mu('DUR') is True
    assert kalkan_dur_mu('DUR ttc=1.2s v=0.5') is True
    assert kalkan_dur_mu('dur') is True
    assert kalkan_dur_mu('YAVAS') is False
    assert kalkan_dur_mu('serbest') is False
    assert kalkan_dur_mu(None) is False

    assert kalkan_yavas_mi('YAVAS') is True
    assert kalkan_yavas_mi('YAVAS ttc=2.5s v=0.3') is True
    assert kalkan_yavas_mi('yavas') is True
    assert kalkan_yavas_mi('DUR') is False
    assert kalkan_yavas_mi('serbest') is False
    assert kalkan_yavas_mi(None) is False


def test_dur_sirasinda_zaman_asimi_dolmaz_saf_sinif():
    """DUR sürerken SonYaklasma.duraklat(dt) ile ölçüm ve azami süre zaman aşımları dolmaz."""
    # 1. Ölçüm zaman aşımı testi (olcum_zaman_asimi = 1.0 s)
    ayar_olcum = SonYaklasmaAyar(
        olcum_zaman_asimi=1.0,
        max_sure=20.0,
        kayip_tolerans_mesafe=0.20,  # toleransı küçük tut ki mesafe koruması değil zaman aşımı devrede olsun
    )
    sy_olcum = SonYaklasma(ayar=ayar_olcum, yaklasma_yaw_odom=0.0)

    # t=0.0'da robot (0,0,0) konumunda nesne (0.50, 0.0) ölçümünü alıyor
    v0, w0, durum0, bilgi0 = sy_olcum.adim((0.0, 0.0, 0.0), t=0.0, olcum_robot_xy=(0.50, 0.0))
    assert durum0 in ('YAKLASIYOR', 'DONUYOR')

    # DUR durumunda 3 saniye bekleniyor (dt=0.1 s x 30 adım = 3.0 s > 1.0 s olcum_zaman_asimi)
    # Yeni ölçüm GELMİYOR (olcum_robot_xy=None)
    for i in range(1, 31):
        t_simdi = i * 0.1
        sy_olcum.duraklat(0.1)
        v, w, durum, bilgi = sy_olcum.adim((0.0, 0.0, 0.0), t=t_simdi, olcum_robot_xy=None)
        # DUR sırasında zaman aşımı sayaçları ötelendiği için HATA:zaman_asimi vermemeli!
        assert durum != 'HATA:zaman_asimi'
        assert durum in ('YAKLASIYOR', 'DONUYOR')

    # 2. Görev azami süre testi (max_sure = 2.0 s)
    ayar_sure = SonYaklasmaAyar(
        olcum_zaman_asimi=100.0,
        max_sure=2.0,
    )
    sy_sure = SonYaklasma(ayar=ayar_sure, yaklasma_yaw_odom=0.0)
    sy_sure.adim((0.0, 0.0, 0.0), t=0.0, olcum_robot_xy=(0.50, 0.0))

    # DUR durumunda 4 saniye bekleniyor (dt=0.1 s x 40 adım = 4.0 s > 2.0 s max_sure)
    for i in range(1, 41):
        t_simdi = i * 0.1
        sy_sure.duraklat(0.1)
        v, w, durum, _ = sy_sure.adim((0.0, 0.0, 0.0), t=t_simdi, olcum_robot_xy=None)
        assert durum != 'HATA:max_sure'


def test_kalkan_bekleme_kontrolu_saf_fonksiyon():
    """kalkan_bekleme_kontrolu fonksiyonunun süre hesabı ve zaman aşımı davranışı."""
    # 1. Kalkan serbest iken: sayaç başlamaz
    asildi, bas, gecen = kalkan_bekleme_kontrolu(None, simdi_t=100.0, kalkan_durum='serbest', maks_bekleme=15.0)
    assert asildi is False
    assert bas is None
    assert gecen == 0.0

    # 2. DUR başladığında: başlangıç zamanı atanır
    asildi, bas, gecen = kalkan_bekleme_kontrolu(None, simdi_t=100.0, kalkan_durum='DUR ttc=1.2s v=0.5', maks_bekleme=15.0)
    assert asildi is False
    assert bas == pytest.approx(100.0)
    assert gecen == pytest.approx(0.0)

    # 3. DUR sürerken (10 saniye < 15 saniye): aşım yok
    asildi, bas2, gecen2 = kalkan_bekleme_kontrolu(bas, simdi_t=110.0, kalkan_durum='DUR ttc=1.0s v=0.4', maks_bekleme=15.0)
    assert asildi is False
    assert bas2 == pytest.approx(100.0)
    assert gecen2 == pytest.approx(10.0)

    # 4. Tam 15.0 saniye: aşım yok (> 15.0 s olmalı)
    asildi, _, gecen_tam = kalkan_bekleme_kontrolu(bas, simdi_t=115.0, kalkan_durum='DUR', maks_bekleme=15.0)
    assert asildi is False
    assert gecen_tam == pytest.approx(15.0)

    # 5. 15.0 saniye aşıldığında (15.1 s > 15.0 s): aşım True
    asildi, _, gecen_asildi = kalkan_bekleme_kontrolu(bas, simdi_t=115.1, kalkan_durum='DUR ttc=0.8s v=0.6', maks_bekleme=15.0)
    assert asildi is True
    assert gecen_asildi == pytest.approx(15.1)

    # 6. Kalkan serbest kalınca: sayaç sıfırlanır
    asildi, bas_serbest, gecen_serbest = kalkan_bekleme_kontrolu(bas, simdi_t=116.0, kalkan_durum='serbest', maks_bekleme=15.0)
    assert asildi is False
    assert bas_serbest is None
    assert gecen_serbest == 0.0


def test_sahte_dongu_kalkan_maks_bekleme_asimi():
    """Simüle edilmiş düğüm döngüsünde DUR 15 s'den uzun sürerse KURTARMA yerine HATA:kalkan_bekleme ile iptal."""
    class SahteKalkanDugumu:
        def __init__(self):
            self.durum_kodu = 'SON_YAKLASMA'
            self.kalkan_uy = True
            self.kalkan_durum = 'DUR ttc=1.2s v=0.5'
            self.kalkan_maks_bekleme = 15.0
            self._kalkan_dur_baslangic_t = None
            self._kalkan_bekleme_bildirildi = False
            self.son_durum_metni = ''
            self.durduruldu = False
            self.kurtarma_baslatildi = False
            self.yayinlanan_hizlar = []

        def _durdur(self):
            self.durduruldu = True

        def durum(self, s: str):
            self.son_durum_metni = s

        def _kurtarma_baslat(self, neden: str):
            self.kurtarma_baslatildi = True
            self.durum_kodu = 'KURTARMA'

        def dongu_adim(self, t: float):
            if self.durum_kodu != 'SON_YAKLASMA':
                return

            kalkan_bekliyor = bool(self.kalkan_uy and kalkan_dur_mu(self.kalkan_durum))
            if self.kalkan_uy:
                asildi, self._kalkan_dur_baslangic_t, gecen_s = kalkan_bekleme_kontrolu(
                    self._kalkan_dur_baslangic_t,
                    t,
                    self.kalkan_durum,
                    self.kalkan_maks_bekleme,
                )
                if asildi:
                    self.durum_kodu = 'HATA'
                    self._durdur()
                    self.durum('HATA:kalkan_bekleme')
                    return

            v_ham = 0.08
            w_ham = 0.10
            v_cikis, w_cikis = kalkan_uygula(v_ham, w_ham, self.kalkan_durum)
            self.yayinlanan_hizlar.append((v_cikis, w_cikis))

            kalkan_bilgi = 'KALKAN_BEKLE ' if kalkan_bekliyor else ''
            ayrinti_metni = f'{kalkan_bilgi}v={v_cikis:.2f} w={w_cikis:.2f} rho=0.25 m'
            self.durum(f'SON_YAKLASMA {ayrinti_metni}')

    dugum = SahteKalkanDugumu()

    # 1. t=0.0'dan t=14.0'e kadar DUR sürüyor (14 s < 15 s)
    for sec in range(15):
        dugum.dongu_adim(float(sec))
        assert dugum.durum_kodu == 'SON_YAKLASMA'
        assert dugum.kurtarma_baslatildi is False
        assert dugum.yayinlanan_hizlar[-1][0] == 0.0  # vx durdurulmuş
        assert 'KALKAN_BEKLE' in dugum.son_durum_metni

    # 2. t=15.1 saniyede kalkan_maks_bekleme aşılır -> KURTARMA değil, HATA:kalkan_bekleme olmalı
    dugum.dongu_adim(15.1)
    assert dugum.durum_kodu == 'HATA'
    assert dugum.durduruldu is True
    assert dugum.kurtarma_baslatildi is False  # KURTARMA çağrılmadı!
    assert 'HATA:kalkan_bekleme' in dugum.son_durum_metni


def test_sahte_dongu_kalkan_kalkinca_yaklasmaya_devam():
    """Kalkan DUR'dan serbest'e geçtiğinde robot son yaklaşmaya normal hızla devam eder."""
    class SahteDevamDugumu:
        def __init__(self):
            self.durum_kodu = 'SON_YAKLASMA'
            self.kalkan_uy = True
            self.kalkan_durum = 'DUR ttc=1.0s v=0.5'
            self.kalkan_maks_bekleme = 15.0
            self._kalkan_dur_baslangic_t = None
            self.son_durum_metni = ''
            self.son_vx = None

        def _durdur(self):
            pass

        def durum(self, s: str):
            self.son_durum_metni = s

        def dongu_adim(self, t: float):
            kalkan_bekliyor = bool(self.kalkan_uy and kalkan_dur_mu(self.kalkan_durum))
            if self.kalkan_uy:
                asildi, self._kalkan_dur_baslangic_t, _ = kalkan_bekleme_kontrolu(
                    self._kalkan_dur_baslangic_t,
                    t,
                    self.kalkan_durum,
                    self.kalkan_maks_bekleme,
                )
                if asildi:
                    self.durum_kodu = 'HATA'
                    self.durum('HATA:kalkan_bekleme')
                    return

            v_cikis, _ = kalkan_uygula(0.08, 0.0, self.kalkan_durum)
            self.son_vx = v_cikis
            kalkan_bilgi = 'KALKAN_BEKLE ' if kalkan_bekliyor else ''
            self.durum(f'SON_YAKLASMA {kalkan_bilgi}v={v_cikis:.2f}')

    node = SahteDevamDugumu()

    # 1. t=0 .. 3 s DUR: hız sıfır, KALKAN_BEKLE bildirimi var
    for sec in range(4):
        node.dongu_adim(float(sec))
        assert node.son_vx == 0.0
        assert 'KALKAN_BEKLE' in node.son_durum_metni

    # 2. t=4 s: İnsan geçti, kalkan 'serbest'
    node.kalkan_durum = 'serbest'
    node.dongu_adim(4.0)
    assert node.durum_kodu == 'SON_YAKLASMA'
    assert node.son_vx == pytest.approx(0.08)
    assert 'KALKAN_BEKLE' not in node.son_durum_metni
    assert node._kalkan_dur_baslangic_t is None


def test_kurtarma_geri_cekilme_kalkan_dur_ve_asimi():
    """KURTARMA sırasında DUR gelirse geri çekilme durur (vx=0.0); 15s aşılırsa HATA:kalkan_bekleme."""
    # 1. Normal kurtarma: vx = -0.08
    v_norm, w_norm = kalkan_uygula(-0.08, 0.0, 'serbest')
    assert v_norm == pytest.approx(-0.08)

    # 2. DUR durumunda: vx = 0.0 (engel arkada olabilir; kalkan yönü bilinmiyor)
    v_dur, w_dur = kalkan_uygula(-0.08, 0.0, 'DUR ttc=0.9s v=0.4')
    assert v_dur == pytest.approx(0.0)

    # 3. KURTARMA döngüsünde 15 saniye DUR aşımı
    dur_baslangic = 50.0
    asildi, bas, gecen = kalkan_bekleme_kontrolu(dur_baslangic, simdi_t=65.1, kalkan_durum='DUR', maks_bekleme=15.0)
    assert asildi is True
    assert gecen == pytest.approx(15.1)


def test_durum_metni_ayristir_hata_kalkan_bekleme():
    """durum_metni_ayristir 'HATA:kalkan_bekleme' durumlarını doğru ayrıştırır."""
    durum, ayrinti = durum_metni_ayristir('HATA:kalkan_bekleme')
    assert durum == 'HATA'
    assert ayrinti == 'kalkan_bekleme'

    durum_ek, ayrinti_ek = durum_metni_ayristir('HATA:kalkan_bekleme 15.1 s > 15.0 s')
    assert durum_ek == 'HATA'
    assert ayrinti_ek == 'kalkan_bekleme 15.1 s > 15.0 s'

    durum_kalkan, ayrinti_kalkan = durum_metni_ayristir('SON_YAKLASMA KALKAN_BEKLE v=0.00 w=0.00 rho=0.25 m')
    assert durum_kalkan == 'SON_YAKLASMA'
    assert 'KALKAN_BEKLE' in ayrinti_kalkan
