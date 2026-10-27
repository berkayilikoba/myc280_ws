"""test_sahte_algi: Sahte nesne algısı görüş testi ve geometri doğrulaması."""

import math
import pytest

from limo_autonomy.sahte_algi import gorus_testi


def test_onde_0_5m_gorunur():
    """Önde 0.5 m mesafedeki nesne kameranın görüş açısı ve menzilindedir."""
    robot_pose = (0.0, 0.0, 0.0)
    # Robotun 0.5 m önünde (x=0.5, y=0.0). Kamera x=0.084'te; kameraya mesafe = 0.416 m
    nesne_xy = (0.5, 0.0)
    gorunur, (x_r, y_r) = gorus_testi(
        robot_pose, nesne_xy, kamera_ofset=0.084, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert gorunur
    assert x_r == pytest.approx(0.5, abs=1e-5)
    assert y_r == pytest.approx(0.0, abs=1e-5)

    # Kameranın tam 0.5 m önündeki nesne (x = 0.084 + 0.5 = 0.584 m)
    nesne_cam_05 = (0.084 + 0.5, 0.0)
    gorunur_cam, (x_r_cam, y_r_cam) = gorus_testi(
        robot_pose, nesne_cam_05, kamera_ofset=0.084, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert gorunur_cam
    assert x_r_cam == pytest.approx(0.584, abs=1e-5)
    assert y_r_cam == pytest.approx(0.0, abs=1e-5)


def test_arkada_gorunmez():
    """Robotun arkasındaki nesne görüş açısı dışında kalır."""
    robot_pose = (0.0, 0.0, 0.0)
    nesne_xy = (-0.5, 0.0)
    gorunur, (x_r, y_r) = gorus_testi(
        robot_pose, nesne_xy, kamera_ofset=0.084, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert not gorunur
    assert x_r == pytest.approx(-0.5, abs=1e-5)
    assert y_r == pytest.approx(0.0, abs=1e-5)


def test_2m_uzakta_gorunmez():
    """2.0 m uzaktaki nesne azami menzili (1.5 m) aştığı için görünmez."""
    robot_pose = (0.0, 0.0, 0.0)
    nesne_xy = (2.0, 0.0)
    gorunur, (x_r, y_r) = gorus_testi(
        robot_pose, nesne_xy, kamera_ofset=0.084, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert not gorunur
    assert x_r == pytest.approx(2.0, abs=1e-5)
    assert y_r == pytest.approx(0.0, abs=1e-5)


def test_40_derece_yanda_gorunmez():
    """Kamera eksenine göre 40° açıdaki nesne (35° yarım açıyı aştığı için) görünmez."""
    robot_pose = (0.0, 0.0, 0.0)
    kamera_ofset = 0.084
    mesafe = 0.5  # menzil içinde (0.12 - 1.5 m)

    # 40° sol (yarım açı 35° olduğu için görüş dışı)
    aci_40 = math.radians(40.0)
    nesne_sol_40 = (kamera_ofset + mesafe * math.cos(aci_40), mesafe * math.sin(aci_40))
    gorunur_sol_40, (x_r, y_r) = gorus_testi(
        robot_pose, nesne_sol_40, kamera_ofset=kamera_ofset, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert not gorunur_sol_40

    # 40° sağ (-40°) de simetrik olarak görüş dışı
    nesne_sag_40 = (kamera_ofset + mesafe * math.cos(-aci_40), mesafe * math.sin(-aci_40))
    gorunur_sag_40, _ = gorus_testi(
        robot_pose, nesne_sag_40, kamera_ofset=kamera_ofset, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert not gorunur_sag_40

    # Karşılaştırma: 30° açı görüş açısı (<= 35°) içinde olduğundan görünmelidir
    aci_30 = math.radians(30.0)
    nesne_sol_30 = (kamera_ofset + mesafe * math.cos(aci_30), mesafe * math.sin(aci_30))
    gorunur_sol_30, _ = gorus_testi(
        robot_pose, nesne_sol_30, kamera_ofset=kamera_ofset, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert gorunur_sol_30


def test_cok_yakin_0_1m_gorunmez():
    """Kameraya 0.1 m yakınlıktaki nesne asgari menzilin (0.12 m) altında kaldığı için görünmez."""
    robot_pose = (0.0, 0.0, 0.0)
    kamera_ofset = 0.084

    # Kameradan 0.10 m mesafede:
    nesne_0_1m = (kamera_ofset + 0.10, 0.0)
    gorunur, _ = gorus_testi(
        robot_pose, nesne_0_1m, kamera_ofset=kamera_ofset, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert not gorunur

    # Robot merkezinden 0.10 m mesafede (kameraya mesafe 0.016 m < 0.12 m):
    nesne_robot_0_1m = (0.10, 0.0)
    gorunur_robot, _ = gorus_testi(
        robot_pose, nesne_robot_0_1m, kamera_ofset=kamera_ofset, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert not gorunur_robot


def test_yonelmis_robot_ve_koordinat_donusumu():
    """Farklı konum ve yöndeki robot için koordinat dönüşümü ve görüş doğrulanır."""
    # Robot (1.0, 2.0) konumunda ve +y yönüne (90 derece) bakıyor
    robot_pose = (1.0, 2.0, math.pi / 2.0)
    kamera_ofset = 0.084

    # Kamera dünya koordinatlarında (1.0, 2.0 + 0.084) = (1.0, 2.084)'tedir.
    # Kameranın 0.5 m önü dünya koordinatlarında y = 2.084 + 0.5 = 2.584
    nesne_xy = (1.0, 2.584)

    gorunur, (x_r, y_r) = gorus_testi(
        robot_pose, nesne_xy, kamera_ofset=kamera_ofset, menzil_min=0.12, menzil_max=1.5, gorus_acisi=70.0
    )
    assert gorunur
    # Robotun base_footprint çerçevesinde: x_r (ileri) = 0.584, y_r (yanal) = 0.0 olmalı
    assert x_r == pytest.approx(0.584, abs=1e-5)
    assert y_r == pytest.approx(0.0, abs=1e-5)


def test_varsayilan_parametreler():
    """Fonksiyonun varsayılan parametrelerle (kamera_ofset=0.084, menzil=[0.12, 1.5], gorus=70.0) çalışması."""
    robot_pose = (0.0, 0.0, 0.0)
    nesne_xy = (0.5, 0.0)
    gorunur, (x_r, y_r) = gorus_testi(robot_pose, nesne_xy)
    assert gorunur
    assert x_r == pytest.approx(0.5, abs=1e-5)
    assert y_r == pytest.approx(0.0, abs=1e-5)


def test_dikey_gorus_uygun_yukseklik_gorunur():
    """Kamera z=0.18 m ve dikey yarı açı 21° iken dolap (z=0.30 m, 0.5 m önde) görüş içindedir."""
    robot_pose = (0.0, 0.0, 0.0)
    # Kameranın 0.50 m önü: x = 0.084 + 0.50 = 0.584 m
    nesne_xy = (0.584, 0.0)
    # dz = 0.30 - 0.18 = 0.12 m; atan2(0.12, 0.50) ~= 13.5° <= 21°
    gorunur, _ = gorus_testi(
        robot_pose=robot_pose,
        nesne_xy=nesne_xy,
        nesne_z=0.30,
        kamera_z=0.18,
        dikey_yari_aci=21.0,
    )
    assert gorunur is True


def test_dikey_gorus_yuksek_nesne_gorunmez():
    """Masa (z=0.75 m) 0.5 m mesafede dikey görüş yarı açısını (21°) aştığı için görünmez."""
    robot_pose = (0.0, 0.0, 0.0)
    nesne_xy = (0.584, 0.0)
    # dz = 0.75 - 0.18 = 0.57 m; atan2(0.57, 0.50) ~= 48.7° > 21°
    gorunur, _ = gorus_testi(
        robot_pose=robot_pose,
        nesne_xy=nesne_xy,
        nesne_z=0.75,
        kamera_z=0.18,
        dikey_yari_aci=21.0,
    )
    assert gorunur is False


def test_dikey_gorus_3b_ve_geriye_uyumluluk():
    """nesne_z verilmediğinde yatay görüş geçerli kalır; 3B tuple olarak nesne_xy verilmesi desteklenir."""
    robot_pose = (0.0, 0.0, 0.0)
    # 1. nesne_z = None -> dikey kısıt uygulanmaz (geriye dönük uyumlu)
    gorunur_eski, _ = gorus_testi(robot_pose, (0.5, 0.0))
    assert gorunur_eski is True

    # 2. 3B demet (x, y, z) olarak verildiğinde nesne_z otomatik çekilir
    gorunur_3b_alcak, _ = gorus_testi(robot_pose, (0.584, 0.0, 0.30), kamera_z=0.18, dikey_yari_aci=21.0)
    assert gorunur_3b_alcak is True

    gorunur_3b_yuksek, _ = gorus_testi(robot_pose, (0.584, 0.0, 0.75), kamera_z=0.18, dikey_yari_aci=21.0)
    assert gorunur_3b_yuksek is False
