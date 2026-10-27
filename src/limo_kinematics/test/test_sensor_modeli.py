"""sensor_modeli_dugumu: gerçek LIMO kusurları (27.10.2026 ölçümleri) doğru uygulanıyor mu."""

import math

import numpy as np
import pytest

from limo_kinematics.sensor_modeli_dugumu import OdomHatasi, derinlik_bozu


def test_yerinde_donus_olcek_hatasi():
    oh = OdomHatasi(k_w=0.9954, k_v=1.0188, sigma_w=0.0)
    toplam, onceki = 0.0, 0.0
    for i in range(2001):                       # gerçekte tam 5 tur, küçük adımlarla
        yaw = 5 * 2 * math.pi * i / 2000
        _, _, oy = oh.adim(0.0, 0.0, math.atan2(math.sin(yaw), math.cos(yaw)))
        toplam += math.atan2(math.sin(oy - onceki), math.cos(oy - onceki))
        onceki = oy
    assert toplam == pytest.approx(10 * math.pi / 0.9954, rel=1e-6)
    # kapsama turunda olduğu gibi: 5 tur ~ -8.3° fark (gerçek - odom)
    assert math.degrees(10 * math.pi - toplam) == pytest.approx(-8.3, abs=0.1)


def test_duz_gidis_mesafe_olcegi():
    oh = OdomHatasi(k_w=0.9954, k_v=1.0188, sigma_w=0.0)
    for i in range(101):
        x, y, yaw = oh.adim(i * 0.01, 0.0, 0.0)
    assert x == pytest.approx(1.0 / 1.0188, rel=1e-9)
    assert y == pytest.approx(0.0, abs=1e-12) and yaw == pytest.approx(0.0, abs=1e-12)


def test_derinlik_gurultusu_ve_aralik():
    rng = np.random.default_rng(0)
    z = np.full((200, 200), 2.0, np.float32)
    d = derinlik_bozu(z, a=0.00176, golge_esik=0, bosluk=0, rng=rng).astype(float) / 1000
    assert d.dtype == float and (d > 0).all()
    assert d.std() == pytest.approx(0.00176 * 4, rel=0.1)          # σ = a z²
    z2 = np.array([[0.1, 0.5, 11.0, np.nan]], np.float32)
    assert derinlik_bozu(z2, golge_esik=0, bosluk=0, rng=rng).tolist() == [[0, 500, 0, 0]] or \
        derinlik_bozu(z2, a=0, golge_esik=0, bosluk=0, rng=rng).tolist() == [[0, 500, 0, 0]]


def test_derinlik_golge_seridi():
    z = np.full((10, 40), 1.0, np.float32)
    z[:, 20:] = 3.0                                                  # sağ taraf uzak: kenarın uzak yanı gölgelenir
    d = derinlik_bozu(z, a=0, golge_esik=0.08, bosluk=0)
    assert (d[:, 20:23] == 0).all() and (d[:, :20] == 1000).all() and (d[:, 24:] == 3000).all()
