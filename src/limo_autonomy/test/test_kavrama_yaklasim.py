"""kavrama_yaklasim: yapay haritalarda yaklaşma pozu doğru mu."""

import math

import numpy as np
import pytest

from limo_autonomy.kavrama_yaklasim import Ayar, Harita, yaklasim_bul


def oda(gen=4.0, yuk=3.0, r=0.02):
    """duvarlarla çevrili boş oda; orijin sol alt köşe (0, 0)"""
    H, W = int(yuk / r), int(gen / r)
    d = np.zeros((H, W), bool)
    d[:3, :] = d[-3:, :] = d[:, :3] = d[:, -3:] = True
    return Harita(d, r, (0.0, 0.0))


def duvar_ekle(h, x0, y0, x1, y1):
    v0, u0 = h.hucre(x0, y1)
    v1, u1 = h.hucre(x1, y0)
    H, W = h.dolu.shape
    h.dolu[max(v0, 0):min(v1, H - 1) + 1, max(u0, 0):min(u1, W - 1) + 1] = True


def test_acik_alanda_mesafe_ve_yon():
    h = oda()
    y = yaklasim_bul(h, (2.0, 1.5))
    assert y is not None
    P, Q = y.kavrama, y.oncesi
    assert math.hypot(2.0 - P[0], 1.5 - P[1]) == pytest.approx(0.22, abs=1e-6)       # d_kavrama
    bak = math.atan2(1.5 - P[1], 2.0 - P[0])
    assert math.atan2(math.sin(bak - P[2]), math.cos(bak - P[2])) == pytest.approx(0, abs=1e-6)   # nesneye bakıyor
    assert math.hypot(P[0] - Q[0], P[1] - Q[1]) == pytest.approx(0.35, abs=1e-6)     # düz son yaklaşma


def test_duvar_dibindeki_nesneye_karsidan_yaklasir():
    h = oda()
    duvar_ekle(h, 3.0, 0.0, 3.06, 3.0)              # x=3'te duvar; nesne duvarın hemen önünde
    y = yaklasim_bul(h, (2.92, 1.5))
    assert y is not None
    assert abs(math.degrees(y.theta)) <= 30         # +x yönünde (duvara doğru) yaklaşmalı


def test_istenen_yon():
    h = oda()
    y = yaklasim_bul(h, (2.0, 1.5), Ayar(yon=math.radians(90)))
    assert abs(math.degrees(y.theta) - 90) <= 10


def test_koseye_yakin_nesne_kose_tarafina_bakarak():
    h = oda()
    y = yaklasim_bul(h, (0.30, 0.30))               # sol alt köşeye 24 cm
    assert y is not None and -180 <= math.degrees(y.theta) <= -90   # köşe çeyreğine bakarak (açık taraftan)


def test_duvar_arkasindan_kavranmaz():
    h = oda()
    duvar_ekle(h, 3.0, 0.0, 3.06, 3.0)
    for y in yaklasim_bul(h, (2.92, 1.5), hepsi=True):
        assert y.kavrama[0] < 3.0                   # hiçbir aday duvarın öbür yanında değil


def test_robotun_bolgesinden_ulasilamayan_reddedilir():
    h = oda()
    duvar_ekle(h, 2.0, 0.0, 2.06, 3.0)              # odayı ikiye bölen duvar
    assert yaklasim_bul(h, (3.0, 1.5), robot=(0.5, 1.5)) is None
    assert yaklasim_bul(h, (3.0, 1.5), robot=(3.5, 1.5)) is not None


def test_dar_bosluktaki_nesne_erisilemez():
    h = oda()
    duvar_ekle(h, 1.0, 1.0, 3.0, 1.3)
    duvar_ekle(h, 1.0, 1.55, 3.0, 1.85)            # 25 cm'lik koridor: robot (22 cm + pay) giremez
    duvar_ekle(h, 0.9, 1.0, 1.0, 1.85)             # iki ucu kapalı
    duvar_ekle(h, 3.0, 1.0, 3.1, 1.85)
    assert yaklasim_bul(h, (2.0, 1.42)) is None
