"""kavrama_yaklasim: yapay haritalarda yaklaşma pozu doğru mu."""

import math

import numpy as np
import pytest

from limo_autonomy.kavrama_yaklasim import Ayar, Harita, yaklasim_bul, yaklasim_tani


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
    assert math.hypot(P[0] - Q[0], P[1] - Q[1]) == pytest.approx(0.60, abs=1e-6)     # uyarlanabilir en büyük d_on
    assert y.d_on == pytest.approx(0.60, abs=1e-6)


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


# --- Görev N3 Yeni Testleri ---

def test_duvar_dibinde_secilen_yaklasma_duvara_dik():
    """(a) duvar dibindeki nesnede seçilen yaklaşma duvara dik (±10°)"""
    h = oda()
    duvar_ekle(h, 3.0, 0.0, 3.06, 3.0)              # x=3.0'da dikey duvar
    y = yaklasim_bul(h, (2.92, 1.5))
    assert y is not None
    # x=3'teki dikey duvara dışarıdan dik yaklaşma açısı 0° dir (±10° tolerans)
    assert abs(math.degrees(y.theta)) <= 10


def test_dar_koridorda_koridor_kosulu_saglanmayan_aci_secilmez():
    """(b) dar koridorda koridor koşulu sağlanmayan açı seçilmez, tanı 'koridor' raporlar."""
    h = oda()
    # Koridor genişliğini ayak izi + Q tarafı payının (0.11 + 0.05 + 0.06 = 0.22) altına indiriyoruz:
    # Toplam genişlik = 2 * (0.11 + 0.05 + 0.06) - 0.02 = 0.42 m (y = 1.5 eksenine göre [-0.21, 0.21]).
    # P=(1.78, 1.5) pozunda pay_p + belirsizlik_p = 0.045 ile genişlik 2*(0.11+0.045) = 0.31 m olup P sığar.
    # Ancak Q ve yaklaşma koridoru boyunca pay Q tarafında 0.11'e ulaştığından koridor koşulu sağlanmaz.
    # Duvarlar x in [1.47, 1.80] aralığına yerleştirilir; böylece d_on=0.60 iken Q=(1.18, 1.5)
    # pozunun ön ucu (1.18 + 0.161 + 0.11 = 1.451) duvara değmez (Q_carpisma olmaz), fakat
    # koridor boyunca ilerlerken koridor duvarına çarparak 'koridor' elenme nedeni üretilir.
    genislik = 2 * (0.11 + 0.05 + 0.06) - 0.02  # 0.42 m
    y_ust = 1.5 + genislik / 2.0                 # 1.71
    y_alt = 1.5 - genislik / 2.0                 # 1.29
    duvar_ekle(h, 1.47, y_ust, 1.80, y_ust + 0.20)
    duvar_ekle(h, 1.47, y_alt - 0.20, 1.80, y_alt)

    y_engelli = yaklasim_bul(h, (2.0, 1.5), Ayar(yon=0.0, yon_tol=math.radians(10)))
    assert y_engelli is None

    # Bu genişlikte yaklasim_tani elenme nedeninin 'koridor' olduğunu doğrula
    tani = yaklasim_tani(h, (2.0, 1.5), Ayar(yon=0.0, yon_tol=math.radians(10)))
    assert tani['koridor'] > 0
    assert tani['nedenler']['koridor'] > 0

    # Serbest seçim yapıldığında koridoru engelli olan açı (theta=0 civarı) seçilmemeli
    y_serbest = yaklasim_bul(h, (2.0, 1.5))
    assert y_serbest is not None
    assert abs(math.degrees(y_serbest.theta)) > 20


def test_d_on_uyarlanabilir_engel_durumu():
    """(c) d_on engel yokken 0.60, önünde 0.45 m'de engel varken daha küçük"""
    h = oda()
    # 1. Engel yokken en büyük d_on (0.60) seçilmeli
    y_bos = yaklasim_bul(h, (2.0, 1.5))
    assert y_bos is not None
    assert y_bos.d_on == pytest.approx(0.60, abs=1e-4)

    # 2. Yaklaşma hattında engel varken:
    # theta=0 için P=(1.78, 1.5). d_on=0.60 için Q=(1.18, 1.5).
    # x in [1.00, 1.15] aralığına engel koyarsak d_on=0.60 engelle çakışır,
    # ancak d_on=0.35 için koridor serbesttir.
    duvar_ekle(h, 1.00, 1.40, 1.15, 1.60)
    y_engelli = yaklasim_bul(h, (2.0, 1.5), Ayar(yon=0.0, yon_tol=math.radians(10)))
    assert y_engelli is not None
    assert y_engelli.d_on < 0.60
    assert y_engelli.d_on >= 0.30


def test_hepsi_sonuclari_en_az_20_derece_ayrik():
    """(d) hepsi=True sonuçlarında açılar en az 20° ayrık"""
    h = oda()
    adaylar = yaklasim_bul(h, (2.0, 1.5), hepsi=True)
    assert len(adaylar) >= 2
    for i in range(len(adaylar)):
        for j in range(i + 1, len(adaylar)):
            fark = abs(math.atan2(math.sin(adaylar[i].theta - adaylar[j].theta),
                                  math.cos(adaylar[i].theta - adaylar[j].theta)))
            assert math.degrees(fark) >= 20.0 - 1e-4


# --- Görev N14 Yeni Testleri ---

def test_ayar_belirsizlik_p_varsayilan():
    """Ayar varsayılan pay_p (0.03 m) ve belirsizlik_p (0.015 m) değerleri doğrulanır."""
    a = Ayar()
    assert a.pay_p == pytest.approx(0.03, abs=1e-6)
    assert a.belirsizlik_p == pytest.approx(0.015, abs=1e-6)


def test_duvardan_5cm_uzaktaki_nesne_duvara_dik_yaklasim_bulunur():
    """Duvardan 5 cm uzaktaki nesne için duvara dik yaklaşma BULUNUR."""
    h = oda()
    duvar_ekle(h, 3.0, 0.0, 3.06, 3.0)              # x=3.0'da dikey duvar
    # Nesne duvardan 5 cm uzakta (x = 3.0 - 0.05 = 2.95)
    y = yaklasim_bul(h, (2.95, 1.5))
    assert y is not None
    # x=3'teki duvara dıştan dik yaklaşma açısı 0° (±10° tolerans)
    assert abs(math.degrees(y.theta)) <= 10
    # P'de en yakın engele açıklık en az geçerlilik eşiği (pay_p + belirsizlik_p) kadar olmalı
    a = Ayar()
    assert y.aciklik >= (a.pay_p + a.belirsizlik_p - 1e-4)


def test_duvara_gomulu_nesne_bulunmaz_ve_P_carpisma_raporlar():
    """2 cm'den yakın (nesne duvara gömülü) için robot tamponu duvara pay+belirsizlik_p'den

    yaklaşacaksa yaklaşma bulunmaz ve yaklasim_tani 'P_carpisma' raporlar.
    """
    h = oda()
    duvar_ekle(h, 3.0, 0.0, 3.06, 3.0)              # x=3.0'da dikey duvar
    # x=3.035: P'de robot önü duvara ~0.029 m kalır < pay_p + belirsizlik_p (0.045) -> elenir
    # (N14b'den sonra 3.01'de ön tampon duvara 0.062 m kalıyor ve bu geçerli bir yaklaşma)
    y = yaklasim_bul(h, (3.035, 1.5))
    assert y is None

    tani = yaklasim_tani(h, (3.035, 1.5))
    assert isinstance(tani, dict)
    assert tani['gecerli_sayisi'] == 0
    # P_carpisma elenme nedeni bulunmalı ve raporlanmalı
    assert tani['P_carpisma'] > 0
    assert tani['nedenler']['P_carpisma'] > 0


def test_yaklasim_tani_neden_sayilari_ve_en_iyi_elenen_acikliklar():
    """yaklasim_tani her aday açı için elenme koşullarını ve en iyi 3 adayın açıklıklarını raporlar."""
    h = oda()
    duvar_ekle(h, 3.0, 0.0, 3.06, 3.0)
    tani = yaklasim_tani(h, (3.01, 1.5))

    # Temel hata koşullarının tümü sözlükte yer almalı
    for kosul in ('P_carpisma', 'koridor', 'Q_carpisma', 'erisim', 'yuzey', 'yon'):
        assert kosul in tani
        assert isinstance(tani[kosul], int)

    assert tani['toplam_aday'] > 0
    assert 'en_iyi_elenen_acikliklar' in tani
    assert isinstance(tani['en_iyi_elenen_acikliklar'], list)
    assert len(tani['en_iyi_elenen_acikliklar']) <= 3


# --- Görev N14b Yeni Testleri ---

def test_duvara_dik_yaklasmada_q_carpisirken_kisa_d_on_ile_bulunur():
    """(Görev N14b) Duvara dik yaklaşmada Q d_on_max'ta arkadaki engele çarparken daha kısa d_on ile aday BULUNUR."""
    h = oda()
    duvar_ekle(h, 3.0, 0.0, 3.06, 3.0)              # x=3.0'da hedef duvar
    duvar_ekle(h, 0.0, 0.0, 1.95, 3.0)              # x=1.95'e kadar arkadaki engel/duvar
    # Nesne x=3.0 duvarının hemen önünde: (2.92, 1.5)
    # P = 2.92 - 0.22 = 2.70.
    # d_on=0.60 iken Q=(2.10, 1.5), ayak izi arkası: 2.10 - 0.161 - 0.11 = 1.829 < 1.95 (Q çarpar!)
    # d_on=0.45 iken Q=(2.25, 1.5), ayak izi arkası: 2.25 - 0.161 - 0.11 = 1.979 > 1.95 (Q temiz!)
    # Sabit d_on=0.60 (d_on_max) ile aday bulunamaz (arkadaki duvara çarpar)
    y_sabit = yaklasim_bul(h, (2.92, 1.5), Ayar(d_on=0.60))
    assert y_sabit is None

    # Dinamik uyarlamalı d_on ile daha kısa d_on seçilerek aday bulunur
    y = yaklasim_bul(h, (2.92, 1.5))
    assert y is not None
    assert abs(math.degrees(y.theta)) <= 15         # duvara dik yaklaşım
    assert y.d_on < 0.60                            # d_on_max'tan daha kısa
    assert y.d_on >= 0.30
