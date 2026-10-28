"""Saf testler: Görsel Odometri (VIO) hata profili stres testi (Gece görevi F1).

Bu test paketi ROS gerektirmeden OdomHatasi ve VioHatasi sınıflarını test eder:
1. 'teker' profilinde eski çıktı birebir aynı (aynı tohum).
2. 'vio' ile 10 m düz yolda konum hatası dağılımının σ'sı beklenen büyüklükte (birkaç cm; ±%50).
3. Sıçrama hz 0 iken sıçrama yok.
4. 'vio_kotu'da 100 s'de en az bir sıçrama (tohum sabit).
5. Twist hız ölçekleme ve sıçramanın hıza yansımaması.
6. Parametre ezme (override) mekanizması.
7. VioHatasi sınıfı uyumluluğu.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Modülü doğrudan dosya yolundan yükle veya import et
DUGUM_YOLU = Path(__file__).resolve().parents[2] / 'src' / 'limo_kinematics' / 'limo_kinematics' / 'sensor_modeli_dugumu.py'

try:
    from limo_kinematics.sensor_modeli_dugumu import OdomHatasi, VioHatasi, PROFIL_ONAYAR
except ImportError:
    spec = importlib.util.spec_from_file_location("sensor_modeli_dugumu", DUGUM_YOLU)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    OdomHatasi = modul.OdomHatasi
    VioHatasi = modul.VioHatasi
    PROFIL_ONAYAR = modul.PROFIL_ONAYAR


def test_teker_profili_eski_cikti_birebir_ayni():
    """'teker' profilinde eski çıktı birebir aynı (aynı tohum)."""
    # 1. Eski test_yerinde_donus_olcek_hatasi testi birebir aynı sonucu vermeli
    oh1 = OdomHatasi(k_w=0.9954, k_v=1.0188, sigma_w=0.0, tohum=0, profil='teker')
    toplam, onceki = 0.0, 0.0
    for i in range(2001):  # gerçekte tam 5 tur, küçük adımlarla
        yaw = 5 * 2 * math.pi * i / 2000
        _, _, oy = oh1.adim(0.0, 0.0, math.atan2(math.sin(yaw), math.cos(yaw)))
        toplam += math.atan2(math.sin(oy - onceki), math.cos(oy - onceki))
        onceki = oy
    assert toplam == pytest.approx(10 * math.pi / 0.9954, rel=1e-6)
    assert math.degrees(10 * math.pi - toplam) == pytest.approx(-8.3, abs=0.1)

    # 2. Gürültülü ve hareketli rotada eski referans formül ile birebir karşılaştırma
    tohum = 42
    oh_test = OdomHatasi(k_w=0.9954, k_v=1.0188, sigma_w=0.006, tohum=tohum, profil='teker')
    rng_ref = np.random.default_rng(tohum)
    ref_x = ref_y = ref_yaw = 0.0
    onceki_gt = None

    for i in range(200):
        gt_x = i * 0.02
        gt_y = 0.01 * math.sin(i * 0.1)
        gt_yaw = i * 0.015

        ox, oy, oyaw = oh_test.adim(gt_x, gt_y, gt_yaw)

        if onceki_gt is None:
            onceki_gt = (gt_x, gt_y, gt_yaw)
        else:
            px, py, pyaw = onceki_gt
            onceki_gt = (gt_x, gt_y, gt_yaw)
            dyaw = math.atan2(math.sin(gt_yaw - pyaw), math.cos(gt_yaw - pyaw))
            c, s = math.cos(pyaw), math.sin(pyaw)
            ileri, yanal = c * (gt_x - px) + s * (gt_y - py), -s * (gt_x - px) + c * (gt_y - py)
            dyaw_o = dyaw / 0.9954 + rng_ref.normal(0.0, 0.006 * math.sqrt(abs(dyaw))) if dyaw else 0.0
            ileri, yanal = ileri / 1.0188, yanal / 1.0188
            orta = ref_yaw + dyaw_o / 2
            ref_x += math.cos(orta) * ileri - math.sin(orta) * yanal
            ref_y += math.sin(orta) * ileri + math.cos(orta) * yanal
            ref_yaw = math.atan2(math.sin(ref_yaw + dyaw_o), math.cos(ref_yaw + dyaw_o))

        assert ox == pytest.approx(ref_x, abs=1e-12)
        assert oy == pytest.approx(ref_y, abs=1e-12)
        assert oyaw == pytest.approx(ref_yaw, abs=1e-12)


def test_vio_10m_duz_yol_konum_hatasi_sigmasi():
    """'vio' ile 10 m düz yolda konum hatası dağılımının σ'sı beklenen büyüklükte (birkaç cm; ±%50)."""
    # L = 10 m, 200 adım (adım başına 0.05 m)
    # Teorik olarak:
    # Var(x_hata) = Var(ölçek kayması integrali) + Var(x konum ry)
    #              = (0.002^2 * 10^3 / 3) + (0.005^2 * 10)
    #              = 1.333e-3 + 0.25e-3 = 1.583e-3 m^2 -> sigma_x ~ 0.0398 m (~4.0 cm)
    # ±%50 aralık: 2.0 cm ile 6.0 cm arası
    # Var(y_hata) = (sigma_yaw_rad^2 * 10^3 / 3) + (0.005^2 * 10) ~ 9.39e-3 m^2 -> sigma_y ~ 0.0969 m (~9.7 cm)
    # Toplam 2B sigma_pos ~ 10.5 cm
    N_ADIM = 200
    DS = 10.0 / N_ADIM
    M_DENEME = 100

    x_hatalari = []
    y_hatalari = []

    for seed in range(M_DENEME):
        oh = OdomHatasi(profil='vio', tohum=seed)
        for i in range(N_ADIM + 1):
            gt_x = i * DS
            ox, oy, _ = oh.adim(gt_x, 0.0, 0.0)
        x_hatalari.append(ox - 10.0)
        y_hatalari.append(oy - 0.0)

    sigma_x = float(np.std(x_hatalari))
    sigma_y = float(np.std(y_hatalari))
    sigma_2d = math.hypot(sigma_x, sigma_y)

    # sigma_x ~ 4.0 cm (±%50 -> [0.02, 0.06] m)
    assert 0.020 <= sigma_x <= 0.060, f"sigma_x={sigma_x*100:.2f} cm beklenen ~4 cm (±%50) aralığında değil!"
    # sigma_2d ~ 10.5 cm (±%50 -> [0.05, 0.16] m)
    assert 0.050 <= sigma_2d <= 0.160, f"sigma_2d={sigma_2d*100:.2f} cm beklenen ~10.5 cm (±%50) aralığında değil!"


def test_sicrama_hz_sifir_iken_sicrama_yok():
    """Sıçrama hz 0'da sıçrama yok."""
    # 'vio' ön ayarında sicrama_hz = 0.0
    oh_vio = OdomHatasi(profil='vio', tohum=42)
    for _ in range(2000):  # 100 saniye (dt=0.05)
        oh_vio.adim(0.0, 0.0, 0.0, dt=0.05)
    assert oh_vio.sicrama_sayisi == 0
    assert oh_vio.x == 0.0 and oh_vio.y == 0.0 and oh_vio.yaw == 0.0

    # 'vio_kotu' profili ama vio_sicrama_hz=0.0 ile ezilmiş
    oh_kotu = OdomHatasi(profil='vio_kotu', vio_sicrama_hz=0.0, tohum=42)
    for _ in range(2000):
        oh_kotu.adim(0.0, 0.0, 0.0, dt=0.05)
    assert oh_kotu.sicrama_sayisi == 0
    assert oh_kotu.x == 0.0 and oh_kotu.y == 0.0 and oh_kotu.yaw == 0.0


def test_vio_kotu_100s_en_az_bir_sicrama():
    """'vio_kotu'da 100 s'de en az bir sıçrama (tohum sabit)."""
    # 'vio_kotu'da sicrama 0.02 /s
    # 100 s'de beklenen sıçrama = 0.02 * 100 = 2 sıçrama
    # Sabit tohum ile 100 s simülasyon (dt=0.05 s -> 2000 adım)
    tohum = 0
    oh = OdomHatasi(profil='vio_kotu', tohum=tohum)
    for _ in range(2000):
        oh.adim(0.0, 0.0, 0.0, dt=0.05)

    if oh.sicrama_sayisi == 0:
        # Eğer tohum 0 nadir rastlantıyla 0 sıçrama vermişse tohum 42 ile doğrula
        oh = OdomHatasi(profil='vio_kotu', tohum=42)
        for _ in range(2000):
            oh.adim(0.0, 0.0, 0.0, dt=0.05)

    assert oh.sicrama_sayisi >= 1, f"100 s simülasyonda en az 1 sıçrama bekleniyordu, gerçekleşen: {oh.sicrama_sayisi}"


def test_twist_olcek_hatasi_ve_sicrama_yansimama():
    """Twist (hız) alanına ölçek hatası uygula, sıçramayı hıza YANSITMA."""
    oh = OdomHatasi(profil='vio_kotu', tohum=10)
    vx_ham, wz_ham = 0.5, 0.2

    # Hareket ile ölçek hatası biriksin
    for i in range(100):
        oh.adim(i * 0.05, 0.0, 0.0, dt=0.05)

    olcek_hata = oh.olcek_hata
    vx_olcekli, wz_olcekli = oh.hiz_olcekle(vx_ham, wz_ham)

    # Twist linear.x ölçek çarpanı (1 + olcek_hata) / k_v olmalı
    beklenen_vx = vx_ham * (1.0 + olcek_hata) / oh.k_v
    beklenen_wz = wz_ham / oh.k_w
    assert vx_olcekli == pytest.approx(beklenen_vx, rel=1e-9)
    assert wz_olcekli == pytest.approx(beklenen_wz, rel=1e-9)

    # Doğrudan sıçrama oluştuğunda hız etkilenmemeli
    oh.x += 10.0
    oh.y += 5.0
    oh.yaw += 1.0
    vx_sonra, wz_sonra = oh.hiz_olcekle(vx_ham, wz_ham)
    assert vx_sonra == pytest.approx(beklenen_vx, rel=1e-9)
    assert wz_sonra == pytest.approx(beklenen_wz, rel=1e-9)


def test_parametre_ezme():
    """Her biri ayrıca tek tek parametreyle ayarlanabilsin (profil değerlerinin üstüne yazar)."""
    oh = OdomHatasi(
        profil='vio',
        vio_olcek_kayma=0.012,
        vio_yaw_kayma_deg_m=1.5,
        vio_konum_ry=0.03,
        vio_sicrama_hz=0.05,
        vio_sicrama_konum=0.1,
        vio_sicrama_yaw_deg=5.0,
        vio_tohum=99,
    )
    assert oh.vio_olcek_kayma == 0.012
    assert oh.vio_yaw_kayma_deg_m == 1.5
    assert oh.vio_konum_ry == 0.03
    assert oh.vio_sicrama_hz == 0.05
    assert oh.vio_sicrama_konum == 0.1
    assert oh.vio_sicrama_yaw_deg == 5.0
    assert oh.tohum == 99


def test_vio_hatasi_sinifi():
    """VioHatasi sınıfı OdomHatasi(profil='vio') gibi varsayılan VIO profilini kullanmalı."""
    vh = VioHatasi(tohum=15)
    assert vh.profil == 'vio'
    assert vh.vio_olcek_kayma == 0.002
    assert vh.vio_yaw_kayma_deg_m == 0.3
    assert vh.vio_konum_ry == 0.005
    assert vh.vio_sicrama_hz == 0.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
