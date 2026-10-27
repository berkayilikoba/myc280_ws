#!/usr/bin/env python3
"""renk_bozu.py modülü için birim testleri (importlib ile doğrudan yükler)."""

import importlib.util
import math
from pathlib import Path

import cv2
import numpy as np


MODUL_YOLU = (
    Path(__file__).resolve().parents[2]
    / "src"
    / "limo_kinematics"
    / "limo_kinematics"
    / "renk_bozu.py"
)
_spec = importlib.util.spec_from_file_location("renk_bozu", MODUL_YOLU)
renk_bozu = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(renk_bozu)


def test_vinyet_koseyi_karartir():
    """Sabit parlaklıktaki görüntüde vinyet köşeleri merkeze göre karartmalı."""
    img = np.full((360, 640, 3), 200, dtype=np.uint8)
    k1, k2 = -0.25, 0.05
    cikti = renk_bozu.vinyet_uygula(img, k1=k1, k2=k2)

    assert cikti.dtype == np.uint8
    assert cikti.shape == img.shape

    merkez = float(cikti[180, 320, 0])
    koseler = [
        float(cikti[0, 0, 0]),
        float(cikti[0, -1, 0]),
        float(cikti[-1, 0, 0]),
        float(cikti[-1, -1, 0]),
    ]
    beklenen_kose = round(200.0 * (1.0 + k1 + k2))
    assert merkez >= 199.0
    for kose in koseler:
        assert kose < merkez
        assert abs(kose - beklenen_kose) <= 1.0


def test_gurultu_std_uyumu():
    """Büyük sabit görüntüde gürültü std'si sqrt(a*I + b) ile %10 tolerans içinde uyumlu olmalı."""
    parlaklik = 120
    img = np.full((600, 800, 3), parlaklik, dtype=np.uint8)
    a = [0.05, 0.08, 0.10]
    b = [3.0, 2.4, 4.0]
    rng = np.random.default_rng(12345)

    cikti = renk_bozu.gurultu_uygula(img, a=a, b=b, rng=rng)
    assert cikti.dtype == np.uint8
    assert cikti.shape == img.shape

    fark = cikti.astype(np.float32) - float(parlaklik)
    for c in range(3):
        beklenen_std = math.sqrt(a[c] * parlaklik + b[c])
        olculen_std = float(np.std(fark[:, :, c]))
        bagil_hata = abs(olculen_std - beklenen_std) / beklenen_std
        assert bagil_hata < 0.10, f"Kanal {c}: ölçülen={olculen_std:.4f}, beklenen={beklenen_std:.4f}"


def test_hareket_bulanikligi_kimlik_ve_etki():
    """w_z=0 (veya L < 1 px) durumunda hareket bulanıklığı birim (kimlik) olmalı."""
    rng = np.random.default_rng(7)
    img = rng.integers(20, 235, size=(360, 640, 3), dtype=np.uint8)

    # w_z = 0 -> tam kimlik
    cikti_sifir = renk_bozu.hareket_bulanikligi(img, w_z=0.0, v_x=0.5, t_poz=0.015, fx=454.5)
    assert cikti_sifir.dtype == np.uint8
    assert cikti_sifir.shape == img.shape
    assert np.array_equal(cikti_sifir, img)

    # L = |w_z| * fx * t_poz < 1.0 -> dokunulmamalı
    cikti_kucuk = renk_bozu.hareket_bulanikligi(img, w_z=0.05, v_x=0.2, t_poz=0.010, fx=454.5)
    assert np.array_equal(cikti_kucuk, img)

    # Yüksek açısal hızda (L > 5 px) Laplacian varyansı düşmeli
    cikti_bulanik = renk_bozu.hareket_bulanikligi(img, w_z=1.2, v_x=0.0, t_poz=0.015, fx=454.5)
    assert cikti_bulanik.dtype == np.uint8
    assert cikti_bulanik.shape == img.shape
    lap_once = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    lap_sonra = cv2.Laplacian(cv2.cvtColor(cikti_bulanik, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    assert lap_sonra < lap_once


def test_beyaz_denge_ve_renk_boz():
    """beyaz_denge_uygula ve renk_boz fonksiyonları uint8 tipi ve görüntü boyutunu korumalı."""
    img = np.full((360, 640, 3), 100, dtype=np.uint8)
    bd = renk_bozu.beyaz_denge_uygula(img, [0.9, 1.0, 1.1])
    assert bd.dtype == np.uint8
    assert bd.shape == img.shape
    assert int(bd[0, 0, 0]) == 90
    assert int(bd[0, 0, 1]) == 100
    assert int(bd[0, 0, 2]) == 110

    param = {
        "k1": -0.18,
        "k2": 0.03,
        "a": [0.04, 0.04, 0.04],
        "b": [2.0, 2.0, 2.0],
        "t_poz": 0.012,
        "fx": 454.5,
        "kazanc_bgr": [0.98, 1.0, 1.02],
    }
    rng = np.random.default_rng(99)
    bozuk = renk_bozu.renk_boz(img, param, w_z=0.6, v_x=0.2, rng=rng)
    assert bozuk.dtype == np.uint8
    assert bozuk.shape == img.shape


if __name__ == "__main__":
    test_vinyet_koseyi_karartir()
    test_gurultu_std_uyumu()
    test_hareket_bulanikligi_kimlik_ve_etki()
    test_beyaz_denge_ve_renk_boz()
    print("Tüm renk_bozu testleri başarıyla geçti.")
