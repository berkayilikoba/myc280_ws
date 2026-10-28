#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_oda.py - saha/oda araçları için birim testleri (pytest).

Test edilen betikler:
  - oda_dunya.py
  - oda_doku.py
  - oda_tsdf.py

Tüm testler sentetik, bağımsız, ROS/Gazebo gereksinimi olmadan saniyeler içinde çalışır.
Open3D gerektiren testler pytest.importorskip("open3d") ile korunmuştur.
"""

import importlib.util
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

# saha/oda dizinini sys.path'e ekle
ODA_DIR = Path(__file__).resolve().parent
if str(ODA_DIR) not in sys.path:
    sys.path.insert(0, str(ODA_DIR))


def _yukle_modul(modul_adi: str):
    """saha/oda dizinindeki betiği importlib ile yükler."""
    dosya_yolu = ODA_DIR / f"{modul_adi}.py"
    if not dosya_yolu.is_file():
        raise FileNotFoundError(f"Modül dosyası bulunamadı: {dosya_yolu}")
    spec = importlib.util.spec_from_file_location(modul_adi, dosya_yolu)
    modul = importlib.util.module_from_spec(spec)
    sys.modules[modul_adi] = modul
    spec.loader.exec_module(modul)
    return modul


@pytest.fixture(scope="session")
def oda_dunya():
    """oda_dunya modülünü yükler."""
    return _yukle_modul("oda_dunya")


@pytest.fixture(scope="session")
def oda_doku():
    """oda_doku modülünü yükler."""
    return _yukle_modul("oda_doku")


@pytest.fixture(scope="session")
def oda_tsdf():
    """oda_tsdf modülünü open3d kontrolü ile yükler."""
    pytest.importorskip("open3d")
    return _yukle_modul("oda_tsdf")


# ==============================================================================
# 1. ODA_DUNYA TESTLERİ
# ==============================================================================


def test_oda_dunya_geometri_yardimcilari(oda_dunya):
    """oda_dunya içerisindeki saf geometri fonksiyonlarının doğruluğunu test eder."""
    # nokta_al
    assert oda_dunya.nokta_al({"x": 1.25, "y": 3.75}) == (1.25, 3.75)
    assert oda_dunya.nokta_al([4.5, -2.5]) == (4.5, -2.5)

    # sinirlar_coz
    bbox_dict = {"min": [0.0, 1.0], "max": [5.0, 6.0]}
    assert oda_dunya.sinirlar_coz(bbox_dict) == (0.0, 1.0, 5.0, 6.0)
    assert oda_dunya.sinirlar_coz([0.0, 1.0, 5.0, 6.0]) == (0.0, 1.0, 5.0, 6.0)
    assert oda_dunya.sinirlar_coz(None) is None

    # nokta_sinirda_mi
    bbox = (0.0, 0.0, 5.0, 5.0)
    assert oda_dunya.nokta_sinirda_mi((2.5, 2.5), bbox) is True
    assert oda_dunya.nokta_sinirda_mi((6.0, 2.5), bbox) is False

    # nokta_cokgende_mi
    kare = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]
    assert oda_dunya.nokta_cokgende_mi((2.0, 2.0), kare) is True
    assert oda_dunya.nokta_cokgende_mi((5.0, 2.0), kare) is False
    # Kenar üzerindeki nokta içeride sayılmaz (False döner)
    assert oda_dunya.nokta_cokgende_mi((0.0, 2.0), kare) is False

    # en_kucuk_dikdortgen
    dikdortgen = [[0.0, 0.0], [3.0, 0.0], [3.0, 2.0], [0.0, 2.0]]
    cx, cy, w, h, yaw = oda_dunya.en_kucuk_dikdortgen(dikdortgen)
    assert math.isclose(cx, 1.5, abs_tol=1e-3)
    assert math.isclose(cy, 1.0, abs_tol=1e-3)
    assert math.isclose(w * h, 6.0, abs_tol=1e-3)


def test_oda_dunya_dunya_uret(oda_dunya, tmp_path):
    """
    oda_dunya.dunya_uret ile sentetik yapıdan SDF üretimi testi:
    - xml.etree ile ayrıştırılabilmeli
    - zemin, tavan, duvarlar modelleri bulunmalı
    - gazebo_ros_state eklentisi bulunmalı
    - tavan ışıkları sayısı > 0 olmalı
    - tavan yüksekliği parametresi duvar boyuna yansımalı
    - mesh bulunmadığında mobilya_kutu modeli üretilmeli
    """
    yapi_dosyasi = tmp_path / "oda_yapi.yaml"
    mesh_dizini = tmp_path / "mesh_bos"
    mesh_dizini.mkdir(parents=True, exist_ok=True)
    cikti_dunya = tmp_path / "cikti" / "lab_oda_3b.world"

    tavan_yuksekligi = 3.20

    sentetik_yapi = {
        "oda_siniri": [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 4.0],
            [0.0, 4.0],
        ],
        "duvarlar": [
            {
                "p1": [0.0, 0.0],
                "p2": [4.0, 0.0],
                "kalinlik": 0.15,
            },
            {
                "p1": [4.0, 0.0],
                "p2": [4.0, 4.0],
                "kalinlik": 0.15,
            },
        ],
        "mobilya": [
            {
                "id": 0,
                "merkez": [2.0, 2.0],
                "cokgen": [
                    [1.8, 1.8],
                    [2.2, 1.8],
                    [2.2, 2.2],
                    [1.8, 2.2],
                ],
                "ince": False,
            }
        ],
    }

    with open(yapi_dosyasi, "w", encoding="utf-8") as f:
        yaml.safe_dump(sentetik_yapi, f)

    oda_dunya.dunya_uret(
        yapi_yol=str(yapi_dosyasi),
        mesh_dizin=str(mesh_dizini),
        cikti_yol=str(cikti_dunya),
        tavan=tavan_yuksekligi,
        mobilya_yuk=0.6,
        isik_aralik=2.0,
    )

    assert cikti_dunya.is_file(), "Üretilen .world dosyası oluşturulmadı"

    # XML ayrıştırma testi
    tree = ET.parse(str(cikti_dunya))
    root = tree.getroot()
    assert root.tag == "sdf", "Kök eleman <sdf> olmalıdır"

    world = root.find("world")
    assert world is not None, "<world> elemanı bulunamadı"

    # Model kontrolleri: zemin, tavan, duvarlar ve mesh dışı mobilya
    modeller = {m.get("name"): m for m in world.findall("model")}
    assert "zemin" in modeller, "zemin modeli SDF içinde bulunamadı"
    assert "tavan" in modeller, "tavan modeli SDF içinde bulunamadı"
    assert "duvarlar" in modeller, "duvarlar modeli SDF içinde bulunamadı"
    assert "mobilya_kutu" in modeller, "mobilya_kutu modeli SDF içinde bulunamadı"

    # gazebo_ros_state eklentisi kontrolü
    plugin = world.find("plugin[@name='gazebo_ros_state']")
    assert plugin is not None, "gazebo_ros_state eklentisi bulunamadı"
    assert plugin.get("filename") == "libgazebo_ros_state.so"
    assert plugin.find("ros/namespace").text == "/gazebo"

    # Işık sayısı kontrolü (isik_aralik=2.0 ile 4x4 oda içinde en az 1 iç ışık olmalı)
    isiklar = world.findall("light")
    assert len(isiklar) > 0, "İç mekân ışıkları üretilmedi"
    ic_isiklar = [l for l in isiklar if (l.get("name") or "").startswith("ic_isik")]
    assert len(ic_isiklar) > 0, "ic_isik elemanları bulunamadı"

    # Tavan yüksekliği parametresinin duvar boyuna yansıması kontrolü
    duvarlar_model = modeller["duvarlar"]
    duvar_linkleri = duvarlar_model.findall("link")
    assert len(duvar_linkleri) == 2, "2 adet duvar linki bekleniyordu"

    for link in duvar_linkleri:
        # collision box boyutu (uzunluk genislik yukseklik)
        box_size_el = link.find("collision/geometry/box/size")
        assert box_size_el is not None
        boyutlar = [float(val) for val in box_size_el.text.strip().split()]
        duvar_z_boyu = boyutlar[2]
        assert math.isclose(duvar_z_boyu, tavan_yuksekligi, abs_tol=1e-3), (
            f"Duvar yüksekliği ({duvar_z_boyu}) tavan parametresi ({tavan_yuksekligi}) ile uyuşmuyor"
        )

        # link pose z yüksekliği (tavan / 2.0 olmalıdır)
        pose_el = link.find("pose")
        assert pose_el is not None
        pose_degerler = [float(val) for val in pose_el.text.strip().split()]
        duvar_mz = pose_degerler[2]
        assert math.isclose(duvar_mz, tavan_yuksekligi / 2.0, abs_tol=1e-3), (
            f"Duvar merkez z ({duvar_mz}) tavan/2 ({tavan_yuksekligi / 2.0}) ile uyuşmuyor"
        )

    # Tavan modelinin yüksekliği kontrolü
    tavan_link = modeller["tavan"].find("link[@name='tavan_link']")
    assert tavan_link is not None
    tavan_pose = [float(val) for val in tavan_link.find("pose").text.strip().split()]
    assert math.isclose(tavan_pose[2], tavan_yuksekligi + 0.01, abs_tol=1e-3)


def test_oda_dunya_sadelestir(oda_dunya, tmp_path):
    """
    oda_dunya.mesh_sadelestir fonksiyonunun küçük bir küre mesh'ini
    hedef üçgen sayısına indirgediğini doğrular.
    """
    o3d = pytest.importorskip("open3d")

    # Sentetik küre üret
    kure = o3d.geometry.TriangleMesh.create_sphere(radius=0.5, resolution=20)
    baslangic_ucgen = len(kure.triangles)
    assert baslangic_ucgen > 100, f"Başlangıç üçgen sayısı beklenenden az: {baslangic_ucgen}"

    giris_obj = tmp_path / "kure_giris.obj"
    cikis_obj = tmp_path / "kure_sadelestirilmis.obj"
    o3d.io.write_triangle_mesh(str(giris_obj), kure)

    hedef_ucgen = 50
    oda_dunya.mesh_sadelestir(str(giris_obj), str(cikis_obj), hedef_ucgen=hedef_ucgen)

    assert cikis_obj.is_file(), "Sadeleştirilmiş OBJ dosyası oluşturulamadı"

    sade_mesh = o3d.io.read_triangle_mesh(str(cikis_obj))
    son_ucgen_sayisi = len(sade_mesh.triangles)

    assert son_ucgen_sayisi > 0, "Sadeleştirilmiş mesh boş olamaz"
    assert son_ucgen_sayisi <= hedef_ucgen, (
        f"Üçgen sayısı ({son_ucgen_sayisi}) hedeften ({hedef_ucgen}) büyük"
    )


def test_oda_dunya_ucgen_duvara_yapisik_atilir(oda_dunya):
    """Duvara yapışık üçgenin tespit edilip atıldığını (True) test eder."""
    duvarlar = [
        {"p1": [0.0, 0.0], "p2": [4.0, 0.0], "kalinlik": 0.15},
    ]
    # Duvar (0,0)-(4,0) boyunca uzanır, kalınlık 0.15, pay 0.08 (sınır y = 0.155)
    # y = 0.05 olan üçgen duvar yüzeyine yapışıktır (atılır -> True)
    ucgen_yapisik = np.array(
        [
            [1.0, 0.05, 1.0],
            [1.2, 0.05, 1.0],
            [1.1, 0.05, 1.2],
        ]
    )
    assert oda_dunya.ucgen_duvara_yapisik(ucgen_yapisik, duvarlar, pay=0.08) is True
    # Vektörel sürüm kontrolü
    maske = oda_dunya.ucgenler_duvara_yapisik(ucgen_yapisik[np.newaxis, ...], duvarlar, pay=0.08)
    assert bool(maske[0]) is True


def test_oda_dunya_ucgen_duvardan_uzak_kalir(oda_dunya):
    """Duvarın 0.5 m içerisindeki üçgenin atılmadığını / kaldığını (False) test eder."""
    duvarlar = [
        {"p1": [0.0, 0.0], "p2": [4.0, 0.0], "kalinlik": 0.15},
    ]
    # y = 0.50 olan üçgen duvar sınırından (0.155) 0.5 m içeridedir (kalır -> False)
    ucgen_uzak = np.array(
        [
            [1.0, 0.50, 1.0],
            [1.2, 0.50, 1.0],
            [1.1, 0.50, 1.2],
        ]
    )
    assert oda_dunya.ucgen_duvara_yapisik(ucgen_uzak, duvarlar, pay=0.08) is False
    # Vektörel sürüm kontrolü
    maske = oda_dunya.ucgenler_duvara_yapisik(ucgen_uzak[np.newaxis, ...], duvarlar, pay=0.08)
    assert bool(maske[0]) is False


def test_oda_dunya_duvar_obj_ve_isiksiz_malzeme(oda_dunya, tmp_path):
    """
    Sentetik 1 duvarlı yapı + doku YAML ile dünya üretir:
    - duvar_doku_0.obj üretilmiş olmalı
    - 4 ön yüz (+ 4 arka yüz) köşe ve UV satırları bulunmalı
    - Ön yüz üçgen normali iç normal yönünde olmalı
    - oda_isiksiz.material içinde "lighting off" geçmeli
    """
    yapi_dosyasi = tmp_path / "oda_yapi_1duvar.yaml"
    mesh_dizini = tmp_path / "mesh_dizin"
    doku_dizini = tmp_path / "doku_dizin"
    cikti_dunya = tmp_path / "cikti" / "isiksiz.world"

    mesh_dizini.mkdir(parents=True, exist_ok=True)
    (doku_dizini / "doku").mkdir(parents=True, exist_ok=True)
    (doku_dizini / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    (doku_dizini / "materials" / "scripts").mkdir(parents=True, exist_ok=True)

    # Sentetik PNG dosyaları
    sahte_png = b"\x89PNG\r\n\x1a\n"
    for png_ad in ("duvar_0.png", "zemin.png"):
        (doku_dizini / "doku" / png_ad).write_bytes(sahte_png)
        (doku_dizini / "materials" / "textures" / png_ad).write_bytes(sahte_png)

    tavan_yuk = 2.7
    sentetik_yapi = {
        "oda_siniri": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
        "duvarlar": [
            {"p1": [0.0, 0.0], "p2": [4.0, 0.0], "kalinlik": 0.15},
        ],
        "mobilya": [],
    }
    with open(yapi_dosyasi, "w", encoding="utf-8") as f:
        yaml.safe_dump(sentetik_yapi, f)

    ic_normal = [0.0, 1.0]
    sentetik_doku = {
        "duvarlar": [
            {
                "i": 0,
                "p1": [0.0, 0.0],
                "p2": [4.0, 0.0],
                "yuz_normali": ic_normal,
                "doku": "doku/duvar_0.png",
                "malzeme": "Oda/duvar_0",
            }
        ],
        "zemin": {
            "kutu": [0.0, 0.0, 4.0, 4.0],
            "doku": "doku/zemin.png",
            "malzeme": "Oda/zemin",
        },
        "tavan": tavan_yuk,
    }
    with open(doku_dizini / "doku.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(sentetik_doku, f)

    oda_dunya.dunya_uret(
        yapi_yol=str(yapi_dosyasi),
        mesh_dizin=str(mesh_dizini),
        cikti_yol=str(cikti_dunya),
        tavan=tavan_yuk,
        doku_dizin=str(doku_dizini),
    )

    duvar_obj_yolu = mesh_dizini / "duvar_doku_0.obj"
    assert duvar_obj_yolu.is_file(), "duvar_doku_0.obj oluşturulmadı"

    satirlar = [s.strip() for s in duvar_obj_yolu.read_text(encoding="utf-8").splitlines() if s.strip()]
    v_satirlar = [
        [float(x) for x in s.split()[1:4]] for s in satirlar if s.startswith("v ")
    ]
    vt_satirlar = [
        [float(x) for x in s.split()[1:3]] for s in satirlar if s.startswith("vt ")
    ]
    vn_satirlar = [
        [float(x) for x in s.split()[1:4]] for s in satirlar if s.startswith("vn ")
    ]
    f_satirlar = [s.split()[1:4] for s in satirlar if s.startswith("f ")]

    # Ön yüz 4 köşe (+ arka yüz 4 köşe) ve 4 UV satırı kontrolü
    assert len(v_satirlar) >= 4, "En az 4 köşe (v) satırı olmalı"
    assert len(vt_satirlar) == 4, "4 adet UV (vt) satırı olmalı"
    np.testing.assert_allclose(
        vt_satirlar, [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], atol=1e-5
    )

    # Ön yüz normali iç normal ([0, 1, 0]) yönünde olmalı
    n_ic_3b = np.array([ic_normal[0], ic_normal[1], 0.0], dtype=float)
    assert len(vn_satirlar) >= 1
    assert float(np.dot(vn_satirlar[0], n_ic_3b)) > 0.99

    assert len(f_satirlar) >= 2
    ilk_ucgen = [int(tok.split("/")[0]) - 1 for tok in f_satirlar[0]]
    p_a = np.array(v_satirlar[ilk_ucgen[0]])
    p_b = np.array(v_satirlar[ilk_ucgen[1]])
    p_c = np.array(v_satirlar[ilk_ucgen[2]])
    ucgen_normali = np.cross(p_b - p_a, p_c - p_a)
    assert float(np.dot(ucgen_normali, n_ic_3b)) > 0.0, "Ön yüz üçgen sarımı iç normal yönünde değil"

    # oda_isiksiz.material kontrolü
    isiksiz_mat_yolu = doku_dizini / "materials" / "scripts" / "oda_isiksiz.material"
    assert isiksiz_mat_yolu.is_file(), "oda_isiksiz.material oluşturulmadı"
    mat_icerik = isiksiz_mat_yolu.read_text(encoding="utf-8")
    assert "lighting off" in mat_icerik
    assert "Oda/isiksiz/duvar_0" in mat_icerik
    assert "Oda/isiksiz/zemin" in mat_icerik


# ==============================================================================
# 2. ODA_DOKU TESTLERİ
# ==============================================================================


def test_oda_doku_poz_ve_duzeltme_matrisi(oda_doku):
    """oda_doku poz ve düzeltme matrisi saf fonksiyonlarını doğrular."""
    # dx=0, dy=0, dyaw=0 iken kimlik matrisi dönmeli
    dT_kimlik = oda_doku.duzeltme_matrisi(dx=0.0, dy=0.0, dyaw_deg=0.0)
    np.testing.assert_allclose(dT_kimlik, np.eye(4), atol=1e-7)

    # dx, dy, dyaw verildiginde oteleme ve donme kontrolu
    dT_rot = oda_doku.duzeltme_matrisi(dx=0.15, dy=-0.25, dyaw_deg=90.0)
    assert math.isclose(dT_rot[0, 3], 0.15, abs_tol=1e-6)
    assert math.isclose(dT_rot[1, 3], -0.25, abs_tol=1e-6)
    # 90 derece: cos(90) ~ 0, sin(90) ~ 1
    assert math.isclose(dT_rot[0, 0], 0.0, abs_tol=1e-6)
    assert math.isclose(dT_rot[0, 1], -1.0, abs_tol=1e-6)
    assert math.isclose(dT_rot[1, 0], 1.0, abs_tol=1e-6)
    assert math.isclose(dT_rot[1, 1], 0.0, abs_tol=1e-6)

    # poz_matrisi kontrolü (x=1, y=2, z=3, yaw=0 -> q=[0, 0, 0, 1])
    satir = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    T = oda_doku.poz_matrisi(satir)
    beklenen_T = np.eye(4, dtype=np.float64)
    beklenen_T[:3, 3] = [1.0, 2.0, 3.0]
    np.testing.assert_allclose(T, beklenen_T, atol=1e-6)


def test_oda_doku_ic_normal_bul(oda_doku):
    """Duvarın oda_siniri içine bakan birim 2B normalini test eder."""
    oda_siniri = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]], dtype=np.float64)

    # Güney duvarı: (0,0) -> (4,0), odanın içi +Y yönündedir (normal [0, 1])
    n_guney = oda_doku.ic_normal_bul(
        np.array([0.0, 0.0], dtype=np.float64),
        np.array([4.0, 0.0], dtype=np.float64),
        oda_siniri,
    )
    np.testing.assert_allclose(n_guney, [0.0, 1.0], atol=1e-5)

    # Doğu duvarı: (4,0) -> (4,4), odanın içi -X yönündedir (normal [-1, 0])
    n_dogu = oda_doku.ic_normal_bul(
        np.array([4.0, 0.0], dtype=np.float64),
        np.array([4.0, 4.0], dtype=np.float64),
        oda_siniri,
    )
    np.testing.assert_allclose(n_dogu, [-1.0, 0.0], atol=1e-5)


def test_oda_doku_izdusum_ve_kare_izdusur(oda_doku):
    """
    Kamera izdüşümünün doğruluğunu test eder:
    1. Bilinen 3B nokta ve bilinen K ile beklenen piksel koordinatı hesabı.
    2. kare_izdusur fonksiyonunun teksel izdüşürme ve ağırlıklı biriktirme davranışı.
    """
    # 1. Saf matematiksel izdüşüm kontrolü
    # u = round(fx * X / Z + cx), v = round(fy * Y / Z + cy)
    fx, fy = 500.0, 500.0
    cx, cy = 320.0, 240.0
    X_c, Y_c, Z_c = 0.5, -0.25, 2.5

    u_hesap = int(round(fx * (X_c / Z_c) + cx))
    v_hesap = int(round(fy * (Y_c / Z_c) + cy))

    assert u_hesap == 420  # 500 * (0.5 / 2.5) + 320 = 100 + 320 = 420
    assert v_hesap == 190  # 500 * (-0.25 / 2.5) + 240 = -50 + 240 = 190

    # 2. kare_izdusur ile sentetik düzlem izdüşümü
    img_h, img_w = 480, 640
    bgr = np.zeros((img_h, img_w, 3), dtype=np.uint8)
    bgr[190, 420] = [120, 140, 160]  # Beklenen piksele renk yerleştir
    depth = np.full((img_h, img_w), 2.5, dtype=np.float32)

    # Kamera (0, 0, 2.5) konumunda -Z yönüne bakıyor.
    # Dünya çerçevesinde z=0 düzlemi (normal=[0,0,1]).
    # Kamera çerçevesinde: X_c = X_w, Y_c = -Y_w, Z_c = 2.5 - Z_w.
    R_cw = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=np.float64,
    )
    cam_pos = np.array([0.0, 0.0, 2.5], dtype=np.float64)
    t_cw = -R_cw @ cam_pos  # [0.0, 0.0, 2.5]

    # Tek bir tekselden oluşan sentetik zemin düzlemi (X_w=0.5, Y_w=0.25, Z_w=0.0)
    duzlem = {
        "tip": "zemin",
        "p0": np.array([0.0, 0.0, 0.0], dtype=np.float64),
        "normal": np.array([0.0, 0.0, 1.0], dtype=np.float64),
        "A": np.array([[0.5, 0.0, 0.0]], dtype=np.float64),
        "B": np.array([[0.0, 0.25, 0.0]], dtype=np.float64),
        "renk_top": np.zeros((1, 1, 3), dtype=np.float32),
        "agirlik_top": np.zeros((1, 1), dtype=np.float32),
        "mobilya_alti": None,
    }

    oda_doku.kare_izdusur(duzlem, bgr, depth, R_cw, t_cw, cam_pos, fx, fy, cx, cy)

    assert duzlem["agirlik_top"][0, 0] > 0.0, "Teksel görünür olmalı ve ağırlık birikmeli"
    biriken_renk = duzlem["renk_top"][0, 0] / duzlem["agirlik_top"][0, 0]
    np.testing.assert_allclose(biriken_renk, [120, 140, 160], atol=1e-3)


def test_oda_doku_mobilya_alti_maskesi(oda_doku):
    """mobilya_alti_maskesi_olustur fonksiyonunun çokgen içini True yaptığını test eder."""
    kutu = [0.0, 0.0, 10.0, 10.0]
    zemin_coz = 0.1
    H = 100
    W = 100

    # (2, 2) ile (6, 6) arasında sentetik mobilya çokgeni
    mobilya_listesi = [
        {
            "id": 1,
            "cokgen": [[2.0, 2.0], [6.0, 2.0], [6.0, 6.0], [2.0, 6.0]],
        }
    ]
    duvar_listesi = []

    maske = oda_doku.mobilya_alti_maskesi_olustur(
        kutu, zemin_coz, H, W, mobilya_listesi, duvar_listesi
    )

    assert maske.shape == (H, W)
    assert maske.dtype == bool

    # Mobilya çokgeninin tam içi: x=4.0, y=4.0
    # Piksel haritası: col = round((x - xmin) / coz), row = round((ymax - y) / coz)
    col_ic = int(round((4.0 - 0.0) / zemin_coz))
    row_ic = int(round((10.0 - 4.0) / zemin_coz))
    assert bool(maske[row_ic, col_ic]) is True, "Mobilya çokgeni içindeki piksel True olmalıdır"

    # Mobilyanın ve genişletme payının tamamen dışındaki bir nokta: x=9.0, y=1.0
    col_dis = int(round((9.0 - 0.0) / zemin_coz))
    row_dis = int(round((10.0 - 1.0) / zemin_coz))
    assert bool(maske[row_dis, col_dis]) is False, "Mobilya dışındaki piksel False olmalıdır"


# ==============================================================================
# 3. ODA_TSDF TESTLERİ
# ==============================================================================


def test_oda_tsdf_poz_duzeltme_kimlik(oda_tsdf):
    """
    oda_tsdf poz düzeltme fonksiyonunun dx=dy=dyaw=0 iken kimlik olduğunu
    (kamera pozunu değiştiremediğini) doğrular.
    """
    dT = oda_tsdf.duzeltme_matrisi(dx=0.0, dy=0.0, dyaw_deg=0.0)
    np.testing.assert_allclose(dT, np.eye(4), atol=1e-7)

    # Gövde ve kamera pozları
    pose_govde = np.array([2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    pose_kam = np.array([2.2, 3.0, 0.5, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    T_wb = oda_tsdf.poz_matrisi(pose_govde)
    T_wc = oda_tsdf.poz_matrisi(pose_kam)
    T_bc = np.linalg.inv(T_wb) @ T_wc
    T_wc_duz = T_wb @ dT @ T_bc

    # Düzeltme matrisi kimlik olduğu için T_wc_duz == T_wc olmalıdır
    np.testing.assert_allclose(T_wc_duz, T_wc, atol=1e-6)


def test_oda_tsdf_poz_duzeltme_dx_kaymasi(oda_tsdf):
    """
    dx=0.1 iken kamera konumunun gövde x ekseninde 0.1 kaydığını doğrular.
    """
    dx_miktari = 0.1
    dT = oda_tsdf.duzeltme_matrisi(dx=dx_miktari, dy=0.0, dyaw_deg=0.0)

    # 1. Senaryo: Robot oryantasyonu yaw = 0 (gövde x ekseni dünya +X ile paralel)
    pose_govde1 = np.array([1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    pose_kam1 = np.array([1.2, 2.0, 0.4, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    T_wb1 = oda_tsdf.poz_matrisi(pose_govde1)
    T_wc1 = oda_tsdf.poz_matrisi(pose_kam1)
    T_bc1 = np.linalg.inv(T_wb1) @ T_wc1
    T_wc1_duz = T_wb1 @ dT @ T_bc1

    delta_w1 = T_wc1_duz[:3, 3] - T_wc1[:3, 3]
    np.testing.assert_allclose(delta_w1, [dx_miktari, 0.0, 0.0], atol=1e-6)

    # 2. Senaryo: Robot 90 derece dönük (yaw = pi/2, gövde x ekseni dünya +Y yönünde)
    # qz = sin(pi/4) ~ 0.70710678, qw = cos(pi/4) ~ 0.70710678
    qw = float(np.cos(np.pi / 4))
    qz = float(np.sin(np.pi / 4))
    pose_govde2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, qz, qw], dtype=np.float64)
    pose_kam2 = np.array([0.0, 0.2, 0.5, 0.0, 0.0, qz, qw], dtype=np.float64)

    T_wb2 = oda_tsdf.poz_matrisi(pose_govde2)
    T_wc2 = oda_tsdf.poz_matrisi(pose_kam2)
    T_bc2 = np.linalg.inv(T_wb2) @ T_wc2
    T_wc2_duz = T_wb2 @ dT @ T_bc2

    # Dünya koordinatlarında kayma gövde x ekseni yönünde, yani +Y yönündedir
    delta_w2 = T_wc2_duz[:3, 3] - T_wc2[:3, 3]
    np.testing.assert_allclose(delta_w2, [0.0, dx_miktari, 0.0], atol=1e-6)

    # Gövde çerçevesine yansıtılan kayma ise her iki senaryoda da tam olarak [0.1, 0, 0]'dır
    R_wb2 = T_wb2[:3, :3]
    delta_govde2 = R_wb2.T @ delta_w2
    np.testing.assert_allclose(delta_govde2, [dx_miktari, 0.0, 0.0], atol=1e-6)


def test_oda_tsdf_obj_ve_mtl_kaydet(oda_tsdf, tmp_path):
    """oda_tsdf.obj_ve_mtl_kaydet fonksiyonunun geçerli OBJ ve MTL ürettiğini doğrular."""
    cikti_dizin = tmp_path / "tsdf_mesh"
    cikti_dizin.mkdir(parents=True, exist_ok=True)

    koseler = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    normaller = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    ucgenler = np.array([[0, 1, 2]], dtype=np.int32)
    etiketler = np.array([0], dtype=np.int32)
    merkezler = np.array([[0.8, 0.8, 0.8]], dtype=np.float32)

    oda_tsdf.obj_ve_mtl_kaydet(cikti_dizin, koseler, normaller, ucgenler, etiketler, merkezler)

    obj_yolu = cikti_dizin / "oda_mesh.obj"
    mtl_yolu = cikti_dizin / "oda_mesh.mtl"

    assert obj_yolu.is_file(), "oda_mesh.obj oluşturulmadı"
    assert mtl_yolu.is_file(), "oda_mesh.mtl oluşturulmadı"

    obj_icerik = obj_yolu.read_text(encoding="utf-8")
    assert "mtllib oda_mesh.mtl" in obj_icerik
    assert "usemtl m00" in obj_icerik
    assert "f 1//1 2//2 3//3" in obj_icerik

    mtl_icerik = mtl_yolu.read_text(encoding="utf-8")
    assert "newmtl m00" in mtl_icerik
    assert "Kd 0.8000 0.8000 0.8000" in mtl_icerik
