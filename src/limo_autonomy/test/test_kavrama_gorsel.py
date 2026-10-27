"""test_kavrama_gorsel: kavrama görselleştirme saf yardımcıları ve işaretçi testleri."""

import math

import numpy as np
import pytest

from limo_autonomy.kavrama_gorsel import (
    AYAK_IZI_DX,
    AYAK_IZI_DY,
    KORIDOR_GENISLIK,
    VARSAYILAN_D_KAVRAMA,
    ayak_izi_koseleri,
    guvenlik_konisi_noktalari,
    guvenlik_konisinde_engel_var_mi,
    kavrama_noktasi_hesapla,
    koridor_koseleri,
    marker_ayak_izi,
    marker_durum,
    marker_guvenlik_konisi,
    marker_kavrama_noktasi,
    marker_koridor,
    marker_nesne,
    marker_ok,
    marker_sil_hepsi,
    quaternion_from_yaw,
    renk_secimi,
    wrap_angle,
    yaw_q,
    KavramaGorsel,
)

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


# ============================================================================
# 1. Temel Açı ve Quaternion Testleri
# ============================================================================

def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(math.radians(190)) == pytest.approx(math.radians(-170))


def test_quaternion_yaw_donusumu():
    for aci_deg in [-180, -90, -45, 0, 30, 60, 90, 135, 180]:
        yaw_beklenen = math.radians(aci_deg)
        qx, qy, qz, qw = quaternion_from_yaw(yaw_beklenen)
        q_obj = Quaternion(x=qx, y=qy, z=qz, w=qw)
        yaw_hesaplanan = yaw_q(q_obj)
        fark = abs(wrap_angle(yaw_hesaplanan - yaw_beklenen))
        assert fark == pytest.approx(0.0, abs=1e-6)


# ============================================================================
# 2. Koridor Köşeleri Saf Yardımcı Testleri
# ============================================================================

def test_koridor_koseleri_yatay():
    """Q=(0, 0) -> P=(1, 0) yatay koridor test edilir."""
    q = (0.0, 0.0)
    p = (1.0, 0.0)
    koseler = koridor_koseleri(q, p, genislik=KORIDOR_GENISLIK)
    assert len(koseler) == 4

    h = KORIDOR_GENISLIK / 2.0  # 0.16 m
    # 0: Q sağ (0, -0.16)
    # 1: P sağ (1, -0.16)
    # 2: P sol (1, +0.16)
    # 3: Q sol (0, +0.16)
    assert koseler[0] == pytest.approx((0.0, -h), abs=1e-6)
    assert koseler[1] == pytest.approx((1.0, -h), abs=1e-6)
    assert koseler[2] == pytest.approx((1.0, h), abs=1e-6)
    assert koseler[3] == pytest.approx((0.0, h), abs=1e-6)

    # Koridor genişliği
    w_q = math.hypot(koseler[3][0] - koseler[0][0], koseler[3][1] - koseler[0][1])
    w_p = math.hypot(koseler[2][0] - koseler[1][0], koseler[2][1] - koseler[1][1])
    assert w_q == pytest.approx(KORIDOR_GENISLIK, abs=1e-6)
    assert w_p == pytest.approx(KORIDOR_GENISLIK, abs=1e-6)


def test_koridor_koseleri_egik():
    """45 derece eğik Q->P koridorunun simetrisi ve genişliği doğrulanır."""
    q = (1.0, 2.0)
    p = (1.0 + 2.0 * math.cos(math.pi / 4), 2.0 + 2.0 * math.sin(math.pi / 4))
    koseler = koridor_koseleri(q, p, genislik=0.32)

    # Q noktasında iki köşe arası mesafe genişliğe eşit olmalı
    d_q = math.hypot(koseler[3][0] - koseler[0][0], koseler[3][1] - koseler[0][1])
    assert d_q == pytest.approx(0.32, abs=1e-6)

    # P noktasında iki köşe arası mesafe
    d_p = math.hypot(koseler[2][0] - koseler[1][0], koseler[2][1] - koseler[1][1])
    assert d_p == pytest.approx(0.32, abs=1e-6)


# ============================================================================
# 3. Ayak İzi Köşeleri Saf Yardımcı Testleri
# ============================================================================

def test_ayak_izi_koseleri_orijin():
    """Robot (0, 0, 0) pozundayken ±0.161 x ±0.11 dikdörtgeni oluşur."""
    pos = (0.0, 0.0)
    yaw = 0.0
    koseler = ayak_izi_koseleri(pos, yaw)
    assert len(koseler) == 4
    # Ön-Sol (+dx, +dy)
    assert koseler[0] == pytest.approx((AYAK_IZI_DX, AYAK_IZI_DY), abs=1e-6)
    # Arka-Sol (-dx, +dy)
    assert koseler[1] == pytest.approx((-AYAK_IZI_DX, AYAK_IZI_DY), abs=1e-6)
    # Arka-Sağ (-dx, -dy)
    assert koseler[2] == pytest.approx((-AYAK_IZI_DX, -AYAK_IZI_DY), abs=1e-6)
    # Ön-Sağ (+dx, -dy)
    assert koseler[3] == pytest.approx((AYAK_IZI_DX, -AYAK_IZI_DY), abs=1e-6)


def test_ayak_izi_koseleri_donme_ve_oteleme():
    """Robot (2.0, 3.0, 90 derece) pozundayken ayak izi döner ve ötelenir."""
    pos = (2.0, 3.0)
    yaw = math.radians(90.0)
    koseler = ayak_izi_koseleri(pos, yaw)

    # 90 derece dönüşte x_harita = pos_x - y_robot, y_harita = pos_y + x_robot
    # Ön-Sol: lx=0.161, ly=0.11 -> (2 - 0.11, 3 + 0.161) = (1.89, 3.161)
    assert koseler[0] == pytest.approx((2.0 - AYAK_IZI_DY, 3.0 + AYAK_IZI_DX), abs=1e-6)

    # Kenar uzunlukları korunmalı
    uzunluk = math.hypot(koseler[0][0] - koseler[1][0], koseler[0][1] - koseler[1][1])
    genislik = math.hypot(koseler[1][0] - koseler[2][0], koseler[1][1] - koseler[2][1])
    assert uzunluk == pytest.approx(2.0 * AYAK_IZI_DX, abs=1e-6)
    assert genislik == pytest.approx(2.0 * AYAK_IZI_DY, abs=1e-6)


# ============================================================================
# 4. Kavrama Noktası Hesaplama ve Nesneyle Çakışma Testleri
# ============================================================================

def test_kavrama_noktasi_hesapla():
    """Robot önünde d_kavrama=0.22 mesafedeki kavrama noktası hesaplanır."""
    # Yaw = 0 -> x + 0.22, y
    kx, ky = kavrama_noktasi_hesapla((1.0, 2.0), 0.0, d_kavrama=0.22)
    assert kx == pytest.approx(1.22, abs=1e-6)
    assert ky == pytest.approx(2.00, abs=1e-6)

    # Yaw = 90 deg -> x, y + 0.22
    kx, ky = kavrama_noktasi_hesapla((1.0, 2.0), math.radians(90.0), d_kavrama=0.22)
    assert kx == pytest.approx(1.00, abs=1e-6)
    assert ky == pytest.approx(2.22, abs=1e-6)


def test_kavrama_noktasi_nesneyle_cakisir():
    """P pozu nesneden d_kavrama kadar geride tanımlandığında,

    P'deki kavrama noktası nesne konumuyla tam olarak çakışır.
    """
    nesne = (2.50, 1.80)
    theta = math.radians(35.0)
    d_kavrama = 0.22
    yanal = 0.0

    # kavrama_yaklasim tanımı: P = nesne - d_kavrama * u(theta)
    u = (math.cos(theta), math.sin(theta))
    P = (nesne[0] - d_kavrama * u[0], nesne[1] - d_kavrama * u[1])

    # P'de robot önündeki kavrama noktasını hesapla
    kx, ky = kavrama_noktasi_hesapla(P, theta, d_kavrama=d_kavrama, yanal=yanal)
    assert kx == pytest.approx(nesne[0], abs=1e-6)
    assert ky == pytest.approx(nesne[1], abs=1e-6)


# ============================================================================
# 5. Renk Seçimi Saf Yardımcı Testleri
# ============================================================================

def test_renk_secimi():
    # 'nesne' (algı varsa yeşil, yoksa turuncu)
    r_algi = renk_secimi('nesne', durum_veya_algili=True)
    assert r_algi[1] > 0.8 and r_algi[0] < 0.2  # Yeşil

    r_hedef = renk_secimi('nesne', durum_veya_algili=False)
    assert r_hedef[0] > 0.8 and r_hedef[2] < 0.2  # Turuncu

    # 'Q' mavi, 'P' yeşil
    r_q = renk_secimi('Q')
    assert r_q[2] > 0.8  # Mavi
    r_p = renk_secimi('P')
    assert r_p[1] > 0.8  # Yeşil

    # 'koridor' yarı saydam (alpha <= 0.5)
    r_koridor = renk_secimi('koridor')
    assert r_koridor[3] <= 0.5

    # 'durum' (HATA kırmızı, HAZIR yeşil, diğer sarı/beyaz)
    r_hata = renk_secimi('durum', 'HATA engelli yol')
    assert r_hata[0] > 0.8 and r_hata[1] < 0.3  # Kırmızı

    r_hazir = renk_secimi('durum', 'HAZIR nesne robotta')
    assert r_hazir[1] > 0.8 and r_hazir[0] < 0.3  # Yeşil

    r_nav = renk_secimi('durum', 'NAV_Q aday 1/3')
    assert r_nav[0] > 0.7 and r_nav[1] > 0.7  # Sarı

    # 'guvenlik_konisi'
    r_koni_engel = renk_secimi('guvenlik_konisi', durum_veya_algili=True)
    assert r_koni_engel[0] > 0.8  # Kırmızı
    r_koni_bos = renk_secimi('guvenlik_konisi', durum_veya_algili=False)
    assert r_koni_bos[0] > 0.8 and r_koni_bos[1] > 0.5  # Turuncu/Sarı


# ============================================================================
# 6. Güvenlik Konisi Noktaları ve Engel Denetimi Testleri
# ============================================================================

def test_guvenlik_konisi_noktalari():
    noktalar = guvenlik_konisi_noktalari(r=0.50, koni_derece=25.0, adim_derece=5.0)
    assert len(noktalar) >= 5
    # İlk ve son nokta (0, 0)
    assert noktalar[0] == (0.0, 0.0)
    assert noktalar[-1] == (0.0, 0.0)

    # Yay üzerindeki tüm noktaların yarıçapı 0.50 m olmalıdır
    for pt in noktalar[1:-1]:
        d = math.hypot(pt[0], pt[1])
        assert d == pytest.approx(0.50, abs=1e-5)
        aci = math.atan2(pt[1], pt[0])
        assert abs(math.degrees(aci)) <= (25.0 + 1e-4)


def test_guvenlik_konisinde_engel_denetimi():
    # 360 derecelik 360 ışınlı yapay lidar (0.50 rad increment ~= 1 deg)
    angle_min = -math.pi
    angle_inc = math.radians(1.0)
    n_rays = 360

    # 1. Engel yokken (tüm ölçümler 2.0 m)
    ranges_bos = [2.0] * n_rays
    assert guvenlik_konisinde_engel_var_mi(ranges_bos, angle_min, angle_inc) is False

    # 2. Ön koni içinde 0.30 m mesafede engel (0 radyan civarı)
    idx_on = int((-angle_min) / angle_inc)
    ranges_engelli = list(ranges_bos)
    ranges_engelli[idx_on] = 0.30
    assert guvenlik_konisinde_engel_var_mi(ranges_engelli, angle_min, angle_inc) is True

    # 3. Koni açısı dışında (45 derece yanda) 0.30 m'de engel -> koni içi değil
    idx_yan = int((-angle_min + math.radians(45)) / angle_inc)
    ranges_yan = list(ranges_bos)
    ranges_yan[idx_yan] = 0.30
    assert guvenlik_konisinde_engel_var_mi(ranges_yan, angle_min, angle_inc) is False

    # 4. Ön koni içinde ama 0.50 m sınırının ötesinde (0.70 m) engel
    ranges_uzak = list(ranges_bos)
    ranges_uzak[idx_on] = 0.70
    assert guvenlik_konisinde_engel_var_mi(ranges_uzak, angle_min, angle_inc) is False


# ============================================================================
# 7. ROS Marker Üretim Fonksiyonları Testleri
# ============================================================================

def test_marker_nesne_ve_ok():
    stamp = Header().stamp

    # Nesne (silindir, r=0.03 -> çap=0.06)
    m_nesne = marker_nesne((1.0, 2.0, 0.45), algi_var=True, stamp=stamp)
    assert m_nesne.ns == 'nesne'
    assert m_nesne.type == Marker.CYLINDER
    assert m_nesne.scale.x == pytest.approx(0.06)
    assert m_nesne.scale.y == pytest.approx(0.06)
    assert m_nesne.pose.position.x == 1.0
    assert m_nesne.pose.position.y == 2.0
    assert m_nesne.pose.position.z == 0.45
    assert m_nesne.color.g > 0.8  # Yeşil

    # Q oku (mavi) ve P oku (yeşil)
    m_q = marker_ok('Q', (0.5, 0.5), 0.0, stamp=stamp)
    assert m_q.ns == 'Q'
    assert m_q.type == Marker.ARROW
    assert m_q.color.b > 0.8

    m_p = marker_ok('P', (1.0, 1.0), math.radians(45.0), stamp=stamp)
    assert m_p.ns == 'P'
    assert m_p.type == Marker.ARROW
    assert m_p.color.g > 0.8


def test_marker_koridor_ve_ayak_izi():
    stamp = Header().stamp

    # Koridor (TRIANGLE_LIST)
    m_koridor = marker_koridor((0.0, 0.0), (1.0, 0.0), stamp=stamp)
    assert m_koridor.ns == 'koridor'
    assert m_koridor.type == Marker.TRIANGLE_LIST
    assert len(m_koridor.points) == 12  # 4 üçgen (çift taraflı)
    assert m_koridor.color.a == pytest.approx(0.35, abs=1e-2)

    # Ayak izi (LINE_STRIP)
    m_ayak = marker_ayak_izi((1.0, 1.0), 0.0, stamp=stamp)
    assert m_ayak.ns == 'ayak_izi_P'
    assert m_ayak.type == Marker.LINE_STRIP
    assert len(m_ayak.points) == 5  # Kapalı dikdörtgen


def test_marker_kavrama_noktasi_ve_durum():
    stamp = Header().stamp

    # Kavrama noktası (SPHERE)
    m_kav = marker_kavrama_noktasi((1.0, 1.0), 0.0, stamp=stamp, d_kavrama=0.22, z=0.30)
    assert m_kav.ns == 'kavrama_noktasi'
    assert m_kav.type == Marker.SPHERE
    assert m_kav.pose.position.x == pytest.approx(1.22, abs=1e-6)
    assert m_kav.pose.position.z == pytest.approx(0.30, abs=1e-6)

    # Durum (TEXT_VIEW_FACING, z=0.50)
    m_durum = marker_durum('HAZIR tamamlandi', stamp=stamp)
    assert m_durum.ns == 'durum'
    assert m_durum.type == Marker.TEXT_VIEW_FACING
    assert m_durum.pose.position.z == pytest.approx(0.50)
    assert m_durum.text == 'HAZIR tamamlandi'
    assert m_durum.color.g > 0.8  # HAZIR -> yeşil


def test_marker_guvenlik_konisi_ve_sil_hepsi():
    stamp = Header().stamp

    # Güvenlik konisi görünürken
    m_koni = marker_guvenlik_konisi(stamp=stamp, gorunur=True, engel_var=False)
    assert m_koni.ns == 'guvenlik_konisi'
    assert m_koni.action == Marker.ADD
    assert len(m_koni.points) > 5

    # Güvenlik konisi gizlenirken
    m_koni_gizli = marker_guvenlik_konisi(stamp=stamp, gorunur=False)
    assert m_koni_gizli.action == Marker.DELETE

    # Sil hepsi
    m_sil = marker_sil_hepsi(stamp=stamp)
    assert m_sil.action == Marker.DELETEALL


# ============================================================================
# 8. Düğüm Mantığı ve Döngü Testleri
# ============================================================================

def test_kavrama_gorsel_dugum_arayuz():
    """KavramaGorsel düğümünün abonelik ve timer döngüsünü simüle eder."""
    # rclpy başlat / kapat
    rclpy.init()
    node = KavramaGorsel()
    try:
        # 1. Plan alımı
        pa = PoseArray()
        pa.header.frame_id = 'map'
        q_pose = Pose()
        q_pose.position.x = 0.5
        q_pose.position.y = 1.0
        q_pose.orientation.w = 1.0

        p_pose = Pose()
        p_pose.position.x = 1.0
        p_pose.position.y = 1.0
        p_pose.orientation.w = 1.0
        pa.poses = [q_pose, p_pose]

        node._plan_cb(pa)
        assert node.plan_q is not None
        assert node.plan_p is not None

        # 2. Hedef alımı
        pt_hedef = PointStamped()
        pt_hedef.header.frame_id = 'map'
        pt_hedef.point.x = 1.22
        pt_hedef.point.y = 1.00
        pt_hedef.point.z = 0.35
        node._hedef_cb(pt_hedef)
        assert node.hedef_nokta == (1.22, 1.00, 0.35)

        # 3. Durum geçişi
        str_msg = String(data='SON_YAKLASMA v=0.08 w=0.00 rho=0.15 m')
        node._durum_cb(str_msg)
        assert node.durum_kodu == 'SON_YAKLASMA'

        # 4. Döngü çalıştırma ve üretilen işaretçilerin kontrolü
        node._dongu_5hz()

        # PLAN durumuna dönüşte temizlik kontrolü
        str_plan = String(data='PLAN yeni hedef')
        node._durum_cb(str_plan)
        assert node.durum_kodu == 'PLAN'
        assert node.plan_q is None
        assert node.plan_p is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
