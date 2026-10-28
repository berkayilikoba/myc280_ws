#!/usr/bin/env python3
"""Dinamik Engel Yardımcı Fonksiyonları Birim Testleri (pytest, ROS'suz)."""

import math
import os
import sys
import pytest

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from dinamik_engel import (
    DEFAULT_FOOTPRINT,
    DEFAULT_CYLINDER_RADIUS,
    yaw_from_quat,
    wrap,
    seg_dist,
    point_inside_polygon,
    point_polygon_distance,
    transform_footprint,
    footprint_to_cylinder_clearance,
    center_distance,
    interpolate_trajectory,
    StopDetector,
    load_scenarios,
    format_terminal_table,
    transform_twist_to_world,
    compute_approach_velocity,
    classify_collision,
    calculate_stop_duration_before_contact
)


# ==============================================================================
# 1. Geometri ve Açı Yardımcıları Testleri
# ==============================================================================

def test_wrap():
    assert abs(wrap(0.0)) < 1e-6
    assert abs(wrap(math.pi) - (-math.pi)) < 1e-6 or abs(wrap(math.pi) - math.pi) < 1e-6
    assert abs(wrap(3 * math.pi) - math.pi) < 1e-5 or abs(wrap(3 * math.pi) - (-math.pi)) < 1e-5
    assert abs(wrap(math.radians(270)) - math.radians(-90)) < 1e-6


def test_seg_dist():
    # Doğru parçası: (0, 0) -> (2, 0)
    # Doğru üzerindeki nokta
    assert abs(seg_dist(1.0, 0.0, 0.0, 0.0, 2.0, 0.0)) < 1e-6
    # Doğruya dik nokta (orta kısım)
    assert abs(seg_dist(1.0, 1.5, 0.0, 0.0, 2.0, 0.0) - 1.5) < 1e-6
    # Başlangıç ucuna yakın nokta
    assert abs(seg_dist(-1.0, 0.0, 0.0, 0.0, 2.0, 0.0) - 1.0) < 1e-6
    # Bitiş ucuna yakın nokta
    assert abs(seg_dist(3.0, 0.0, 0.0, 0.0, 2.0, 0.0) - 1.0) < 1e-6
    # Sıfır uzunluklu segment
    assert abs(seg_dist(0.0, 3.0, 0.0, 0.0, 0.0, 0.0) - 3.0) < 1e-6


def test_point_inside_polygon():
    poly = [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)]
    assert point_inside_polygon(0.0, 0.0, poly) is True
    assert point_inside_polygon(0.5, 0.5, poly) is True
    assert point_inside_polygon(2.0, 0.0, poly) is False
    assert point_inside_polygon(0.0, 2.0, poly) is False


def test_transform_footprint():
    fp = [(-0.161, -0.11), (-0.161, 0.11), (0.161, 0.11), (0.161, -0.11)]

    # Konum: (0, 0, 0) -> Değişmemeli
    transformed_0 = transform_footprint(fp, (0.0, 0.0, 0.0))
    for (orig_x, orig_y), (tx, ty) in zip(fp, transformed_0):
        assert abs(orig_x - tx) < 1e-6
        assert abs(orig_y - ty) < 1e-6

    # Konum: (1.0, 2.0, 0.0) -> Öteleme
    transformed_t = transform_footprint(fp, (1.0, 2.0, 0.0))
    for (orig_x, orig_y), (tx, ty) in zip(fp, transformed_t):
        assert abs(tx - (orig_x + 1.0)) < 1e-6
        assert abs(ty - (orig_y + 2.0)) < 1e-6

    # Konum: (0, 0, pi/2) -> 90 derece dönüş: (x, y) -> (-y, x)
    transformed_r = transform_footprint(fp, (0.0, 0.0, math.pi / 2.0))
    for (orig_x, orig_y), (tx, ty) in zip(fp, transformed_r):
        assert abs(tx - (-orig_y)) < 1e-5
        assert abs(ty - orig_x) < 1e-5


# ==============================================================================
# 2. Ayak İzi - Silindir Açıklığı ve Çarpışma Testleri
# ==============================================================================

def test_footprint_to_cylinder_clearance_separated():
    # Robot (0, 0, 0) konumunda, ön kenar x = 0.161 m
    # Silindir merkezi (1.0, 0.0), yarıçap 0.20 m
    # Beklenen kenar uzaklığı = 1.0 - 0.161 = 0.839 m
    # Beklenen açıklık = 0.839 - 0.20 = 0.639 m
    clearance = footprint_to_cylinder_clearance(
        robot_pose=(0.0, 0.0, 0.0),
        footprint=DEFAULT_FOOTPRINT,
        cyl_center=(1.0, 0.0),
        cyl_radius=0.20
    )
    assert abs(clearance - 0.639) < 1e-5
    assert clearance > 0.0


def test_footprint_to_cylinder_clearance_touching():
    # Teğet temas: Kenar uzaklığı = 0.20 m
    # Silindir merkezi x = 0.161 + 0.20 = 0.361 m
    clearance = footprint_to_cylinder_clearance(
        robot_pose=(0.0, 0.0, 0.0),
        footprint=DEFAULT_FOOTPRINT,
        cyl_center=(0.361, 0.0),
        cyl_radius=0.20
    )
    assert abs(clearance) < 1e-5


def test_footprint_to_cylinder_clearance_collision():
    # Çarpışma: Kenar uzaklığı 0.10 m < yarıçap 0.20 m
    # Silindir merkezi x = 0.161 + 0.10 = 0.261 m
    # Beklenen açıklık = 0.10 - 0.20 = -0.10 m (penetrasyon)
    clearance = footprint_to_cylinder_clearance(
        robot_pose=(0.0, 0.0, 0.0),
        footprint=DEFAULT_FOOTPRINT,
        cyl_center=(0.261, 0.0),
        cyl_radius=0.20
    )
    assert clearance < 0.0
    assert abs(clearance - (-0.10)) < 1e-5


def test_footprint_to_cylinder_clearance_center_inside():
    # Silindir merkezi doğrudan robot merkezinde (0, 0)
    # Robot içine girdiğinde açıklık kesinlikle negatif olmalı
    clearance = footprint_to_cylinder_clearance(
        robot_pose=(0.0, 0.0, 0.0),
        footprint=DEFAULT_FOOTPRINT,
        cyl_center=(0.0, 0.0),
        cyl_radius=0.20
    )
    assert clearance < -0.20


def test_center_distance():
    d = center_distance((1.0, 2.0), (4.0, 6.0))
    assert abs(d - 5.0) < 1e-6


# ==============================================================================
# 3. Yörünge Enterpolasyonu Testleri
# ==============================================================================

def test_interpolate_trajectory_single_point():
    # Tek nokta (duran_engel: [(1.6, 0.3)])
    wps = [(1.6, 0.3)]
    x, y, yaw = interpolate_trajectory(wps, speed=0.0, elapsed_time=5.0)
    assert abs(x - 1.6) < 1e-6
    assert abs(y - 0.3) < 1e-6
    assert abs(yaw - 0.0) < 1e-6


def test_interpolate_trajectory_karsidan():
    # karsidan: [(4.5, 0.3), (0.5, 0.3)], hız: 0.6 m/s, gecikme: 0.0
    # Toplam mesafe = 4.0 m, yürüyüş yönü: -x ekseni (yaw = pi)
    wps = [(4.5, 0.3), (0.5, 0.3)]
    speed = 0.6

    # Başlangıç anı (t=0)
    x0, y0, yaw0 = interpolate_trajectory(wps, speed=speed, elapsed_time=0.0)
    assert abs(x0 - 4.5) < 1e-6
    assert abs(y0 - 0.3) < 1e-6
    assert abs(wrap(yaw0 - math.pi)) < 1e-5

    # 2 saniye sonra (d = 1.2 m -> x = 4.5 - 1.2 = 3.3)
    x2, y2, yaw2 = interpolate_trajectory(wps, speed=speed, elapsed_time=2.0)
    assert abs(x2 - 3.3) < 1e-5
    assert abs(y2 - 0.3) < 1e-5
    assert abs(wrap(yaw2 - math.pi)) < 1e-5

    # Yolun sonuna varıştan sonra (t=10.0 s, d = 6.0 m > 4.0 m) -> (0.5, 0.3)'te sabitlenmeli
    x_end, y_end, yaw_end = interpolate_trajectory(wps, speed=speed, elapsed_time=10.0)
    assert abs(x_end - 0.5) < 1e-5
    assert abs(y_end - 0.3) < 1e-5
    assert abs(wrap(yaw_end - math.pi)) < 1e-5


def test_interpolate_trajectory_yandan_kesen_delay():
    # yandan_kesen: [(1.5, 1.5), (1.5, -0.6)], hız: 0.8 m/s, gecikme: 2.0 s
    # Yürüyüş yönü: -y ekseni (yaw = -pi/2)
    wps = [(1.5, 1.5), (1.5, -0.6)]
    speed = 0.8
    delay = 2.0

    # Gecikme süresi içi (t=1.0 s) -> Başlangıç noktasında durmalı
    x1, y1, yaw1 = interpolate_trajectory(wps, speed=speed, elapsed_time=1.0, delay=delay)
    assert abs(x1 - 1.5) < 1e-6
    assert abs(y1 - 1.5) < 1e-6
    assert abs(yaw1 - (-math.pi / 2.0)) < 1e-5

    # Gecikme bitiş anı (t=2.0 s)
    x2, y2, yaw2 = interpolate_trajectory(wps, speed=speed, elapsed_time=2.0, delay=delay)
    assert abs(x2 - 1.5) < 1e-6
    assert abs(y2 - 1.5) < 1e-6

    # Hareket başladıktan 1 saniye sonra (t=3.0 s, d = 0.8 * 1.0 = 0.8 m -> y = 1.5 - 0.8 = 0.7)
    x3, y3, yaw3 = interpolate_trajectory(wps, speed=speed, elapsed_time=3.0, delay=delay)
    assert abs(x3 - 1.5) < 1e-5
    assert abs(y3 - 0.7) < 1e-5
    assert abs(yaw3 - (-math.pi / 2.0)) < 1e-5


def test_interpolate_trajectory_multi_segment():
    wps = [(0.0, 0.0), (2.0, 0.0), (2.0, 3.0)]
    speed = 1.0

    # Segment 1 üzerinde (t=1.0 s -> (1.0, 0.0), yaw = 0)
    x, y, yaw = interpolate_trajectory(wps, speed=speed, elapsed_time=1.0)
    assert abs(x - 1.0) < 1e-5
    assert abs(y - 0.0) < 1e-5
    assert abs(yaw - 0.0) < 1e-5

    # Segment 2 üzerinde (t=3.0 s -> segment 1 bitti (2m), segment 2'de 1m -> (2.0, 1.0), yaw = pi/2)
    x2, y2, yaw2 = interpolate_trajectory(wps, speed=speed, elapsed_time=3.0)
    assert abs(x2 - 2.0) < 1e-5
    assert abs(y2 - 1.0) < 1e-5
    assert abs(yaw2 - (math.pi / 2.0)) < 1e-5


def test_interpolate_trajectory_empty():
    with pytest.raises(ValueError):
        interpolate_trajectory([], speed=1.0, elapsed_time=1.0)


# ==============================================================================
# 4. Durdurma Sayacı (StopDetector) Testleri
# ==============================================================================

def test_stop_detector_cmd_vel_edges():
    det = StopDetector(vel_threshold=0.005)

    # Robot hareket ediyor: cmd_vel_smoothed != 0, cmd_vel != 0
    assert det.update_from_cmd_vel(0.2, 0.0, 0.2, 0.0) == 0

    # Collision monitor devreye girdi ve robota 0 hız bastı (durdurma 1)
    assert det.update_from_cmd_vel(0.2, 0.0, 0.0, 0.0) == 1

    # Durma durumu devam ediyor -> Sayı artmamalı (yükselen kenar)
    assert det.update_from_cmd_vel(0.2, 0.0, 0.0, 0.0) == 1
    assert det.update_from_cmd_vel(0.2, 0.0, 0.0, 0.0) == 1

    # Robot tekrar hareket etmeye başladı
    assert det.update_from_cmd_vel(0.2, 0.0, 0.2, 0.0) == 1

    # İkinci kez durduruldu (durdurma 2)
    assert det.update_from_cmd_vel(0.2, 0.0, 0.0, 0.0) == 2


def test_stop_detector_state_action():
    det = StopDetector()

    # DO_NOTHING = 0
    assert det.update_from_state(0) == 0

    # STOP = 1
    assert det.update_from_state(1) == 1

    # STOP devam ediyor
    assert det.update_from_state(1) == 1

    # Serbest kaldı
    assert det.update_from_state(0) == 1

    # Tekrar STOP
    assert det.update_from_state(1) == 2


def test_stop_detector_priority():
    det = StopDetector()
    # State mesajı kullanıldığında cmd_vel fallback dikkate alınmaz
    det.update_from_state(1)
    assert det.stop_count == 1

    # cmd_vel çağrılsa bile çift sayma yapmaz
    det.update_from_cmd_vel(0.2, 0.0, 0.0, 0.0)
    assert det.stop_count == 1


def test_stop_detector_reset():
    det = StopDetector()
    det.update_from_state(1)
    assert det.stop_count == 1

    det.reset()
    assert det.stop_count == 0
    assert det.is_stopped is False
    assert det.using_state_msg is False


# ==============================================================================
# 5. Senaryo YAML Okuma ve Çıktı Tablosu Testleri
# ==============================================================================

def test_load_scenarios_file():
    yaml_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'dinamik_engel_senaryolar.yaml'
    )
    assert os.path.exists(yaml_path), f"Senaryo dosyası bulunamadı: {yaml_path}"

    scenarios = load_scenarios(yaml_path)
    assert 'karsidan' in scenarios
    assert 'yandan_kesen' in scenarios
    assert 'duran_engel' in scenarios
    assert 'takip' in scenarios

    # karsidan senaryosu doğrulaması
    s_kar = scenarios['karsidan']
    assert s_kar['robot_hedef'] == (3.0, 0.3, 0.0)
    assert len(s_kar['waypoints']) == 2
    assert s_kar['waypoints'][0] == (4.5, 0.3)
    assert s_kar['waypoints'][1] == (0.5, 0.3)
    assert s_kar['speed'] == 0.6
    assert s_kar['delay'] == 0.0

    # yandan_kesen senaryosu doğrulaması
    s_yan = scenarios['yandan_kesen']
    assert s_yan['robot_hedef'] == (3.0, 0.3, 0.0)
    assert s_yan['waypoints'][0] == (1.5, 1.5)
    assert s_yan['waypoints'][1] == (1.5, -0.6)
    assert s_yan['speed'] == 0.8
    assert s_yan['delay'] == 2.0

    # duran_engel senaryosu doğrulaması
    s_dur = scenarios['duran_engel']
    assert s_dur['robot_hedef'] == (3.0, 0.3, 0.0)
    assert s_dur['waypoints'][0] == (1.6, 0.3)
    assert s_dur['speed'] == 0.0

    # takip senaryosu doğrulaması
    s_tak = scenarios['takip']
    assert s_tak['robot_hedef'] == (4.6, 0.8, 0.0)
    assert s_tak['waypoints'][0] == (1.0, 0.0)
    assert s_tak['waypoints'][1] == (4.0, 0.0)
    assert s_tak['speed'] == 0.25


def test_format_terminal_table():
    sample_results = [
        {
            'senaryo': 'karsidan',
            'status': 'SUCCEEDED',
            'sure_s': 12.4,
            'min_aciklik_m': 0.342,
            'carpisma': False,
            'min_merkez_mesafesi_m': 0.702,
            'durdurma_sayisi': 1,
            'kurtarma_sayisi': 0,
            'varis_hatasi_m': 0.045
        }
    ]
    table = format_terminal_table(sample_results)
    assert "karsidan" in table
    assert "SUCCEEDED" in table
    assert "0.342" in table
    assert "HAYIR" in table
    assert "0.702" in table
    assert "Sınıf" in table
    assert "Yaklaşma v (m/s)" in table


# ==============================================================================
# 6. F6 Görevi Hız Dönüşümü, Yaklaşma Hızı ve Kusur Ayrımı Testleri
# ==============================================================================

def test_transform_twist_to_world():
    # Yaw = 0 -> robot x yönü dünya x yönü ile aynı
    vx_w, vy_w = transform_twist_to_world(0.5, 0.0, 0.0)
    assert abs(vx_w - 0.5) < 1e-6
    assert abs(vy_w - 0.0) < 1e-6

    # Yaw = pi/2 (90 derece) -> ileri hız (vx) dünya +y yönüne döner
    vx_w90, vy_w90 = transform_twist_to_world(0.5, 0.0, math.pi / 2.0)
    assert abs(vx_w90 - 0.0) < 1e-5
    assert abs(vy_w90 - 0.5) < 1e-5

    # Yaw = pi (180 derece) -> ileri hız dünya -x yönüne döner
    vx_w180, vy_w180 = transform_twist_to_world(0.5, 0.0, math.pi)
    assert abs(vx_w180 - (-0.5)) < 1e-5
    assert abs(vy_w180 - 0.0) < 1e-5


def test_compute_approach_velocity():
    # Robot (0, 0), Engel (2, 0) -> robot engele doğru 0.3 m/s ile gidiyor
    v_yak, v_mag = compute_approach_velocity((0.0, 0.0), (0.3, 0.0), (2.0, 0.0))
    assert abs(v_yak - 0.3) < 1e-5
    assert abs(v_mag - 0.3) < 1e-5

    # Robot (0, 0), Engel (2, 0) -> robot engelden uzaklaşıyor (-0.2 m/s)
    v_yak_away, _ = compute_approach_velocity((0.0, 0.0), (-0.2, 0.0), (2.0, 0.0))
    assert abs(v_yak_away - (-0.2)) < 1e-5

    # Robot (0, 0), Engel (2, 0) -> robot dik hareket ediyor (0, 0.4 m/s)
    v_yak_perp, v_mag_perp = compute_approach_velocity((0.0, 0.0), (0.0, 0.4), (2.0, 0.0))
    assert abs(v_yak_perp - 0.0) < 1e-5
    assert abs(v_mag_perp - 0.4) < 1e-5

    # Robot duruyor (0, 0) -> yaklaşma hızı 0
    v_yak_stop, v_mag_stop = compute_approach_velocity((0.0, 0.0), (0.0, 0.0), (2.0, 0.0))
    assert abs(v_yak_stop) < 1e-6
    assert abs(v_mag_stop) < 1e-6


def test_classify_collision():
    # Çarpışma yoksa None döner
    assert classify_collision(0.2, 0.2, has_collision=False) is None

    # robot_carpti: temas anında v_yaklasma > 0.05 m/s
    assert classify_collision(0.10, 0.20, has_collision=True) == 'robot_carpti'
    assert classify_collision(0.06, 0.08, has_collision=True) == 'robot_carpti'

    # engel_carpti: robot |v| < 0.03 m/s ve v_yaklasma <= 0.05 m/s
    assert classify_collision(0.00, 0.01, has_collision=True) == 'engel_carpti'
    assert classify_collision(-0.01, 0.00, has_collision=True) == 'engel_carpti'
    assert classify_collision(0.02, 0.025, has_collision=True) == 'engel_carpti'

    # karisik: robot hareket halinde (|v| >= 0.03) ancak v_yaklasma <= 0.05 m/s
    # Örneğin robot dik dönüyor veya kaçıyor
    assert classify_collision(0.02, 0.15, has_collision=True) == 'karisik'
    assert classify_collision(-0.10, 0.20, has_collision=True) == 'karisik'
    assert classify_collision(0.05, 0.03, has_collision=True) == 'karisik'


def test_calculate_stop_duration_before_contact():
    # Senaryo 1: Robot temas anından (t=10.0s) önce 2.5 saniyedir duruyor (t=7.5'ten itibaren v=0.0)
    history_stopped_2_5s = [
        (5.0, 0.3),
        (7.0, 0.1),
        (7.4, 0.05),
        (7.5, 0.01),
        (8.0, 0.00),
        (9.0, 0.00),
        (9.5, 0.01),
        (10.0, 0.01),
    ]
    durma_s, durmus_2s = calculate_stop_duration_before_contact(history_stopped_2_5s, contact_time=10.0)
    assert durma_s >= 2.5
    assert durmus_2s is True

    # Senaryo 2: Robot temastan önce yalnızca 0.8 saniyedir duruyor (t=9.2'den itibaren v=0.0)
    history_stopped_short = [
        (8.0, 0.3),
        (9.0, 0.1),
        (9.2, 0.01),
        (9.6, 0.00),
        (10.0, 0.00),
    ]
    durma_s_short, durmus_2s_short = calculate_stop_duration_before_contact(history_stopped_short, contact_time=10.0)
    assert abs(durma_s_short - 0.8) < 1e-2
    assert durmus_2s_short is False

    # Senaryo 3: Robot temas anında hareket halinde (v=0.2 m/s)
    history_moving = [
        (8.0, 0.3),
        (9.0, 0.25),
        (10.0, 0.20),
    ]
    durma_s_mov, durmus_2s_mov = calculate_stop_duration_before_contact(history_moving, contact_time=10.0)
    assert abs(durma_s_mov - 0.0) < 1e-2
    assert durmus_2s_mov is False


def test_format_terminal_table_with_classification():
    sample_results = [
        {
            'senaryo': 'karsidan',
            'status': 'SUCCEEDED',
            'sure_s': 15.2,
            'min_aciklik_m': -0.015,
            'carpisma': True,
            'carpisma_sinifi': 'engel_carpti',
            'v_yaklasma_m_s': 0.008,
            'min_merkez_mesafesi_m': 0.385,
            'durdurma_sayisi': 1,
            'kurtarma_sayisi': 3,
            'varis_hatasi_m': 0.05
        },
        {
            'senaryo': 'yandan_kesen',
            'status': 'SUCCEEDED',
            'sure_s': 18.0,
            'min_aciklik_m': -0.068,
            'carpisma': True,
            'carpisma_sinifi': 'robot_carpti',
            'v_yaklasma_m_s': 0.124,
            'min_merkez_mesafesi_m': 0.332,
            'durdurma_sayisi': 9,
            'kurtarma_sayisi': 15,
            'varis_hatasi_m': 0.08
        }
    ]
    table = format_terminal_table(sample_results)
    assert "engel_carpti" in table
    assert "robot_carpti" in table
    assert "0.008" in table
    assert "0.124" in table


if __name__ == '__main__':
    sys.exit(pytest.main([__file__]))

