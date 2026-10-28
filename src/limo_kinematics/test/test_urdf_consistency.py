"""Faz 1 sözleşmesi: simülasyon URDF'leri gerçek robotla aynı geometriyi tanımlamalı.

Hedef değerler ve kaynakları:
  * dingil mesafesi 0.200, iz 0.172, iç direksiyon sınırı 0.48869 (28°)
      -> limo_base/include/limo_base/limo_driver.h:130-132
  * tekerlek yarıçapı 0.045 -> tüm URDF'lerde zaten böyle, Gazebo wheel_diameter 0.09
  * sensör konumları -> robottaki ROS 1 limo_start.launch (static_transform_publisher)
      laser : 0.105 0 0.08, YAW -0.15  (konumsal argüman sırası x y z yaw pitch roll;
              tf2_echo ile ölçüldü: RPY [0, 0, -0.150])
      imu   : 0 0 0
  * kamera -> resmi AgileX URDF'i (limo_ros1 limo_description): 0.084 0 0.03, eğimsiz; 09.10.2026
      sahada doğrulandı (saha/config/kamera_kalibrasyon.yaml). Çerçeve adı camera_link.
  * diff modeli 27.10.2026'dan beri sahada ölçülen lidar/D435 konumlarını kullanır (SENSORS_DIFF)

Bu testler xacro'yu çalıştırır; ROS ortamı ve workspace source edilmiş olmalı.
"""

import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

from limo_kinematics.geometry import load_geometry
from limo_kinematics.kinematics import min_turning_radius_rear_axle, radius_at_offset

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# ad -> (xacro dosyası, xacro argümanları). arm_*: limo_sim_nav2'nin yüklediği kollu model.
ARM_XACRO = os.path.join(SRC, 'limo_description', 'urdf', 'limo_mycobot.xacro')
ACKERMANN_SOURCES = {
    'sim_ackermann': (os.path.join(SRC, 'limo_car', 'gazebo', 'ackermann_with_sensor.xacro'), []),
    'arm_ackermann': (ARM_XACRO, ['mode:=ackermann']),
}
DIFF_SOURCES = {
    'description_four_diff': (os.path.join(SRC, 'limo_description', 'urdf', 'limo_four_diff.xacro'), []),
    'arm_diff': (ARM_XACRO, ['mode:=diff']),
}
ALL_SOURCES = dict(ACKERMANN_SOURCES, **DIFF_SOURCES)

L, T, INNER, WHEEL_R = 0.200, 0.172, 0.48869, 0.045
TOL = 1e-4

SENSORS = {
    'laser_link': ((0.105, 0.0, 0.08), (0.0, 0.0, -0.15)),
    'camera_link': ((0.084, 0.0, 0.03), (0.0, 0.0, 0.0)),
    'imu_link': ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
}
# Diff (gerçek robotun modu, FOUR_DIFF) 27.10.2026'dan beri sahada ölçülen değerleri kullanır:
#   lidar: tarama-harita ICP (saha/analiz/lidar_kalibrasyon.py): x 0.137, y 0.015, yaw -0.1686; z AgileX 0.08
#   kamera: D435, saha/config/kamera_kalibrasyon_d435.yaml (0.084, 0.0175, 0.028; roll 0.0015, pitch -0.0191)
SENSORS_DIFF = {
    'laser_link': ((0.137, 0.015, 0.08), (0.0, 0.0, -0.1686)),
    'camera_link': ((0.084, 0.0175, 0.028), (0.0015, -0.0191, 0.0)),
    'imu_link': ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
}

pytestmark = pytest.mark.skipif(shutil.which('xacro') is None,
                                reason='xacro yok — ROS ortamını source edin')


def _expand(name, src, args, tmp_path_factory):
    out = tmp_path_factory.mktemp('urdf') / (name + '.urdf')
    res = subprocess.run(['xacro', src, *args], capture_output=True, text=True)
    assert res.returncode == 0, f'xacro başarısız: {src}\n{res.stderr}'
    out.write_text(res.stdout)
    return str(out)


@pytest.fixture(scope='module')
def urdfs(tmp_path_factory):
    return {k: _expand(k, src, args, tmp_path_factory) for k, (src, args) in ALL_SOURCES.items()}


def _root(path):
    return ET.parse(path).getroot()


def _floats(s, n=3):
    vals = [float(v) for v in (s or '').split()]
    return tuple(vals) if vals else (0.0,) * n


def _joint_to(root, child):
    for j in root.findall('joint'):
        if j.find('child').get('link') == child:
            return j
    return None


# ---------------------------------------------------------------- geometri

@pytest.mark.parametrize('name', sorted(ACKERMANN_SOURCES))
def test_ackermann_geometry_matches_driver(urdfs, name):
    g = load_geometry(urdfs[name])
    assert g.wheelbase == pytest.approx(L, abs=TOL), name
    assert g.track == pytest.approx(T, abs=TOL), name
    assert g.max_inner_steer == pytest.approx(INNER, abs=TOL), name
    assert g.wheel_radius == pytest.approx(WHEEL_R, abs=TOL), name
    assert g.rear_axle_x == pytest.approx(-L / 2, abs=TOL), name


def test_ackermann_models_match_theoretical_turning_radius(urdfs):
    radii = []
    for name in ACKERMANN_SOURCES:
        g = load_geometry(urdfs[name])
        r = radius_at_offset(min_turning_radius_rear_axle(g.wheelbase, g.track, g.max_inner_steer),
                             abs(g.rear_axle_x))
        radii.append(r)
    assert radii == pytest.approx([0.47282] * len(radii), abs=1e-3)


@pytest.mark.parametrize('name', sorted(DIFF_SOURCES))
def test_four_diff_wheel_positions(urdfs, name):
    root = _root(urdfs[name])
    wheels = [j for j in root.findall('joint') if j.get('type') == 'continuous']
    assert len(wheels) == 4
    xs = sorted(round(_floats(j.find('origin').get('xyz'))[0], 4) for j in wheels)
    ys = sorted(round(abs(_floats(j.find('origin').get('xyz'))[1]), 4) for j in wheels)
    assert xs == pytest.approx([-L / 2, -L / 2, L / 2, L / 2], abs=TOL)
    assert ys == pytest.approx([T / 2] * 4, abs=TOL)


# ---------------------------------------------------------------- sensörler

@pytest.mark.parametrize('name, link', [(n, l) for n in sorted(ACKERMANN_SOURCES) for l in sorted(SENSORS)] +
                         [('description_four_diff', l) for l in sorted(SENSORS_DIFF)])
def test_sensor_pose_matches_field_calibration(urdfs, name, link):
    root = _root(urdfs[name])
    j = _joint_to(root, link)
    assert j is not None, f'{name}: {link} yok'
    assert j.find('parent').get('link') == 'base_link', f'{name}: {link} base_link altında değil'
    o = j.find('origin')
    xyz, rpy = (SENSORS_DIFF if name == 'description_four_diff' else SENSORS)[link]
    assert _floats(o.get('xyz')) == pytest.approx(xyz, abs=TOL), f'{name}:{link} xyz'
    assert _floats(o.get('rpy')) == pytest.approx(rpy, abs=TOL), f'{name}:{link} rpy'


@pytest.mark.parametrize('name', sorted(ALL_SOURCES))
def test_old_camera_frame_name_is_gone(urdfs, name):
    names = {l.get('name') for l in _root(urdfs[name]).findall('link')}
    assert 'depth_camera_link' not in names


@pytest.mark.parametrize('name', sorted(ALL_SOURCES))
def test_gazebo_references_point_to_existing_links(urdfs, name):
    root = _root(urdfs[name])
    links = {l.get('name') for l in root.findall('link')}
    refs = {g.get('reference') for g in root.findall('gazebo') if g.get('reference')}
    assert refs <= links, f'{name}: tanımsız link\'e gazebo referansı: {sorted(refs - links)}'


# ---------------------------------------------------------------- Gazebo eklentisi

@pytest.mark.parametrize('name', sorted(ACKERMANN_SOURCES))
def test_sim_ackermann_plugin_matches_driver(urdfs, name):
    xml = open(urdfs[name]).read()
    root = ET.fromstring(xml)
    plugin = next(p for p in root.iter('plugin') if 'ackermann_drive' in (p.get('filename') or ''))
    assert float(plugin.find('max_steer').text) == pytest.approx(INNER, abs=TOL)
    assert float(plugin.find('wheel_separation').text) == pytest.approx(T, abs=TOL)
    assert float(plugin.find('wheel_diameter').text) == pytest.approx(2 * WHEEL_R, abs=TOL)


# ---------------------------------------------------------------- bütünlük

@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='check_urdf yok')
@pytest.mark.parametrize('name', sorted(ALL_SOURCES))
def test_check_urdf_passes(urdfs, name):
    res = subprocess.run(['check_urdf', urdfs[name]], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr + res.stdout
    assert 'root Link: base_footprint' in res.stdout


def test_four_diff_d435_frames_match_realsense_driver(urdfs):
    """Diff sim kamerası gerçek realsense2_camera sürücüsünün çerçeve ağacını ve kayıttaki fabrika dış kalibrasyonunu
    (camera_link -> camera_color_frame y +0.0148) kullanmalı; algı/VIO kodu sim ve gerçekte aynı çerçeveleri görür."""
    root = _root(urdfs['description_four_diff'])
    for child, parent, xyz in (('camera_depth_frame', 'camera_link', (0, 0, 0)),
                               ('camera_color_frame', 'camera_link', (0, 0.0148, 0.0002)),
                               ('camera_depth_optical_frame', 'camera_depth_frame', (0, 0, 0)),
                               ('camera_color_optical_frame', 'camera_color_frame', (0, 0, 0))):
        j = _joint_to(root, child)
        assert j is not None and j.find('parent').get('link') == parent, child
        assert _floats(j.find('origin').get('xyz')) == pytest.approx(xyz, abs=TOL), child
