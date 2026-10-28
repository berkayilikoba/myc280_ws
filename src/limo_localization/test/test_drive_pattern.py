"""
drive_pattern birim testleri: yardımcı fonksiyonlar ve güvenlik durdurmaları.

Düğüm gerçek robotu sürdüğü için yayıncı sahte bir nesneyle değiştirilir ve tick()
doğrudan çağrılır; hiçbir şey ROS ağına yayınlanmaz.
"""

import math

from geometry_msgs.msg import Quaternion
from limo_localization.drive_pattern import DrivePattern, wrap, yaw_of
import pytest
import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan


class FakePub:

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    @property
    def last(self):
        return self.msgs[-1]


def make_scan(ranges, angle_min=-math.pi / 2, angle_max=math.pi / 2, range_min=0.1):
    m = LaserScan()
    m.angle_min = angle_min
    m.angle_max = angle_max
    m.angle_increment = (angle_max - angle_min) / (len(ranges) - 1)
    m.range_min = range_min
    m.range_max = 12.0
    m.ranges = [float(r) for r in ranges]
    return m


def is_zero(twist):
    return twist.linear.x == 0.0 and twist.angular.z == 0.0


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = DrivePattern()
    n.pub = FakePub()
    yield n
    n.destroy_node()


def start_step(node, index, odom=(0.0, 0.0, 0.0), scan=None):
    """Düğümü index numaralı adımın başına getirir (duraklama bitmiş, sensörler taze)."""
    node.odom = odom
    node.last = odom
    node.scan = scan if scan is not None else make_scan([5.0] * 181)
    node.scan_time = node.get_clock().now()
    node.step = index
    node.travelled = 0.0
    node.step_t0 = node.get_clock().now().nanoseconds * 1e-9


# ---------------------------------------------------------------- yardımcılar

@pytest.mark.parametrize('a, expected', [
    (0.0, 0.0),
    (3 * math.pi / 2, -math.pi / 2),
    (-3 * math.pi / 2, math.pi / 2),
    (2 * math.pi + 0.1, 0.1),
])
def test_wrap(a, expected):
    assert wrap(a) == pytest.approx(expected)


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.5, math.pi - 1e-6])
def test_yaw_of_roundtrip(yaw):
    q = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    assert yaw_of(q) == pytest.approx(yaw, abs=1e-9)


def test_min_range_only_looks_inside_sector(node):
    ranges = [5.0] * 181                 # -90..90°, 1° aralık
    ranges[0] = 0.3                      # -90°: sektör dışında
    ranges[100] = 1.2                    # +10°: sektör içinde
    node.scan = make_scan(ranges)
    assert node.min_range(math.radians(30)) == pytest.approx(1.2)
    assert node.min_range(math.pi) == pytest.approx(0.3)


def test_min_range_ignores_invalid_readings(node):
    ranges = [5.0] * 181
    ranges[90] = float('inf')
    ranges[91] = float('nan')
    ranges[92] = 0.05                    # range_min (0.1) altında: gövde/gürültü
    ranges[93] = 0.0
    node.scan = make_scan(ranges)
    assert node.min_range(math.radians(30)) == pytest.approx(5.0)


def test_min_range_empty_sector_is_inf(node):
    node.scan = make_scan([float('inf')] * 181)
    assert node.min_range(math.radians(30)) == math.inf


# ---------------------------------------------------------------- güvenlik

def test_waits_without_sensors(node):
    node.tick()
    assert node.pub.msgs == []
    assert not node.done


def test_stale_scan_stops(node):
    start_step(node, 1)
    node.scan_time = node.get_clock().now() - Duration(seconds=1.0)
    node.tick()
    assert node.done
    assert is_zero(node.pub.last)


def test_forward_obstacle_stops(node):
    ranges = [5.0] * 181
    ranges[90] = node.safety_dist - 0.05     # tam önde
    start_step(node, 1, scan=make_scan(ranges))
    node.tick()
    assert node.done
    assert is_zero(node.pub.last)


def test_forward_ignores_side_obstacle(node):
    ranges = [5.0] * 181
    ranges[0] = 0.2                          # -90°: ön sektör dışında
    start_step(node, 1, scan=make_scan(ranges))
    node.tick()
    assert not node.done
    assert node.pub.last.linear.x == pytest.approx(node.lin_speed)
    assert node.pub.last.angular.z == 0.0


def test_turn_obstacle_stops(node):
    ranges = [5.0] * 181
    ranges[0] = node.turn_safety_dist - 0.05
    start_step(node, 0, scan=make_scan(ranges))
    node.tick()
    assert node.done
    assert is_zero(node.pub.last)


def test_turn_publishes_only_angular(node):
    start_step(node, 0)
    node.tick()
    assert not node.done
    assert node.pub.last.angular.z == pytest.approx(node.ang_speed)
    assert node.pub.last.linear.x == 0.0


def test_step_timeout_stops(node):
    start_step(node, 1)
    node.step_t0 -= 2.0 * node.distance / node.lin_speed + 6.0
    node.tick()
    assert node.done
    assert is_zero(node.pub.last)


# ---------------------------------------------------------------- adım ilerleyişi

def test_forward_step_completes_at_distance(node):
    start_step(node, 1)
    node.odom = (node.distance + 0.01, 0.0, 0.0)
    node.tick()
    assert not node.done
    assert is_zero(node.pub.last)
    assert node.pause_until is not None
    assert node.travelled == pytest.approx(node.distance + 0.01)


def test_turn_accumulates_across_wrap(node):
    # 170° -> -170° geçişi 20°'lik dönüştür, 340° değil
    start_step(node, 2, odom=(0.0, 0.0, math.radians(170)))
    node.odom = (0.0, 0.0, math.radians(-170))
    node.tick()
    assert node.travelled == pytest.approx(math.radians(20))


def test_last_step_finishes(node):
    start_step(node, len(node.steps) - 1)
    node.pause_until = 0.0                   # duraklama bitti
    node.tick()
    assert node.done
    assert is_zero(node.pub.last)


def test_pattern_returns_to_start(node):
    # desen geri gitmez ve başlangıç pozuna döner: net yer değiştirme ve dönüş sıfır
    x = y = th = 0.0
    for kind, v in node.steps:
        assert v > 0
        if kind == 'fwd':
            x += v * math.cos(th)
            y += v * math.sin(th)
        else:
            th += v
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert wrap(th) == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------- dikdörtgen deseni

@pytest.fixture
def rect():
    n = DrivePattern()
    n.pattern = 'dikdortgen'
    n.steps = [('fwd', 2.0), ('turn', math.pi / 2), ('fwd', 1.0), ('turn', math.pi / 2)] * 2
    n.pub = FakePub()
    yield n
    n.destroy_node()


def advance(node, odom):
    """Duraklamayı bitirip bir sonraki adıma geçirir."""
    node.odom = odom
    node.scan = make_scan([5.0] * 181)
    node.scan_time = node.get_clock().now()
    node.pause_until = 0.0
    node.tick()


def test_rect_start_sets_goal_heading(rect):
    rect.odom = (0.0, 0.0, 0.3)
    rect.scan = make_scan([5.0] * 181)
    rect.scan_time = rect.get_clock().now()
    rect.tick()
    assert rect.goal == pytest.approx(0.3)


def test_rect_forward_holds_heading(rect):
    rect.goal = 0.0
    advance(rect, (0.0, 0.0, 0.0))           # adım 0: ileri
    rect.odom = (0.1, 0.0, 0.05)             # sola sapmış
    rect.tick()
    assert rect.pub.last.linear.x == pytest.approx(rect.lin_speed)
    assert rect.pub.last.angular.z < 0.0
    assert abs(rect.pub.last.angular.z) <= rect.max_heading_rate


def test_rect_turn_compensates_previous_overshoot(rect):
    rect.goal = 0.0
    rect.step = 0
    advance(rect, (2.0, 0.0, math.radians(3)))   # adım 1: dönüş; yön 3° fazla
    assert rect.goal == pytest.approx(math.pi / 2)
    assert rect.target == pytest.approx(math.radians(87))


def test_rect_turn_slows_near_goal(rect):
    rect.goal = 0.0
    rect.step = 0
    advance(rect, (2.0, 0.0, 0.0))
    rect.odom = (2.0, 0.0, math.radians(85))
    rect.tick()
    assert rect.pub.last.angular.z == pytest.approx(rect.slow_turn_speed)
