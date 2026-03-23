"""ControlModule Mock 노드 테스트 - ROS2 의존 없이 제어 로직 검증."""

import math
from unittest.mock import MagicMock

import pytest

from geometry_msgs.msg import Twist


# ── Mock Node ─────────────────────────────────────────────────


class MockParameter:
    def __init__(self, value):
        self.value = value


class MockPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_mock_node(params=None):
    defaults = {
        'topics.cmd_vel': '/cmd_vel',
        'control.speed_kp': 1.0,
        'control.speed_ki': 0.01,
        'control.speed_kd': 0.1,
        'control.steer_kp': 1.5,
        'control.calibration_file': None,
        'control.calibration_ema_alpha': 0.05,
        'control.calibration_enabled': False,
    }
    if params:
        defaults.update(params)

    node = MagicMock()
    node.get_parameter = lambda name: MockParameter(defaults.get(name, None))
    publisher = MockPublisher()
    node.create_publisher = MagicMock(return_value=publisher)
    node._mock_publisher = publisher
    return node


from ad_control.control_node import ControlModule, VehicleState


# ── 초기화 ────────────────────────────────────────────────────


class TestControlModuleInit:
    def test_basic_init(self):
        cm = ControlModule(make_mock_node())
        assert cm.slip_compensation_enabled is False
        assert cm.tracker is not None
        assert cm.slip_tracker is None

    def test_slip_compensation_init(self):
        cm = ControlModule(make_mock_node(), enable_slip_compensation=True)
        assert cm.slip_compensation_enabled is True
        assert cm.slip_tracker is not None

    def test_publisher_created(self):
        node = make_mock_node()
        ControlModule(node)
        node.create_publisher.assert_called_once()

    def test_calibration_disabled(self):
        cm = ControlModule(make_mock_node({'control.calibration_enabled': False}))
        assert cm.skid_model.calibration_enabled is False


# ── 경로 설정 ─────────────────────────────────────────────────


class TestSetPath:
    @pytest.fixture
    def cm(self):
        return ControlModule(make_mock_node())

    def test_set_valid_path(self, cm):
        waypoints = [(0, 0), (5, 0), (10, 0)]
        cm.set_path(waypoints)
        assert cm._path_tracking_active is True
        assert len(cm.tracker.path) == 3

    def test_set_empty_path(self, cm):
        cm.set_path([])
        assert cm._path_tracking_active is False


# ── compute_control ───────────────────────────────────────────


class TestComputeControl:
    @pytest.fixture
    def cm(self):
        return ControlModule(make_mock_node())

    def test_idle_returns_zero(self, cm):
        v_left, v_right = cm.compute_control(VehicleState())
        assert v_left == 0.0
        assert v_right == 0.0

    def test_path_tracking_produces_output(self, cm):
        cm.set_path([(0, 0), (10, 0)])
        v_left, v_right = cm.compute_control(
            VehicleState(x=0.0, y=0.0, yaw=0.0, speed=0.5))
        assert not (v_left == 0.0 and v_right == 0.0)

    def test_goal_reached_stops(self, cm):
        cm.set_path([(0, 0), (0.1, 0)])
        cm.compute_control(VehicleState(x=0.1, y=0.0, yaw=0.0, speed=0.0))
        v_left, v_right = cm.compute_control(VehicleState(x=0.1, y=0.0))
        assert v_left == 0.0
        assert v_right == 0.0

    def test_updates_current_speed(self, cm):
        cm.compute_control(VehicleState(speed=1.5, steering=0.3))
        assert cm._current_speed == 1.5
        assert cm._current_steering == 0.3


# ── PID 속도 제어 ─────────────────────────────────────────────


class TestPIDSpeed:
    @pytest.fixture
    def cm(self):
        return ControlModule(make_mock_node({
            'control.speed_kp': 1.0, 'control.speed_ki': 0.0,
            'control.speed_kd': 0.0,
        }))

    def test_zero_error(self, cm):
        cm._current_speed = 1.0
        assert cm._pid_speed(1.0) == pytest.approx(0.0, abs=1e-6)

    def test_positive_error(self, cm):
        cm._current_speed = 0.0
        assert cm._pid_speed(1.0) > 0.0

    def test_clamps_negative(self, cm):
        cm._current_speed = 2.0
        assert cm._pid_speed(0.0) == 0.0

    def test_integral_accumulates(self):
        cm = ControlModule(make_mock_node({
            'control.speed_kp': 0.0, 'control.speed_ki': 1.0,
            'control.speed_kd': 0.0,
        }))
        cm._current_speed = 0.0
        cm._pid_speed(1.0)
        assert cm._pid_speed(1.0) > 1.0

    def test_derivative_responds(self):
        cm = ControlModule(make_mock_node({
            'control.speed_kp': 0.0, 'control.speed_ki': 0.0,
            'control.speed_kd': 1.0,
        }))
        cm._current_speed = 0.0
        out1 = cm._pid_speed(1.0)
        cm._current_speed = 0.5
        out2 = cm._pid_speed(1.0)
        assert out2 < out1


# ── P 조향 제어 ───────────────────────────────────────────────


class TestPSteering:
    @pytest.fixture
    def cm(self):
        return ControlModule(make_mock_node({'control.steer_kp': 1.5}))

    def test_zero_error(self, cm):
        cm._current_steering = 0.5
        assert cm._p_steering(0.5) == pytest.approx(0.0, abs=1e-6)

    def test_proportional_response(self, cm):
        cm._current_steering = 0.0
        assert cm._p_steering(1.0) == pytest.approx(1.5, abs=1e-6)


# ── execute (Twist 퍼블리시) ──────────────────────────────────


class TestExecute:
    def test_execute_publishes_twist_pid(self):
        node = make_mock_node()
        cm = ControlModule(node)
        cm.execute({'target_speed': 1.0, 'target_steering': 0.5})
        assert len(node._mock_publisher.messages) == 1
        assert node._mock_publisher.messages[0].linear.x > 0.0

    def test_execute_path_tracking_publishes(self):
        node = make_mock_node()
        cm = ControlModule(node)
        cm.set_path([(0, 0), (10, 0)])
        cm.execute({'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'current_speed': 0.5})
        assert len(node._mock_publisher.messages) == 1


# ── 슬립 보상 모드 ────────────────────────────────────────────


class TestSlipCompensation:
    @pytest.fixture
    def cm(self):
        return ControlModule(make_mock_node(), enable_slip_compensation=True)

    def test_set_terrain(self, cm):
        cm.set_terrain('ASPHALT')
        cm.set_terrain('MUD')

    def test_get_slip_info(self, cm):
        assert cm.get_slip_info() is not None

    def test_slip_path_tracking(self, cm):
        cm.set_path([(0, 0), (10, 0)])
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, speed=0.5,
                             odom_x=0.0, odom_y=0.0, odom_yaw=0.0,
                             terrain_name='GRASS')
        v_left, v_right = cm.compute_control(state)
        assert not (v_left == 0.0 and v_right == 0.0)

    def test_no_slip_info_without_slip_mode(self):
        cm = ControlModule(make_mock_node())
        assert cm.get_slip_info() is None


# ── 캘리브레이션 ──────────────────────────────────────────────


class TestCalibration:
    def test_get_calibration_status(self):
        cm = ControlModule(make_mock_node({'control.calibration_enabled': True}))
        assert 'track_width_correction' in cm.get_calibration_status()

    def test_reset_calibration(self):
        cm = ControlModule(make_mock_node({'control.calibration_enabled': True}))
        cm._calibration_update_count = 50
        cm.reset_calibration()
        assert cm._calibration_update_count == 0

    def test_reset_pid(self):
        cm = ControlModule(make_mock_node())
        cm._speed_error_integral = 10.0
        cm._speed_error_prev = 5.0
        cm.reset_pid()
        assert cm._speed_error_integral == 0.0
        assert cm._speed_error_prev == 0.0
