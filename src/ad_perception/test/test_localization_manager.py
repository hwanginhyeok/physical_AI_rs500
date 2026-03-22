"""LocalizationManager / GPSHealthMonitor Mock 노드 테스트."""

import math
from unittest.mock import MagicMock

import pytest

from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry


# ── Mock Node ─────────────────────────────────────────────────


class MockPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_mock_node():
    node = MagicMock()
    node.create_publisher = MagicMock(side_effect=lambda *a, **kw: MockPublisher())
    node.create_subscription = MagicMock(return_value=MagicMock())
    node.create_timer = MagicMock(return_value=MagicMock())
    return node


from ad_perception.localization_node import (
    GPSHealthConfig,
    GPSHealthMonitor,
    GPSHealthStatus,
    LocalizationConfig,
    LocalizationManager,
    LocalizationMode,
)


# ── Helper ────────────────────────────────────────────────────


def make_gps_fix(fix_type=0, covariance_type=2, var_x=1.0, var_y=1.0):
    msg = NavSatFix()
    msg.status.status = fix_type
    msg.position_covariance_type = covariance_type
    msg.position_covariance[0] = var_x
    msg.position_covariance[4] = var_y
    return msg


# ── GPSHealthMonitor ──────────────────────────────────────────


class TestGPSHealthMonitor:
    @pytest.fixture
    def monitor(self):
        config = GPSHealthConfig(consecutive_bad_count=3, consecutive_good_count=3)
        return GPSHealthMonitor(make_mock_node(), config)

    def test_initial_status_unavailable(self, monitor):
        assert monitor.status == GPSHealthStatus.UNAVAILABLE

    def test_decent_fix_transitions_to_degraded(self, monitor):
        monitor._gps_callback(make_gps_fix(fix_type=0, var_x=1.0, var_y=1.0))
        assert monitor.status == GPSHealthStatus.DEGRADED

    def test_good_fixes_transition_to_good(self, monitor):
        good = make_gps_fix(fix_type=2, var_x=0.1, var_y=0.1)
        for _ in range(5):
            monitor._gps_callback(good)
        assert monitor.status == GPSHealthStatus.GOOD

    def test_bad_fixes_degrade(self, monitor):
        # Get to GOOD first
        good = make_gps_fix(fix_type=2, var_x=0.1, var_y=0.1)
        for _ in range(5):
            monitor._gps_callback(good)
        # Send bad fixes
        bad = make_gps_fix(fix_type=-1, covariance_type=0)
        for _ in range(5):
            monitor._gps_callback(bad)
        assert monitor.status != GPSHealthStatus.GOOD

    def test_hdop_from_covariance(self, monitor):
        monitor._gps_callback(make_gps_fix(fix_type=0, var_x=4.0, var_y=4.0))
        assert monitor.last_hdop == pytest.approx(math.sqrt(8), abs=0.01)

    def test_seconds_since_fix_initially_inf(self, monitor):
        assert monitor.seconds_since_last_fix == float('inf')

    def test_seconds_since_fix_updates(self, monitor):
        monitor._gps_callback(make_gps_fix(fix_type=0))
        assert monitor.seconds_since_last_fix < 1.0

    def test_health_summary(self, monitor):
        summary = monitor.get_health_summary()
        assert 'status' in summary
        assert 'hdop' in summary
        assert 'num_satellites' in summary

    def test_on_status_change_callback(self, monitor):
        called = []
        monitor.on_status_change(lambda old, new: called.append((old, new)))
        monitor._gps_callback(make_gps_fix(fix_type=0, var_x=1.0, var_y=1.0))
        assert len(called) >= 1
        assert called[0][0] == GPSHealthStatus.UNAVAILABLE


# ── LocalizationManager ──────────────────────────────────────


class TestLocalizationManager:
    @pytest.fixture
    def manager(self):
        config = GPSHealthConfig(consecutive_bad_count=2, consecutive_good_count=2)
        return LocalizationManager(make_mock_node(), config)

    def test_initial_mode_slam_only(self, manager):
        assert manager.mode == LocalizationMode.SLAM_ONLY

    def test_gps_weight_slam_only(self, manager):
        assert manager.hybrid_gps_weight == 0.0

    def test_force_mode_gps_ekf(self, manager):
        manager.force_mode(LocalizationMode.GPS_EKF)
        assert manager.mode == LocalizationMode.GPS_EKF
        assert manager.hybrid_gps_weight == 1.0

    def test_force_mode_hybrid(self, manager):
        manager.force_mode(LocalizationMode.HYBRID)
        assert manager.mode == LocalizationMode.HYBRID

    def test_force_mode_slam_only(self, manager):
        manager.force_mode(LocalizationMode.GPS_EKF)
        manager.force_mode(LocalizationMode.SLAM_ONLY)
        assert manager.mode == LocalizationMode.SLAM_ONLY
        assert manager.hybrid_gps_weight == 0.0

    def test_transition_log(self, manager):
        manager.force_mode(LocalizationMode.GPS_EKF)
        manager.force_mode(LocalizationMode.SLAM_ONLY)
        assert len(manager.transition_log) >= 2

    def test_get_status(self, manager):
        status = manager.get_status()
        assert status['mode'] == 'SLAM_ONLY'
        assert 'gps_health' in status
        assert 'gps_weight' in status
        assert 'pose_offset' in status


# ── 포즈 블렌딩 ──────────────────────────────────────────────


class TestPoseBlending:
    @pytest.fixture
    def manager(self):
        return LocalizationManager(make_mock_node())

    def test_blend_full_gps(self, manager):
        slam, gps = Odometry(), Odometry()
        slam.pose.pose.position.x = 1.0
        slam.pose.pose.position.y = 2.0
        gps.pose.pose.position.x = 3.0
        gps.pose.pose.position.y = 4.0
        blended = manager._blend_poses(slam, gps, gps_weight=1.0)
        assert blended.pose.pose.position.x == pytest.approx(3.0, abs=0.01)
        assert blended.pose.pose.position.y == pytest.approx(4.0, abs=0.01)

    def test_blend_full_slam(self, manager):
        slam, gps = Odometry(), Odometry()
        slam.pose.pose.position.x = 1.0
        gps.pose.pose.position.x = 3.0
        blended = manager._blend_poses(slam, gps, gps_weight=0.0)
        assert blended.pose.pose.position.x == pytest.approx(1.0, abs=0.01)

    def test_blend_half(self, manager):
        slam, gps = Odometry(), Odometry()
        slam.pose.pose.position.x = 0.0
        gps.pose.pose.position.x = 10.0
        blended = manager._blend_poses(slam, gps, gps_weight=0.5)
        assert blended.pose.pose.position.x == pytest.approx(5.0, abs=0.01)


# ── 유틸리티 ──────────────────────────────────────────────────


class TestLocalizationUtils:
    def test_quaternion_to_yaw_zero(self):
        assert LocalizationManager._quaternion_to_yaw(0.0, 1.0) == pytest.approx(0.0)

    def test_quaternion_to_yaw_90deg(self):
        qz = math.sin(math.pi / 4)
        qw = math.cos(math.pi / 4)
        assert LocalizationManager._quaternion_to_yaw(qz, qw) == pytest.approx(
            math.pi / 2, abs=0.01)

    def test_normalize_angle_zero(self):
        assert LocalizationManager._normalize_angle(0.0) == pytest.approx(0.0)

    def test_normalize_angle_wraps(self):
        assert LocalizationManager._normalize_angle(3 * math.pi) == pytest.approx(
            math.pi, abs=0.01)
        assert LocalizationManager._normalize_angle(-3 * math.pi) == pytest.approx(
            -math.pi, abs=0.01)
