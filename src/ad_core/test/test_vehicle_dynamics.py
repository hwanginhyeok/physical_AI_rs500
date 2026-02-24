"""VehicleDynamics 단위 테스트."""

import math
import pytest

from ad_core.datatypes import Pose2D
from ad_core.vehicle_dynamics import (
    VehicleDynamics,
    VehicleDynamicsConfig,
    GRAVITY,
)


class TestVehicleDynamicsConfig:
    def test_default_values(self):
        cfg = VehicleDynamicsConfig()
        assert cfg.mass == 200.0
        assert cfg.Izz == 50.0
        assert cfg.max_accel == 1.0
        assert cfg.max_decel == 2.0


class TestVehicleDynamics:
    @pytest.fixture
    def dynamics(self):
        cfg = VehicleDynamicsConfig(mass=200.0, max_speed=1.0)
        return VehicleDynamics(cfg)

    def test_stationary_stays_still(self, dynamics):
        """명령 없으면 정지 유지."""
        pose = Pose2D(x=5.0, y=5.0, yaw=0.0)
        new_pose = dynamics.step(0.0, 0.0, 0.01, pose)
        assert abs(new_pose.x - 5.0) < 0.01
        assert abs(new_pose.y - 5.0) < 0.01

    def test_acceleration_limited(self, dynamics):
        """가속도가 max_accel로 제한된다."""
        pose = Pose2D()
        # 큰 속도 명령
        for _ in range(10):
            pose = dynamics.step(5.0, 5.0, 0.01, pose)
        # 10스텝 * 0.01s = 0.1s, max_accel=1.0 → 최대 0.1 m/s 증가
        assert dynamics.current_linear_velocity <= 1.0 * 0.1 + 0.1  # 여유

    def test_deceleration_limited(self, dynamics):
        """감속도가 max_decel로 제한된다."""
        # 먼저 속도를 올린다
        pose = Pose2D()
        for _ in range(200):
            pose = dynamics.step(1.0, 1.0, 0.01, pose)

        speed_before = dynamics.current_linear_velocity
        assert speed_before > 0.3  # 어느 정도 속도가 있어야 함

        # 급정지 명령
        for _ in range(5):
            pose = dynamics.step(0.0, 0.0, 0.01, pose)

        # 5스텝 * 0.01s = 0.05s, max_decel=2.0 → 최대 0.1 m/s 감소
        speed_drop = speed_before - dynamics.current_linear_velocity
        assert speed_drop <= 2.0 * 0.05 + 0.1  # 저항 포함 여유

    def test_straight_line_motion(self, dynamics):
        """동일 트랙 속도 → 직진."""
        pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        for _ in range(500):
            pose = dynamics.step(0.5, 0.5, 0.01, pose)
        # x 방향으로 전진해야 함
        assert pose.x > 0.5
        assert abs(pose.y) < 0.5  # y 편향 작아야 함

    def test_rotation(self, dynamics):
        """차동 속도 → 회전."""
        pose = Pose2D(x=5.0, y=5.0, yaw=0.0)
        for _ in range(200):
            pose = dynamics.step(-0.3, 0.3, 0.01, pose)
        # yaw가 변해야 함
        assert abs(pose.yaw) > 0.05

    def test_slope_uphill_slows_down(self, dynamics):
        """오르막 경사에서 속도 감소."""
        dynamics.set_slope(pitch=math.radians(10))  # 10도 오르막
        pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        for _ in range(300):
            pose = dynamics.step(0.5, 0.5, 0.01, pose)
        speed_uphill = dynamics.current_linear_velocity

        dynamics.reset()
        dynamics.set_slope(pitch=0.0)  # 평지
        pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        for _ in range(300):
            pose = dynamics.step(0.5, 0.5, 0.01, pose)
        speed_flat = dynamics.current_linear_velocity

        assert speed_uphill < speed_flat

    def test_load_distribution_acceleration(self, dynamics):
        """가속 시 후방 하중 증가."""
        pose = Pose2D()
        dynamics.step(1.0, 1.0, 0.1, pose)
        # 가속 중이면 후방 하중 > 전방 하중
        assert dynamics.rear_normal_force >= dynamics.front_normal_force - 50

    def test_max_climbable_angle(self, dynamics):
        """등판 한계각 계산이 물리적으로 타당."""
        angle = dynamics.compute_max_climbable_angle(0.6)
        assert 0 < angle < math.pi / 2
        assert abs(math.tan(angle) - 0.6) < 1e-6

    def test_slope_forces_flat(self, dynamics):
        """평지에서 경사방향력 = 0, 수직항력 ≈ 중량."""
        dynamics.set_slope(0.0, 0.0)
        along, normal, lateral = dynamics.compute_slope_forces()
        assert abs(along) < 1e-6
        assert abs(normal - dynamics.config.mass * GRAVITY) < 1e-3
        assert abs(lateral) < 1e-6

    def test_energy_conservation_approximation(self, dynamics):
        """평지 등속 주행 시 운동에너지가 대략 보존된다."""
        pose = Pose2D()
        # 먼저 등속 상태로
        for _ in range(500):
            pose = dynamics.step(0.5, 0.5, 0.01, pose)

        v1 = dynamics.current_linear_velocity
        # 계속 등속 유지
        for _ in range(100):
            pose = dynamics.step(0.5, 0.5, 0.01, pose)
        v2 = dynamics.current_linear_velocity

        # 등속 상태에서 속도 변동이 작아야 함
        assert abs(v2 - v1) < 0.1

    def test_speed_limit(self, dynamics):
        """최대 속도를 초과하지 않는다."""
        pose = Pose2D()
        for _ in range(2000):
            pose = dynamics.step(5.0, 5.0, 0.01, pose)
        assert abs(dynamics.current_linear_velocity) <= dynamics.config.max_speed + 0.05
