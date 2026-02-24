"""DrivetrainModel 단위 테스트."""

import math
import pytest

from ad_core.drivetrain_model import (
    DrivetrainConfig,
    DrivetrainModel,
    SingleMotorModel,
)


class TestSingleMotorModel:
    @pytest.fixture
    def motor(self):
        return SingleMotorModel(DrivetrainConfig(time_constant=0.15, deadzone=0.05))

    def test_deadzone(self, motor):
        """데드존 이하 명령은 무반응."""
        for _ in range(100):
            motor.update(0.03, 0.01)  # < deadzone(0.05)
        assert abs(motor.actual_velocity) < 0.01

    def test_deadzone_boundary(self, motor):
        """데드존 초과 명령은 반응."""
        for _ in range(200):
            motor.update(0.1, 0.01)
        assert motor.actual_velocity > 0.01

    def test_step_response_convergence(self, motor):
        """스텝 입력에 대해 목표 속도에 수렴한다."""
        target = 0.5
        for _ in range(500):
            motor.update(target, 0.01)
        # 5 * tau = 0.75s → 500 * 0.01 = 5s 충분히 수렴
        assert abs(motor.actual_velocity - target * motor.config.gear_efficiency_forward) < 0.15

    def test_no_overshoot(self, motor):
        """1차 지연 모델이므로 오버슈트가 없어야 한다."""
        target = 0.5
        max_seen = 0.0
        for _ in range(500):
            motor.update(target, 0.01)
            max_seen = max(max_seen, motor.actual_velocity)
        # 오버슈트 없음: 최대값이 target을 크게 넘지 않아야 함
        assert max_seen <= target + 0.05

    def test_reverse_direction(self, motor):
        """역방향 명령 처리."""
        for _ in range(300):
            motor.update(-0.3, 0.01)
        assert motor.actual_velocity < -0.01

    def test_reset(self, motor):
        """리셋 후 속도 0."""
        motor.update(0.5, 0.1)
        motor.reset()
        assert motor.actual_velocity == 0.0

    def test_motor_torque_at_stall(self, motor):
        """정지 상태에서 최대 토크."""
        torque = motor.get_motor_torque(0.0)
        cfg = motor.config
        expected = cfg.max_motor_torque * cfg.gear_ratio * cfg.gear_efficiency_forward
        assert abs(torque - expected) < 1e-3

    def test_motor_torque_at_max_speed(self, motor):
        """최대 속도에서 토크 ≈ 0."""
        max_speed = motor._max_track_speed()
        torque = motor.get_motor_torque(max_speed)
        assert torque < 1.0  # 거의 0


class TestDrivetrainModel:
    @pytest.fixture
    def drivetrain(self):
        return DrivetrainModel(DrivetrainConfig(time_constant=0.10))

    def test_symmetric_response(self, drivetrain):
        """좌/우 동일 명령이면 동일 응답."""
        for _ in range(200):
            drivetrain.update(0.5, 0.5, 0.01)
        left, right = drivetrain.actual_velocities
        assert abs(left - right) < 1e-6

    def test_independent_motors(self):
        """좌/우 다른 설정 시 비대칭 응답."""
        left_cfg = DrivetrainConfig(time_constant=0.10)
        right_cfg = DrivetrainConfig(time_constant=0.30)  # 느린 우측
        dt = DrivetrainModel(left_config=left_cfg, right_config=right_cfg)

        for _ in range(50):
            dt.update(0.5, 0.5, 0.01)

        left, right = dt.actual_velocities
        # 좌측이 더 빨리 수렴하므로 더 큰 값
        assert left > right

    def test_reset(self, drivetrain):
        drivetrain.update(0.5, 0.5, 0.1)
        drivetrain.reset()
        left, right = drivetrain.actual_velocities
        assert left == 0.0
        assert right == 0.0

    def test_total_traction_force(self, drivetrain):
        """정지 상태에서 양쪽 견인력이 0 이상."""
        force = drivetrain.get_total_traction_force(0.0, 0.0)
        assert force > 0
