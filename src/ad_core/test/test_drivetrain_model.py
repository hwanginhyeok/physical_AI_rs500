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


class TestSingleMotorModelSSSD:
    """MD2K SS/SD 가감속 프로파일 테스트."""

    @pytest.fixture
    def motor_ss(self):
        """SS=2s, SD=1s 설정."""
        return SingleMotorModel(DrivetrainConfig(
            slow_start_time=2.0,
            slow_down_time=1.0,
            time_constant=0.15,
            deadzone=0.05,
        ))

    def test_ss_ramp_acceleration(self, motor_ss):
        """SS 설정 시 선형 램프 가속."""
        # 2초간 가속
        for _ in range(200):
            motor_ss.update(1.0, 0.01)
        v_at_2s = motor_ss.actual_velocity
        # 2초 동안 max_speed/2s 램프 → max_speed에 가까워야 함
        assert v_at_2s > 0.3

    def test_sd_ramp_deceleration(self, motor_ss):
        """SD 설정 시 선형 램프 감속."""
        # 먼저 속도를 올린다
        for _ in range(300):
            motor_ss.update(0.8, 0.01)
        speed_before = motor_ss.actual_velocity
        assert speed_before > 0.3

        # 정지 명령 → SD 램프 감속
        for _ in range(150):
            motor_ss.update(0.0, 0.01)
        # 1s SD → 1.5s 이내 거의 정지
        assert motor_ss.actual_velocity < speed_before * 0.5

    def test_ss_zero_uses_first_order(self):
        """SS=0이면 기존 1차 지연 사용 (디폴트 동작 유지)."""
        motor_default = SingleMotorModel(DrivetrainConfig(
            slow_start_time=0.0, slow_down_time=0.0,
        ))
        motor_lag = SingleMotorModel(DrivetrainConfig(
            time_constant=0.20,
        ))
        # 동일 명령으로 10스텝
        for _ in range(10):
            motor_default.update(0.5, 0.01)
            motor_lag.update(0.5, 0.01)
        # 거의 동일해야 함
        assert abs(motor_default.actual_velocity - motor_lag.actual_velocity) < 0.01

    def test_ss_time_proportional(self):
        """SS 시간이 길수록 도달 시간이 길다."""
        motor_fast = SingleMotorModel(DrivetrainConfig(slow_start_time=1.0, slow_down_time=1.0))
        motor_slow = SingleMotorModel(DrivetrainConfig(slow_start_time=5.0, slow_down_time=5.0))

        for _ in range(100):
            motor_fast.update(1.0, 0.01)
            motor_slow.update(1.0, 0.01)

        # 1초 후 fast가 더 빠름
        assert motor_fast.actual_velocity > motor_slow.actual_velocity

    def test_sd_stops_completely(self, motor_ss):
        """SD 감속 후 완전히 정지."""
        for _ in range(500):
            motor_ss.update(0.8, 0.01)
        for _ in range(500):
            motor_ss.update(0.0, 0.01)
        assert abs(motor_ss.actual_velocity) < 0.01


class TestSingleMotorModelBrake:
    """전자브레이크 테스트."""

    @pytest.fixture
    def motor(self):
        return SingleMotorModel(DrivetrainConfig(
            time_constant=0.15, deadzone=0.05, brake_torque=16.0,
        ))

    def test_brake_slows_down(self, motor):
        """브레이크 체결 시 감속."""
        for _ in range(200):
            motor.update(0.5, 0.01)
        speed_before = motor.actual_velocity
        assert speed_before > 0.1

        motor.set_brake(True)
        for _ in range(100):
            motor.update(0.5, 0.01)  # 명령 무시, 브레이크 적용
        assert motor.actual_velocity < speed_before

    def test_brake_stops_completely(self, motor):
        """브레이크 장시간 체결 시 완전 정지."""
        for _ in range(200):
            motor.update(0.5, 0.01)
        motor.set_brake(True)
        for _ in range(1000):
            motor.update(0.5, 0.01)
        assert motor.actual_velocity == 0.0

    def test_brake_release_resumes(self, motor):
        """브레이크 해제 후 재가속."""
        for _ in range(200):
            motor.update(0.5, 0.01)
        motor.set_brake(True)
        for _ in range(500):
            motor.update(0.5, 0.01)
        assert motor.actual_velocity == 0.0

        motor.set_brake(False)
        for _ in range(200):
            motor.update(0.5, 0.01)
        assert motor.actual_velocity > 0.1

    def test_brake_initial_state(self, motor):
        """초기 상태에서 브레이크 해제."""
        assert not motor.brake_engaged

    def test_reset_clears_brake(self, motor):
        """리셋 시 브레이크 해제."""
        motor.set_brake(True)
        motor.reset()
        assert not motor.brake_engaged


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
