"""시뮬레이션 파라미터 물리적 타당성 검증 테스트.

C48~C50에서 교정된 파라미터가 물리적으로 유효한 범위에 있는지 검증한다.
파라미터 변경 시 회귀 방지 역할.

검증 대상:
- DrivetrainConfig: 모터/감속기/MD2K 파라미터
- VehicleDynamicsConfig: 차량 동역학
- ProtectionConfig: 보호 기능 열 모델
- MD2K CAN 코덱: 메시지 ID, 바이트 범위
"""

import math
import pytest

from ad_core.drivetrain_model import DrivetrainConfig
from ad_core.vehicle_dynamics import VehicleDynamicsConfig
from ad_core.motor_protection import ProtectionConfig
from ad_core.md2k_codec import (
    MD2K_TX_ID, MD2K_RX_ID,
    rpm_to_track_speed, track_speed_to_rpm,
)


class TestDrivetrainConfigValidation:
    """DrivetrainConfig 파라미터 범위 검증."""

    @pytest.fixture
    def cfg(self):
        return DrivetrainConfig()

    # --- 기본 모터 파라미터 ---

    def test_time_constant_positive(self, cfg):
        """시정수는 양수."""
        assert cfg.time_constant > 0

    def test_time_constant_reasonable(self, cfg):
        """시정수 0.01~5초 범위 (DC모터 일반적 범위)."""
        assert 0.01 <= cfg.time_constant <= 5.0

    def test_gear_ratio_positive(self, cfg):
        assert cfg.gear_ratio > 0

    def test_gear_ratio_reasonable(self, cfg):
        """감속비 1~100 범위."""
        assert 1.0 <= cfg.gear_ratio <= 100.0

    def test_gear_efficiency_range(self, cfg):
        """감속기 효율 0~1 범위."""
        assert 0.0 < cfg.gear_efficiency_forward <= 1.0
        assert 0.0 < cfg.gear_efficiency_reverse <= 1.0

    def test_forward_efficiency_greater_than_reverse(self, cfg):
        """정방향 효율 >= 역방향 효율."""
        assert cfg.gear_efficiency_forward >= cfg.gear_efficiency_reverse

    def test_deadzone_non_negative(self, cfg):
        assert cfg.deadzone >= 0.0

    def test_deadzone_reasonable(self, cfg):
        """데드존 < 최대 속도의 50%."""
        max_speed = rpm_to_track_speed(cfg.max_motor_rpm, cfg.gear_ratio, cfg.sprocket_radius)
        assert cfg.deadzone < max_speed * 0.5

    def test_max_motor_torque_positive(self, cfg):
        assert cfg.max_motor_torque > 0

    def test_max_motor_rpm_positive(self, cfg):
        assert cfg.max_motor_rpm > 0

    def test_sprocket_radius_positive(self, cfg):
        assert cfg.sprocket_radius > 0

    def test_sprocket_radius_reasonable(self, cfg):
        """스프로킷 반경 0.01~1.0m."""
        assert 0.01 <= cfg.sprocket_radius <= 1.0

    # --- 파생값 물리적 일관성 ---

    def test_max_speed_from_rpm(self, cfg):
        """RPM 기반 최대 속도가 합리적 (0.1~10 m/s)."""
        max_speed = rpm_to_track_speed(cfg.max_motor_rpm, cfg.gear_ratio, cfg.sprocket_radius)
        assert 0.1 <= max_speed <= 10.0

    def test_motor_power_matches_spec(self, cfg):
        """모터 출력 ≈ 3000W (DB130-48 정격)."""
        # P = T × ω = torque × (RPM × 2π/60)
        omega = cfg.max_motor_rpm * 2.0 * math.pi / 60.0
        power = cfg.max_motor_torque * omega
        assert 2000.0 <= power <= 4000.0  # 3000W ±33%

    def test_output_torque_reasonable(self, cfg):
        """감속기 출력 토크가 합리적."""
        output_torque = cfg.max_motor_torque * cfg.gear_ratio * cfg.gear_efficiency_forward
        assert 100.0 <= output_torque <= 500.0  # 200~300Nm 예상

    # --- MD2K 신규 파라미터 ---

    def test_rated_voltage_positive(self, cfg):
        assert cfg.rated_voltage > 0

    def test_rated_voltage_matches_motor(self, cfg):
        """정격 전압 = DB130-48 정격 48V."""
        assert cfg.rated_voltage == 48.0

    def test_rated_current_positive(self, cfg):
        assert cfg.rated_current > 0

    def test_max_current_gte_rated(self, cfg):
        """최대 전류 >= 정격 전류."""
        assert cfg.max_current >= cfg.rated_current

    def test_max_current_md2k_spec(self, cfg):
        """MD2K 최대 전류 = 100A."""
        assert cfg.max_current == 100.0

    def test_slow_start_time_non_negative(self, cfg):
        assert cfg.slow_start_time >= 0.0

    def test_slow_down_time_non_negative(self, cfg):
        assert cfg.slow_down_time >= 0.0

    def test_ss_sd_range(self, cfg):
        """SS/SD 범위: 0~15초 (MD2K 사양)."""
        assert cfg.slow_start_time <= 15.0
        assert cfg.slow_down_time <= 15.0

    def test_brake_torque_non_negative(self, cfg):
        assert cfg.brake_torque >= 0.0

    def test_brake_torque_spec(self, cfg):
        """전자브레이크 = 16Nm (DB130-48 사양)."""
        assert cfg.brake_torque == 16.0

    def test_over_temp_threshold_reasonable(self, cfg):
        """과온도 임계: 40~150°C."""
        assert 40.0 <= cfg.over_temp_threshold <= 150.0

    def test_stall_alarm_threshold_range(self, cfg):
        """STALL 임계: 0~1."""
        assert 0.0 < cfg.stall_alarm_threshold < 1.0


class TestVehicleDynamicsConfigValidation:
    """VehicleDynamicsConfig 파라미터 범위 검증."""

    @pytest.fixture
    def cfg(self):
        return VehicleDynamicsConfig()

    def test_mass_positive(self, cfg):
        assert cfg.mass > 0

    def test_mass_reasonable(self, cfg):
        """차량 질량 100~5000 kg."""
        assert 100.0 <= cfg.mass <= 5000.0

    def test_izz_positive(self, cfg):
        assert cfg.Izz > 0

    def test_izz_reasonable(self, cfg):
        """관성 모멘트: 질량 기반 합리적 범위."""
        # Izz ≈ (1/12) × m × (L² + W²) 직육면체 근사
        # 800kg, 2m×1.2m → ~293
        min_izz = cfg.mass * 0.05  # 매우 작은 차량
        max_izz = cfg.mass * 2.0   # 매우 큰 차량
        assert min_izz <= cfg.Izz <= max_izz

    def test_cog_height_positive(self, cfg):
        assert cfg.cog_height > 0

    def test_cog_height_below_vehicle_height(self, cfg):
        """무게중심 높이 < 2m (차량 전체 높이 미만)."""
        assert cfg.cog_height < 2.0

    def test_wheelbase_positive(self, cfg):
        assert cfg.wheelbase > 0

    def test_track_width_positive(self, cfg):
        assert cfg.track_width > 0

    def test_max_accel_positive(self, cfg):
        assert cfg.max_accel > 0

    def test_max_decel_positive(self, cfg):
        assert cfg.max_decel > 0

    def test_decel_greater_than_accel(self, cfg):
        """감속이 가속보다 빠름 (브레이크 > 엔진)."""
        assert cfg.max_decel >= cfg.max_accel

    def test_max_speed_positive(self, cfg):
        assert cfg.max_speed > 0

    def test_max_speed_reasonable(self, cfg):
        """최대 속도 0.1~10 m/s (농업용 차량)."""
        assert 0.1 <= cfg.max_speed <= 10.0

    def test_drag_coefficient_non_negative(self, cfg):
        assert cfg.drag_coefficient >= 0.0

    def test_rolling_resistance_range(self, cfg):
        """구름 저항 0~1."""
        assert 0.0 <= cfg.rolling_resistance <= 1.0

    def test_max_speed_matches_drivetrain(self, cfg):
        """동역학 max_speed ≈ 구동계 RPM 기반 최대 속도."""
        dt_cfg = DrivetrainConfig()
        motor_max_speed = rpm_to_track_speed(
            dt_cfg.max_motor_rpm, dt_cfg.gear_ratio, dt_cfg.sprocket_radius,
        )
        # 10% 이내 일치
        assert abs(cfg.max_speed - motor_max_speed) / motor_max_speed < 0.10


class TestProtectionConfigValidation:
    """ProtectionConfig 열 모델 파라미터 검증."""

    @pytest.fixture
    def cfg(self):
        return ProtectionConfig()

    def test_rated_current_positive(self, cfg):
        assert cfg.rated_current > 0

    def test_max_current_gte_rated(self, cfg):
        assert cfg.max_current >= cfg.rated_current

    def test_soft_fuse_time_positive(self, cfg):
        assert cfg.soft_fuse_time > 0

    def test_over_temp_threshold_above_ambient(self, cfg):
        """과온도 임계 > 주변 온도."""
        assert cfg.over_temp_threshold > cfg.ambient_temp

    def test_ambient_temp_reasonable(self, cfg):
        """주변 온도 -40~60°C."""
        assert -40.0 <= cfg.ambient_temp <= 60.0

    def test_winding_resistance_positive(self, cfg):
        assert cfg.winding_resistance > 0

    def test_winding_resistance_reasonable(self, cfg):
        """권선 저항 0.001~1.0Ω (BLDC 모터 일반 범위)."""
        assert 0.001 <= cfg.winding_resistance <= 1.0

    def test_thermal_resistance_positive(self, cfg):
        assert cfg.thermal_resistance > 0

    def test_cooling_time_constant_positive(self, cfg):
        assert cfg.cooling_time_constant > 0

    def test_steady_state_temp_at_rated(self, cfg):
        """정격 전류 연속 운전 시 정상상태 온도 < 과온도 임계.

        T_ss = T_amb + I² × R_winding × R_thermal × tau_cool
        """
        t_ss = (cfg.ambient_temp
                + cfg.rated_current ** 2
                * cfg.winding_resistance
                * cfg.thermal_resistance
                * cfg.cooling_time_constant)
        assert t_ss < cfg.over_temp_threshold, (
            f"정격 정상상태 {t_ss:.1f}°C >= 과온도 {cfg.over_temp_threshold}°C"
        )

    def test_steady_state_temp_at_max_current_exceeds_threshold(self, cfg):
        """최대 전류 시 정상상태 온도 > 과온도 임계 (보호 동작 가능)."""
        t_ss = (cfg.ambient_temp
                + cfg.max_current ** 2
                * cfg.winding_resistance
                * cfg.thermal_resistance
                * cfg.cooling_time_constant)
        assert t_ss > cfg.over_temp_threshold, (
            f"최대전류 정상상태 {t_ss:.1f}°C <= 과온도 {cfg.over_temp_threshold}°C (보호 불가)"
        )

    def test_voltage_tolerance_range(self, cfg):
        """전압 허용 오차 0~50%."""
        assert 0.0 < cfg.voltage_tolerance <= 0.5

    def test_stall_threshold_range(self, cfg):
        assert 0.0 < cfg.stall_threshold < 1.0


class TestCANCodecValidation:
    """MD2K CAN 코덱 상수 검증."""

    def test_tx_id_standard_range(self):
        """Tx CAN ID가 표준 11비트 범위 (0x000~0x7FF)."""
        assert 0x000 <= MD2K_TX_ID <= 0x7FF

    def test_rx_id_standard_range(self):
        """Rx CAN ID가 표준 11비트 범위."""
        assert 0x000 <= MD2K_RX_ID <= 0x7FF

    def test_tx_rx_different(self):
        """Tx ≠ Rx."""
        assert MD2K_TX_ID != MD2K_RX_ID

    def test_rpm_conversion_identity(self):
        """RPM 변환 왕복 오차 < 0.01%."""
        for rpm in [0, 100, 500, 1000, 2000]:
            speed = rpm_to_track_speed(rpm)
            rpm_back = track_speed_to_rpm(speed)
            if rpm > 0:
                assert abs(rpm_back - rpm) / rpm < 0.0001

    def test_max_rpm_gives_max_speed(self):
        """2000 RPM → ~1.111 m/s (SS500 최대 속도)."""
        cfg = DrivetrainConfig()
        speed = rpm_to_track_speed(cfg.max_motor_rpm, cfg.gear_ratio, cfg.sprocket_radius)
        assert abs(speed - 1.111) < 0.01


class TestCrossValidation:
    """C49 리포트 교차검증: 코드 파라미터 vs 데이터시트 값 일치."""

    def test_motor_rpm_matches_datasheet(self):
        """DB130-48 정격 RPM = 2000."""
        assert DrivetrainConfig().max_motor_rpm == 2000.0

    def test_gear_ratio_matches_datasheet(self):
        """SCK75-20 감속비 = 20:1."""
        assert DrivetrainConfig().gear_ratio == 20.0

    def test_motor_torque_matches_calculation(self):
        """모터 토크 = 3000W / (2000rpm × 2π/60) ≈ 14.32Nm."""
        cfg = DrivetrainConfig()
        expected = 3000.0 / (2000.0 * 2.0 * math.pi / 60.0)
        assert abs(cfg.max_motor_torque - expected) < 0.01

    def test_vehicle_mass_matches_estimate(self):
        """차량 질량 = 800kg (공차 추정)."""
        assert VehicleDynamicsConfig().mass == 800.0

    def test_max_speed_matches_can_dbc(self):
        """최대 속도 = 4km/h = 1.111 m/s (CAN DBC 범위)."""
        assert abs(VehicleDynamicsConfig().max_speed - 1.111) < 0.001

    def test_brake_torque_matches_datasheet(self):
        """전자브레이크 = 16Nm (DB130-48)."""
        assert DrivetrainConfig().brake_torque == 16.0

    def test_md2k_max_current_matches_spec(self):
        """MD2K 채널당 최대 전류 = 100A."""
        assert DrivetrainConfig().max_current == 100.0

    def test_rated_current_matches_datasheet(self):
        """DB130-48 정격 전류 = 78A."""
        assert DrivetrainConfig().rated_current == 78.0
