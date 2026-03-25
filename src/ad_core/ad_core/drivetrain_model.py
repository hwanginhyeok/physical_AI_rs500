"""구동계(drivetrain) 모델 모듈.

모터 명령 → 실제 궤도 속도 사이의 지연, 데드존, 비선형성을 모델링한다.
좌/우 독립 모터를 각각 시뮬레이션하여 비대칭 응답을 재현할 수 있다.

모터 컨트롤러: MDROBOT MD2K (듀얼채널 DC 모터 드라이버, BLDC 구동)
모터: DB130-48 (BLDC 3000W DC48V) + SCK75-20 감속기
"""

import math
from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class DrivetrainConfig:
    """구동계 파라미터.

    Attributes:
        time_constant: 모터 응답 시정수 (s). 1차 지연 모델의 tau.
        gear_ratio: 감속비.
        gear_efficiency_forward: 정방향(구동) 감속기 효율 (0~1).
        gear_efficiency_reverse: 역방향(제동/역구동) 감속기 효율 (0~1).
        deadzone: 최소 유효 명령 속도 (m/s). 이하는 무반응.
        max_motor_torque: 모터 최대 토크 (Nm).
        max_motor_rpm: 모터 최대 RPM.
        sprocket_radius: 구동 스프로킷 반경 (m). 토크→견인력 변환에 사용.
        rated_voltage: DC 정격 전압 (V).
        rated_current: 정격 전류 (A).
        max_current: MD2K 채널당 최대 전류 (A).
        slow_start_time: SS 가속 시간 (s). 0이면 기존 1차 지연 사용.
        slow_down_time: SD 감속 시간 (s). 0이면 기존 1차 지연 사용.
        brake_torque: 전자브레이크 토크 (Nm).
        over_temp_threshold: 과온도 임계 (°C).
        stall_alarm_threshold: STALL 속도 오차 비율 (0~1).
    """

    # 기존 추정값 → HIH-2 SS500 실차 기반 교정 (2026-03-23, C48)
    # 모터: DB130-48 (3000W DC48V 2000rpm) + SCK75-20 감속기
    # 컨트롤러: MDROBOT MD2K (듀얼채널, 100A, CAN 50kbps) — C50 반영
    time_constant: float = 0.20   # 기존: 0.15s → BLACKTAN 돌입 전류 시험 기반 추정 (~0.2s)
    gear_ratio: float = 20.0      # 기존: 30 → DB130-48 SCK75-20 Helical-Hypoid 감속비 20:1
    gear_efficiency_forward: float = 0.85  # Helical-Hypoid 감속기 일반적 효율 ~85%
    gear_efficiency_reverse: float = 0.75
    deadzone: float = 0.05        # BLACKTAN 무부하 Spd1 PWM ~10.4%, 보수적 유지
    max_motor_torque: float = 14.32  # 기존: 20Nm → 3000W / (2000rpm * 2π/60) = 14.32Nm
    max_motor_rpm: float = 2000.0    # 기존: 3000 → DB130-48 정격 2000rpm ±10%
    sprocket_radius: float = 0.106   # 기존: 0.15m → v_max / ω_out = 1.111 / (100rpm*2π/60)

    # === MD2K 신규 파라미터 (C50) ===
    rated_voltage: float = 48.0        # DC 정격 전압 (V)
    rated_current: float = 78.0        # DB130-48 정격 전류 (A)
    max_current: float = 100.0         # MD2K 채널당 최대 전류 (A)
    slow_start_time: float = 0.0       # SS 가속 시간 (s), 0=기존 1차 지연
    slow_down_time: float = 0.0        # SD 감속 시간 (s), 0=기존 1차 지연
    brake_torque: float = 16.0         # 전자브레이크 (Nm)
    over_temp_threshold: float = 65.0  # MD2K 과온도 임계 (°C)
    stall_alarm_threshold: float = 0.15  # STALL 속도 오차 비율 (15%)


class SingleMotorModel:
    """단일 모터+감속기 모델.

    1차 지연(first-order lag) 기반 모터 응답과
    데드존, 토크 한계, 감속기 효율을 시뮬레이션한다.

    MD2K SS/SD 가감속 프로파일 지원:
    - slow_start_time > 0: 선형 램프 가속 (MD2K SS 가변저항)
    - slow_down_time > 0: 선형 램프 감속 (MD2K SD 가변저항)
    - 둘 다 0: 기존 1차 지연 동작 유지
    """

    def __init__(self, config: DrivetrainConfig | None = None) -> None:
        self.config = config or DrivetrainConfig()
        self._actual_velocity: float = 0.0
        self._brake_engaged: bool = False

    @property
    def actual_velocity(self) -> float:
        """현재 실제 출력 속도 (m/s)."""
        return self._actual_velocity

    @property
    def brake_engaged(self) -> bool:
        """브레이크 체결 상태."""
        return self._brake_engaged

    def set_brake(self, engaged: bool) -> None:
        """전자브레이크 체결/해제."""
        self._brake_engaged = engaged

    def reset(self, velocity: float = 0.0) -> None:
        """모터 상태를 초기화한다."""
        self._actual_velocity = velocity
        self._brake_engaged = False

    def update(self, command_velocity: float, dt: float) -> float:
        """모터 명령을 받아 실제 출력 속도를 갱신한다.

        Args:
            command_velocity: 명령 속도 (m/s).
            dt: 시간 간격 (s).

        Returns:
            실제 출력 속도 (m/s).
        """
        cfg = self.config

        # 0. 브레이크 체결 시 감속
        if self._brake_engaged:
            self._apply_brake(dt)
            return self._actual_velocity

        # 1. 데드존 적용: |cmd| < deadzone이면 목표 = 0
        if abs(command_velocity) < cfg.deadzone:
            target_velocity = 0.0
        else:
            target_velocity = command_velocity

        # 2. 최대 속도 제한 (RPM 한계에서 역산)
        max_track_speed = self._max_track_speed()
        target_velocity = max(-max_track_speed, min(target_velocity, max_track_speed))

        # 3. SS/SD 램프 또는 1차 지연 선택
        is_accelerating = abs(target_velocity) > abs(self._actual_velocity)
        use_ss = is_accelerating and cfg.slow_start_time > 0
        use_sd = (not is_accelerating) and cfg.slow_down_time > 0

        if use_ss or use_sd:
            # MD2K SS/SD 선형 램프 가감속
            self._apply_ss_sd_ramp(target_velocity, dt, max_track_speed)
        else:
            # 기존 1차 지연 모델
            self._apply_first_order_lag(target_velocity, dt)

        return self._actual_velocity

    def _apply_first_order_lag(self, target_velocity: float, dt: float) -> None:
        """기존 1차 지연 모델로 속도를 갱신한다."""
        cfg = self.config

        # 감속기 효율 적용
        delta = target_velocity - self._actual_velocity
        if delta * self._actual_velocity >= 0:
            efficiency = cfg.gear_efficiency_forward
        else:
            efficiency = cfg.gear_efficiency_reverse

        effective_target = self._actual_velocity + delta * efficiency

        # 1차 지연: v += (target - v) * (1 - exp(-dt/tau))
        tau = max(cfg.time_constant, 1e-6)
        alpha = 1.0 - math.exp(-dt / tau)
        self._actual_velocity += (effective_target - self._actual_velocity) * alpha

    def _apply_ss_sd_ramp(
        self, target_velocity: float, dt: float, max_track_speed: float
    ) -> None:
        """MD2K SS/SD 선형 램프 가감속을 적용한다.

        가속: ramp_rate = max_speed / slow_start_time
        감속: ramp_rate = max_speed / slow_down_time
        """
        cfg = self.config

        is_accelerating = abs(target_velocity) > abs(self._actual_velocity)

        if is_accelerating and cfg.slow_start_time > 0:
            ramp_rate = max_track_speed / cfg.slow_start_time
        elif cfg.slow_down_time > 0:
            ramp_rate = max_track_speed / cfg.slow_down_time
        else:
            # fallback: 1차 지연
            self._apply_first_order_lag(target_velocity, dt)
            return

        # 감속기 효율 적용
        delta = target_velocity - self._actual_velocity
        if delta * self._actual_velocity >= 0:
            efficiency = cfg.gear_efficiency_forward
        else:
            efficiency = cfg.gear_efficiency_reverse

        effective_delta = delta * efficiency
        max_dv = ramp_rate * dt

        # 클램프: 램프 속도 이내로 변화량 제한
        if abs(effective_delta) > max_dv:
            effective_delta = math.copysign(max_dv, effective_delta)

        self._actual_velocity += effective_delta

    def _apply_brake(self, dt: float) -> None:
        """전자브레이크에 의한 감속을 적용한다."""
        cfg = self.config
        if abs(self._actual_velocity) < 1e-6:
            self._actual_velocity = 0.0
            return

        # 브레이크 토크 → 감속기 출력 기준 감속력
        # 감속도 = brake_torque / sprocket_radius / mass_approx
        # mass를 모르므로 토크→속도 변화율로 근사 (F = T/r)
        brake_force = cfg.brake_torque * cfg.gear_ratio / max(cfg.sprocket_radius, 1e-6)
        # 대략적 질량 800kg 가정하여 감속도 계산 (VehicleDynamics에서 정밀 처리)
        approx_mass = 800.0
        decel = brake_force / approx_mass * dt

        if abs(self._actual_velocity) <= decel:
            self._actual_velocity = 0.0
        else:
            self._actual_velocity -= math.copysign(decel, self._actual_velocity)

    def get_motor_torque(self, load_velocity: float) -> float:
        """현재 부하 속도에서의 모터 출력 토크를 계산한다.

        선형 토크-RPM 특성을 가정한다 (디폴트).

        Args:
            load_velocity: 부하측(트랙) 속도 (m/s).

        Returns:
            모터 출력 토크 (Nm), 감속기 출력 기준.
        """
        cfg = self.config
        max_speed = self._max_track_speed()

        if max_speed < 1e-6:
            return cfg.max_motor_torque * cfg.gear_ratio * cfg.gear_efficiency_forward

        # 선형 토크-속도 특성: T = T_max * (1 - v/v_max)
        speed_ratio = min(abs(load_velocity) / max_speed, 1.0)
        motor_torque = cfg.max_motor_torque * (1.0 - speed_ratio)

        # 감속기 출력 토크
        output_torque = motor_torque * cfg.gear_ratio * cfg.gear_efficiency_forward
        return output_torque

    def _max_track_speed(self) -> float:
        """최대 트랙 속도 (RPM 한계 기반)."""
        cfg = self.config
        # v = (RPM * 2*pi*r) / (60 * gear_ratio)
        return (cfg.max_motor_rpm * 2.0 * math.pi * cfg.sprocket_radius) / (
            60.0 * cfg.gear_ratio
        )

    def _max_traction_force(self) -> float:
        """최대 견인력 (N)."""
        cfg = self.config
        return (
            cfg.max_motor_torque
            * cfg.gear_ratio
            * cfg.gear_efficiency_forward
            / max(cfg.sprocket_radius, 1e-6)
        )


class DrivetrainModel:
    """좌/우 독립 구동계 모델.

    두 개의 SingleMotorModel을 조합하여 차량 전체 구동계를 시뮬레이션한다.
    좌/우 모터의 파라미터를 다르게 설정하여 비대칭 응답을 모델링할 수 있다.
    """

    def __init__(
        self,
        config: DrivetrainConfig | None = None,
        left_config: DrivetrainConfig | None = None,
        right_config: DrivetrainConfig | None = None,
    ) -> None:
        """초기화.

        Args:
            config: 좌/우 공통 설정. left_config/right_config가 None인 쪽에 적용.
            left_config: 좌측 모터 전용 설정. None이면 config 사용.
            right_config: 우측 모터 전용 설정. None이면 config 사용.
        """
        base = config or DrivetrainConfig()
        self.left_motor = SingleMotorModel(left_config or base)
        self.right_motor = SingleMotorModel(right_config or base)

    def reset(self) -> None:
        """양쪽 모터 상태를 초기화한다."""
        self.left_motor.reset()
        self.right_motor.reset()

    def update(
        self,
        cmd_left: float,
        cmd_right: float,
        dt: float,
    ) -> Tuple[float, float]:
        """좌/우 모터 명령을 받아 실제 출력 속도를 반환한다.

        Args:
            cmd_left: 좌측 트랙 명령 속도 (m/s).
            cmd_right: 우측 트랙 명령 속도 (m/s).
            dt: 시간 간격 (s).

        Returns:
            (actual_left, actual_right) 실제 좌/우 트랙 속도 (m/s).
        """
        actual_left = self.left_motor.update(cmd_left, dt)
        actual_right = self.right_motor.update(cmd_right, dt)
        return actual_left, actual_right

    @property
    def actual_velocities(self) -> Tuple[float, float]:
        """현재 실제 좌/우 트랙 속도."""
        return self.left_motor.actual_velocity, self.right_motor.actual_velocity

    def get_total_traction_force(self, v_left: float, v_right: float) -> float:
        """현재 속도에서의 총 견인력을 계산한다.

        Args:
            v_left: 좌측 트랙 속도.
            v_right: 우측 트랙 속도.

        Returns:
            총 견인력 (N).
        """
        f_left = self.left_motor.get_motor_torque(v_left) / max(
            self.left_motor.config.sprocket_radius, 1e-6
        )
        f_right = self.right_motor.get_motor_torque(v_right) / max(
            self.right_motor.config.sprocket_radius, 1e-6
        )
        return f_left + f_right
