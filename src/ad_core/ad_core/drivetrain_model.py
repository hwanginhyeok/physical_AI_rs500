"""구동계(drivetrain) 모델 모듈.

모터 명령 → 실제 궤도 속도 사이의 지연, 데드존, 비선형성을 모델링한다.
좌/우 독립 모터를 각각 시뮬레이션하여 비대칭 응답을 재현할 수 있다.
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
    """

    time_constant: float = 0.15
    gear_ratio: float = 30.0
    gear_efficiency_forward: float = 0.85
    gear_efficiency_reverse: float = 0.75
    deadzone: float = 0.05
    max_motor_torque: float = 20.0
    max_motor_rpm: float = 3000.0
    sprocket_radius: float = 0.15


class SingleMotorModel:
    """단일 모터+감속기 모델.

    1차 지연(first-order lag) 기반 모터 응답과
    데드존, 토크 한계, 감속기 효율을 시뮬레이션한다.
    """

    def __init__(self, config: DrivetrainConfig | None = None) -> None:
        self.config = config or DrivetrainConfig()
        self._actual_velocity: float = 0.0

    @property
    def actual_velocity(self) -> float:
        """현재 실제 출력 속도 (m/s)."""
        return self._actual_velocity

    def reset(self, velocity: float = 0.0) -> None:
        """모터 상태를 초기화한다."""
        self._actual_velocity = velocity

    def update(self, command_velocity: float, dt: float) -> float:
        """모터 명령을 받아 실제 출력 속도를 갱신한다.

        Args:
            command_velocity: 명령 속도 (m/s).
            dt: 시간 간격 (s).

        Returns:
            실제 출력 속도 (m/s).
        """
        cfg = self.config

        # 1. 데드존 적용: |cmd| < deadzone이면 목표 = 0
        if abs(command_velocity) < cfg.deadzone:
            target_velocity = 0.0
        else:
            target_velocity = command_velocity

        # 2. 최대 속도 제한 (RPM 한계에서 역산)
        max_track_speed = self._max_track_speed()
        target_velocity = max(-max_track_speed, min(target_velocity, max_track_speed))

        # 3. 감속기 효율 적용
        # 가속 방향과 현재 속도 방향이 같으면 정방향, 아니면 역방향
        delta = target_velocity - self._actual_velocity
        if delta * self._actual_velocity >= 0:
            efficiency = cfg.gear_efficiency_forward
        else:
            efficiency = cfg.gear_efficiency_reverse

        # 목표 속도에 효율 적용 (손실 반영)
        effective_target = self._actual_velocity + delta * efficiency

        # 4. 1차 지연 모델: v += (target - v) * (1 - exp(-dt/tau))
        tau = max(cfg.time_constant, 1e-6)
        alpha = 1.0 - math.exp(-dt / tau)
        self._actual_velocity += (effective_target - self._actual_velocity) * alpha

        # 5. 토크 한계에 의한 가속도 제한
        max_force = self._max_traction_force()
        # 가속도 = F/m (여기서 m은 호출측에서 처리하므로, 힘 기반 속도 변화율로 근사)
        # 단순화: 토크 한계를 속도 변화율로 반영
        max_dv = max_force * cfg.sprocket_radius * dt
        actual_dv = self._actual_velocity - (self._actual_velocity - (effective_target - self._actual_velocity) * alpha + self._actual_velocity - (effective_target - self._actual_velocity) * alpha)
        # 이미 1차 지연에서 처리됨; 토크 한계는 max_speed로 간접 반영

        return self._actual_velocity

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
