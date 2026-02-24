"""차량 동역학 모델 모듈.

기존 SkidSteerModel(운동학)을 감싸서 뉴턴 역학 기반 동역학 레이어를 추가한다.
질량, 관성, 하중 전이, 가속도 제한 등을 모델링하여
Level 2 엔지니어링 시뮬레이션을 지원한다.
"""

import math
from dataclasses import dataclass
from typing import Tuple

from ad_core.datatypes import Pose2D
from ad_core.skid_steer_model import SkidSteerModel


@dataclass
class VehicleDynamicsConfig:
    """차량 동역학 파라미터.

    Attributes:
        mass: 차량 총 질량 (kg).
        Izz: 수직축(yaw) 관성 모멘트 (kg*m^2).
        cog_height: 무게중심 높이 (m). 하중 전이 계산에 사용.
        wheelbase: 차량 전후 길이 (m). 하중 전이 분배에 사용.
        track_width: 궤도 폭 (m).
        max_accel: 최대 가속도 (m/s^2).
        max_decel: 최대 감속도 (m/s^2). 양수로 입력.
        max_speed: 최대 속도 (m/s).
        drag_coefficient: 공기/구름 저항 계수 (N/(m/s)^2).
        rolling_resistance: 구름 저항 계수 (무차원).
    """

    mass: float = 200.0
    Izz: float = 50.0
    cog_height: float = 0.4
    wheelbase: float = 1.5
    track_width: float = 1.4
    max_accel: float = 1.0
    max_decel: float = 2.0
    max_speed: float = 1.0
    drag_coefficient: float = 0.5
    rolling_resistance: float = 0.02


GRAVITY = 9.81  # m/s^2


class VehicleDynamics:
    """뉴턴 역학 기반 차량 동역학 모델.

    SkidSteerModel의 운동학적 예측을 기반으로, 질량/관성에 의한
    가속도 제한, 하중 전이, 경사면 효과를 추가한다.

    시뮬 스텝:
        1. 명령 트랙 속도 → 운동학 모델로 목표 선속도/각속도 계산
        2. F=ma로 종방향 가속도 제한 적용
        3. tau=I*alpha로 요 각가속도 제한 적용
        4. 하중 전이 계산 (전후 축 하중 변화)
        5. 경사면 중력 분해 적용
        6. 최종 Pose2D 갱신
    """

    def __init__(
        self,
        config: VehicleDynamicsConfig | None = None,
        kinematic_model: SkidSteerModel | None = None,
    ) -> None:
        self.config = config or VehicleDynamicsConfig()
        self.kinematic = kinematic_model or SkidSteerModel(
            track_width=self.config.track_width,
            max_speed=self.config.max_speed * 2.0,
        )

        # 내부 상태
        self._current_linear_vel: float = 0.0
        self._current_angular_vel: float = 0.0
        self._pitch: float = 0.0  # 경사각 (rad), 양수 = 오르막
        self._roll: float = 0.0   # 횡경사 (rad)

        # 하중 분배 (전방/후방 수직력)
        half_weight = self.config.mass * GRAVITY / 2.0
        self._front_normal_force: float = half_weight
        self._rear_normal_force: float = half_weight

    # ------------------------------------------------------------------ #
    #  공개 API
    # ------------------------------------------------------------------ #

    @property
    def current_linear_velocity(self) -> float:
        """현재 종방향 속도 (m/s)."""
        return self._current_linear_vel

    @property
    def current_angular_velocity(self) -> float:
        """현재 요 각속도 (rad/s)."""
        return self._current_angular_vel

    @property
    def front_normal_force(self) -> float:
        """전방 수직력 (N)."""
        return self._front_normal_force

    @property
    def rear_normal_force(self) -> float:
        """후방 수직력 (N)."""
        return self._rear_normal_force

    def set_slope(self, pitch: float, roll: float = 0.0) -> None:
        """경사면 각도를 설정한다.

        Args:
            pitch: 전후 경사각 (rad). 양수 = 오르막.
            roll: 좌우 횡경사 (rad). 양수 = 우측 높음.
        """
        self._pitch = pitch
        self._roll = roll

    def reset(self, linear_vel: float = 0.0, angular_vel: float = 0.0) -> None:
        """내부 상태를 초기화한다."""
        self._current_linear_vel = linear_vel
        self._current_angular_vel = angular_vel
        self._pitch = 0.0
        self._roll = 0.0
        self._update_load_distribution(0.0)

    def step(
        self,
        commanded_left: float,
        commanded_right: float,
        dt: float,
        current_pose: Pose2D | None = None,
    ) -> Pose2D:
        """한 타임스텝 동역학 시뮬레이션을 수행한다.

        Args:
            commanded_left: 좌측 트랙 명령 속도 (m/s).
            commanded_right: 우측 트랙 명령 속도 (m/s).
            dt: 시간 간격 (s).
            current_pose: 현재 위치. None이면 원점에서 시작.

        Returns:
            갱신된 Pose2D.
        """
        if current_pose is None:
            current_pose = Pose2D()

        cfg = self.config

        # 1. 운동학 모델로 명령 속도를 선속도/각속도로 변환
        target_linear, target_angular = self.kinematic.tracks_to_twist(
            commanded_left, commanded_right
        )

        # 2. 경사면 중력 효과
        gravity_accel = self._compute_slope_acceleration()

        # 3. 저항력 계산
        resistance_accel = self._compute_resistance_deceleration()

        # 4. 종방향 가속도 제한 (F=ma)
        desired_accel = (target_linear - self._current_linear_vel) / max(dt, 1e-6)
        # 중력 + 저항을 가속 요구에 반영
        desired_accel += gravity_accel - resistance_accel

        # 가속도 클램프
        if desired_accel > 0:
            clamped_accel = min(desired_accel, cfg.max_accel)
        else:
            clamped_accel = max(desired_accel, -cfg.max_decel)

        # 선속도 갱신
        self._current_linear_vel += clamped_accel * dt

        # 속도 제한
        self._current_linear_vel = max(
            -cfg.max_speed, min(self._current_linear_vel, cfg.max_speed)
        )

        # 5. 요 각가속도 제한 (tau = I*alpha)
        desired_alpha = (target_angular - self._current_angular_vel) / max(dt, 1e-6)

        # 최대 요 토크를 트랙 견인력으로 추정
        # 단순 추정: max_torque = mass * max_accel * track_width / 2
        max_yaw_torque = cfg.mass * cfg.max_accel * cfg.track_width / 2.0
        max_alpha = max_yaw_torque / max(cfg.Izz, 1e-6)

        clamped_alpha = max(-max_alpha, min(desired_alpha, max_alpha))
        self._current_angular_vel += clamped_alpha * dt

        # 6. 하중 전이 계산
        self._update_load_distribution(clamped_accel)

        # 7. 운동학 모델로 트랙 속도 역산 후 포즈 예측
        v_left, v_right = self.kinematic.twist_to_tracks(
            self._current_linear_vel, self._current_angular_vel
        )
        new_pose = self.kinematic.predict_pose(current_pose, v_left, v_right, dt)

        return new_pose

    # ------------------------------------------------------------------ #
    #  하중 전이
    # ------------------------------------------------------------------ #

    def _update_load_distribution(self, longitudinal_accel: float) -> None:
        """종방향 가속도에 의한 전후 하중 전이를 계산한다.

        가속 시 후방에 하중 집중, 감속 시 전방에 하중 집중.

        Args:
            longitudinal_accel: 종방향 가속도 (m/s^2). 양수=가속.
        """
        cfg = self.config
        W = cfg.mass * GRAVITY  # 총 중량 (N)
        L = max(cfg.wheelbase, 1e-6)
        h = cfg.cog_height

        # 경사면 효과: 오르막이면 후방 하중 증가
        slope_transfer = W * math.sin(self._pitch) * h / L

        # 가속에 의한 하중 전이: delta_F = m * a * h / L
        accel_transfer = cfg.mass * longitudinal_accel * h / L

        # 정적 분배 (50:50 가정) + 전이
        half_W = W * math.cos(self._pitch) / 2.0
        self._front_normal_force = max(0.0, half_W - accel_transfer + slope_transfer)
        self._rear_normal_force = max(0.0, half_W + accel_transfer - slope_transfer)

    def _compute_slope_acceleration(self) -> float:
        """경사면에 의한 중력 가속 성분을 계산한다.

        Returns:
            경사 가속도 (m/s^2). 양수 = 내리막 방향 가속.
        """
        # 내리막: pitch < 0 → sin(pitch) < 0 → 양의 가속 (전진 방향)
        # 오르막: pitch > 0 → sin(pitch) > 0 → 음의 가속 (감속)
        return -GRAVITY * math.sin(self._pitch)

    def _compute_resistance_deceleration(self) -> float:
        """구름 저항 + 공기 저항에 의한 감속 성분을 계산한다.

        Returns:
            저항 감속도 (m/s^2). 항상 양수 (속도 반대 방향).
        """
        cfg = self.config
        speed = abs(self._current_linear_vel)

        # 구름 저항: F_roll = mu_r * m * g * cos(pitch)
        rolling = cfg.rolling_resistance * GRAVITY * math.cos(self._pitch)

        # 공기/점성 저항: F_drag = c * v^2
        drag = cfg.drag_coefficient * speed * speed / max(cfg.mass, 1e-6)

        return rolling + drag

    # ------------------------------------------------------------------ #
    #  경사면 유틸리티
    # ------------------------------------------------------------------ #

    def compute_max_climbable_angle(self, traction_coefficient: float = 0.6) -> float:
        """최대 등판 가능 각도를 계산한다.

        Args:
            traction_coefficient: 지면-트랙 마찰 계수.

        Returns:
            최대 등판 각도 (rad).
        """
        # 등판 한계: mu * cos(theta) >= sin(theta) → tan(theta) <= mu
        return math.atan(traction_coefficient)

    def compute_slope_forces(self) -> Tuple[float, float, float]:
        """현재 경사면에서의 힘 성분을 반환한다.

        Returns:
            (along_slope, normal, lateral) 경사 방향력, 수직항력, 횡력 (N).
        """
        W = self.config.mass * GRAVITY
        along = W * math.sin(self._pitch)
        normal = W * math.cos(self._pitch) * math.cos(self._roll)
        lateral = W * math.cos(self._pitch) * math.sin(self._roll)
        return along, normal, lateral
