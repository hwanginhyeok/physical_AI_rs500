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

    # 기존 추정값 → HIH-2 SS500 실차 기반 교정 (2026-03-23, C48)
    mass: float = 800.0           # 기존: 200kg → 실차: 800kg 공차 추정 (TODO: xlsx 확인, 만차 ~1300kg)
    Izz: float = 250.0            # 기존: 50 → 800kg 차체 기반 추정 (TODO: System Identification)
    cog_height: float = 0.5       # 기존: 0.4m → 궤도차+배터리 하부 배치 감안 0.5m 추정
    wheelbase: float = 1.8        # 기존: 1.5m → 궤도 접지 길이 기반 추정 (TODO: xlsx 확인)
    track_width: float = 1.2      # 기존: 1.4m → 윤거 추정 (TODO: xlsx 확인)
    max_accel: float = 0.5        # 기존: 1.0 → 800kg 차량에 맞게 하향 (3kW 모터 2기)
    max_decel: float = 1.5        # 기존: 2.0 → 800kg + 브레이크 16Nm 감안
    max_speed: float = 1.111      # 기존: 1.0 → CAN CF_VehSpdAct 범위 ±4km/h = 1.111m/s
    drag_coefficient: float = 0.5
    rolling_resistance: float = 0.02


GRAVITY = 9.81  # m/s^2


class VehicleDynamics:
    """뉴턴 역학 기반 차량 동역학 모델.

    SkidSteerModel의 운동학적 예측을 기반으로, 질량/관성에 의한
    가속도 제한, 하중 전이, 경사면 효과를 추가한다.

    안전 상태 (C50 P1):
    - 비상정지(e-stop): 즉시 출력 0, 브레이크 체결
    - 자율주행 모드: False면 명령 무시 (속도 0으로 수렴)
    - 시스템 폴트: 모터 출력 차단

    시뮬 스텝:
        0. 안전 상태 체크 (e-stop > fault > autonomous_mode)
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
        terrain_interaction: "TrackTerrainInteraction | None" = None,
    ) -> None:
        self.config = config or VehicleDynamicsConfig()
        self.kinematic = kinematic_model or SkidSteerModel(
            track_width=self.config.track_width,
            max_speed=self.config.max_speed * 2.0,
        )
        self.terrain_interaction = terrain_interaction

        # 내부 상태
        self._current_linear_vel: float = 0.0
        self._current_angular_vel: float = 0.0
        self._pitch: float = 0.0  # 경사각 (rad), 양수 = 오르막
        self._roll: float = 0.0   # 횡경사 (rad)

        # 하중 분배 (전방/후방 수직력)
        half_weight = self.config.mass * GRAVITY / 2.0
        self._front_normal_force: float = half_weight
        self._rear_normal_force: float = half_weight

        # 지면 상호작용 결과 (외부 조회용)
        self.last_sinkage: float = 0.0
        self.last_slip_ratio: float = 0.0

        # === 안전 상태 (C50 P1) ===
        self._emergency_stop: bool = False
        self._autonomous_mode: bool = True   # 디폴트 True → 기존 동작 유지
        self._system_fault: bool = False

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

    # --- 안전 상태 API (C50 P1) ---

    @property
    def emergency_stop(self) -> bool:
        """비상정지 상태."""
        return self._emergency_stop

    def set_emergency_stop(self, active: bool) -> None:
        """비상정지를 설정/해제한다. 활성화 시 즉시 출력 차단."""
        self._emergency_stop = active

    @property
    def autonomous_mode(self) -> bool:
        """자율주행 모드 상태."""
        return self._autonomous_mode

    def set_autonomous_mode(self, enabled: bool) -> None:
        """자율주행 모드를 전환한다. 비활성화 시 명령 무시."""
        self._autonomous_mode = enabled

    @property
    def system_fault(self) -> bool:
        """시스템 폴트 상태."""
        return self._system_fault

    def set_system_fault(self, fault: bool) -> None:
        """시스템 폴트를 설정/해제한다. 활성화 시 모터 출력 차단."""
        self._system_fault = fault

    @property
    def is_operational(self) -> bool:
        """운행 가능 상태인지 반환한다 (e-stop, fault, autonomous 모두 정상)."""
        return (
            not self._emergency_stop
            and not self._system_fault
            and self._autonomous_mode
        )

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
        self._emergency_stop = False
        self._autonomous_mode = True
        self._system_fault = False
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

        # 0. 안전 상태 체크 (우선순위: e-stop > fault > autonomous_mode)
        if self._emergency_stop or self._system_fault or not self._autonomous_mode:
            # 명령 무시, 자연 감속
            commanded_left = 0.0
            commanded_right = 0.0

        # 1. 운동학 모델로 명령 속도를 선속도/각속도로 변환
        target_linear, target_angular = self.kinematic.tracks_to_twist(
            commanded_left, commanded_right
        )

        # 2. 경사면 중력 효과
        gravity_accel = self._compute_slope_acceleration()

        # 3. 저항력 계산 (지면 상호작용 반영)
        resistance_accel = self._compute_resistance_deceleration()
        terrain_rolling_resistance = 0.0
        terrain_turning_resistance = 0.0
        sinkage_drag = 0.0

        if self.terrain_interaction is not None:
            total_normal = self._front_normal_force + self._rear_normal_force
            half_normal = total_normal / 2.0

            # 슬립비 계산: 명령 속도 vs 실제 속도 차이
            cmd_speed = (abs(commanded_left) + abs(commanded_right)) / 2.0
            slip_ratio = abs(cmd_speed - abs(self._current_linear_vel)) / max(cmd_speed, 0.1)
            self.last_slip_ratio = slip_ratio

            # 침하량 → 추가 구름 저항
            sinkage = self.terrain_interaction.compute_sinkage(half_normal)
            self.last_sinkage = sinkage
            # 침하에 비례하는 저항 (침하 깊을수록 더 많은 흙을 밀어야 함)
            sinkage_drag = sinkage * cfg.mass * GRAVITY * 0.5 / max(cfg.mass, 1e-6)

            # 지면 구름 저항 (지형별 다른 계수)
            rolling_force = self.terrain_interaction.compute_rolling_resistance(total_normal)
            terrain_rolling_resistance = rolling_force / max(cfg.mass, 1e-6)

            # 선회 저항 모멘트
            turning_moment = self.terrain_interaction.compute_turning_resistance(
                self._current_angular_vel, total_normal
            )
            terrain_turning_resistance = turning_moment
        else:
            self.last_sinkage = 0.0
            self.last_slip_ratio = 0.0

        # 4. 종방향 가속도 제한 (F=ma)
        desired_accel = (target_linear - self._current_linear_vel) / max(dt, 1e-6)
        # 중력 + 저항 + 지면 저항을 가속 요구에 반영
        total_resistance = resistance_accel + terrain_rolling_resistance + sinkage_drag
        desired_accel += gravity_accel - total_resistance

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
        max_yaw_torque = cfg.mass * cfg.max_accel * cfg.track_width / 2.0
        max_alpha = max_yaw_torque / max(cfg.Izz, 1e-6)

        # 선회 저항을 요 각가속도에 반영
        if self.terrain_interaction is not None and abs(self._current_angular_vel) > 1e-6:
            resistance_alpha = terrain_turning_resistance / max(cfg.Izz, 1e-6)
            # 선회 저항은 각속도 반대 방향
            sign = -1.0 if self._current_angular_vel > 0 else 1.0
            desired_alpha += sign * resistance_alpha

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
