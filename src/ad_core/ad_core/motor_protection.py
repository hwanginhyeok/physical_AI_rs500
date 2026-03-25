"""MD2K 모터 컨트롤러 보호 기능 시뮬레이션.

MDROBOT MD2K 듀얼채널 DC 모터 드라이버의 보호 알람을 모델링한다:
- STALL: 명령 대비 실제 속도 오차 > 임계값
- 과전류: Soft fuse (정격 초과 지속 시간 기반 차단)
- 과온도: 단순 열 모델 (I²R 발열 + 자연 냉각)
- 과전압/저전압: 전압 범위 이탈
- 방향 불일치: 명령 방향 ≠ 실제 이동 방향
"""

from dataclasses import dataclass
from enum import Flag, auto


class MotorAlarm(Flag):
    """MD2K 알람 비트 플래그."""
    NONE = 0
    STALL = auto()
    OVER_CURRENT = auto()
    OVER_TEMP = auto()
    OVER_VOLTAGE = auto()
    UNDER_VOLTAGE = auto()
    INV_VEL = auto()         # 방향 불일치
    HALL_FAILURE = auto()


@dataclass
class ProtectionConfig:
    """보호 기능 파라미터.

    Attributes:
        rated_current: 정격 전류 (A).
        max_current: 최대 전류 (A).
        soft_fuse_time: Soft fuse 지속 시간 임계 (s).
        over_temp_threshold: 과온도 임계 (°C).
        ambient_temp: 주변 온도 (°C).
        winding_resistance: 모터 권선 저항 (Ω). I²R 발열 계산.
        thermal_resistance: 열저항 (°C/W). 발열→온도 변환 계수.
        cooling_time_constant: 냉각 시정수 (s).
        stall_threshold: STALL 속도 오차 비율 (0~1).
        rated_voltage: 정격 전압 (V).
        voltage_tolerance: 전압 허용 오차 비율 (0~1).

    열 모델 설계:
        정상상태 온도: T_ss = T_amb + I² × R_winding × R_thermal × tau_cool
        정격(78A) → T_ss = 25 + 78² × 0.05 × 0.002 × 60 = 61.5°C (65°C 직전)
        과부하(95A) → T_ss = 25 + 95² × 0.05 × 0.002 × 60 = 79.2°C (65°C 초과 → 알람)
    """
    rated_current: float = 78.0
    max_current: float = 100.0
    soft_fuse_time: float = 2.0
    over_temp_threshold: float = 65.0
    ambient_temp: float = 25.0
    winding_resistance: float = 0.05   # DB130-48 권선 저항 추정 (Ω)
    thermal_resistance: float = 0.002  # 열저항 (°C/W), 정격에서 ~60°C 도달하도록 교정
    cooling_time_constant: float = 60.0  # 자연 냉각 시정수 (s)
    stall_threshold: float = 0.15
    rated_voltage: float = 48.0
    voltage_tolerance: float = 0.10    # ±10%


class MotorProtection:
    """단일 채널 모터 보호 시뮬레이션.

    update()를 매 타임스텝 호출하여 알람 상태를 갱신한다.
    is_fault가 True이면 모터 출력을 차단해야 한다.
    """

    def __init__(self, config: ProtectionConfig | None = None) -> None:
        self.config = config or ProtectionConfig()
        self._temperature: float = self.config.ambient_temp
        self._over_current_timer: float = 0.0
        self._alarm: MotorAlarm = MotorAlarm.NONE
        self._is_fault: bool = False

    @property
    def temperature(self) -> float:
        """현재 추정 온도 (°C)."""
        return self._temperature

    @property
    def alarm(self) -> MotorAlarm:
        """현재 알람 상태."""
        return self._alarm

    @property
    def is_fault(self) -> bool:
        """차단(fault) 상태 여부."""
        return self._is_fault

    def reset(self) -> None:
        """보호 상태를 초기화한다."""
        self._temperature = self.config.ambient_temp
        self._over_current_timer = 0.0
        self._alarm = MotorAlarm.NONE
        self._is_fault = False

    def clear_fault(self) -> None:
        """수동 폴트 해제. 온도/전류가 정상 범위여야 해제 가능."""
        cfg = self.config
        if self._temperature < cfg.over_temp_threshold and self._over_current_timer == 0.0:
            self._is_fault = False
            self._alarm = MotorAlarm.NONE

    def update(
        self,
        command_speed: float,
        actual_speed: float,
        current: float,
        voltage: float,
        dt: float,
    ) -> MotorAlarm:
        """한 타임스텝 보호 로직을 수행한다.

        Args:
            command_speed: 명령 속도 (m/s 또는 RPM, 단위 무관).
            actual_speed: 실제 속도 (command_speed와 동일 단위).
            current: 현재 전류 (A).
            voltage: 현재 전압 (V).
            dt: 시간 간격 (s).

        Returns:
            현재 알람 상태.
        """
        cfg = self.config
        self._alarm = MotorAlarm.NONE

        # 1. STALL 검사
        if abs(command_speed) > 1e-3:
            speed_error = abs(command_speed - actual_speed) / abs(command_speed)
            if speed_error > cfg.stall_threshold:
                self._alarm |= MotorAlarm.STALL

        # 2. 과전류 (Soft fuse)
        if abs(current) > cfg.rated_current:
            self._over_current_timer += dt
            if self._over_current_timer >= cfg.soft_fuse_time:
                self._alarm |= MotorAlarm.OVER_CURRENT
                self._is_fault = True
        else:
            self._over_current_timer = max(0.0, self._over_current_timer - dt)

        # 3. 열 모델 (I²R 발열 + 자연 냉각)
        # 발열: P = I²R → dT = P * R_thermal * dt
        power_dissipation = current * current * cfg.winding_resistance
        heating = power_dissipation * cfg.thermal_resistance * dt

        # 냉각: 뉴턴 냉각 법칙
        cooling = (self._temperature - cfg.ambient_temp) / max(cfg.cooling_time_constant, 1e-6) * dt

        self._temperature += heating - cooling

        if self._temperature >= cfg.over_temp_threshold:
            self._alarm |= MotorAlarm.OVER_TEMP
            self._is_fault = True

        # 4. 과전압/저전압
        v_min = cfg.rated_voltage * (1.0 - cfg.voltage_tolerance)
        v_max = cfg.rated_voltage * (1.0 + cfg.voltage_tolerance)
        if voltage > v_max:
            self._alarm |= MotorAlarm.OVER_VOLTAGE
        if voltage < v_min:
            self._alarm |= MotorAlarm.UNDER_VOLTAGE

        # 5. 방향 불일치
        if abs(command_speed) > 1e-3 and abs(actual_speed) > 1e-3:
            if (command_speed > 0) != (actual_speed > 0):
                self._alarm |= MotorAlarm.INV_VEL

        return self._alarm
