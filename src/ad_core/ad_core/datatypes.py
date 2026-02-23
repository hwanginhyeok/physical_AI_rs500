"""ad_core 공유 타입 정의.

순수 Python 데이터 클래스로, ROS2 의존성 없이 사용 가능하다.
adtpc(실차 배포용) 및 ROS2 노드 양쪽에서 공통으로 사용한다.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Pose2D:
    """2D 위치 및 방향."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class VehicleCommand:
    """차량 제어 명령 (SS500 CAN ICD 0x301 대응).

    Attributes:
        linear_velocity: 목표 직선 속도 (m/s).
        angular_velocity: 목표 각속도 (rad/s).
        left_track_speed: 좌측 트랙 속도 (m/s).
        right_track_speed: 우측 트랙 속도 (m/s).
        brake: 브레이크 강도 (0.0 ~ 1.0).
        emergency_stop: 비상 정지 여부.
    """

    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    left_track_speed: float = 0.0
    right_track_speed: float = 0.0
    brake: float = 0.0
    emergency_stop: bool = False


@dataclass
class VehicleFeedback:
    """차량 상태 피드백 (SS500 CAN ICD 0x302 대응).

    Attributes:
        x: 현재 위치 x (m).
        y: 현재 위치 y (m).
        yaw: 현재 방향 (rad).
        speed: 현재 속도 (m/s).
        yaw_rate: 현재 yaw rate (rad/s).
        left_track_speed: 좌측 트랙 실제 속도 (m/s).
        right_track_speed: 우측 트랙 실제 속도 (m/s).
        battery_voltage: 배터리 전압 (V).
        engine_rpm: 엔진 RPM.
        error_code: 에러 코드 (0 = 정상).
    """

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0
    left_track_speed: float = 0.0
    right_track_speed: float = 0.0
    battery_voltage: float = 0.0
    engine_rpm: float = 0.0
    error_code: int = 0
