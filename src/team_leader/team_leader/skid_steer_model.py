"""ICR 기반 스키드 스티어 조향 모델 모듈.

궤도차량(tracked vehicle)의 운동학적 모델을 구현한다.
좌/우 트랙 속도 차이에 의한 회전을 ICR(Instantaneous Center of Rotation)
기반으로 모델링하며, 슬립 보정을 지원한다.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Pose2D:
    """2D 위치 및 방향."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class SkidSteerModel:
    """ICR 기반 스키드 스티어 운동학 모델.

    좌/우 트랙 속도로부터 차량의 선속도와 각속도를 계산하고,
    ICR(순간 회전 중심)을 추정하여 다음 위치를 예측한다.

    Attributes:
        track_width: 좌/우 트랙 사이 간격 (m).
        max_speed: 트랙 최대 속도 (m/s).
        steering_efficiency: 조향 효율 계수 (0~1). 미끄러짐으로 인한 손실 반영.
        slip_ratio: 슬립 보정 팩터 (0.0 = 슬립 없음).
    """

    def __init__(
        self,
        track_width: float = 1.4,
        max_speed: float = 1.0,
        steering_efficiency: float = 0.8,
    ) -> None:
        """초기화.

        Args:
            track_width: 트랙 간격 (m). 좌/우 트랙 중심 사이 거리.
            max_speed: 각 트랙의 최대 허용 속도 (m/s).
            steering_efficiency: 조향 효율 (0~1). 이상적 조향 대비 실제 효율.
        """
        self.track_width: float = track_width
        self.max_speed: float = max_speed
        self.steering_efficiency: float = steering_efficiency
        self.slip_ratio: float = 0.0

    def twist_to_tracks(
        self, linear: float, angular: float
    ) -> Tuple[float, float]:
        """(선속도, 각속도)를 좌/우 트랙 속도로 변환한다.

        차동 구동 역기구학:
            v_left  = v - (omega * W) / (2 * eta)
            v_right = v + (omega * W) / (2 * eta)

        여기서 eta는 조향 효율이며, 효율이 낮을수록 같은 각속도를 내기 위해
        더 큰 트랙 속도 차이가 필요하다.

        Args:
            linear: 목표 선속도 (m/s).
            angular: 목표 각속도 (rad/s).

        Returns:
            (v_left, v_right) 포화 처리된 트랙 속도 (m/s).
        """
        # 조향 효율을 반영한 유효 트랙 간격
        effective_width = self.track_width / max(self.steering_efficiency, 1e-6)
        half_w = effective_width / 2.0

        v_left = linear - angular * half_w
        v_right = linear + angular * half_w

        # 포화 처리 (비율 유지)
        v_left, v_right = self._saturate(v_left, v_right)

        return v_left, v_right

    def tracks_to_twist(
        self, v_left: float, v_right: float
    ) -> Tuple[float, float]:
        """좌/우 트랙 속도를 (선속도, 각속도)로 역변환한다.

        차동 구동 순기구학:
            v     = (v_right + v_left) / 2
            omega = (v_right - v_left) * eta / W

        슬립 보정이 적용된 경우 유효 속도를 보정한다.

        Args:
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).

        Returns:
            (linear, angular) 선속도(m/s)와 각속도(rad/s).
        """
        # 슬립 보정 적용
        effective_left = v_left * (1.0 - self.slip_ratio)
        effective_right = v_right * (1.0 - self.slip_ratio)

        linear = (effective_right + effective_left) / 2.0
        angular = (
            (effective_right - effective_left)
            * self.steering_efficiency
            / self.track_width
        )

        return linear, angular

    def predict_pose(
        self,
        current_pose: Pose2D,
        v_left: float,
        v_right: float,
        dt: float,
    ) -> Pose2D:
        """ICR 기반으로 다음 타임스텝의 위치를 예측한다.

        직진(v_left ~= v_right) 시에는 직선 모델을 사용하고,
        회전 시에는 ICR(순간 회전 중심) 주위의 원호 모델을 사용한다.

        Args:
            current_pose: 현재 위치 및 방향.
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).
            dt: 시간 간격 (s).

        Returns:
            예측된 다음 위치 (Pose2D).
        """
        linear, angular = self.tracks_to_twist(v_left, v_right)

        new_pose = Pose2D(
            x=current_pose.x,
            y=current_pose.y,
            yaw=current_pose.yaw,
        )

        if abs(angular) < 1e-6:
            # 직진 모델: 곡률이 거의 0
            new_pose.x += linear * math.cos(current_pose.yaw) * dt
            new_pose.y += linear * math.sin(current_pose.yaw) * dt
        else:
            # ICR 기반 원호 모델
            radius = linear / angular  # 회전 반경 = v / omega
            icr_x = current_pose.x - radius * math.sin(current_pose.yaw)
            icr_y = current_pose.y + radius * math.cos(current_pose.yaw)

            d_theta = angular * dt
            new_pose.yaw = _normalize_angle(current_pose.yaw + d_theta)

            # ICR을 중심으로 회전
            new_pose.x = icr_x + radius * math.sin(new_pose.yaw)
            new_pose.y = icr_y - radius * math.cos(new_pose.yaw)

        return new_pose

    def estimate_icr(
        self, v_left: float, v_right: float
    ) -> float:
        """순간 회전 중심(ICR)의 y좌표를 추정한다.

        ICR은 차량 좌표계에서 좌/우 트랙 사이에 위치하며,
        그 y좌표는 회전 반경의 부호와 크기를 나타낸다.

        - v_left == v_right: ICR이 무한대 (직진)
        - v_left == -v_right: ICR이 차량 중심 (제자리 회전)
        - v_left == 0: ICR이 좌측 트랙 위 (좌측 고정 회전)

        Args:
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).

        Returns:
            ICR의 y좌표 (m). 차량 좌표계 기준.
            좌/우 속도가 동일하면 float('inf')를 반환.
        """
        # 슬립 보정 적용
        eff_left = v_left * (1.0 - self.slip_ratio)
        eff_right = v_right * (1.0 - self.slip_ratio)

        diff = eff_right - eff_left
        if abs(diff) < 1e-9:
            return float('inf')  # 직진 — ICR이 무한대

        half_w = self.track_width / 2.0

        # ICR y좌표: y_icr = W/2 * (v_right + v_left) / (v_right - v_left)
        # 양수 → 좌측에 ICR (우회전), 음수 → 우측에 ICR (좌회전)
        icr_y = half_w * (eff_right + eff_left) / diff

        return icr_y

    def _saturate(
        self, v_left: float, v_right: float
    ) -> Tuple[float, float]:
        """트랙 속도를 최대 속도 이내로 포화 처리한다.

        두 트랙 속도의 비율을 유지하면서 포화시킨다.

        Args:
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).

        Returns:
            포화 처리된 (v_left, v_right).
        """
        max_abs = max(abs(v_left), abs(v_right))
        if max_abs > self.max_speed and max_abs > 1e-9:
            scale = self.max_speed / max_abs
            v_left *= scale
            v_right *= scale
        return v_left, v_right


def _normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi) 범위로 정규화한다.

    Args:
        angle: 입력 각도 (rad).

    Returns:
        정규화된 각도 (rad).
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle
