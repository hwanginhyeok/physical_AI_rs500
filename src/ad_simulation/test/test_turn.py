"""선회 SIL 테스트.

제자리 회전, 곡선 주행, 속도 포화 처리를 검증한다.
시뮬레이션 없이도 MockVehicleState 기반 로직 검증을 지원한다.

실행:
    pytest src/ad_simulation/test/test_turn.py -v
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Tuple

import pytest

# --------------------------------------------------------------------------- #
# ROS2 메시지 타입 (없으면 mock으로 대체)
# --------------------------------------------------------------------------- #
try:
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    HAS_ROS_MSGS = True
except ImportError:
    HAS_ROS_MSGS = False


# --------------------------------------------------------------------------- #
# Mock 차량 모델 (test_straight_drive.py와 동일한 구조)
# --------------------------------------------------------------------------- #

@dataclass
class MockVehicleState:
    """차동 구동 차량 상태 시뮬레이터.

    포화(saturation) 처리를 포함한 이상적 운동 모델이다.
    """
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0         # heading [rad]
    vx: float = 0.0          # 선속도 [m/s]
    wz: float = 0.0          # 각속도 [rad/s]
    max_vx: float = 2.0      # 최대 선속도 [m/s]
    max_wz: float = 1.5      # 최대 각속도 [rad/s]
    dt: float = 0.01         # 시뮬레이션 스텝 [s]
    odom_history: List[Tuple[float, float, float, float, float]] = field(
        default_factory=list,
    )

    def apply_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        """cmd_vel 명령 적용 (포화 처리 포함)."""
        self.vx = max(-self.max_vx, min(self.max_vx, linear_x))
        self.wz = max(-self.max_wz, min(self.max_wz, angular_z))

        self.yaw += self.wz * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.vx * math.cos(self.yaw) * self.dt
        self.y += self.vx * math.sin(self.yaw) * self.dt

    def step(self, linear_x: float, angular_z: float, steps: int = 1) -> None:
        """여러 스텝 동안 cmd_vel을 적용한다."""
        for _ in range(steps):
            self.apply_cmd_vel(linear_x, angular_z)
            self.odom_history.append(
                (self.x, self.y, self.yaw, self.vx, self.wz)
            )

    def heading_deg(self) -> float:
        """현재 heading을 도(degree) 단위로 반환."""
        return math.degrees(self.yaw)


# --------------------------------------------------------------------------- #
# 유틸리티
# --------------------------------------------------------------------------- #

def normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화한다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def path_following_error(
    actual: List[Tuple[float, float]],
    reference: List[Tuple[float, float]],
) -> float:
    """실제 경로와 참조 경로 사이의 평균 횡방향 오차를 계산한다.

    각 actual 점에 대해 reference 경로 상의 가장 가까운 점까지의 거리를 사용한다.
    """
    if not actual or not reference:
        return float('inf')

    errors = []
    for ax, ay in actual:
        min_dist = float('inf')
        for rx, ry in reference:
            dist = math.sqrt((ax - rx) ** 2 + (ay - ry) ** 2)
            min_dist = min(min_dist, dist)
        errors.append(min_dist)

    return sum(errors) / len(errors)


# --------------------------------------------------------------------------- #
# 테스트: 제자리 회전 (Pivot Turn)
# --------------------------------------------------------------------------- #

class TestPivotTurn:
    """제자리 회전 명령 후 heading 변화를 검증한다."""

    def test_pivot_turn(self) -> None:
        """[Mock] 제자리 90도 회전 명령 후 heading이 90도에 근접."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        # 제자리 회전: linear.x=0, angular.z=1.0 rad/s
        # 90도(pi/2 ≈ 1.5708 rad) 회전에 필요한 시간 ≈ 1.5708초
        target_yaw = math.pi / 2.0
        angular_speed = 1.0  # rad/s
        turn_time = target_yaw / angular_speed
        steps = int(turn_time / vehicle.dt)

        vehicle.step(linear_x=0.0, angular_z=angular_speed, steps=steps)

        # heading 변화 확인 (허용 오차 5도)
        heading_error = abs(normalize_angle(vehicle.yaw - target_yaw))
        heading_error_deg = math.degrees(heading_error)
        assert heading_error_deg < 5.0, (
            f'제자리 회전 오차: {heading_error_deg:.2f}도 (허용: 5.0도)'
        )

        # 위치 변화가 거의 없어야 함 (제자리 회전이므로)
        displacement = math.sqrt(vehicle.x ** 2 + vehicle.y ** 2)
        assert displacement < 0.1, (
            f'제자리 회전 중 위치 이동: {displacement:.4f}m (허용: 0.1m)'
        )

    def test_pivot_turn_180(self) -> None:
        """[Mock] 180도 제자리 회전."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        target_yaw = math.pi
        angular_speed = 1.0
        turn_time = target_yaw / angular_speed
        steps = int(turn_time / vehicle.dt)

        vehicle.step(linear_x=0.0, angular_z=angular_speed, steps=steps)

        # 180도 회전 확인 (yaw ≈ pi 또는 -pi)
        heading_error = abs(normalize_angle(vehicle.yaw - target_yaw))
        heading_error_deg = math.degrees(heading_error)
        assert heading_error_deg < 5.0, (
            f'180도 회전 오차: {heading_error_deg:.2f}도 (허용: 5.0도)'
        )

    def test_pivot_turn_negative(self) -> None:
        """[Mock] 음의 방향(시계방향) 90도 제자리 회전."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        target_yaw = -math.pi / 2.0
        angular_speed = -1.0
        turn_time = abs(target_yaw / angular_speed)
        steps = int(turn_time / vehicle.dt)

        vehicle.step(linear_x=0.0, angular_z=angular_speed, steps=steps)

        heading_error = abs(normalize_angle(vehicle.yaw - target_yaw))
        heading_error_deg = math.degrees(heading_error)
        assert heading_error_deg < 5.0, (
            f'시계방향 회전 오차: {heading_error_deg:.2f}도 (허용: 5.0도)'
        )


# --------------------------------------------------------------------------- #
# 테스트: 곡선 주행 (Curve Turn)
# --------------------------------------------------------------------------- #

class TestCurveTurn:
    """곡선 주행 시 경로 추종 정확도를 검증한다."""

    @staticmethod
    def _generate_arc_reference(
        radius: float,
        arc_angle: float,
        num_points: int = 100,
    ) -> List[Tuple[float, float]]:
        """원호 참조 경로를 생성한다.

        원점에서 시작하여 반시계 방향으로 arc_angle만큼 회전하는 원호.
        회전 중심은 (0, radius)에 위치.
        """
        points = []
        for i in range(num_points):
            theta = arc_angle * i / (num_points - 1)
            x = radius * math.sin(theta)
            y = radius * (1.0 - math.cos(theta))
            points.append((x, y))
        return points

    def test_curve_turn(self) -> None:
        """[Mock] 반경 5m 원호 주행 시 경로 추종 오차 확인."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        # 원호 주행 파라미터
        radius = 5.0           # 회전 반경 [m]
        linear_speed = 1.0     # 선속도 [m/s]
        angular_speed = linear_speed / radius  # w = v / r

        # 90도(pi/2) 원호 주행
        arc_angle = math.pi / 2.0
        arc_length = radius * arc_angle  # 호의 길이
        travel_time = arc_length / linear_speed
        steps = int(travel_time / vehicle.dt)

        vehicle.step(linear_x=linear_speed, angular_z=angular_speed, steps=steps)

        # 참조 경로 생성
        reference = self._generate_arc_reference(radius, arc_angle, num_points=200)

        # 실제 경로 추출
        actual = [(rec[0], rec[1]) for rec in vehicle.odom_history]

        # 경로 추종 오차 계산
        avg_error = path_following_error(actual, reference)
        assert avg_error < 0.5, (
            f'곡선 추종 평균 오차: {avg_error:.4f}m (허용: 0.5m)'
        )

    def test_curve_turn_tight(self) -> None:
        """[Mock] 반경 2m 급선회 시 경로 추종 확인."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        radius = 2.0
        linear_speed = 0.5
        angular_speed = linear_speed / radius

        arc_angle = math.pi / 2.0
        arc_length = radius * arc_angle
        travel_time = arc_length / linear_speed
        steps = int(travel_time / vehicle.dt)

        vehicle.step(linear_x=linear_speed, angular_z=angular_speed, steps=steps)

        reference = self._generate_arc_reference(radius, arc_angle, num_points=200)
        actual = [(rec[0], rec[1]) for rec in vehicle.odom_history]

        avg_error = path_following_error(actual, reference)
        assert avg_error < 0.5, (
            f'급선회 추종 평균 오차: {avg_error:.4f}m (허용: 0.5m)'
        )

    def test_curve_final_heading(self) -> None:
        """[Mock] 90도 원호 주행 후 heading이 약 90도인지 확인."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        radius = 5.0
        linear_speed = 1.0
        angular_speed = linear_speed / radius

        arc_angle = math.pi / 2.0
        arc_length = radius * arc_angle
        travel_time = arc_length / linear_speed
        steps = int(travel_time / vehicle.dt)

        vehicle.step(linear_x=linear_speed, angular_z=angular_speed, steps=steps)

        expected_heading = math.pi / 2.0
        heading_error = abs(normalize_angle(vehicle.yaw - expected_heading))
        heading_error_deg = math.degrees(heading_error)
        assert heading_error_deg < 5.0, (
            f'원호 종료 후 heading 오차: {heading_error_deg:.2f}도 (허용: 5.0도)'
        )


# --------------------------------------------------------------------------- #
# 테스트: 속도 포화 (Speed Saturation)
# --------------------------------------------------------------------------- #

class TestSpeedSaturation:
    """최대 속도 초과 명령 시 포화 처리가 올바른지 검증."""

    def test_speed_saturation_linear(self) -> None:
        """[Mock] 최대 선속도 초과 명령 → 포화 처리."""
        vehicle = MockVehicleState(max_vx=2.0, max_wz=1.5)

        # 최대 속도(2.0)를 초과하는 명령 (5.0 m/s)
        vehicle.step(linear_x=5.0, angular_z=0.0, steps=100)

        assert abs(vehicle.vx - vehicle.max_vx) < 0.001, (
            f'선속도 포화 실패: {vehicle.vx:.4f} m/s (기대: {vehicle.max_vx})'
        )

    def test_speed_saturation_angular(self) -> None:
        """[Mock] 최대 각속도 초과 명령 → 포화 처리."""
        vehicle = MockVehicleState(max_vx=2.0, max_wz=1.5)

        # 최대 각속도(1.5)를 초과하는 명령 (3.0 rad/s)
        vehicle.step(linear_x=0.0, angular_z=3.0, steps=100)

        assert abs(vehicle.wz - vehicle.max_wz) < 0.001, (
            f'각속도 포화 실패: {vehicle.wz:.4f} rad/s (기대: {vehicle.max_wz})'
        )

    def test_speed_saturation_negative(self) -> None:
        """[Mock] 음의 방향 최대 속도 초과 → 포화 처리."""
        vehicle = MockVehicleState(max_vx=2.0, max_wz=1.5)

        # 음의 방향 최대 속도 초과
        vehicle.step(linear_x=-5.0, angular_z=-3.0, steps=100)

        assert abs(vehicle.vx - (-vehicle.max_vx)) < 0.001, (
            f'음의 선속도 포화 실패: {vehicle.vx:.4f} m/s (기대: {-vehicle.max_vx})'
        )
        assert abs(vehicle.wz - (-vehicle.max_wz)) < 0.001, (
            f'음의 각속도 포화 실패: {vehicle.wz:.4f} rad/s (기대: {-vehicle.max_wz})'
        )

    def test_speed_within_limits(self) -> None:
        """[Mock] 허용 범위 내 명령 → 그대로 적용."""
        vehicle = MockVehicleState(max_vx=2.0, max_wz=1.5)

        # 허용 범위 내 명령
        target_vx = 1.0
        target_wz = 0.8
        vehicle.step(linear_x=target_vx, angular_z=target_wz, steps=100)

        assert abs(vehicle.vx - target_vx) < 0.001, (
            f'범위 내 선속도 적용 오류: {vehicle.vx:.4f} m/s (기대: {target_vx})'
        )
        assert abs(vehicle.wz - target_wz) < 0.001, (
            f'범위 내 각속도 적용 오류: {vehicle.wz:.4f} rad/s (기대: {target_wz})'
        )

    def test_speed_saturation_combined(self) -> None:
        """[Mock] 선속도+각속도 동시 포화 처리."""
        vehicle = MockVehicleState(max_vx=2.0, max_wz=1.5)

        # 선속도, 각속도 모두 초과
        vehicle.step(linear_x=10.0, angular_z=5.0, steps=100)

        assert abs(vehicle.vx - vehicle.max_vx) < 0.001, (
            f'동시 포화 시 선속도 오류: {vehicle.vx:.4f} m/s'
        )
        assert abs(vehicle.wz - vehicle.max_wz) < 0.001, (
            f'동시 포화 시 각속도 오류: {vehicle.wz:.4f} rad/s'
        )
