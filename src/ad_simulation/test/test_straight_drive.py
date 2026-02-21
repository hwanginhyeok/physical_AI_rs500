"""직진 주행 SIL 테스트.

cmd_vel 명령 발행 후 odom(또는 mock) 응답을 검증한다.
시뮬레이션이 실행 중이지 않아도 mock 기반으로 로직을 검증할 수 있다.

실행:
    # mock 전용 (시뮬레이션 불필요)
    pytest src/ad_simulation/test/test_straight_drive.py -v

    # 시뮬레이션 연동 (Gazebo + rclpy 필요)
    pytest src/ad_simulation/test/test_straight_drive.py -v --run-sim
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional
from unittest.mock import MagicMock

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

    # Mock 메시지 클래스 정의
    class _Vec3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class _Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class _TwistInner:
        def __init__(self):
            self.linear = _Vec3()
            self.angular = _Vec3()

    class Twist:  # type: ignore[no-redef]
        def __init__(self):
            self.linear = _Vec3()
            self.angular = _Vec3()

    class _Pose:
        def __init__(self):
            self.position = _Vec3()
            self.orientation = _Quaternion()

    class _PoseWithCovariance:
        def __init__(self):
            self.pose = _Pose()

    class _TwistWithCovariance:
        def __init__(self):
            self.twist = _TwistInner()

    class Odometry:  # type: ignore[no-redef]
        def __init__(self):
            self.pose = _PoseWithCovariance()
            self.twist = _TwistWithCovariance()


# --------------------------------------------------------------------------- #
# 유틸리티 함수
# --------------------------------------------------------------------------- #

def quaternion_to_yaw(q) -> float:
    """쿼터니언에서 yaw(z축 회전) 각도를 추출한다 (라디안)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    """yaw 각도(라디안)를 쿼터니언으로 변환한다."""
    q = _Quaternion() if not HAS_ROS_MSGS else type(
        'Q', (), {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    )()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# --------------------------------------------------------------------------- #
# Mock 차량 모델: 시뮬레이션 없이 주행 로직 검증용
# --------------------------------------------------------------------------- #

@dataclass
class MockVehicleState:
    """간단한 차동 구동 차량 상태 시뮬레이터.

    물리 시뮬레이션 없이 cmd_vel 명령에 대한 이상적(ideal) 응답을 계산한다.
    """
    x: float = 0.0           # 위치 [m]
    y: float = 0.0
    yaw: float = 0.0         # heading [rad]
    vx: float = 0.0          # 선속도 [m/s]
    wz: float = 0.0          # 각속도 [rad/s]
    max_vx: float = 2.0      # 최대 선속도 [m/s]
    max_wz: float = 1.5      # 최대 각속도 [rad/s]
    dt: float = 0.01         # 시뮬레이션 스텝 [s]
    odom_history: List = field(default_factory=list)

    def apply_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        """cmd_vel 명령을 적용하고 상태를 갱신한다 (포화 처리 포함)."""
        # 속도 포화 (saturation)
        self.vx = max(-self.max_vx, min(self.max_vx, linear_x))
        self.wz = max(-self.max_wz, min(self.max_wz, angular_z))

        # 상태 갱신 (오일러 적분)
        self.yaw += self.wz * self.dt
        # yaw를 [-pi, pi] 범위로 정규화
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.vx * math.cos(self.yaw) * self.dt
        self.y += self.vx * math.sin(self.yaw) * self.dt

    def step(self, linear_x: float, angular_z: float, steps: int = 1) -> None:
        """여러 스텝 동안 cmd_vel을 적용한다."""
        for _ in range(steps):
            self.apply_cmd_vel(linear_x, angular_z)
            self.odom_history.append((self.x, self.y, self.yaw, self.vx, self.wz))

    def get_odom(self) -> 'Odometry':
        """현재 상태를 Odometry 메시지 형태로 반환한다."""
        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = yaw_to_quaternion(self.yaw)
        odom.pose.pose.orientation.x = q.x
        odom.pose.pose.orientation.y = q.y
        odom.pose.pose.orientation.z = q.z
        odom.pose.pose.orientation.w = q.w
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        return odom


# --------------------------------------------------------------------------- #
# pytest 커맨드라인 옵션
# --------------------------------------------------------------------------- #

def pytest_addoption(parser):
    """시뮬레이션 연동 테스트 옵션을 추가한다."""
    try:
        parser.addoption(
            '--run-sim',
            action='store_true',
            default=False,
            help='Gazebo 시뮬레이션 연동 테스트 실행',
        )
    except ValueError:
        pass  # 이미 등록됨


# --------------------------------------------------------------------------- #
# 테스트: 속도 응답 (cmd_vel -> odom)
# --------------------------------------------------------------------------- #

class TestVelocityResponse:
    """cmd_vel 발행 후 odom에서 속도 응답을 확인하는 테스트."""

    def test_velocity_response_mock(self) -> None:
        """[Mock] cmd_vel 발행 후 odom 속도가 명령값과 일치하는지 확인."""
        vehicle = MockVehicleState()

        # 전진 명령: linear.x = 1.0 m/s
        target_vx = 1.0
        vehicle.step(linear_x=target_vx, angular_z=0.0, steps=100)

        odom = vehicle.get_odom()
        # 속도 응답 확인 (mock에서는 즉시 반영)
        assert abs(odom.twist.twist.linear.x - target_vx) < 0.01, (
            f'속도 응답 오차: {abs(odom.twist.twist.linear.x - target_vx):.4f} m/s'
        )

    def test_velocity_response_zero(self) -> None:
        """[Mock] 정지 명령 후 속도가 0인지 확인."""
        vehicle = MockVehicleState()

        # 먼저 전진
        vehicle.step(linear_x=1.0, angular_z=0.0, steps=50)
        # 정지 명령
        vehicle.step(linear_x=0.0, angular_z=0.0, steps=50)

        odom = vehicle.get_odom()
        assert abs(odom.twist.twist.linear.x) < 0.01, (
            f'정지 후 잔여 속도: {odom.twist.twist.linear.x:.4f} m/s'
        )

    @pytest.mark.skipif(not HAS_ROS_MSGS, reason='ROS2 메시지 타입 미설치')
    def test_velocity_response_sim(self, ros2_node, gazebo_process) -> None:
        """[시뮬레이션] cmd_vel 발행 후 실제 odom 속도 응답 확인."""
        if gazebo_process is None:
            pytest.skip('Gazebo가 실행되지 않음')

        node, executor = ros2_node
        pub = node.create_publisher(Twist, '/cmd_vel', 10)
        received_odom = []

        def odom_cb(msg: Odometry) -> None:
            received_odom.append(msg)

        sub = node.create_subscription(Odometry, '/odom', odom_cb, 10)

        # cmd_vel 발행
        cmd = Twist()
        cmd.linear.x = 0.5

        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            pub.publish(cmd)
            executor.spin_once(timeout_sec=0.1)
            if len(received_odom) >= 10:
                break

        assert len(received_odom) > 0, 'odom 메시지를 수신하지 못했습니다.'

        # 마지막 odom의 선속도가 0보다 큰지 확인
        last_vx = received_odom[-1].twist.twist.linear.x
        assert last_vx > 0.1, f'속도 응답 부족: {last_vx:.4f} m/s'

        node.destroy_publisher(pub)
        node.destroy_subscription(sub)


# --------------------------------------------------------------------------- #
# 테스트: 직진 정확도 (10m 주행 후 횡방향 오차)
# --------------------------------------------------------------------------- #

class TestStraightLineAccuracy:
    """10m 직진 후 횡방향(lateral) 오차가 0.5m 이내인지 검증."""

    @staticmethod
    def _drive_straight(vehicle: MockVehicleState, distance: float, speed: float) -> None:
        """목표 거리만큼 직진한다."""
        steps_needed = int(distance / (speed * vehicle.dt))
        vehicle.step(linear_x=speed, angular_z=0.0, steps=steps_needed)

    def test_straight_line_accuracy(self) -> None:
        """[Mock] 10m 직진 후 횡방향 오차 0.5m 이내."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        # 10m 직진 (1.0 m/s)
        self._drive_straight(vehicle, distance=10.0, speed=1.0)

        # 횡방향 오차 = |y|  (초기 heading=0 이므로 x 방향 직진)
        lateral_error = abs(vehicle.y)
        assert lateral_error < 0.5, (
            f'횡방향 오차 초과: {lateral_error:.4f}m (허용: 0.5m)'
        )

        # 전진 거리 확인 (오차 10% 이내)
        forward_distance = vehicle.x
        assert abs(forward_distance - 10.0) < 1.0, (
            f'전진 거리 오차: {forward_distance:.2f}m (목표: 10.0m)'
        )

    def test_straight_line_with_initial_offset(self) -> None:
        """[Mock] 초기 yaw 오프셋이 있어도 직진 시 횡방향 오차 확인."""
        # 초기 heading에 약간의 오프셋 (2도)
        initial_yaw = math.radians(2.0)
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=initial_yaw)

        self._drive_straight(vehicle, distance=10.0, speed=1.0)

        # 2도 오프셋으로 10m 주행 시 횡방향 편차 ≈ 10 * sin(2deg) ≈ 0.35m
        lateral_error = abs(vehicle.y)
        assert lateral_error < 0.5, (
            f'횡방향 오차 초과 (초기 오프셋 포함): {lateral_error:.4f}m (허용: 0.5m)'
        )


# --------------------------------------------------------------------------- #
# 테스트: 헤딩 안정성 (직진 중 heading 변동 5도 이내)
# --------------------------------------------------------------------------- #

class TestHeadingStability:
    """직진 주행 중 heading 변동이 5도 이내인지 검증."""

    def test_heading_stability(self) -> None:
        """[Mock] 직진 주행 시 heading 변동이 5도 이내."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        # 10m 직진
        total_steps = int(10.0 / (1.0 * vehicle.dt))
        vehicle.step(linear_x=1.0, angular_z=0.0, steps=total_steps)

        # 기록된 heading 값에서 최대/최소 차이 계산
        headings = [record[2] for record in vehicle.odom_history]
        if headings:
            heading_range = max(headings) - min(headings)
            heading_range_deg = math.degrees(heading_range)
            assert heading_range_deg < 5.0, (
                f'heading 변동 초과: {heading_range_deg:.2f}도 (허용: 5.0도)'
            )

    def test_heading_stability_with_disturbance(self) -> None:
        """[Mock] 미세한 각속도 노이즈가 있어도 heading 변동 5도 이내."""
        vehicle = MockVehicleState(x=0.0, y=0.0, yaw=0.0)

        # 약간의 각속도 노이즈를 포함한 직진 (회전 명령 0.01 rad/s)
        # 현실적으로 제어기에서 미세한 보정이 들어가는 상황 시뮬레이션
        noise_wz = 0.01  # 매우 작은 회전 노이즈

        total_steps = int(10.0 / (1.0 * vehicle.dt))
        for i in range(total_steps):
            # 홀수 스텝에서 양의 노이즈, 짝수에서 음의 노이즈 (진동)
            wz = noise_wz if i % 2 == 0 else -noise_wz
            vehicle.step(linear_x=1.0, angular_z=wz, steps=1)

        headings = [record[2] for record in vehicle.odom_history]
        heading_range = max(headings) - min(headings)
        heading_range_deg = math.degrees(heading_range)
        assert heading_range_deg < 5.0, (
            f'heading 변동 초과 (노이즈 포함): {heading_range_deg:.2f}도 (허용: 5.0도)'
        )
