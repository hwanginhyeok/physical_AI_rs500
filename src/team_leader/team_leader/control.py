"""제어 모듈 - 계획된 속도/조향을 실제 제어 명령으로 변환.

Pure Pursuit 경로 추종과 스키드 스티어 운동학 모델을 통합하여
궤도차량에 적합한 제어 명령을 생성한다.
기존 PID 제어는 경로가 설정되지 않았을 때 fallback으로 동작한다.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from rclpy.node import Node
from geometry_msgs.msg import Twist

from team_leader.pure_pursuit import Pose2D, PurePursuitConfig, PurePursuitTracker
from team_leader.skid_steer_model import SkidSteerModel


@dataclass
class VehicleState:
    """차량 상태 정보."""

    x: float = 0.0
    """위치 x (m)."""

    y: float = 0.0
    """위치 y (m)."""

    yaw: float = 0.0
    """방향 (rad)."""

    speed: float = 0.0
    """현재 속도 (m/s)."""

    steering: float = 0.0
    """현재 조향각 (rad)."""


class ControlModule:
    """판단 모듈의 목표값을 실제 차량 제어 명령으로 변환하여 퍼블리시.

    경로가 설정된 경우 Pure Pursuit + SkidSteer 모델을 사용하고,
    경로가 없으면 기존 PID 제어를 fallback으로 사용한다.

    Attributes:
        tracker: Pure Pursuit 경로 추종기.
        skid_model: 스키드 스티어 운동학 모델.
    """

    def __init__(self, node: Node) -> None:
        """초기화.

        Args:
            node: ROS2 노드 인스턴스 (파라미터 및 퍼블리셔 접근).
        """
        self._node = node

        # 제어 명령 퍼블리셔
        cmd_vel_topic = node.get_parameter('topics.cmd_vel').value
        self._cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

        # 현재 상태
        self._current_speed: float = 0.0
        self._current_steering: float = 0.0

        # PID 게인 (파라미터에서 읽기)
        self._speed_kp: float = node.get_parameter('control.speed_kp').value
        self._speed_ki: float = node.get_parameter('control.speed_ki').value
        self._speed_kd: float = node.get_parameter('control.speed_kd').value
        self._steer_kp: float = node.get_parameter('control.steer_kp').value

        # PID 상태
        self._speed_error_integral: float = 0.0
        self._speed_error_prev: float = 0.0

        # Pure Pursuit 경로 추종기
        pp_config = PurePursuitConfig(
            lookahead_gain=0.8,
            min_lookahead=1.0,
            max_lookahead=5.0,
            goal_tolerance=0.5,
            max_linear_speed=1.0,
            track_width=1.4,
        )
        self.tracker: PurePursuitTracker = PurePursuitTracker(config=pp_config)

        # 스키드 스티어 운동학 모델
        self.skid_model: SkidSteerModel = SkidSteerModel(
            track_width=1.4,
            max_speed=1.0,
            steering_efficiency=0.8,
        )

        # 경로 추종 모드 활성화 여부
        self._path_tracking_active: bool = False

        node.get_logger().info('[제어] 모듈 초기화 완료 (Pure Pursuit + SkidSteer)')

    # ------------------------------------------------------------------ #
    #  경로 설정 / 추종
    # ------------------------------------------------------------------ #

    def set_path(self, waypoints) -> None:
        """추종할 경로를 설정하고 경로 추종 모드를 활성화한다.

        Args:
            waypoints: 경로 데이터. list of (x, y) 또는 nav_msgs/Path 호환 객체.
        """
        self.tracker.set_path(waypoints)
        self._path_tracking_active = bool(self.tracker.path)
        if self._path_tracking_active:
            self._node.get_logger().info(
                f'[제어] 경로 설정 완료 ({len(self.tracker.path)}개 웨이포인트)')
        else:
            self._node.get_logger().warn('[제어] 빈 경로가 설정됨')

    def compute_control(self, vehicle_state: VehicleState) -> Tuple[float, float]:
        """차량 상태를 기반으로 좌/우 트랙 속도를 계산한다.

        경로가 활성화된 경우 Pure Pursuit 결과를 SkidSteer 모델로 변환하고,
        그렇지 않으면 기존 PID fallback을 사용한다.

        Args:
            vehicle_state: 현재 차량 상태.

        Returns:
            (v_left, v_right) 좌/우 트랙 속도 (m/s).
        """
        # 내부 상태 갱신
        self._current_speed = vehicle_state.speed
        self._current_steering = vehicle_state.steering

        if self._path_tracking_active and not self.tracker.is_goal_reached:
            return self._compute_path_tracking(vehicle_state)

        # 경로 완료 또는 미설정 → 정지
        if self._path_tracking_active and self.tracker.is_goal_reached:
            self._node.get_logger().info('[제어] 경로 추종 완료 — 정지')
            self._path_tracking_active = False
            return 0.0, 0.0

        # Fallback: PID 기반 (경로 미설정 시)
        return 0.0, 0.0

    def _compute_path_tracking(
        self, vehicle_state: VehicleState
    ) -> Tuple[float, float]:
        """Pure Pursuit + SkidSteer 경로 추종 제어를 수행한다.

        Args:
            vehicle_state: 현재 차량 상태.

        Returns:
            (v_left, v_right) 트랙 속도 (m/s).
        """
        pose = Pose2D(
            x=vehicle_state.x,
            y=vehicle_state.y,
            yaw=vehicle_state.yaw,
        )

        # Pure Pursuit으로 선속도/각속도 계산
        linear_vel, angular_vel = self.tracker.compute(
            pose, vehicle_state.speed)

        # SkidSteer 모델로 트랙 속도 변환
        v_left, v_right = self.skid_model.twist_to_tracks(
            linear_vel, angular_vel)

        return v_left, v_right

    # ------------------------------------------------------------------ #
    #  기존 PID 기반 제어 (fallback)
    # ------------------------------------------------------------------ #

    def execute(self, plan_result: dict) -> None:
        """계획 결과를 기반으로 제어 명령 생성 및 퍼블리시.

        경로 추종 모드가 활성화되어 있으면 Pure Pursuit 결과를 사용하고,
        그렇지 않으면 기존 PID 제어를 사용한다.

        Args:
            plan_result: PlanningModule.get_plan_result()의 반환값.
        """
        if self._path_tracking_active and not self.tracker.is_goal_reached:
            # Pure Pursuit 모드: vehicle_state 기반 제어
            pose = Pose2D(
                x=plan_result.get('x', 0.0),
                y=plan_result.get('y', 0.0),
                yaw=plan_result.get('yaw', 0.0),
            )
            speed = plan_result.get('current_speed', self._current_speed)

            linear_vel, angular_vel = self.tracker.compute(pose, speed)
            v_left, v_right = self.skid_model.twist_to_tracks(
                linear_vel, angular_vel)

            # 트랙 속도를 Twist로 변환하여 퍼블리시
            linear_pub, angular_pub = self.skid_model.tracks_to_twist(
                v_left, v_right)

            twist = Twist()
            twist.linear.x = linear_pub
            twist.angular.z = angular_pub
            self._cmd_pub.publish(twist)

            if self.tracker.is_goal_reached:
                self._node.get_logger().info('[제어] 경로 추종 완료')
                self._path_tracking_active = False
            return

        # Fallback: 기존 PID 제어
        target_speed = plan_result.get('target_speed', 0.0)
        target_steering = plan_result.get('target_steering', 0.0)

        speed_cmd = self._pid_speed(target_speed)
        steer_cmd = self._p_steering(target_steering)

        twist = Twist()
        twist.linear.x = speed_cmd
        twist.angular.z = steer_cmd
        self._cmd_pub.publish(twist)

    def _pid_speed(self, target: float) -> float:
        """속도 PID 제어.

        Args:
            target: 목표 속도 (m/s).

        Returns:
            PID 제어 출력 속도 (m/s). 음수 방지.
        """
        error = target - self._current_speed
        self._speed_error_integral += error
        derivative = error - self._speed_error_prev
        self._speed_error_prev = error

        output = (
            self._speed_kp * error
            + self._speed_ki * self._speed_error_integral
            + self._speed_kd * derivative
        )
        return max(0.0, output)

    def _p_steering(self, target: float) -> float:
        """조향 P 제어.

        Args:
            target: 목표 조향각 (rad).

        Returns:
            P 제어 출력 조향 명령 (rad/s).
        """
        error = target - self._current_steering
        return self._steer_kp * error

    def update_current_state(self, speed: float, steering: float) -> None:
        """외부에서 현재 차량 상태를 업데이트한다.

        Args:
            speed: 현재 속도 (m/s).
            steering: 현재 조향각 (rad).
        """
        self._current_speed = speed
        self._current_steering = steering

    def reset_pid(self) -> None:
        """PID 적분기 및 이전 오차를 초기화한다."""
        self._speed_error_integral = 0.0
        self._speed_error_prev = 0.0
