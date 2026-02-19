"""제어 모듈 - 계획된 속도/조향을 실제 제어 명령으로 변환."""

import math

from rclpy.node import Node
from geometry_msgs.msg import Twist


class ControlModule:
    """판단 모듈의 목표값을 실제 차량 제어 명령으로 변환하여 퍼블리시.

    PID 제어를 통해 목표 속도와 조향 각도를 추종하는
    Twist 메시지를 생성한다.
    """

    def __init__(self, node: Node):
        self._node = node

        # 제어 명령 퍼블리셔
        self._cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

        # 현재 상태
        self._current_speed = 0.0
        self._current_steering = 0.0

        # PID 게인 (파라미터에서 읽기)
        self._speed_kp = node.get_parameter('control.speed_kp').value
        self._speed_ki = node.get_parameter('control.speed_ki').value
        self._speed_kd = node.get_parameter('control.speed_kd').value

        self._steer_kp = node.get_parameter('control.steer_kp').value

        # PID 상태
        self._speed_error_integral = 0.0
        self._speed_error_prev = 0.0

        node.get_logger().info('[제어] 모듈 초기화 완료')

    def execute(self, plan_result: dict):
        """계획 결과를 기반으로 제어 명령 생성 및 퍼블리시.

        Args:
            plan_result: PlanningModule.get_plan_result()의 반환값
        """
        target_speed = plan_result.get('target_speed', 0.0)
        target_steering = plan_result.get('target_steering', 0.0)

        # 속도 PID 제어
        speed_cmd = self._pid_speed(target_speed)

        # 조향 P 제어
        steer_cmd = self._p_steering(target_steering)

        # Twist 메시지 생성 및 퍼블리시
        twist = Twist()
        twist.linear.x = speed_cmd
        twist.angular.z = steer_cmd
        self._cmd_pub.publish(twist)

    def _pid_speed(self, target: float) -> float:
        """속도 PID 제어."""
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
        """조향 P 제어."""
        error = target - self._current_steering
        return self._steer_kp * error

    def update_current_state(self, speed: float, steering: float):
        """외부에서 현재 차량 상태를 업데이트."""
        self._current_speed = speed
        self._current_steering = steering
