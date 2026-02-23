"""제어 노드 - 계획된 속도/조향을 실제 제어 명령으로 변환.

Pure Pursuit 경로 추종과 스키드 스티어 운동학 모델을 통합하여
궤도차량에 적합한 제어 명령을 생성한다.
"""

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from ad_core.datatypes import Pose2D
from ad_core.pure_pursuit import (
    PurePursuitConfig,
    PurePursuitTracker,
    SlipCompensatedPurePursuit,
    SlipCompensatedPurePursuitConfig,
    TerrainType as SlipTerrainType,
    terrain_type_from_name,
)
from ad_core.skid_steer_model import (
    CalibratedSkidSteerModel,
    SkidSteerModel,
)


@dataclass
class VehicleState:
    """차량 상태 정보."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    steering: float = 0.0
    yaw_rate: float = 0.0
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    terrain_name: str = ""


class ControlModule:
    """판단 모듈의 목표값을 실제 차량 제어 명령으로 변환하여 퍼블리시."""

    def __init__(
        self,
        node: Node,
        enable_slip_compensation: bool = False,
    ) -> None:
        self._node = node
        self.slip_compensation_enabled: bool = enable_slip_compensation

        cmd_vel_topic = node.get_parameter('topics.cmd_vel').value
        self._cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

        self._current_speed: float = 0.0
        self._current_steering: float = 0.0

        self._speed_kp: float = node.get_parameter('control.speed_kp').value
        self._speed_ki: float = node.get_parameter('control.speed_ki').value
        self._speed_kd: float = node.get_parameter('control.speed_kd').value
        self._steer_kp: float = node.get_parameter('control.steer_kp').value

        self._speed_error_integral: float = 0.0
        self._speed_error_prev: float = 0.0

        if enable_slip_compensation:
            slip_pp_config = SlipCompensatedPurePursuitConfig(
                lookahead_gain=0.8,
                min_lookahead=1.0,
                max_lookahead=5.0,
                goal_tolerance=0.5,
                max_linear_speed=1.0,
                track_width=1.4,
                slip_lookahead_gain=2.0,
                max_slip_lookahead_ratio=2.0,
                longitudinal_compensation_gain=1.0,
                lateral_compensation_gain=0.8,
                slip_speed_reduction_gain=0.5,
                min_speed_ratio=0.2,
                ema_alpha=0.15,
                enable_terrain_adaptation=True,
            )
            self.slip_tracker: SlipCompensatedPurePursuit = (
                SlipCompensatedPurePursuit(config=slip_pp_config)
            )
            self.tracker: PurePursuitTracker = self.slip_tracker
        else:
            pp_config = PurePursuitConfig(
                lookahead_gain=0.8,
                min_lookahead=1.0,
                max_lookahead=5.0,
                goal_tolerance=0.5,
                max_linear_speed=1.0,
                track_width=1.4,
            )
            self.tracker = PurePursuitTracker(config=pp_config)
            self.slip_tracker = None  # type: ignore[assignment]

        calibration_file = self._get_param_safe(
            'control.calibration_file', None)
        ema_alpha = self._get_param_safe(
            'control.calibration_ema_alpha', 0.05)
        calibration_enabled = self._get_param_safe(
            'control.calibration_enabled', True)

        self.skid_model: CalibratedSkidSteerModel = CalibratedSkidSteerModel(
            track_width=1.4,
            max_speed=1.0,
            steering_efficiency=0.8,
            ema_alpha=ema_alpha,
            calibration_file=calibration_file,
            calibration_enabled=calibration_enabled,
        )

        self._path_tracking_active: bool = False
        self._last_commanded_linear: float = 0.0
        self._last_commanded_angular: float = 0.0
        self._calibration_save_interval: int = 100
        self._calibration_update_count: int = 0

        mode_str = "슬립 보상 " if enable_slip_compensation else ""
        cal_status = '활성화' if calibration_enabled else '비활성화'
        node.get_logger().info(
            f'[제어] 모듈 초기화 완료 ({mode_str}Pure Pursuit + '
            f'CalibratedSkidSteer, 캘리브레이션 {cal_status})')
        if calibration_file and self.skid_model.calibration.calibration_count > 0:
            node.get_logger().info(
                f'[제어] 캘리브레이션 데이터 로드 완료: '
                f'궤도폭 보정={self.skid_model.calibration.track_width_correction:.4f}, '
                f'L/R 게인={self.skid_model.calibration.left_motor_gain:.4f}/'
                f'{self.skid_model.calibration.right_motor_gain:.4f}')

    def _get_param_safe(self, name: str, default):
        try:
            param = self._node.get_parameter(name)
            return param.value
        except Exception:
            return default

    def set_path(self, waypoints) -> None:
        self.tracker.set_path(waypoints)
        self._path_tracking_active = bool(self.tracker.path)
        if self._path_tracking_active:
            self._node.get_logger().info(
                f'[제어] 경로 설정 완료 ({len(self.tracker.path)}개 웨이포인트)')
        else:
            self._node.get_logger().warn('[제어] 빈 경로가 설정됨')

    def compute_control(self, vehicle_state: VehicleState) -> Tuple[float, float]:
        self._current_speed = vehicle_state.speed
        self._current_steering = vehicle_state.steering

        self._update_calibration_feedback(vehicle_state)

        if self._path_tracking_active and not self.tracker.is_goal_reached:
            return self._compute_path_tracking(vehicle_state)

        if self._path_tracking_active and self.tracker.is_goal_reached:
            self._node.get_logger().info('[제어] 경로 추종 완료 -- 정지')
            self._path_tracking_active = False
            return 0.0, 0.0

        return 0.0, 0.0

    def set_terrain(self, terrain_name: str) -> None:
        if self.slip_compensation_enabled and self.slip_tracker is not None:
            terrain = terrain_type_from_name(terrain_name)
            self.slip_tracker.set_terrain(terrain)
            self._node.get_logger().debug(
                f'[제어] 지형 설정: {terrain_name} -> {terrain.name}')

    def get_slip_info(self) -> Optional[dict]:
        if self.slip_compensation_enabled and self.slip_tracker is not None:
            return self.slip_tracker.get_slip_info()
        return None

    def _update_calibration_feedback(
        self, vehicle_state: VehicleState
    ) -> None:
        if not self.skid_model.calibration_enabled:
            return

        measured_yaw_rate = vehicle_state.yaw_rate
        cmd_linear = self._last_commanded_linear
        cmd_angular = self._last_commanded_angular

        self.skid_model.update_track_width_correction(
            commanded_angular=cmd_angular,
            measured_angular=measured_yaw_rate,
        )

        self.skid_model.update_motor_asymmetry(
            commanded_linear=cmd_linear,
            commanded_angular=cmd_angular,
            measured_yaw_rate=measured_yaw_rate,
        )

        self._calibration_update_count += 1
        if (self._calibration_update_count % self._calibration_save_interval == 0
                and self.skid_model.calibration.calibration_count > 0):
            if self.skid_model.save_calibration():
                self._node.get_logger().debug(
                    '[제어] 캘리브레이션 데이터 자동 저장 완료')

    def update_imu_feedback(
        self, measured_yaw_rate: float
    ) -> None:
        if not self.skid_model.calibration_enabled:
            return

        self.skid_model.update_track_width_correction(
            commanded_angular=self._last_commanded_angular,
            measured_angular=measured_yaw_rate,
        )

        self.skid_model.update_motor_asymmetry(
            commanded_linear=self._last_commanded_linear,
            commanded_angular=self._last_commanded_angular,
            measured_yaw_rate=measured_yaw_rate,
        )

    def get_calibration_status(self) -> Dict:
        return self.skid_model.get_calibration_status()

    def save_calibration(self, file_path: Optional[str] = None) -> bool:
        success = self.skid_model.save_calibration(file_path)
        if success:
            self._node.get_logger().info('[제어] 캘리브레이션 데이터 저장 완료')
        else:
            self._node.get_logger().warn('[제어] 캘리브레이션 데이터 저장 실패')
        return success

    def reset_calibration(self) -> None:
        self.skid_model.reset_calibration()
        self._calibration_update_count = 0
        self._node.get_logger().info('[제어] 캘리브레이션 초기화 완료')

    def _compute_path_tracking(
        self, vehicle_state: VehicleState
    ) -> Tuple[float, float]:
        pose = Pose2D(
            x=vehicle_state.x,
            y=vehicle_state.y,
            yaw=vehicle_state.yaw,
        )

        if self.slip_compensation_enabled and self.slip_tracker is not None:
            odom_pose = Pose2D(
                x=vehicle_state.odom_x,
                y=vehicle_state.odom_y,
                yaw=vehicle_state.odom_yaw,
            )
            self.slip_tracker.update_slip(odom_pose, pose)

            if vehicle_state.terrain_name:
                terrain = terrain_type_from_name(vehicle_state.terrain_name)
                self.slip_tracker.set_terrain(terrain)

            linear_vel, angular_vel = self.slip_tracker.compute(
                pose, vehicle_state.speed)
        else:
            linear_vel, angular_vel = self.tracker.compute(
                pose, vehicle_state.speed)

        self._last_commanded_linear = linear_vel
        self._last_commanded_angular = angular_vel

        v_left, v_right = self.skid_model.twist_to_tracks(
            linear_vel, angular_vel)

        return v_left, v_right

    def execute(self, plan_result: dict) -> None:
        if self._path_tracking_active and not self.tracker.is_goal_reached:
            pose = Pose2D(
                x=plan_result.get('x', 0.0),
                y=plan_result.get('y', 0.0),
                yaw=plan_result.get('yaw', 0.0),
            )
            speed = plan_result.get('current_speed', self._current_speed)

            if self.slip_compensation_enabled and self.slip_tracker is not None:
                odom_pose = Pose2D(
                    x=plan_result.get('odom_x', pose.x),
                    y=plan_result.get('odom_y', pose.y),
                    yaw=plan_result.get('odom_yaw', pose.yaw),
                )
                self.slip_tracker.update_slip(odom_pose, pose)

                terrain_name = plan_result.get('terrain_name', '')
                if terrain_name:
                    terrain = terrain_type_from_name(terrain_name)
                    self.slip_tracker.set_terrain(terrain)

                linear_vel, angular_vel = self.slip_tracker.compute(
                    pose, speed)
            else:
                linear_vel, angular_vel = self.tracker.compute(pose, speed)

            self._last_commanded_linear = linear_vel
            self._last_commanded_angular = angular_vel

            v_left, v_right = self.skid_model.twist_to_tracks(
                linear_vel, angular_vel)

            linear_pub, angular_pub = self.skid_model.tracks_to_twist(
                v_left, v_right)

            twist = Twist()
            twist.linear.x = linear_pub
            twist.angular.z = angular_pub
            self._cmd_pub.publish(twist)

            yaw_rate = plan_result.get('yaw_rate', 0.0)
            if yaw_rate != 0.0:
                self.update_imu_feedback(yaw_rate)

            if self.tracker.is_goal_reached:
                self._node.get_logger().info('[제어] 경로 추종 완료')
                self._path_tracking_active = False
            return

        target_speed = plan_result.get('target_speed', 0.0)
        target_steering = plan_result.get('target_steering', 0.0)

        speed_cmd = self._pid_speed(target_speed)
        steer_cmd = self._p_steering(target_steering)

        twist = Twist()
        twist.linear.x = speed_cmd
        twist.angular.z = steer_cmd
        self._cmd_pub.publish(twist)

    def _pid_speed(self, target: float) -> float:
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
        error = target - self._current_steering
        return self._steer_kp * error

    def update_current_state(self, speed: float, steering: float) -> None:
        self._current_speed = speed
        self._current_steering = steering

    def reset_pid(self) -> None:
        self._speed_error_integral = 0.0
        self._speed_error_prev = 0.0


class ControlNode(Node):
    """제어 독립 ROS2 노드."""

    def __init__(self):
        super().__init__('control_node')

        # 파라미터 선언
        self.declare_parameter('topics.cmd_vel', '/cmd_vel')
        self.declare_parameter('control.speed_kp', 1.0)
        self.declare_parameter('control.speed_ki', 0.01)
        self.declare_parameter('control.speed_kd', 0.1)
        self.declare_parameter('control.steer_kp', 1.5)

        # 제어 모듈 초기화
        self.control = ControlModule(self)

        self.get_logger().info('[제어 노드] 시작')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[제어 노드] 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
