"""제어 모듈 - 계획된 속도/조향을 실제 제어 명령으로 변환.

Pure Pursuit 경로 추종과 스키드 스티어 운동학 모델을 통합하여
궤도차량에 적합한 제어 명령을 생성한다.
기존 PID 제어는 경로가 설정되지 않았을 때 fallback으로 동작한다.

Phase 2 확장:
- CalibratedSkidSteerModel을 통한 유효 궤도 폭 온라인 보정
- IMU 피드백 기반 모터 비대칭 보상
- 캘리브레이션 데이터 자동 저장/로드
- SlipCompensatedPurePursuit을 통한 슬립 보상 적응형 경로 추종
"""

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from rclpy.node import Node
from geometry_msgs.msg import Twist

from team_leader.pure_pursuit import (
    Pose2D,
    PurePursuitConfig,
    PurePursuitTracker,
    SlipCompensatedPurePursuit,
    SlipCompensatedPurePursuitConfig,
    TerrainType as SlipTerrainType,
    terrain_type_from_name,
)
from team_leader.skid_steer_model import (
    CalibratedSkidSteerModel,
    SkidSteerModel,
)


@dataclass
class VehicleState:
    """차량 상태 정보."""

    x: float = 0.0
    """위치 x (m). 실측(로컬라이제이션) 기반."""

    y: float = 0.0
    """위치 y (m). 실측(로컬라이제이션) 기반."""

    yaw: float = 0.0
    """방향 (rad). 실측(로컬라이제이션) 기반."""

    speed: float = 0.0
    """현재 속도 (m/s)."""

    steering: float = 0.0
    """현재 조향각 (rad)."""

    yaw_rate: float = 0.0
    """IMU 측정 yaw rate (rad/s). 캘리브레이션 피드백에 사용."""

    odom_x: float = 0.0
    """오도메트리 기반 예측 위치 x (m). 슬립 추정에 사용."""

    odom_y: float = 0.0
    """오도메트리 기반 예측 위치 y (m). 슬립 추정에 사용."""

    odom_yaw: float = 0.0
    """오도메트리 기반 예측 방향 (rad). 슬립 추정에 사용."""

    terrain_name: str = ""
    """현재 지형 이름 문자열. 예: "PAVED", "MUD". 비어있으면 미설정."""


class ControlModule:
    """판단 모듈의 목표값을 실제 차량 제어 명령으로 변환하여 퍼블리시.

    경로가 설정된 경우 Pure Pursuit + SkidSteer 모델을 사용하고,
    경로가 없으면 기존 PID 제어를 fallback으로 사용한다.

    슬립 보상 모드(enable_slip_compensation=True)가 활성화되면
    SlipCompensatedPurePursuit을 사용하여 슬립에 적응하는 경로 추종을 수행한다.

    Phase 2에서 CalibratedSkidSteerModel을 통합하여:
    - IMU 피드백으로 유효 궤도 폭을 온라인 보정
    - 직진 시 모터 비대칭을 자동 감지/보상
    - 캘리브레이션 데이터를 파일로 영속 저장

    Attributes:
        tracker: Pure Pursuit 경로 추종기 (기본 또는 슬립 보상).
        slip_tracker: 슬립 보상 추종기 (슬립 보상 모드일 때만 유효).
        skid_model: 보정 기능 통합 스키드 스티어 운동학 모델.
        slip_compensation_enabled: 슬립 보상 모드 활성화 여부.
    """

    def __init__(
        self,
        node: Node,
        enable_slip_compensation: bool = False,
    ) -> None:
        """초기화.

        Args:
            node: ROS2 노드 인스턴스 (파라미터 및 퍼블리셔 접근).
            enable_slip_compensation: True이면 슬립 보상 Pure Pursuit을 사용.
        """
        self._node = node
        self.slip_compensation_enabled: bool = enable_slip_compensation

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

        # Pure Pursuit 경로 추종기 설정
        if enable_slip_compensation:
            # 슬립 보상 Pure Pursuit
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
            # tracker는 slip_tracker를 가리킴 (호환 인터페이스)
            self.tracker: PurePursuitTracker = self.slip_tracker
        else:
            # 기본 Pure Pursuit
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

        # 캘리브레이션 파라미터 로드 (파라미터 서버에서 선택적으로 읽기)
        calibration_file = self._get_param_safe(
            'control.calibration_file', None)
        ema_alpha = self._get_param_safe(
            'control.calibration_ema_alpha', 0.05)
        calibration_enabled = self._get_param_safe(
            'control.calibration_enabled', True)

        # 보정 기능 통합 스키드 스티어 운동학 모델 (Phase 2)
        self.skid_model: CalibratedSkidSteerModel = CalibratedSkidSteerModel(
            track_width=1.4,
            max_speed=1.0,
            steering_efficiency=0.8,
            ema_alpha=ema_alpha,
            calibration_file=calibration_file,
            calibration_enabled=calibration_enabled,
        )

        # 경로 추종 모드 활성화 여부
        self._path_tracking_active: bool = False

        # 마지막 명령 저장 (캘리브레이션 피드백에 사용)
        self._last_commanded_linear: float = 0.0
        self._last_commanded_angular: float = 0.0

        # 캘리브레이션 자동 저장 주기 (갱신 카운트 기준)
        self._calibration_save_interval: int = 100
        self._calibration_update_count: int = 0

        # 초기화 로그
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
        """파라미터를 안전하게 읽는다. 선언되지 않은 경우 기본값을 반환한다.

        Args:
            name: 파라미터 이름.
            default: 기본값.

        Returns:
            파라미터 값 또는 기본값.
        """
        try:
            param = self._node.get_parameter(name)
            return param.value
        except Exception:
            return default

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

        IMU yaw_rate가 제공된 경우 캘리브레이션 피드백도 수행한다.

        Args:
            vehicle_state: 현재 차량 상태.

        Returns:
            (v_left, v_right) 좌/우 트랙 속도 (m/s).
        """
        # 내부 상태 갱신
        self._current_speed = vehicle_state.speed
        self._current_steering = vehicle_state.steering

        # IMU 피드백 기반 캘리브레이션 갱신
        self._update_calibration_feedback(vehicle_state)

        if self._path_tracking_active and not self.tracker.is_goal_reached:
            return self._compute_path_tracking(vehicle_state)

        # 경로 완료 또는 미설정 -> 정지
        if self._path_tracking_active and self.tracker.is_goal_reached:
            self._node.get_logger().info('[제어] 경로 추종 완료 -- 정지')
            self._path_tracking_active = False
            return 0.0, 0.0

        # Fallback: PID 기반 (경로 미설정 시)
        return 0.0, 0.0

    def set_terrain(self, terrain_name: str) -> None:
        """현재 주행 지형을 설정한다.

        슬립 보상 모드가 활성화된 경우에만 효과가 있다.

        Args:
            terrain_name: 지형 이름 문자열. 예: "PAVED", "MUD", "DIRT_ROAD".
        """
        if self.slip_compensation_enabled and self.slip_tracker is not None:
            terrain = terrain_type_from_name(terrain_name)
            self.slip_tracker.set_terrain(terrain)
            self._node.get_logger().debug(
                f'[제어] 지형 설정: {terrain_name} -> {terrain.name}')

    def get_slip_info(self) -> Optional[dict]:
        """현재 슬립 추정 정보를 반환한다.

        슬립 보상 모드가 비활성화되어 있으면 None을 반환한다.

        Returns:
            슬립 진단 정보 딕셔너리, 또는 None.
        """
        if self.slip_compensation_enabled and self.slip_tracker is not None:
            return self.slip_tracker.get_slip_info()
        return None

    # ------------------------------------------------------------------ #
    #  IMU 피드백 기반 캘리브레이션 (Phase 2)
    # ------------------------------------------------------------------ #

    def _update_calibration_feedback(
        self, vehicle_state: VehicleState
    ) -> None:
        """IMU yaw rate 피드백으로 캘리브레이션을 갱신한다.

        매 제어 주기마다 호출되며, 조건에 따라 두 가지 보정을 수행한다:
        1. 회전 시: 유효 궤도 폭 보정 (명령 각속도 vs 측정 각속도)
        2. 직진 시: 모터 비대칭 보상 (직진 중 yaw drift 감지)

        Args:
            vehicle_state: 현재 차량 상태 (yaw_rate 포함).
        """
        if not self.skid_model.calibration_enabled:
            return

        measured_yaw_rate = vehicle_state.yaw_rate
        cmd_linear = self._last_commanded_linear
        cmd_angular = self._last_commanded_angular

        # 1. 유효 궤도 폭 보정: 회전 중일 때
        self.skid_model.update_track_width_correction(
            commanded_angular=cmd_angular,
            measured_angular=measured_yaw_rate,
        )

        # 2. 모터 비대칭 보상: 직진 중일 때
        self.skid_model.update_motor_asymmetry(
            commanded_linear=cmd_linear,
            commanded_angular=cmd_angular,
            measured_yaw_rate=measured_yaw_rate,
        )

        # 자동 저장 주기 확인
        self._calibration_update_count += 1
        if (self._calibration_update_count % self._calibration_save_interval == 0
                and self.skid_model.calibration.calibration_count > 0):
            if self.skid_model.save_calibration():
                self._node.get_logger().debug(
                    '[제어] 캘리브레이션 데이터 자동 저장 완료')

    def update_imu_feedback(
        self, measured_yaw_rate: float
    ) -> None:
        """외부에서 IMU yaw rate 피드백을 직접 전달한다.

        compute_control()을 사용하지 않는 경우에도 캘리브레이션을
        수행할 수 있도록 하는 편의 메서드이다.

        Args:
            measured_yaw_rate: IMU에서 측정된 yaw rate (rad/s).
        """
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
        """현재 캘리브레이션 상태를 반환한다.

        Returns:
            캘리브레이션 진단 정보 딕셔너리.
        """
        return self.skid_model.get_calibration_status()

    def save_calibration(self, file_path: Optional[str] = None) -> bool:
        """캘리브레이션 데이터를 명시적으로 저장한다.

        Args:
            file_path: 저장 파일 경로. None이면 설정된 기본 경로를 사용.

        Returns:
            저장 성공 여부.
        """
        success = self.skid_model.save_calibration(file_path)
        if success:
            self._node.get_logger().info('[제어] 캘리브레이션 데이터 저장 완료')
        else:
            self._node.get_logger().warn('[제어] 캘리브레이션 데이터 저장 실패')
        return success

    def reset_calibration(self) -> None:
        """모든 캘리브레이션 보정 계수를 기본값으로 초기화한다."""
        self.skid_model.reset_calibration()
        self._calibration_update_count = 0
        self._node.get_logger().info('[제어] 캘리브레이션 초기화 완료')

    def _compute_path_tracking(
        self, vehicle_state: VehicleState
    ) -> Tuple[float, float]:
        """Pure Pursuit + SkidSteer 경로 추종 제어를 수행한다.

        슬립 보상 모드가 활성화되어 있으면 오도메트리와 실측 위치를
        비교하여 슬립을 추정하고, 보상된 제어 명령을 생성한다.

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

        # 슬립 보상 모드: 슬립 추정 업데이트
        if self.slip_compensation_enabled and self.slip_tracker is not None:
            odom_pose = Pose2D(
                x=vehicle_state.odom_x,
                y=vehicle_state.odom_y,
                yaw=vehicle_state.odom_yaw,
            )
            self.slip_tracker.update_slip(odom_pose, pose)

            # 지형 정보가 있으면 갱신
            if vehicle_state.terrain_name:
                terrain = terrain_type_from_name(vehicle_state.terrain_name)
                self.slip_tracker.set_terrain(terrain)

            # SlipCompensatedPurePursuit에서 선속도/각속도 계산
            linear_vel, angular_vel = self.slip_tracker.compute(
                pose, vehicle_state.speed)
        else:
            # 기본 모드: Pure Pursuit으로 선속도/각속도 계산
            linear_vel, angular_vel = self.tracker.compute(
                pose, vehicle_state.speed)

        # 마지막 명령 저장 (캘리브레이션 피드백에 사용)
        self._last_commanded_linear = linear_vel
        self._last_commanded_angular = angular_vel

        # CalibratedSkidSteer 모델로 트랙 속도 변환 (보정 적용)
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
        슬립 보상 모드에서는 오도메트리 정보도 활용한다.

        Args:
            plan_result: PlanningModule.get_plan_result()의 반환값.
                슬립 보상 모드에서는 추가로 'odom_x', 'odom_y', 'odom_yaw',
                'terrain_name' 키를 포함할 수 있다.
                캘리브레이션 피드백을 위해 'yaw_rate' 키를 포함할 수 있다.
        """
        if self._path_tracking_active and not self.tracker.is_goal_reached:
            # Pure Pursuit 모드: vehicle_state 기반 제어
            pose = Pose2D(
                x=plan_result.get('x', 0.0),
                y=plan_result.get('y', 0.0),
                yaw=plan_result.get('yaw', 0.0),
            )
            speed = plan_result.get('current_speed', self._current_speed)

            # 슬립 보상 모드
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

            # 마지막 명령 저장 (캘리브레이션 피드백에 사용)
            self._last_commanded_linear = linear_vel
            self._last_commanded_angular = angular_vel

            # CalibratedSkidSteer 모델로 트랙 속도 변환 (보정 적용)
            v_left, v_right = self.skid_model.twist_to_tracks(
                linear_vel, angular_vel)

            # 트랙 속도를 Twist로 변환하여 퍼블리시
            linear_pub, angular_pub = self.skid_model.tracks_to_twist(
                v_left, v_right)

            twist = Twist()
            twist.linear.x = linear_pub
            twist.angular.z = angular_pub
            self._cmd_pub.publish(twist)

            # IMU 피드백이 있으면 캘리브레이션 갱신
            yaw_rate = plan_result.get('yaw_rate', 0.0)
            if yaw_rate != 0.0:
                self.update_imu_feedback(yaw_rate)

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
