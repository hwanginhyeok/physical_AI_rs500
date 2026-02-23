"""위치 추정 관리 노드 - GPS/SLAM 자동 전환.

GPS 상태를 모니터링하고, GPS 가용성에 따라 위치 추정 모드를
자동으로 전환한다.

모드 전환 전략:
  - GPS 양호 (GOOD)     -> GPS_EKF   : Dual-EKF (robot_localization) 사용
  - GPS 불량 (UNAVAILABLE) -> SLAM_ONLY : LIO-SAM LiDAR-IMU SLAM 의존
  - GPS 복구 (DEGRADED)  -> HYBRID    : SLAM + GPS 점진적 전환
"""

import math
import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, List, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ======================================================================
# Enum / 상수
# ======================================================================

class LocalizationMode(Enum):
    """위치 추정 모드."""
    GPS_EKF = auto()
    SLAM_ONLY = auto()
    HYBRID = auto()


class GPSHealthStatus(Enum):
    """GPS 건강 상태."""
    GOOD = auto()
    DEGRADED = auto()
    UNAVAILABLE = auto()


# ======================================================================
# GPS 건강 모니터
# ======================================================================

@dataclass
class GPSHealthConfig:
    """GPS 건강 판단 기준 설정."""
    hdop_good_threshold: float = 2.0
    hdop_degraded_threshold: float = 5.0
    min_satellites_good: int = 6
    min_satellites_degraded: int = 4
    timeout_sec: float = 3.0
    consecutive_bad_count: int = 5
    consecutive_good_count: int = 10


class GPSHealthMonitor:
    """GPS fix 품질 모니터링 클래스."""

    def __init__(
        self,
        node: Node,
        config: Optional[GPSHealthConfig] = None,
    ):
        self._node = node
        self._config = config if config is not None else GPSHealthConfig()

        self._status = GPSHealthStatus.UNAVAILABLE
        self._last_fix_time: Optional[float] = None
        self._last_fix_type: int = -1
        self._last_hdop: float = 99.0
        self._last_num_satellites: int = 0
        self._consecutive_good: int = 0
        self._consecutive_bad: int = 0

        self._on_status_change_callbacks: List[Callable] = []

        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._gps_sub = node.create_subscription(
            NavSatFix,
            '/sensor/gps',
            self._gps_callback,
            gps_qos,
        )

        node.get_logger().info(
            '[GPS모니터] 초기화 완료 '
            f'(HDOP 임계값: {self._config.hdop_good_threshold}/'
            f'{self._config.hdop_degraded_threshold}, '
            f'타임아웃: {self._config.timeout_sec}초)'
        )

    @property
    def status(self) -> GPSHealthStatus:
        return self._status

    @property
    def last_hdop(self) -> float:
        return self._last_hdop

    @property
    def last_num_satellites(self) -> int:
        return self._last_num_satellites

    @property
    def last_fix_type(self) -> int:
        return self._last_fix_type

    @property
    def seconds_since_last_fix(self) -> float:
        if self._last_fix_time is None:
            return float('inf')
        return time.monotonic() - self._last_fix_time

    def on_status_change(self, callback: Callable) -> None:
        self._on_status_change_callbacks.append(callback)

    def _gps_callback(self, msg: NavSatFix) -> None:
        now = time.monotonic()

        fix_type = msg.status.status
        self._last_fix_type = fix_type
        self._last_fix_time = now

        if (msg.position_covariance_type !=
                NavSatFix.COVARIANCE_TYPE_UNKNOWN):
            var_x = msg.position_covariance[0]
            var_y = msg.position_covariance[4]
            self._last_hdop = math.sqrt(max(0.0, var_x) + max(0.0, var_y))
        else:
            if fix_type >= NavSatStatus.STATUS_GBAS_FIX:
                self._last_hdop = 0.5
            elif fix_type >= NavSatStatus.STATUS_FIX:
                self._last_hdop = 1.5
            else:
                self._last_hdop = 99.0

        if fix_type >= NavSatStatus.STATUS_GBAS_FIX:
            self._last_num_satellites = max(self._last_num_satellites, 8)
        elif fix_type >= NavSatStatus.STATUS_SBAS_FIX:
            self._last_num_satellites = max(6, self._last_num_satellites)
        elif fix_type >= NavSatStatus.STATUS_FIX:
            self._last_num_satellites = max(4, self._last_num_satellites)
        else:
            self._last_num_satellites = 0

        is_good = self._evaluate_quality_good()
        is_degraded = self._evaluate_quality_degraded()

        if is_good:
            self._consecutive_good += 1
            self._consecutive_bad = 0
        elif is_degraded:
            self._consecutive_good = 0
            self._consecutive_bad = 0
        else:
            self._consecutive_good = 0
            self._consecutive_bad += 1

        self._update_status()

    def _evaluate_quality_good(self) -> bool:
        return (
            self._last_fix_type >= NavSatStatus.STATUS_FIX
            and self._last_hdop <= self._config.hdop_good_threshold
            and self._last_num_satellites >= self._config.min_satellites_good
        )

    def _evaluate_quality_degraded(self) -> bool:
        return (
            self._last_fix_type >= NavSatStatus.STATUS_FIX
            and self._last_hdop <= self._config.hdop_degraded_threshold
            and self._last_num_satellites >= self._config.min_satellites_degraded
        )

    def _update_status(self) -> None:
        old_status = self._status

        if self._status == GPSHealthStatus.GOOD:
            if self._consecutive_bad >= self._config.consecutive_bad_count:
                self._status = GPSHealthStatus.DEGRADED
                self._node.get_logger().warn(
                    f'[GPS모니터] 상태 전환: GOOD -> DEGRADED '
                    f'(HDOP={self._last_hdop:.1f}, '
                    f'위성={self._last_num_satellites})'
                )

        elif self._status == GPSHealthStatus.DEGRADED:
            if self._consecutive_good >= self._config.consecutive_good_count:
                self._status = GPSHealthStatus.GOOD
                self._node.get_logger().info(
                    f'[GPS모니터] 상태 전환: DEGRADED -> GOOD '
                    f'(HDOP={self._last_hdop:.1f}, '
                    f'위성={self._last_num_satellites})'
                )
            elif self._consecutive_bad >= self._config.consecutive_bad_count * 2:
                self._status = GPSHealthStatus.UNAVAILABLE
                self._node.get_logger().warn(
                    '[GPS모니터] 상태 전환: DEGRADED -> UNAVAILABLE'
                )

        elif self._status == GPSHealthStatus.UNAVAILABLE:
            if self._evaluate_quality_degraded():
                self._status = GPSHealthStatus.DEGRADED
                self._node.get_logger().info(
                    f'[GPS모니터] 상태 전환: UNAVAILABLE -> DEGRADED '
                    f'(HDOP={self._last_hdop:.1f})'
                )
            if (self._evaluate_quality_good()
                    and self._consecutive_good >= self._config.consecutive_good_count):
                self._status = GPSHealthStatus.GOOD
                self._node.get_logger().info(
                    '[GPS모니터] 상태 전환: UNAVAILABLE -> GOOD'
                )

        if old_status != self._status:
            for cb in self._on_status_change_callbacks:
                try:
                    cb(old_status, self._status)
                except Exception as e:
                    self._node.get_logger().error(
                        f'[GPS모니터] 콜백 오류: {e}'
                    )

    def check_timeout(self) -> None:
        if (self._last_fix_time is not None
                and self.seconds_since_last_fix > self._config.timeout_sec
                and self._status != GPSHealthStatus.UNAVAILABLE):
            old_status = self._status
            self._status = GPSHealthStatus.UNAVAILABLE
            self._consecutive_good = 0
            self._consecutive_bad = 0
            self._node.get_logger().warn(
                f'[GPS모니터] GPS 타임아웃 '
                f'({self.seconds_since_last_fix:.1f}초 미수신) '
                f'-> UNAVAILABLE'
            )
            for cb in self._on_status_change_callbacks:
                try:
                    cb(old_status, self._status)
                except Exception as e:
                    self._node.get_logger().error(
                        f'[GPS모니터] 콜백 오류: {e}'
                    )
        elif (self._last_fix_time is None
              and self._status != GPSHealthStatus.UNAVAILABLE):
            old_status = self._status
            self._status = GPSHealthStatus.UNAVAILABLE
            if old_status != self._status:
                for cb in self._on_status_change_callbacks:
                    try:
                        cb(old_status, self._status)
                    except Exception as e:
                        self._node.get_logger().error(
                            f'[GPS모니터] 콜백 오류: {e}'
                        )

    def get_health_summary(self) -> dict:
        return {
            'status': self._status.name,
            'fix_type': self._last_fix_type,
            'hdop': self._last_hdop,
            'num_satellites': self._last_num_satellites,
            'seconds_since_fix': self.seconds_since_last_fix,
            'consecutive_good': self._consecutive_good,
            'consecutive_bad': self._consecutive_bad,
        }


# ======================================================================
# 위치 추정 관리자
# ======================================================================

@dataclass
class LocalizationConfig:
    """위치 추정 모드 전환 설정."""
    hybrid_transition_duration_sec: float = 10.0
    hybrid_gps_weight_initial: float = 0.1
    hybrid_gps_weight_final: float = 1.0
    pose_alignment_threshold_m: float = 2.0
    pose_alignment_rate: float = 0.05
    slam_odom_topic: str = '/odometry/slam'
    gps_odom_topic: str = '/odometry/gps'
    fused_odom_topic: str = '/odometry/localization'
    mode_status_topic: str = '/localization/mode'


class LocalizationManager:
    """GPS 상태 기반 위치 추정 모드 자동 전환 관리자."""

    def __init__(
        self,
        node: Node,
        gps_health_config: Optional[GPSHealthConfig] = None,
        localization_config: Optional[LocalizationConfig] = None,
    ):
        self._node = node
        self._config = (localization_config
                        if localization_config is not None
                        else LocalizationConfig())

        self._mode = LocalizationMode.SLAM_ONLY
        self._hybrid_start_time: Optional[float] = None
        self._hybrid_gps_weight: float = 0.0

        self._slam_pose: Optional[PoseStamped] = None
        self._gps_pose: Optional[PoseStamped] = None
        self._aligned_pose: Optional[PoseStamped] = None
        self._pose_offset_x: float = 0.0
        self._pose_offset_y: float = 0.0
        self._pose_offset_yaw: float = 0.0

        self._transition_log: List[dict] = []

        self._gps_monitor = GPSHealthMonitor(node, gps_health_config)
        self._gps_monitor.on_status_change(self._on_gps_status_change)

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._slam_odom_sub = node.create_subscription(
            Odometry,
            self._config.slam_odom_topic,
            self._slam_odom_callback,
            odom_qos,
        )

        self._gps_odom_sub = node.create_subscription(
            Odometry,
            self._config.gps_odom_topic,
            self._gps_odom_callback,
            odom_qos,
        )

        self._fused_odom_pub = node.create_publisher(
            Odometry,
            self._config.fused_odom_topic,
            10,
        )

        self._mode_pub = node.create_publisher(
            String,
            self._config.mode_status_topic,
            10,
        )

        self._timeout_timer = node.create_timer(1.0, self._timeout_check)
        self._status_timer = node.create_timer(0.5, self._publish_mode_status)

        node.get_logger().info(
            f'[위치관리] LocalizationManager 초기화 완료 '
            f'(초기 모드: {self._mode.name})'
        )

    @property
    def mode(self) -> LocalizationMode:
        return self._mode

    @property
    def gps_monitor(self) -> GPSHealthMonitor:
        return self._gps_monitor

    @property
    def hybrid_gps_weight(self) -> float:
        if self._mode == LocalizationMode.GPS_EKF:
            return 1.0
        elif self._mode == LocalizationMode.SLAM_ONLY:
            return 0.0
        return self._hybrid_gps_weight

    @property
    def transition_log(self) -> List[dict]:
        return self._transition_log.copy()

    def _on_gps_status_change(
        self,
        old_status: GPSHealthStatus,
        new_status: GPSHealthStatus,
    ) -> None:
        old_mode = self._mode

        if new_status == GPSHealthStatus.GOOD:
            if self._mode == LocalizationMode.SLAM_ONLY:
                self._transition_to_hybrid()
            elif self._mode == LocalizationMode.HYBRID:
                pass

        elif new_status == GPSHealthStatus.DEGRADED:
            if self._mode == LocalizationMode.GPS_EKF:
                self._transition_to_hybrid_from_gps()
            elif self._mode == LocalizationMode.SLAM_ONLY:
                self._transition_to_hybrid()

        elif new_status == GPSHealthStatus.UNAVAILABLE:
            if self._mode != LocalizationMode.SLAM_ONLY:
                self._transition_to_slam_only()

        if old_mode != self._mode:
            self._log_transition(old_mode, self._mode, new_status)

    def _transition_to_slam_only(self) -> None:
        self._mode = LocalizationMode.SLAM_ONLY
        self._hybrid_start_time = None
        self._hybrid_gps_weight = 0.0

        if self._slam_pose is not None and self._gps_pose is not None:
            self._compute_pose_offset()

        self._node.get_logger().warn(
            '[위치관리] 모드 전환: -> SLAM_ONLY '
            '(GPS 불가, LIO-SAM 단독 위치 추정)'
        )

    def _transition_to_hybrid(self) -> None:
        self._mode = LocalizationMode.HYBRID
        self._hybrid_start_time = time.monotonic()
        self._hybrid_gps_weight = self._config.hybrid_gps_weight_initial

        if self._slam_pose is not None and self._gps_pose is not None:
            self._compute_pose_offset()

        self._node.get_logger().info(
            '[위치관리] 모드 전환: -> HYBRID '
            f'(GPS 복구 중, 초기 GPS 가중치: '
            f'{self._config.hybrid_gps_weight_initial:.2f})'
        )

    def _transition_to_hybrid_from_gps(self) -> None:
        self._mode = LocalizationMode.HYBRID
        self._hybrid_start_time = time.monotonic()
        self._hybrid_gps_weight = self._config.hybrid_gps_weight_final

        self._node.get_logger().info(
            '[위치관리] 모드 전환: GPS_EKF -> HYBRID '
            '(GPS 성능 저하, GPS 가중치 점진적 감소)'
        )

    def _transition_to_gps_ekf(self) -> None:
        old_mode = self._mode
        self._mode = LocalizationMode.GPS_EKF
        self._hybrid_start_time = None
        self._hybrid_gps_weight = 1.0
        self._pose_offset_x = 0.0
        self._pose_offset_y = 0.0
        self._pose_offset_yaw = 0.0

        self._node.get_logger().info(
            '[위치관리] 모드 전환: -> GPS_EKF '
            '(GPS 양호, Dual-EKF 위치 추정 활성)'
        )

        if old_mode != self._mode:
            self._log_transition(
                old_mode, self._mode, self._gps_monitor.status
            )

    def _compute_pose_offset(self) -> None:
        if self._slam_pose is None or self._gps_pose is None:
            return

        slam_x = self._slam_pose.pose.position.x
        slam_y = self._slam_pose.pose.position.y
        gps_x = self._gps_pose.pose.position.x
        gps_y = self._gps_pose.pose.position.y

        self._pose_offset_x = gps_x - slam_x
        self._pose_offset_y = gps_y - slam_y

        slam_yaw = self._quaternion_to_yaw(
            self._slam_pose.pose.orientation.z,
            self._slam_pose.pose.orientation.w,
        )
        gps_yaw = self._quaternion_to_yaw(
            self._gps_pose.pose.orientation.z,
            self._gps_pose.pose.orientation.w,
        )
        self._pose_offset_yaw = self._normalize_angle(gps_yaw - slam_yaw)

        offset_distance = math.sqrt(
            self._pose_offset_x ** 2 + self._pose_offset_y ** 2
        )
        self._node.get_logger().info(
            f'[위치관리] 포즈 오프셋 계산: '
            f'dx={self._pose_offset_x:.3f}m, '
            f'dy={self._pose_offset_y:.3f}m, '
            f'dyaw={math.degrees(self._pose_offset_yaw):.1f}deg '
            f'(거리: {offset_distance:.3f}m)'
        )

    def _blend_poses(
        self,
        slam_odom: Odometry,
        gps_odom: Odometry,
        gps_weight: float,
    ) -> Odometry:
        slam_weight = 1.0 - gps_weight

        blended = Odometry()
        blended.header = slam_odom.header
        blended.header.frame_id = 'map'
        blended.child_frame_id = 'base_link'

        slam_x = slam_odom.pose.pose.position.x + self._pose_offset_x
        slam_y = slam_odom.pose.pose.position.y + self._pose_offset_y
        gps_x = gps_odom.pose.pose.position.x
        gps_y = gps_odom.pose.pose.position.y

        blended.pose.pose.position.x = (
            slam_weight * slam_x + gps_weight * gps_x
        )
        blended.pose.pose.position.y = (
            slam_weight * slam_y + gps_weight * gps_y
        )
        blended.pose.pose.position.z = 0.0

        slam_yaw = self._quaternion_to_yaw(
            slam_odom.pose.pose.orientation.z,
            slam_odom.pose.pose.orientation.w,
        ) + self._pose_offset_yaw
        gps_yaw = self._quaternion_to_yaw(
            gps_odom.pose.pose.orientation.z,
            gps_odom.pose.pose.orientation.w,
        )

        yaw_diff = self._normalize_angle(gps_yaw - slam_yaw)
        blended_yaw = slam_yaw + gps_weight * yaw_diff

        blended.pose.pose.orientation.z = math.sin(blended_yaw / 2.0)
        blended.pose.pose.orientation.w = math.cos(blended_yaw / 2.0)
        blended.pose.pose.orientation.x = 0.0
        blended.pose.pose.orientation.y = 0.0

        blended.twist = slam_odom.twist

        base_cov = 0.01
        gps_cov_factor = max(0.001, 1.0 - gps_weight * 0.9)
        blended.pose.covariance[0] = base_cov * gps_cov_factor
        blended.pose.covariance[7] = base_cov * gps_cov_factor
        blended.pose.covariance[35] = base_cov * gps_cov_factor

        return blended

    def _slam_odom_callback(self, msg: Odometry) -> None:
        if self._slam_pose is None:
            self._slam_pose = PoseStamped()
        self._slam_pose.header = msg.header
        self._slam_pose.pose = msg.pose.pose

        if self._mode == LocalizationMode.SLAM_ONLY:
            output = Odometry()
            output.header = msg.header
            output.header.frame_id = 'map'
            output.child_frame_id = 'base_link'
            output.pose.pose.position.x = (
                msg.pose.pose.position.x + self._pose_offset_x
            )
            output.pose.pose.position.y = (
                msg.pose.pose.position.y + self._pose_offset_y
            )
            output.pose.pose.position.z = 0.0

            slam_yaw = self._quaternion_to_yaw(
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            adjusted_yaw = slam_yaw + self._pose_offset_yaw
            output.pose.pose.orientation.z = math.sin(adjusted_yaw / 2.0)
            output.pose.pose.orientation.w = math.cos(adjusted_yaw / 2.0)
            output.pose.pose.orientation.x = 0.0
            output.pose.pose.orientation.y = 0.0

            output.twist = msg.twist
            output.pose.covariance[0] = 0.05
            output.pose.covariance[7] = 0.05
            output.pose.covariance[35] = 0.02

            self._fused_odom_pub.publish(output)

        elif self._mode == LocalizationMode.HYBRID:
            self._update_hybrid_weight()
            if self._gps_pose is not None:
                gps_odom = Odometry()
                gps_odom.header = msg.header
                gps_odom.pose.pose = self._gps_pose.pose
                blended = self._blend_poses(
                    msg, gps_odom, self._hybrid_gps_weight
                )
                self._fused_odom_pub.publish(blended)
            else:
                self._publish_slam_only(msg)

    def _gps_odom_callback(self, msg: Odometry) -> None:
        if self._gps_pose is None:
            self._gps_pose = PoseStamped()
        self._gps_pose.header = msg.header
        self._gps_pose.pose = msg.pose.pose

        if self._mode == LocalizationMode.GPS_EKF:
            output = Odometry()
            output.header = msg.header
            output.header.frame_id = 'map'
            output.child_frame_id = 'base_link'
            output.pose = msg.pose
            output.twist = msg.twist
            self._fused_odom_pub.publish(output)

    def _publish_slam_only(self, slam_odom: Odometry) -> None:
        output = Odometry()
        output.header = slam_odom.header
        output.header.frame_id = 'map'
        output.child_frame_id = 'base_link'
        output.pose.pose.position.x = (
            slam_odom.pose.pose.position.x + self._pose_offset_x
        )
        output.pose.pose.position.y = (
            slam_odom.pose.pose.position.y + self._pose_offset_y
        )
        output.pose.pose.position.z = 0.0

        slam_yaw = self._quaternion_to_yaw(
            slam_odom.pose.pose.orientation.z,
            slam_odom.pose.pose.orientation.w,
        )
        adjusted_yaw = slam_yaw + self._pose_offset_yaw
        output.pose.pose.orientation.z = math.sin(adjusted_yaw / 2.0)
        output.pose.pose.orientation.w = math.cos(adjusted_yaw / 2.0)
        output.pose.pose.orientation.x = 0.0
        output.pose.pose.orientation.y = 0.0

        output.twist = slam_odom.twist
        output.pose.covariance[0] = 0.05
        output.pose.covariance[7] = 0.05
        output.pose.covariance[35] = 0.02

        self._fused_odom_pub.publish(output)

    def _update_hybrid_weight(self) -> None:
        if self._hybrid_start_time is None:
            return

        elapsed = time.monotonic() - self._hybrid_start_time
        duration = self._config.hybrid_transition_duration_sec

        gps_status = self._gps_monitor.status

        if gps_status == GPSHealthStatus.GOOD:
            progress = min(1.0, elapsed / duration)
            self._hybrid_gps_weight = (
                self._config.hybrid_gps_weight_initial
                + progress * (
                    self._config.hybrid_gps_weight_final
                    - self._config.hybrid_gps_weight_initial
                )
            )

            decay = 1.0 - self._config.pose_alignment_rate
            self._pose_offset_x *= decay
            self._pose_offset_y *= decay
            self._pose_offset_yaw *= decay

            if progress >= 1.0:
                self._transition_to_gps_ekf()

        elif gps_status == GPSHealthStatus.DEGRADED:
            self._hybrid_gps_weight = max(
                self._config.hybrid_gps_weight_initial,
                self._hybrid_gps_weight * 0.99
            )

    def _timeout_check(self) -> None:
        self._gps_monitor.check_timeout()

    def _publish_mode_status(self) -> None:
        msg = String()
        gps_summary = self._gps_monitor.get_health_summary()
        msg.data = (
            f'{{"mode": "{self._mode.name}", '
            f'"gps_status": "{gps_summary["status"]}", '
            f'"gps_weight": {self.hybrid_gps_weight:.3f}, '
            f'"hdop": {gps_summary["hdop"]:.2f}, '
            f'"satellites": {gps_summary["num_satellites"]}, '
            f'"offset_x": {self._pose_offset_x:.3f}, '
            f'"offset_y": {self._pose_offset_y:.3f}}}'
        )
        self._mode_pub.publish(msg)

    def _log_transition(
        self,
        old_mode: LocalizationMode,
        new_mode: LocalizationMode,
        gps_status: GPSHealthStatus,
    ) -> None:
        entry = {
            'timestamp': time.monotonic(),
            'old_mode': old_mode.name,
            'new_mode': new_mode.name,
            'gps_status': gps_status.name,
            'gps_hdop': self._gps_monitor.last_hdop,
            'gps_satellites': self._gps_monitor.last_num_satellites,
            'pose_offset_x': self._pose_offset_x,
            'pose_offset_y': self._pose_offset_y,
            'pose_offset_yaw': self._pose_offset_yaw,
        }
        self._transition_log.append(entry)

        if len(self._transition_log) > 100:
            self._transition_log = self._transition_log[-100:]

        self._node.get_logger().info(
            f'[위치관리] 전환 기록: {old_mode.name} -> {new_mode.name} '
            f'(GPS: {gps_status.name}, '
            f'HDOP: {entry["gps_hdop"]:.1f}, '
            f'위성: {entry["gps_satellites"]})'
        )

    @staticmethod
    def _quaternion_to_yaw(qz: float, qw: float) -> float:
        return 2.0 * math.atan2(qz, qw)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_status(self) -> dict:
        return {
            'mode': self._mode.name,
            'gps_health': self._gps_monitor.get_health_summary(),
            'gps_weight': self.hybrid_gps_weight,
            'pose_offset': {
                'x': self._pose_offset_x,
                'y': self._pose_offset_y,
                'yaw': self._pose_offset_yaw,
            },
            'transition_count': len(self._transition_log),
        }

    def force_mode(self, mode: LocalizationMode) -> None:
        old_mode = self._mode
        self._mode = mode

        if mode == LocalizationMode.GPS_EKF:
            self._hybrid_gps_weight = 1.0
            self._hybrid_start_time = None
        elif mode == LocalizationMode.SLAM_ONLY:
            self._hybrid_gps_weight = 0.0
            self._hybrid_start_time = None
            if self._slam_pose is not None and self._gps_pose is not None:
                self._compute_pose_offset()
        elif mode == LocalizationMode.HYBRID:
            self._hybrid_start_time = time.monotonic()
            self._hybrid_gps_weight = self._config.hybrid_gps_weight_initial

        self._node.get_logger().warn(
            f'[위치관리] 강제 모드 전환: {old_mode.name} -> {mode.name}'
        )

        if old_mode != mode:
            self._log_transition(
                old_mode, mode, self._gps_monitor.status
            )


class LocalizationNode(Node):
    """위치 추정 독립 ROS2 노드."""

    def __init__(self):
        super().__init__('localization_node')

        self.localization_manager = LocalizationManager(self)

        self.get_logger().info('[위치 추정 노드] 시작')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[위치 추정 노드] 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
