"""LIO-SAM SLAM 통합 위치 추정 관리 모듈 - GPS 가용/불가 자동 전환.

GPS 상태를 모니터링하고, GPS 가용성에 따라 위치 추정 모드를
자동으로 전환한다.

모드 전환 전략:
  - GPS 양호 (GOOD)     -> GPS_EKF   : Dual-EKF (robot_localization) 사용
  - GPS 불량 (UNAVAILABLE) -> SLAM_ONLY : LIO-SAM LiDAR-IMU SLAM 의존
  - GPS 복구 (DEGRADED)  -> HYBRID    : SLAM + GPS 점진적 전환

선행연구 참조:
  - LIO-SAM: Shan et al., IROS 2020 (인수 그래프 기반 LiDAR-IMU-GPS 융합)
  - robot_localization: Moore & Stouch, 2016 (Dual-EKF 센서 융합)
  - GPS-Denied SLAM: Jiang et al., IET 2025 (GPS 차단 LiDAR SLAM 전략)
"""

import math
import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, List, Callable

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
    """위치 추정 모드.

    GPS_EKF:   GPS 양호 - Dual-EKF (robot_localization) 기반 위치 추정.
               ekf_local (IMU+Wheel) + ekf_global (GPS 융합) 활성.
    SLAM_ONLY: GPS 불가 - LIO-SAM LiDAR-IMU SLAM 단독 위치 추정.
               GPS 팩터 비활성, LiDAR 오도메트리 + 루프 클로저 의존.
    HYBRID:    GPS 복구 중 - SLAM 기반 유지하면서 GPS를 점진적으로 융합.
               GPS 가중치를 서서히 높여 급격한 포즈 점프를 방지.
    """
    GPS_EKF = auto()
    SLAM_ONLY = auto()
    HYBRID = auto()


class GPSHealthStatus(Enum):
    """GPS 건강 상태.

    GOOD:        fix_type >= 2 (3D Fix 이상), HDOP <= 임계값, 위성 수 충분.
    DEGRADED:    GPS 수신되나 품질 저하 (HDOP 높음, 위성 수 부족, fix 불안정).
    UNAVAILABLE: GPS 미수신 (연속 N초 이상 데이터 없음 또는 No Fix).
    """
    GOOD = auto()
    DEGRADED = auto()
    UNAVAILABLE = auto()


# ======================================================================
# GPS 건강 모니터
# ======================================================================

@dataclass
class GPSHealthConfig:
    """GPS 건강 판단 기준 설정.

    Attributes:
        hdop_good_threshold: GOOD 판정 HDOP 상한 (m).
        hdop_degraded_threshold: DEGRADED 판정 HDOP 상한 (m). 이를 초과하면 UNAVAILABLE.
        min_satellites_good: GOOD 판정 최소 위성 수.
        min_satellites_degraded: DEGRADED 판정 최소 위성 수.
        timeout_sec: GPS 메시지 미수신 시 UNAVAILABLE 판정까지 대기 시간 (초).
        consecutive_bad_count: 연속 N회 불량 시 상태 전환.
        consecutive_good_count: 연속 N회 양호 시 GOOD 복귀.
    """
    hdop_good_threshold: float = 2.0
    hdop_degraded_threshold: float = 5.0
    min_satellites_good: int = 6
    min_satellites_degraded: int = 4
    timeout_sec: float = 3.0
    consecutive_bad_count: int = 5
    consecutive_good_count: int = 10


class GPSHealthMonitor:
    """GPS fix 품질 모니터링 클래스.

    NavSatFix 메시지를 수신하여 GPS의 fix 유형, HDOP,
    위성 수 등을 분석하고 GPS 건강 상태를 판정한다.

    상태 전이:
      GOOD -> DEGRADED: 연속 consecutive_bad_count회 품질 저하 감지
      DEGRADED -> UNAVAILABLE: timeout_sec 이상 미수신 또는 연속 불량
      UNAVAILABLE -> DEGRADED: GPS 재수신 시작 (품질 불충분)
      DEGRADED -> GOOD: 연속 consecutive_good_count회 양호 감지
    """

    def __init__(
        self,
        node: Node,
        config: Optional[GPSHealthConfig] = None,
    ):
        """GPSHealthMonitor를 초기화한다.

        Args:
            node: ROS2 노드 인스턴스.
            config: GPS 건강 판단 기준 설정. None이면 기본값 사용.
        """
        self._node = node
        self._config = config if config is not None else GPSHealthConfig()

        # 내부 상태
        self._status = GPSHealthStatus.UNAVAILABLE
        self._last_fix_time: Optional[float] = None
        self._last_fix_type: int = -1  # NavSatStatus 상수
        self._last_hdop: float = 99.0
        self._last_num_satellites: int = 0
        self._consecutive_good: int = 0
        self._consecutive_bad: int = 0

        # 상태 변경 콜백 리스트
        self._on_status_change_callbacks: List[Callable] = []

        # GPS NavSatFix 구독
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

    # ------------------------------------------------------------------
    # 속성
    # ------------------------------------------------------------------

    @property
    def status(self) -> GPSHealthStatus:
        """현재 GPS 건강 상태를 반환한다."""
        return self._status

    @property
    def last_hdop(self) -> float:
        """마지막 수신된 HDOP 값을 반환한다."""
        return self._last_hdop

    @property
    def last_num_satellites(self) -> int:
        """마지막 수신된 위성 수를 반환한다."""
        return self._last_num_satellites

    @property
    def last_fix_type(self) -> int:
        """마지막 fix 유형을 반환한다.

        Returns:
            NavSatStatus 상수값:
              -1 = STATUS_NO_FIX
               0 = STATUS_FIX
               1 = STATUS_SBAS_FIX
               2 = STATUS_GBAS_FIX (RTK 등)
        """
        return self._last_fix_type

    @property
    def seconds_since_last_fix(self) -> float:
        """마지막 GPS fix 이후 경과 시간(초)을 반환한다.

        한 번도 수신하지 않은 경우 float('inf')를 반환한다.
        """
        if self._last_fix_time is None:
            return float('inf')
        return time.monotonic() - self._last_fix_time

    # ------------------------------------------------------------------
    # 콜백 등록
    # ------------------------------------------------------------------

    def on_status_change(self, callback: Callable) -> None:
        """GPS 상태 변경 시 호출될 콜백을 등록한다.

        콜백 시그니처: callback(old_status: GPSHealthStatus, new_status: GPSHealthStatus)

        Args:
            callback: 상태 변경 콜백 함수.
        """
        self._on_status_change_callbacks.append(callback)

    # ------------------------------------------------------------------
    # GPS 콜백 및 상태 업데이트
    # ------------------------------------------------------------------

    def _gps_callback(self, msg: NavSatFix) -> None:
        """NavSatFix 메시지 수신 콜백.

        GPS 데이터를 분석하여 건강 상태를 업데이트한다.

        Args:
            msg: NavSatFix 메시지.
        """
        now = time.monotonic()

        # fix 정보 추출
        fix_type = msg.status.status  # -1, 0, 1, 2
        self._last_fix_type = fix_type
        self._last_fix_time = now

        # HDOP 추출: position_covariance 행렬의 대각 요소로 근사
        # NavSatFix에는 HDOP 필드가 없으므로 공분산에서 추정
        # position_covariance[0] = variance_x, [4] = variance_y
        if (msg.position_covariance_type !=
                NavSatFix.COVARIANCE_TYPE_UNKNOWN):
            var_x = msg.position_covariance[0]
            var_y = msg.position_covariance[4]
            # HDOP 근사: sqrt(var_x + var_y) (미터 단위 공분산 가정)
            self._last_hdop = math.sqrt(max(0.0, var_x) + max(0.0, var_y))
        else:
            # 공분산 정보 없으면 fix 유형 기반 추정
            if fix_type >= NavSatStatus.STATUS_GBAS_FIX:
                self._last_hdop = 0.5  # RTK 수준
            elif fix_type >= NavSatStatus.STATUS_FIX:
                self._last_hdop = 1.5  # 일반 Fix
            else:
                self._last_hdop = 99.0  # No Fix

        # 위성 수: NavSatFix 표준 메시지에는 위성 수 필드가 없음
        # position_covariance의 품질로 간접 추정하거나, 별도 토픽 필요
        # 여기서는 fix 유형과 HDOP으로 위성 수를 간접 추정
        if fix_type >= NavSatStatus.STATUS_GBAS_FIX:
            self._last_num_satellites = max(self._last_num_satellites, 8)
        elif fix_type >= NavSatStatus.STATUS_SBAS_FIX:
            self._last_num_satellites = max(6, self._last_num_satellites)
        elif fix_type >= NavSatStatus.STATUS_FIX:
            self._last_num_satellites = max(4, self._last_num_satellites)
        else:
            self._last_num_satellites = 0

        # 품질 판정
        is_good = self._evaluate_quality_good()
        is_degraded = self._evaluate_quality_degraded()

        if is_good:
            self._consecutive_good += 1
            self._consecutive_bad = 0
        elif is_degraded:
            self._consecutive_good = 0
            self._consecutive_bad = 0  # DEGRADED는 bad count 리셋하지 않음
        else:
            self._consecutive_good = 0
            self._consecutive_bad += 1

        self._update_status()

    def _evaluate_quality_good(self) -> bool:
        """현재 GPS 품질이 GOOD 기준을 충족하는지 판단한다.

        Returns:
            True이면 GOOD 기준 충족.
        """
        return (
            self._last_fix_type >= NavSatStatus.STATUS_FIX
            and self._last_hdop <= self._config.hdop_good_threshold
            and self._last_num_satellites >= self._config.min_satellites_good
        )

    def _evaluate_quality_degraded(self) -> bool:
        """현재 GPS 품질이 DEGRADED 기준을 충족하는지 판단한다.

        GOOD 기준에는 미달하지만 UNAVAILABLE은 아닌 상태.

        Returns:
            True이면 DEGRADED 기준 충족.
        """
        return (
            self._last_fix_type >= NavSatStatus.STATUS_FIX
            and self._last_hdop <= self._config.hdop_degraded_threshold
            and self._last_num_satellites >= self._config.min_satellites_degraded
        )

    def _update_status(self) -> None:
        """내부 카운터를 기반으로 GPS 건강 상태를 전이한다."""
        old_status = self._status

        if self._status == GPSHealthStatus.GOOD:
            # GOOD -> DEGRADED: 연속 불량 감지
            if self._consecutive_bad >= self._config.consecutive_bad_count:
                self._status = GPSHealthStatus.DEGRADED
                self._node.get_logger().warn(
                    f'[GPS모니터] 상태 전환: GOOD -> DEGRADED '
                    f'(HDOP={self._last_hdop:.1f}, '
                    f'위성={self._last_num_satellites})'
                )

        elif self._status == GPSHealthStatus.DEGRADED:
            # DEGRADED -> GOOD: 연속 양호 복귀
            if self._consecutive_good >= self._config.consecutive_good_count:
                self._status = GPSHealthStatus.GOOD
                self._node.get_logger().info(
                    f'[GPS모니터] 상태 전환: DEGRADED -> GOOD '
                    f'(HDOP={self._last_hdop:.1f}, '
                    f'위성={self._last_num_satellites})'
                )
            # DEGRADED -> UNAVAILABLE: 연속 불량 또는 타임아웃
            elif self._consecutive_bad >= self._config.consecutive_bad_count * 2:
                self._status = GPSHealthStatus.UNAVAILABLE
                self._node.get_logger().warn(
                    '[GPS모니터] 상태 전환: DEGRADED -> UNAVAILABLE'
                )

        elif self._status == GPSHealthStatus.UNAVAILABLE:
            # UNAVAILABLE -> DEGRADED: GPS 재수신 (품질 불충분)
            if self._evaluate_quality_degraded():
                self._status = GPSHealthStatus.DEGRADED
                self._node.get_logger().info(
                    f'[GPS모니터] 상태 전환: UNAVAILABLE -> DEGRADED '
                    f'(HDOP={self._last_hdop:.1f})'
                )
            # UNAVAILABLE -> GOOD: GPS 재수신 (품질 양호, 연속 충분)
            if (self._evaluate_quality_good()
                    and self._consecutive_good >= self._config.consecutive_good_count):
                self._status = GPSHealthStatus.GOOD
                self._node.get_logger().info(
                    '[GPS모니터] 상태 전환: UNAVAILABLE -> GOOD'
                )

        # 상태 변경 시 콜백 호출
        if old_status != self._status:
            for cb in self._on_status_change_callbacks:
                try:
                    cb(old_status, self._status)
                except Exception as e:
                    self._node.get_logger().error(
                        f'[GPS모니터] 콜백 오류: {e}'
                    )

    def check_timeout(self) -> None:
        """GPS 타임아웃을 확인하고 상태를 업데이트한다.

        주기적 타이머에서 호출되어, GPS 메시지 미수신 시
        UNAVAILABLE로 전환한다.
        """
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
            # 한 번도 GPS를 수신하지 못한 경우
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
        """GPS 건강 상태 요약 정보를 딕셔너리로 반환한다.

        Returns:
            GPS 건강 상태 요약 딕셔너리.
        """
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
    """위치 추정 모드 전환 설정.

    Attributes:
        hybrid_transition_duration_sec: HYBRID 모드 전환 기간 (초).
            GPS 복구 시 이 기간 동안 GPS 가중치를 점진적으로 증가시킨다.
        hybrid_gps_weight_initial: HYBRID 전환 시작 GPS 가중치 (0.0 ~ 1.0).
        hybrid_gps_weight_final: HYBRID 전환 완료 GPS 가중치 (0.0 ~ 1.0).
        pose_alignment_threshold_m: 모드 전환 시 포즈 정합 최대 허용 차이 (m).
            SLAM 포즈와 GPS 포즈 차이가 이보다 크면 점진적 정합 수행.
        pose_alignment_rate: 포즈 정합 속도 (0.0 ~ 1.0, 1이면 즉시 정합).
        slam_odom_topic: LIO-SAM 오도메트리 출력 토픽.
        gps_odom_topic: GPS 기반 오도메트리 토픽 (navsat_transform 출력).
        fused_odom_topic: 최종 융합 오도메트리 출력 토픽.
        mode_status_topic: 현재 모드 상태 퍼블리시 토픽.
    """
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
    """GPS 상태 기반 위치 추정 모드 자동 전환 관리자.

    GPS 건강 상태에 따라 위치 추정 소스를 자동으로 전환하며,
    모드 전환 시 초기 포즈 정합(pose alignment)을 수행하여
    급격한 위치 점프를 방지한다.

    모드 전환 규칙:
      GPS GOOD       -> GPS_EKF   (Dual-EKF 활성, SLAM은 백그라운드)
      GPS UNAVAILABLE -> SLAM_ONLY (LIO-SAM 단독, GPS 팩터 비활성)
      GPS DEGRADED   -> HYBRID    (SLAM 기반 + GPS 점진적 융합)

    아키텍처:
      GPS NavSatFix -> GPSHealthMonitor -> LocalizationManager
                                              |
                    LIO-SAM Odom -----> [모드별 포즈 선택/블렌딩]
                    GPS EKF Odom -----> [      |      ]
                                              v
                                    /odometry/localization (융합 출력)
    """

    def __init__(
        self,
        node: Node,
        gps_health_config: Optional[GPSHealthConfig] = None,
        localization_config: Optional[LocalizationConfig] = None,
    ):
        """LocalizationManager를 초기화한다.

        Args:
            node: ROS2 노드 인스턴스.
            gps_health_config: GPS 건강 판단 기준. None이면 기본값 사용.
            localization_config: 위치 추정 모드 전환 설정. None이면 기본값 사용.
        """
        self._node = node
        self._config = (localization_config
                        if localization_config is not None
                        else LocalizationConfig())

        # 현재 모드 및 상태
        self._mode = LocalizationMode.SLAM_ONLY  # 초기값: GPS 미확인
        self._hybrid_start_time: Optional[float] = None
        self._hybrid_gps_weight: float = 0.0

        # 포즈 정합 상태
        self._slam_pose: Optional[PoseStamped] = None
        self._gps_pose: Optional[PoseStamped] = None
        self._aligned_pose: Optional[PoseStamped] = None
        self._pose_offset_x: float = 0.0
        self._pose_offset_y: float = 0.0
        self._pose_offset_yaw: float = 0.0

        # 모드 전환 이력
        self._transition_log: List[dict] = []

        # GPS 건강 모니터 초기화
        self._gps_monitor = GPSHealthMonitor(node, gps_health_config)
        self._gps_monitor.on_status_change(self._on_gps_status_change)

        # 오도메트리 구독: LIO-SAM 출력
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

        # 오도메트리 구독: GPS EKF (ekf_global) 출력
        self._gps_odom_sub = node.create_subscription(
            Odometry,
            self._config.gps_odom_topic,
            self._gps_odom_callback,
            odom_qos,
        )

        # 융합 오도메트리 퍼블리셔
        self._fused_odom_pub = node.create_publisher(
            Odometry,
            self._config.fused_odom_topic,
            10,
        )

        # 모드 상태 퍼블리셔
        self._mode_pub = node.create_publisher(
            String,
            self._config.mode_status_topic,
            10,
        )

        # GPS 타임아웃 체크 타이머 (1Hz)
        self._timeout_timer = node.create_timer(1.0, self._timeout_check)

        # 모드 상태 퍼블리시 타이머 (2Hz)
        self._status_timer = node.create_timer(0.5, self._publish_mode_status)

        node.get_logger().info(
            f'[위치관리] LocalizationManager 초기화 완료 '
            f'(초기 모드: {self._mode.name})'
        )

    # ------------------------------------------------------------------
    # 속성
    # ------------------------------------------------------------------

    @property
    def mode(self) -> LocalizationMode:
        """현재 위치 추정 모드를 반환한다."""
        return self._mode

    @property
    def gps_monitor(self) -> GPSHealthMonitor:
        """GPS 건강 모니터 인스턴스를 반환한다."""
        return self._gps_monitor

    @property
    def hybrid_gps_weight(self) -> float:
        """현재 HYBRID 모드에서의 GPS 가중치를 반환한다.

        GPS_EKF 모드이면 1.0, SLAM_ONLY이면 0.0,
        HYBRID이면 0.0 ~ 1.0 사이 전환 중 값.
        """
        if self._mode == LocalizationMode.GPS_EKF:
            return 1.0
        elif self._mode == LocalizationMode.SLAM_ONLY:
            return 0.0
        return self._hybrid_gps_weight

    @property
    def transition_log(self) -> List[dict]:
        """모드 전환 이력 리스트를 반환한다."""
        return self._transition_log.copy()

    # ------------------------------------------------------------------
    # GPS 상태 변경 핸들러
    # ------------------------------------------------------------------

    def _on_gps_status_change(
        self,
        old_status: GPSHealthStatus,
        new_status: GPSHealthStatus,
    ) -> None:
        """GPS 건강 상태 변경 시 모드 전환을 처리한다.

        Args:
            old_status: 이전 GPS 건강 상태.
            new_status: 새 GPS 건강 상태.
        """
        old_mode = self._mode

        if new_status == GPSHealthStatus.GOOD:
            if self._mode == LocalizationMode.SLAM_ONLY:
                # SLAM -> HYBRID (점진적 GPS 복구)
                self._transition_to_hybrid()
            elif self._mode == LocalizationMode.HYBRID:
                # HYBRID 상태에서 GOOD이면 전환 가속화
                # (이미 HYBRID 중이므로 계속 진행)
                pass
            # 이미 GPS_EKF이면 변경 없음

        elif new_status == GPSHealthStatus.DEGRADED:
            if self._mode == LocalizationMode.GPS_EKF:
                # GPS_EKF -> HYBRID (GPS 성능 저하, 점진적 SLAM 전환)
                self._transition_to_hybrid_from_gps()
            elif self._mode == LocalizationMode.SLAM_ONLY:
                # SLAM -> HYBRID (GPS 일부 복구)
                self._transition_to_hybrid()
            # 이미 HYBRID이면 계속 유지

        elif new_status == GPSHealthStatus.UNAVAILABLE:
            if self._mode != LocalizationMode.SLAM_ONLY:
                # 어떤 모드든 -> SLAM_ONLY
                self._transition_to_slam_only()

        if old_mode != self._mode:
            self._log_transition(old_mode, self._mode, new_status)

    # ------------------------------------------------------------------
    # 모드 전환
    # ------------------------------------------------------------------

    def _transition_to_slam_only(self) -> None:
        """SLAM_ONLY 모드로 전환한다.

        GPS 불가 시 LIO-SAM 단독 위치 추정으로 전환.
        현재 포즈를 기준점으로 유지하여 위치 연속성을 보장한다.
        """
        self._mode = LocalizationMode.SLAM_ONLY
        self._hybrid_start_time = None
        self._hybrid_gps_weight = 0.0

        # 현재 GPS 포즈와 SLAM 포즈 간 오프셋 계산 (포즈 연속성)
        if self._slam_pose is not None and self._gps_pose is not None:
            self._compute_pose_offset()

        self._node.get_logger().warn(
            '[위치관리] 모드 전환: -> SLAM_ONLY '
            '(GPS 불가, LIO-SAM 단독 위치 추정)'
        )

    def _transition_to_hybrid(self) -> None:
        """SLAM_ONLY에서 HYBRID 모드로 전환한다.

        GPS가 복구되기 시작할 때 SLAM 기반을 유지하면서
        GPS 가중치를 점진적으로 증가시킨다.
        """
        self._mode = LocalizationMode.HYBRID
        self._hybrid_start_time = time.monotonic()
        self._hybrid_gps_weight = self._config.hybrid_gps_weight_initial

        # 포즈 정합 시작: SLAM 포즈와 GPS 포즈 간 오프셋 확인
        if self._slam_pose is not None and self._gps_pose is not None:
            self._compute_pose_offset()

        self._node.get_logger().info(
            '[위치관리] 모드 전환: -> HYBRID '
            f'(GPS 복구 중, 초기 GPS 가중치: '
            f'{self._config.hybrid_gps_weight_initial:.2f})'
        )

    def _transition_to_hybrid_from_gps(self) -> None:
        """GPS_EKF에서 HYBRID 모드로 전환한다.

        GPS 성능이 저하될 때 GPS 가중치를 점진적으로 감소시키면서
        SLAM으로 전환 준비한다.
        """
        self._mode = LocalizationMode.HYBRID
        self._hybrid_start_time = time.monotonic()
        # GPS에서 오는 것이므로 높은 GPS 가중치에서 시작
        self._hybrid_gps_weight = self._config.hybrid_gps_weight_final

        self._node.get_logger().info(
            '[위치관리] 모드 전환: GPS_EKF -> HYBRID '
            '(GPS 성능 저하, GPS 가중치 점진적 감소)'
        )

    def _transition_to_gps_ekf(self) -> None:
        """GPS_EKF 모드로 전환한다.

        HYBRID 전환 기간이 완료되고 GPS 상태가 양호하면 전환.
        """
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

    # ------------------------------------------------------------------
    # 포즈 정합
    # ------------------------------------------------------------------

    def _compute_pose_offset(self) -> None:
        """SLAM 포즈와 GPS 포즈 간 오프셋을 계산한다.

        모드 전환 시 두 소스 간의 위치 차이를 기록하여
        전환 시 급격한 포즈 점프를 방지한다.
        """
        if self._slam_pose is None or self._gps_pose is None:
            return

        slam_x = self._slam_pose.pose.position.x
        slam_y = self._slam_pose.pose.position.y
        gps_x = self._gps_pose.pose.position.x
        gps_y = self._gps_pose.pose.position.y

        self._pose_offset_x = gps_x - slam_x
        self._pose_offset_y = gps_y - slam_y

        # Yaw 오프셋 계산 (쿼터니언 -> yaw 변환)
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
        """SLAM 오도메트리와 GPS 오도메트리를 가중 블렌딩한다.

        포즈 정합 오프셋을 고려하여 두 소스를 보간하고,
        급격한 위치 점프 없이 부드럽게 전환한다.

        Args:
            slam_odom: LIO-SAM 오도메트리.
            gps_odom: GPS EKF 오도메트리.
            gps_weight: GPS 가중치 (0.0 = SLAM only, 1.0 = GPS only).

        Returns:
            블렌딩된 Odometry 메시지.
        """
        slam_weight = 1.0 - gps_weight

        blended = Odometry()
        blended.header = slam_odom.header
        blended.header.frame_id = 'map'
        blended.child_frame_id = 'base_link'

        # 위치 블렌딩
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
        blended.pose.pose.position.z = 0.0  # 2D 모드

        # 방향(Yaw) 블렌딩 (SLERP 근사: 2D에서는 각도 선형 보간)
        slam_yaw = self._quaternion_to_yaw(
            slam_odom.pose.pose.orientation.z,
            slam_odom.pose.pose.orientation.w,
        ) + self._pose_offset_yaw
        gps_yaw = self._quaternion_to_yaw(
            gps_odom.pose.pose.orientation.z,
            gps_odom.pose.pose.orientation.w,
        )

        # 각도 차이가 pi를 넘지 않도록 정규화
        yaw_diff = self._normalize_angle(gps_yaw - slam_yaw)
        blended_yaw = slam_yaw + gps_weight * yaw_diff

        blended.pose.pose.orientation.z = math.sin(blended_yaw / 2.0)
        blended.pose.pose.orientation.w = math.cos(blended_yaw / 2.0)
        blended.pose.pose.orientation.x = 0.0
        blended.pose.pose.orientation.y = 0.0

        # 속도: SLAM 기반 유지 (더 높은 주파수)
        blended.twist = slam_odom.twist

        # 공분산 블렌딩: GPS 가중치에 반비례하여 불확실성 조정
        # GPS 가중치가 높을수록 공분산이 작아짐 (더 확실)
        base_cov = 0.01  # 기본 공분산
        gps_cov_factor = max(0.001, 1.0 - gps_weight * 0.9)
        blended.pose.covariance[0] = base_cov * gps_cov_factor   # x
        blended.pose.covariance[7] = base_cov * gps_cov_factor   # y
        blended.pose.covariance[35] = base_cov * gps_cov_factor  # yaw

        return blended

    # ------------------------------------------------------------------
    # 오도메트리 콜백
    # ------------------------------------------------------------------

    def _slam_odom_callback(self, msg: Odometry) -> None:
        """LIO-SAM 오도메트리 수신 콜백.

        Args:
            msg: LIO-SAM 오도메트리 메시지.
        """
        # SLAM 포즈 캐시
        if self._slam_pose is None:
            self._slam_pose = PoseStamped()
        self._slam_pose.header = msg.header
        self._slam_pose.pose = msg.pose.pose

        # 모드별 출력 처리
        if self._mode == LocalizationMode.SLAM_ONLY:
            # SLAM 단독: SLAM 오도메트리를 오프셋 적용하여 퍼블리시
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

            # Yaw 오프셋 적용
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
            # SLAM 단독 시 공분산 증가 (GPS 없으므로 불확실성 높음)
            output.pose.covariance[0] = 0.05   # x
            output.pose.covariance[7] = 0.05   # y
            output.pose.covariance[35] = 0.02  # yaw

            self._fused_odom_pub.publish(output)

        elif self._mode == LocalizationMode.HYBRID:
            # HYBRID: GPS 오도메트리가 있으면 블렌딩
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
                # GPS 포즈 아직 없으면 SLAM 단독으로 퍼블리시
                self._publish_slam_only(msg)

    def _gps_odom_callback(self, msg: Odometry) -> None:
        """GPS EKF 오도메트리 수신 콜백.

        Args:
            msg: GPS EKF (ekf_global/navsat_transform) 오도메트리 메시지.
        """
        # GPS 포즈 캐시
        if self._gps_pose is None:
            self._gps_pose = PoseStamped()
        self._gps_pose.header = msg.header
        self._gps_pose.pose = msg.pose.pose

        # GPS_EKF 모드: GPS 오도메트리를 직접 퍼블리시
        if self._mode == LocalizationMode.GPS_EKF:
            output = Odometry()
            output.header = msg.header
            output.header.frame_id = 'map'
            output.child_frame_id = 'base_link'
            output.pose = msg.pose
            output.twist = msg.twist
            self._fused_odom_pub.publish(output)

    def _publish_slam_only(self, slam_odom: Odometry) -> None:
        """SLAM 오도메트리를 오프셋 적용하여 퍼블리시한다.

        _slam_odom_callback의 SLAM_ONLY 처리 로직을 재사용.

        Args:
            slam_odom: LIO-SAM 오도메트리 메시지.
        """
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

    # ------------------------------------------------------------------
    # HYBRID 가중치 갱신
    # ------------------------------------------------------------------

    def _update_hybrid_weight(self) -> None:
        """HYBRID 모드에서 GPS 가중치를 시간 경과에 따라 갱신한다.

        GPS가 복구되어 HYBRID로 진입한 경우:
          시간에 따라 GPS 가중치를 initial -> final로 선형 증가.
          전환 기간이 완료되고 GPS 상태가 GOOD이면 GPS_EKF로 전환.

        GPS가 저하되어 GPS_EKF -> HYBRID로 진입한 경우:
          시간에 따라 GPS 가중치를 final -> initial로 선형 감소.
        """
        if self._hybrid_start_time is None:
            return

        elapsed = time.monotonic() - self._hybrid_start_time
        duration = self._config.hybrid_transition_duration_sec

        gps_status = self._gps_monitor.status

        if gps_status == GPSHealthStatus.GOOD:
            # GPS 양호: 가중치 증가 방향
            progress = min(1.0, elapsed / duration)
            self._hybrid_gps_weight = (
                self._config.hybrid_gps_weight_initial
                + progress * (
                    self._config.hybrid_gps_weight_final
                    - self._config.hybrid_gps_weight_initial
                )
            )

            # 포즈 오프셋 점진적 감소 (정합 완료)
            decay = 1.0 - self._config.pose_alignment_rate
            self._pose_offset_x *= decay
            self._pose_offset_y *= decay
            self._pose_offset_yaw *= decay

            # 전환 완료 판정
            if progress >= 1.0:
                self._transition_to_gps_ekf()

        elif gps_status == GPSHealthStatus.DEGRADED:
            # GPS 저하: 가중치 유지 또는 미세 감소
            self._hybrid_gps_weight = max(
                self._config.hybrid_gps_weight_initial,
                self._hybrid_gps_weight * 0.99
            )

        elif gps_status == GPSHealthStatus.UNAVAILABLE:
            # GPS 불가: SLAM_ONLY로 즉시 전환 (콜백에서 처리)
            pass

    # ------------------------------------------------------------------
    # 타이머 콜백
    # ------------------------------------------------------------------

    def _timeout_check(self) -> None:
        """GPS 타임아웃 확인 타이머 콜백 (1Hz)."""
        self._gps_monitor.check_timeout()

    def _publish_mode_status(self) -> None:
        """현재 위치 추정 모드 상태를 퍼블리시한다 (2Hz)."""
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

    # ------------------------------------------------------------------
    # 이벤트 로깅
    # ------------------------------------------------------------------

    def _log_transition(
        self,
        old_mode: LocalizationMode,
        new_mode: LocalizationMode,
        gps_status: GPSHealthStatus,
    ) -> None:
        """모드 전환 이벤트를 로깅한다.

        Args:
            old_mode: 이전 위치 추정 모드.
            new_mode: 새 위치 추정 모드.
            gps_status: 전환 시점 GPS 건강 상태.
        """
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

        # 최대 100개 이력 유지
        if len(self._transition_log) > 100:
            self._transition_log = self._transition_log[-100:]

        self._node.get_logger().info(
            f'[위치관리] 전환 기록: {old_mode.name} -> {new_mode.name} '
            f'(GPS: {gps_status.name}, '
            f'HDOP: {entry["gps_hdop"]:.1f}, '
            f'위성: {entry["gps_satellites"]})'
        )

    # ------------------------------------------------------------------
    # 유틸리티
    # ------------------------------------------------------------------

    @staticmethod
    def _quaternion_to_yaw(qz: float, qw: float) -> float:
        """2D 쿼터니언(z, w)에서 yaw 각도를 추출한다.

        Z축 회전만 사용하는 2D 모드 가정.

        Args:
            qz: 쿼터니언 z 성분.
            qw: 쿼터니언 w 성분.

        Returns:
            Yaw 각도 (rad, -pi ~ pi).
        """
        return 2.0 * math.atan2(qz, qw)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """각도를 -pi ~ pi 범위로 정규화한다.

        Args:
            angle: 입력 각도 (rad).

        Returns:
            정규화된 각도 (rad, -pi ~ pi).
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ------------------------------------------------------------------
    # 외부 인터페이스
    # ------------------------------------------------------------------

    def get_status(self) -> dict:
        """현재 위치 추정 상태 요약을 반환한다.

        Returns:
            위치 추정 상태 딕셔너리:
              - mode: 현재 모드 이름
              - gps_health: GPS 건강 상태 요약
              - gps_weight: 현재 GPS 가중치
              - pose_offset: 포즈 오프셋 (x, y, yaw)
              - transition_count: 총 전환 횟수
        """
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
        """위치 추정 모드를 강제 전환한다.

        디버깅 또는 테스트 목적으로 수동 모드 전환 시 사용.

        Args:
            mode: 강제 전환할 모드.
        """
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
