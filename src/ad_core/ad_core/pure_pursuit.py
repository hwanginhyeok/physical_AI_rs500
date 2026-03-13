"""Pure Pursuit 경로 추종 알고리즘 모듈.

속도 비례 lookahead distance를 사용하여 목표 경로를 추종하는
Pure Pursuit 제어기를 구현한다. 궤도차량(스키드 스티어) 특성을 고려하여
cmd_vel을 좌/우 트랙 속도로 변환하는 기능을 포함한다.

Phase 2: 슬립 보상 적응형 Pure Pursuit
    - SlipEstimator: 오도메트리 vs 실제 위치 비교로 종/횡방향 슬립 비율 실시간 추정
    - TerrainSlipProfile: 지형별 기본 슬립 파라미터 테이블
    - SlipCompensatedPurePursuit: 슬립 기반 adaptive lookahead 및 조향 보상
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple, Union

from ad_core.datatypes import Pose2D
from ad_core.utils import normalize_angle, clamp

# 상수 정의
DEFAULT_LOOKAHEAD_SEARCH_WINDOW = 50  # 최근접점 탐색 윈도우 크기
MIN_CURVATURE_DISTANCE = 0.1  # 곡률 계산 시 최소 거리 (0으로 나누기 방지)
EPSILON = 1e-9  # 부동소수점 비교용
EPSILON_SQ = 1e-12  # 제곱 연산용
MAX_SLIP_COMPENSATION = 1.5  # 최대 슬립 보상 비율
DEFAULT_MIN_SPEED_RATIO = 0.3  # 곡률 기반 감속 시 최소 속도 비율
CURVATURE_SPEED_DECAY = 2.0  # 곡률 기반 속도 감소 계수

# 슬립 추정기 기본값
DEFAULT_EMA_ALPHA = 0.15
DEFAULT_MIN_DISPLACEMENT = 0.01
DEFAULT_CONFIDENCE_RATE = 0.05


@dataclass
class PurePursuitConfig:
    """Pure Pursuit 파라미터 설정.
    
    Attributes:
        lookahead_gain: 속도에 대한 lookahead distance 비례 계수.
        min_lookahead: 최소 lookahead distance (m).
        max_lookahead: 최대 lookahead distance (m).
        goal_tolerance: 목표 도달 판정 거리 (m).
        max_linear_speed: 최대 직선 속도 (m/s).
        track_width: 궤도차량 트랙 간격 (m).
    """

    lookahead_gain: float = 0.8
    min_lookahead: float = 1.0
    max_lookahead: float = 5.0
    goal_tolerance: float = 0.5
    max_linear_speed: float = 1.0
    track_width: float = 1.4


class PurePursuitTracker:
    """Pure Pursuit 기반 경로 추종 제어기.

    주어진 경로(waypoints)를 추종하기 위해 lookahead point를 계산하고,
    해당 점을 향한 곡률로부터 선속도/각속도 명령을 생성한다.

    Attributes:
        config: Pure Pursuit 파라미터 설정.
        path: 추종할 경로 웨이포인트 리스트 [(x, y), ...].
        nearest_idx: 현재 가장 가까운 경로점 인덱스.
        is_goal_reached: 경로 완료 여부.
    """

    def __init__(self, config: Optional[PurePursuitConfig] = None) -> None:
        """초기화.

        Args:
            config: Pure Pursuit 파라미터. None이면 기본값 사용.
        """
        self.config: PurePursuitConfig = config or PurePursuitConfig()
        self.path: List[Tuple[float, float]] = []
        self.nearest_idx: int = 0
        self.is_goal_reached: bool = False

    def set_path(self, waypoints: Union[List[Tuple[float, float]], object]) -> None:
        """추종할 경로를 설정한다.

        nav_msgs/Path 메시지 또는 (x, y) 튜플 리스트를 받는다.

        Args:
            waypoints: 경로 데이터. list of (x, y) 튜플이거나,
                       poses 속성을 가진 nav_msgs/Path 호환 객체.
        """
        self.nearest_idx = 0
        self.is_goal_reached = False

        # nav_msgs/Path 호환 객체인 경우
        if hasattr(waypoints, 'poses'):
            self.path = [
                (float(p.pose.position.x), float(p.pose.position.y))
                for p in waypoints.poses
            ]
        else:
            self.path = [(float(wp[0]), float(wp[1])) for wp in waypoints]

    def compute(
        self, current_pose: Pose2D, current_speed: float
    ) -> Tuple[float, float]:
        """현재 위치와 속도를 기반으로 제어 명령을 계산한다.

        Args:
            current_pose: 현재 차량 위치 및 방향 (x, y, yaw).
            current_speed: 현재 차량 속도 (m/s), 부호는 방향.

        Returns:
            (linear_vel, angular_vel) 튜플 (m/s, rad/s).
            경로가 비었거나 완료된 경우 (0.0, 0.0)을 반환.
        """
        if not self.path or self.is_goal_reached:
            return 0.0, 0.0

        # 1. 최근접 경로점 탐색
        self._update_nearest(current_pose)

        # 2. 경로 완료 감지
        if self._check_goal_reached(current_pose):
            self.is_goal_reached = True
            return 0.0, 0.0

        # 3. Lookahead distance 계산 (속도 비례, 범위 제한)
        ld = self._compute_lookahead_distance(current_speed)

        # 4. Lookahead 점 탐색 및 보간
        lookahead_pt = self._find_lookahead_point(current_pose, ld)
        if lookahead_pt is None:
            lookahead_pt = self.path[-1]

        # 5. 곡률 계산 (차량 좌표계 기준)
        curvature = self._compute_curvature(current_pose, lookahead_pt, ld)

        # 6. 속도 결정: 곡률이 클수록 감속
        linear_vel = self._compute_speed(curvature)

        # 7. 각속도 = 선속도 * 곡률
        angular_vel = linear_vel * curvature

        return linear_vel, angular_vel

    def compute_track_velocities(
        self, current_pose: Pose2D, current_speed: float
    ) -> Tuple[float, float]:
        """Pure Pursuit 결과를 좌/우 트랙 속도로 변환한다.

        궤도차량(스키드 스티어)을 위한 편의 메서드.

        Args:
            current_pose: 현재 차량 위치 및 방향.
            current_speed: 현재 차량 속도 (m/s).

        Returns:
            (v_left, v_right) 좌/우 트랙 속도 (m/s).
        """
        linear, angular = self.compute(current_pose, current_speed)
        return self._twist_to_tracks(linear, angular)

    # ------------------------------------------------------------------ #
    #  낮은 수준 메서드 (낮은 결합도를 위해 private)
    # ------------------------------------------------------------------ #

    def _compute_lookahead_distance(self, speed: float) -> float:
        """속도 비례 lookahead distance를 계산한다.

        ld = clamp(lookahead_gain * |speed|, min_lookahead, max_lookahead)

        Args:
            speed: 현재 속도 (m/s).

        Returns:
            클램핑된 lookahead distance (m).
        """
        ld = self.config.lookahead_gain * abs(speed)
        return clamp(ld, self.config.min_lookahead, self.config.max_lookahead)

    def _update_nearest(self, pose: Pose2D) -> None:
        """현재 위치에서 가장 가까운 경로점 인덱스를 갱신한다.

        탐색은 이전 nearest_idx부터 전방으로만 진행하여 뒤로 돌아가지 않는다.

        Args:
            pose: 현재 차량 위치.
        """
        min_dist_sq = float('inf')
        best_idx = self.nearest_idx

        search_end = min(
            self.nearest_idx + DEFAULT_LOOKAHEAD_SEARCH_WINDOW, 
            len(self.path)
        )
        for i in range(self.nearest_idx, search_end):
            dx = self.path[i][0] - pose.x
            dy = self.path[i][1] - pose.y
            dist_sq = dx * dx + dy * dy
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_idx = i

        self.nearest_idx = best_idx

    def _check_goal_reached(self, pose: Pose2D) -> bool:
        """마지막 경로점에 도달했는지 확인한다.

        Args:
            pose: 현재 차량 위치.

        Returns:
            목표 도달 여부.
        """
        goal = self.path[-1]
        dist = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
        return dist <= self.config.goal_tolerance

    def _find_lookahead_point(
        self, pose: Pose2D, ld: float
    ) -> Optional[Tuple[float, float]]:
        """경로 위에서 lookahead distance에 해당하는 보간 점을 찾는다.

        nearest_idx부터 전방으로 탐색하며, 차량과의 거리가
        ld를 처음으로 넘는 구간에서 선형 보간하여 정확한 점을 계산한다.

        Args:
            pose: 현재 차량 위치.
            ld: lookahead distance (m).

        Returns:
            보간된 (x, y) 좌표. 찾지 못하면 None.
        """
        for i in range(self.nearest_idx, len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]

            d1 = math.hypot(p1[0] - pose.x, p1[1] - pose.y)
            d2 = math.hypot(p2[0] - pose.x, p2[1] - pose.y)

            # p1은 ld 안쪽, p2는 ld 바깥쪽인 구간에서 보간
            if d1 < ld <= d2:
                ratio = (ld - d1) / (d2 - d1) if (d2 - d1) > EPSILON else 0.0
                x = p1[0] + ratio * (p2[0] - p1[0])
                y = p1[1] + ratio * (p2[1] - p1[1])
                return (x, y)

            # 원-직선 교차 방식 (보다 정확한 보간)
            pt = self._circle_line_intersection(pose, ld, p1, p2)
            if pt is not None:
                return pt

        return None

    @staticmethod
    def _circle_line_intersection(
        pose: Pose2D, 
        radius: float,
        p1: Tuple[float, float], 
        p2: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        """차량 위치를 중심으로 한 원과 경로 선분의 교점을 계산한다.

        전방(p2 방향) 교점을 우선 반환한다.

        Args:
            pose: 원 중심 (차량 위치).
            radius: 원 반지름 (lookahead distance).
            p1: 선분 시작점.
            p2: 선분 끝점.

        Returns:
            교점 (x, y) 또는 교점이 없으면 None.
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        fx = p1[0] - pose.x
        fy = p1[1] - pose.y

        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius

        discriminant = b * b - 4.0 * a * c

        if a < EPSILON_SQ or discriminant < 0.0:
            return None

        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)

        # t가 [0, 1] 범위인 교점 중 전방(t가 큰) 것을 선택
        t = None
        if 0.0 <= t2 <= 1.0:
            t = t2
        if 0.0 <= t1 <= 1.0:
            if t is None or t1 > t:
                t = t1

        if t is None:
            return None

        x = p1[0] + t * dx
        y = p1[1] + t * dy
        return (x, y)

    @staticmethod
    def _compute_curvature(
        pose: Pose2D,
        target: Tuple[float, float],
        ld: float
    ) -> float:
        """차량 좌표계에서 lookahead 점까지의 곡률을 계산한다.

        Pure Pursuit 공식: curvature = 2 * sin(alpha) / ld
        여기서 alpha는 차량 전방 방향과 lookahead 점 방향 사이의 각도.

        Args:
            pose: 현재 차량 위치 및 방향.
            target: lookahead 점 (x, y).
            ld: lookahead distance (m).

        Returns:
            곡률 (1/m). 양수=좌회전, 음수=우회전.
        """
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        actual_dist = math.hypot(dx, dy)

        if actual_dist < EPSILON:
            return 0.0

        target_angle = math.atan2(dy, dx)
        alpha = normalize_angle(target_angle - pose.yaw)

        curvature = 2.0 * math.sin(alpha) / max(actual_dist, MIN_CURVATURE_DISTANCE)
        return curvature

    def _compute_speed(self, curvature: float) -> float:
        """곡률에 따라 적절한 선속도를 계산한다.

        곡률이 클수록 감속하여 안정적인 추종을 보장한다.

        Args:
            curvature: 현재 곡률 (1/m).

        Returns:
            목표 선속도 (m/s).
        """
        max_speed = self.config.max_linear_speed
        curvature_factor = 1.0 / (1.0 + CURVATURE_SPEED_DECAY * abs(curvature))
        return max_speed * max(DEFAULT_MIN_SPEED_RATIO, curvature_factor)

    def _twist_to_tracks(
        self, linear: float, angular: float
    ) -> Tuple[float, float]:
        """(linear, angular) 속도를 좌/우 트랙 속도로 변환한다.

        차동 구동 모델:
            v_left  = v - omega * W / 2
            v_right = v + omega * W / 2

        Args:
            linear: 선속도 (m/s).
            angular: 각속도 (rad/s).

        Returns:
            (v_left, v_right) 트랙 속도 (m/s).
        """
        half_width = self.config.track_width / 2.0
        v_left = linear - angular * half_width
        v_right = linear + angular * half_width

        # 최대 속도 포화 처리 (비율 유지)
        max_abs = max(abs(v_left), abs(v_right))
        max_speed = self.config.max_linear_speed
        if max_abs > max_speed and max_abs > EPSILON:
            scale = max_speed / max_abs
            v_left *= scale
            v_right *= scale

        return v_left, v_right


# ====================================================================== #
#  Phase 2: 슬립 보상 적응형 Pure Pursuit
# ====================================================================== #


class TerrainType(Enum):
    """슬립 프로파일에서 사용하는 지형 유형."""

    PAVED = "paved"
    DIRT_ROAD = "dirt_road"
    GRASS = "grass"
    GRAVEL = "gravel"
    CROP_FIELD = "crop_field"
    MUD = "mud"


@dataclass
class TerrainSlipProfile:
    """지형별 슬립 특성 프로파일.

    Attributes:
        longitudinal_slip: 종방향(전후) 기본 슬립 비율 (0.0 ~ 1.0).
        lateral_slip: 횡방향(좌우) 기본 슬립 비율 (0.0 ~ 1.0).
        slip_variability: 슬립 변동성 계수 (0.0 ~ 1.0).
        max_safe_speed: 해당 지형에서의 최대 안전 속도 (m/s).
    """

    longitudinal_slip: float = 0.0
    lateral_slip: float = 0.0
    slip_variability: float = 0.0
    max_safe_speed: float = 1.0


# 지형별 기본 슬립 파라미터 테이블
TERRAIN_SLIP_TABLE: Dict[TerrainType, TerrainSlipProfile] = {
    TerrainType.PAVED: TerrainSlipProfile(
        longitudinal_slip=0.02,
        lateral_slip=0.03,
        slip_variability=0.05,
        max_safe_speed=1.0,
    ),
    TerrainType.DIRT_ROAD: TerrainSlipProfile(
        longitudinal_slip=0.08,
        lateral_slip=0.12,
        slip_variability=0.15,
        max_safe_speed=0.8,
    ),
    TerrainType.GRASS: TerrainSlipProfile(
        longitudinal_slip=0.10,
        lateral_slip=0.15,
        slip_variability=0.20,
        max_safe_speed=0.7,
    ),
    TerrainType.GRAVEL: TerrainSlipProfile(
        longitudinal_slip=0.06,
        lateral_slip=0.10,
        slip_variability=0.12,
        max_safe_speed=0.8,
    ),
    TerrainType.CROP_FIELD: TerrainSlipProfile(
        longitudinal_slip=0.12,
        lateral_slip=0.18,
        slip_variability=0.25,
        max_safe_speed=0.6,
    ),
    TerrainType.MUD: TerrainSlipProfile(
        longitudinal_slip=0.25,
        lateral_slip=0.30,
        slip_variability=0.40,
        max_safe_speed=0.4,
    ),
}

# 기본 프로파일 (알 수 없는 지형에 사용)
DEFAULT_SLIP_PROFILE = TerrainSlipProfile(
    longitudinal_slip=0.10,
    lateral_slip=0.15,
    slip_variability=0.20,
    max_safe_speed=0.7,
)


def get_terrain_slip_profile(terrain: TerrainType) -> TerrainSlipProfile:
    """지형 유형에 해당하는 슬립 프로파일을 반환한다.

    Args:
        terrain: 지형 유형.

    Returns:
        해당 지형의 TerrainSlipProfile. 미등록 지형이면 기본값 반환.
    """
    return TERRAIN_SLIP_TABLE.get(terrain, DEFAULT_SLIP_PROFILE)


def terrain_type_from_name(name: str) -> TerrainType:
    """문자열 이름으로부터 TerrainType을 조회한다.

    Args:
        name: 지형 이름 문자열 (대소문자 무관). 예: "MUD", "paved".

    Returns:
        매칭되는 TerrainType. 매칭 실패 시 TerrainType.DIRT_ROAD 반환.
    """
    name_upper = name.upper()
    for t in TerrainType:
        if t.name == name_upper:
            return t
    return TerrainType.DIRT_ROAD


class SlipEstimator:
    """실시간 슬립 추정기.

    오도메트리(명령 기반 예측 위치)와 실측 위치(로컬라이제이션 결과)를
    비교하여 종방향 및 횡방향 슬립 비율을 실시간으로 추정한다.

    Attributes:
        longitudinal_slip: 현재 추정 종방향 슬립 비율 (0.0 ~ 1.0).
        lateral_slip: 현재 추정 횡방향 슬립 비율 (0.0 ~ 1.0).
        slip_confidence: 슬립 추정 신뢰도 (0.0 ~ 1.0).
    """

    def __init__(
        self,
        ema_alpha: float = DEFAULT_EMA_ALPHA,
        min_displacement: float = DEFAULT_MIN_DISPLACEMENT,
        confidence_rate: float = DEFAULT_CONFIDENCE_RATE,
    ) -> None:
        """초기화.

        Args:
            ema_alpha: EMA 스무딩 계수 (0 < alpha <= 1).
            min_displacement: 슬립 계산을 위한 최소 변위 (m).
            confidence_rate: 업데이트마다 신뢰도 증가 비율.
        """
        self._ema_alpha: float = clamp(ema_alpha, 0.01, 1.0)
        self._min_displacement: float = min_displacement
        self._confidence_rate: float = confidence_rate

        self.longitudinal_slip: float = 0.0
        self.lateral_slip: float = 0.0
        self.slip_confidence: float = 0.0

        self._prev_odom_pose: Optional[Pose2D] = None
        self._prev_actual_pose: Optional[Pose2D] = None
        self._prev_time: Optional[float] = None
        self._terrain_prior: Optional[TerrainSlipProfile] = None

    @property
    def total_slip_magnitude(self) -> float:
        """종/횡방향 슬립의 결합 크기.

        Returns:
            유클리드 노름 기반의 전체 슬립 크기.
        """
        return math.hypot(self.longitudinal_slip, self.lateral_slip)

    def reset(self) -> None:
        """추정기 상태를 초기화한다."""
        self.longitudinal_slip = 0.0
        self.lateral_slip = 0.0
        self.slip_confidence = 0.0
        self._prev_odom_pose = None
        self._prev_actual_pose = None
        self._prev_time = None

    def set_terrain_prior(self, terrain: TerrainType) -> None:
        """지형 유형에 따른 사전 슬립 정보를 설정한다.

        Args:
            terrain: 현재 지형 유형.
        """
        self._terrain_prior = get_terrain_slip_profile(terrain)

        if self.slip_confidence < 0.3:
            prior_weight = 1.0 - self.slip_confidence
            self.longitudinal_slip = (
                self.longitudinal_slip * (1.0 - prior_weight)
                + self._terrain_prior.longitudinal_slip * prior_weight
            )
            self.lateral_slip = (
                self.lateral_slip * (1.0 - prior_weight)
                + self._terrain_prior.lateral_slip * prior_weight
            )

    def update(
        self,
        odom_pose: Pose2D,
        actual_pose: Pose2D,
        current_time: Optional[float] = None,
    ) -> Tuple[float, float]:
        """오도메트리와 실측 위치를 비교하여 슬립을 추정한다.

        Args:
            odom_pose: 오도메트리 기반 예측 위치.
            actual_pose: 로컬라이제이션 기반 실측 위치.
            current_time: 현재 시각 (초). None이면 time.monotonic() 사용.

        Returns:
            (longitudinal_slip, lateral_slip) 현재 추정값.
        """
        if current_time is None:
            current_time = time.monotonic()

        if self._prev_odom_pose is None or self._prev_actual_pose is None:
            self._prev_odom_pose = Pose2D(
                x=odom_pose.x, y=odom_pose.y, yaw=odom_pose.yaw)
            self._prev_actual_pose = Pose2D(
                x=actual_pose.x, y=actual_pose.y, yaw=actual_pose.yaw)
            self._prev_time = current_time
            return self.longitudinal_slip, self.lateral_slip

        odom_dx = odom_pose.x - self._prev_odom_pose.x
        odom_dy = odom_pose.y - self._prev_odom_pose.y
        odom_disp = math.hypot(odom_dx, odom_dy)

        actual_dx = actual_pose.x - self._prev_actual_pose.x
        actual_dy = actual_pose.y - self._prev_actual_pose.y

        if odom_disp < self._min_displacement:
            self._prev_odom_pose = Pose2D(
                x=odom_pose.x, y=odom_pose.y, yaw=odom_pose.yaw)
            self._prev_actual_pose = Pose2D(
                x=actual_pose.x, y=actual_pose.y, yaw=actual_pose.yaw)
            self._prev_time = current_time
            return self.longitudinal_slip, self.lateral_slip

        ref_yaw = self._prev_actual_pose.yaw
        cos_yaw = math.cos(ref_yaw)
        sin_yaw = math.sin(ref_yaw)

        odom_forward = odom_dx * cos_yaw + odom_dy * sin_yaw
        odom_lateral = -odom_dx * sin_yaw + odom_dy * cos_yaw

        actual_forward = actual_dx * cos_yaw + actual_dy * sin_yaw
        actual_lateral = -actual_dx * sin_yaw + actual_dy * cos_yaw

        if abs(odom_forward) > self._min_displacement:
            raw_long_slip = 1.0 - (actual_forward / odom_forward)
            raw_long_slip = clamp(raw_long_slip, 0.0, 1.0)
        else:
            raw_long_slip = self.longitudinal_slip

        denom = max(abs(actual_forward), self._min_displacement)
        raw_lat_slip = abs(actual_lateral - odom_lateral) / denom
        raw_lat_slip = clamp(raw_lat_slip, 0.0, 1.0)

        alpha = self._ema_alpha
        self.longitudinal_slip = (
            alpha * raw_long_slip + (1.0 - alpha) * self.longitudinal_slip
        )
        self.lateral_slip = (
            alpha * raw_lat_slip + (1.0 - alpha) * self.lateral_slip
        )

        self.slip_confidence = min(1.0, self.slip_confidence + self._confidence_rate)

        self._prev_odom_pose = Pose2D(
            x=odom_pose.x, y=odom_pose.y, yaw=odom_pose.yaw)
        self._prev_actual_pose = Pose2D(
            x=actual_pose.x, y=actual_pose.y, yaw=actual_pose.yaw)
        self._prev_time = current_time

        return self.longitudinal_slip, self.lateral_slip


@dataclass
class SlipCompensatedPurePursuitConfig(PurePursuitConfig):
    """슬립 보상 Pure Pursuit 파라미터 설정."""

    slip_lookahead_gain: float = 2.0
    max_slip_lookahead_ratio: float = 2.0
    longitudinal_compensation_gain: float = 1.0
    lateral_compensation_gain: float = 0.8
    slip_speed_reduction_gain: float = 0.5
    min_speed_ratio: float = 0.2
    ema_alpha: float = DEFAULT_EMA_ALPHA
    enable_terrain_adaptation: bool = True


class SlipCompensatedPurePursuit(PurePursuitTracker):
    """슬립 보상 적응형 Pure Pursuit 경로 추종 제어기."""

    def __init__(
        self,
        config: Optional[SlipCompensatedPurePursuitConfig] = None,
    ) -> None:
        """초기화.

        Args:
            config: 슬립 보상 Pure Pursuit 파라미터. None이면 기본값 사용.
        """
        if config is None:
            config = SlipCompensatedPurePursuitConfig()
        super().__init__(config=config)

        self.slip_config: SlipCompensatedPurePursuitConfig = config
        self.slip_estimator: SlipEstimator = SlipEstimator(
            ema_alpha=config.ema_alpha,
        )
        self.current_terrain: Optional[TerrainType] = None
        self._current_terrain_profile: TerrainSlipProfile = DEFAULT_SLIP_PROFILE

    def set_path(self, waypoints: Union[List[Tuple[float, float]], object]) -> None:
        """경로를 설정하고 슬립 추정기를 리셋한다."""
        super().set_path(waypoints)
        self.slip_estimator.reset()
        if self.current_terrain is not None:
            self.slip_estimator.set_terrain_prior(self.current_terrain)

    def set_terrain(self, terrain: TerrainType) -> None:
        """현재 주행 지형을 설정한다."""
        self.current_terrain = terrain
        self._current_terrain_profile = get_terrain_slip_profile(terrain)
        if self.slip_config.enable_terrain_adaptation:
            self.slip_estimator.set_terrain_prior(terrain)

    def update_slip(
        self,
        odom_pose: Pose2D,
        actual_pose: Pose2D,
        current_time: Optional[float] = None,
    ) -> Tuple[float, float]:
        """슬립 추정을 갱신한다.

        Returns:
            (longitudinal_slip, lateral_slip) 현재 추정값.
        """
        return self.slip_estimator.update(odom_pose, actual_pose, current_time)

    def get_slip_info(self) -> dict:
        """현재 슬립 관련 진단 정보를 반환한다."""
        return {
            "longitudinal_slip": self.slip_estimator.longitudinal_slip,
            "lateral_slip": self.slip_estimator.lateral_slip,
            "total_slip": self.slip_estimator.total_slip_magnitude,
            "confidence": self.slip_estimator.slip_confidence,
            "terrain": (
                self.current_terrain.name
                if self.current_terrain is not None
                else "UNKNOWN"
            ),
            "terrain_base_long_slip": self._current_terrain_profile.longitudinal_slip,
            "terrain_base_lat_slip": self._current_terrain_profile.lateral_slip,
        }

    def _compute_lookahead_distance(self, speed: float) -> float:
        """슬립 적응형 lookahead distance를 계산한다."""
        base_ld = self.config.lookahead_gain * abs(speed)
        base_ld = max(self.config.min_lookahead, base_ld)

        total_slip = self.slip_estimator.total_slip_magnitude
        slip_addition = self.slip_config.slip_lookahead_gain * total_slip * base_ld
        ld = base_ld + slip_addition

        effective_max = self.config.max_lookahead * self.slip_config.max_slip_lookahead_ratio
        return clamp(ld, self.config.min_lookahead, effective_max)

    def _compute_speed(self, curvature: float) -> float:
        """슬립을 반영하여 안전한 속도를 계산한다."""
        base_speed = super()._compute_speed(curvature)

        total_slip = self.slip_estimator.total_slip_magnitude
        slip_factor = 1.0 - (self.slip_config.slip_speed_reduction_gain * total_slip)
        slip_factor = max(self.slip_config.min_speed_ratio, slip_factor)

        speed = base_speed * slip_factor
        return min(speed, self._current_terrain_profile.max_safe_speed)

    def _twist_to_tracks(
        self, linear: float, angular: float
    ) -> Tuple[float, float]:
        """슬립 보상된 좌/우 트랙 속도를 계산한다."""
        long_slip = self.slip_estimator.longitudinal_slip
        long_gain = self.slip_config.longitudinal_compensation_gain
        long_comp = 1.0 + long_gain * long_slip
        long_comp = min(long_comp, MAX_SLIP_COMPENSATION)
        compensated_linear = linear * long_comp

        lat_slip = self.slip_estimator.lateral_slip
        lat_gain = self.slip_config.lateral_compensation_gain
        lat_comp = 1.0 + lat_gain * lat_slip
        lat_comp = min(lat_comp, MAX_SLIP_COMPENSATION)
        compensated_angular = angular * lat_comp

        half_width = self.config.track_width / 2.0
        v_left = compensated_linear - compensated_angular * half_width
        v_right = compensated_linear + compensated_angular * half_width

        max_abs = max(abs(v_left), abs(v_right))
        max_speed = self.config.max_linear_speed
        if max_abs > max_speed and max_abs > EPSILON:
            scale = max_speed / max_abs
            v_left *= scale
            v_right *= scale

        return v_left, v_right
