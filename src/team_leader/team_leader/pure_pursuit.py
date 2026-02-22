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
from typing import Dict, List, Optional, Tuple


@dataclass
class Pose2D:
    """2D 위치 및 방향."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class PurePursuitConfig:
    """Pure Pursuit 파라미터 설정."""

    lookahead_gain: float = 0.8
    """속도에 대한 lookahead distance 비례 계수."""

    min_lookahead: float = 1.0
    """최소 lookahead distance (m)."""

    max_lookahead: float = 5.0
    """최대 lookahead distance (m)."""

    goal_tolerance: float = 0.5
    """목표 도달 판정 거리 (m)."""

    max_linear_speed: float = 1.0
    """최대 직선 속도 (m/s)."""

    track_width: float = 1.4
    """궤도차량 트랙 간격 (m)."""


class PurePursuitTracker:
    """Pure Pursuit 기반 경로 추종 제어기.

    주어진 경로(waypoints)를 추종하기 위해 lookahead point를 계산하고,
    해당 점을 향한 곡률로부터 선속도/각속도 명령을 생성한다.

    Attributes:
        config: Pure Pursuit 파라미터 설정.
        path: 추종할 경로 웨이포인트 리스트.
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

    def set_path(self, waypoints) -> None:
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
                (p.pose.position.x, p.pose.position.y)
                for p in waypoints.poses
            ]
        else:
            self.path = [(float(wp[0]), float(wp[1])) for wp in waypoints]

    def compute(
        self, current_pose: Pose2D, current_speed: float
    ) -> Tuple[float, float]:
        """현재 위치와 속도를 기반으로 제어 명령을 계산한다.

        Args:
            current_pose: 현재 차량 위치 및 방향.
            current_speed: 현재 차량 속도 (m/s).

        Returns:
            (linear_vel, angular_vel) 튜플.
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
            # lookahead 점을 찾지 못하면 마지막 경로점 사용
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
    #  내부 메서드
    # ------------------------------------------------------------------ #

    def _compute_lookahead_distance(self, speed: float) -> float:
        """속도 비례 lookahead distance를 계산한다.

        Args:
            speed: 현재 속도 (m/s).

        Returns:
            클램핑된 lookahead distance (m).
        """
        ld = self.config.lookahead_gain * abs(speed)
        return max(self.config.min_lookahead,
                   min(ld, self.config.max_lookahead))

    def _update_nearest(self, pose: Pose2D) -> None:
        """현재 위치에서 가장 가까운 경로점 인덱스를 갱신한다.

        탐색은 이전 nearest_idx부터 전방으로만 진행하여 뒤로 돌아가지 않는다.

        Args:
            pose: 현재 차량 위치.
        """
        min_dist_sq = float('inf')
        best_idx = self.nearest_idx

        # 현재 인덱스부터 전방 탐색 (전체 경로 길이까지)
        search_end = min(self.nearest_idx + 50, len(self.path))
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
        dx = goal[0] - pose.x
        dy = goal[1] - pose.y
        dist = math.hypot(dx, dy)
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
                # 선형 보간 비율
                ratio = (ld - d1) / (d2 - d1) if (d2 - d1) > 1e-9 else 0.0
                x = p1[0] + ratio * (p2[0] - p1[0])
                y = p1[1] + ratio * (p2[1] - p1[1])
                return (x, y)

            # 원-직선 교차 방식 (보다 정확한 보간)
            pt = self._circle_line_intersection(pose, ld, p1, p2)
            if pt is not None:
                return pt

        # 경로 끝에 도달 — 마지막 점 반환
        return None

    @staticmethod
    def _circle_line_intersection(
        pose: Pose2D, radius: float,
        p1: Tuple[float, float], p2: Tuple[float, float]
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

        if a < 1e-12 or discriminant < 0.0:
            return None

        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)

        # t가 [0, 1] 범위인 교점 중 전방(t가 큰) 것을 선택
        t = None
        if 0.0 <= t2 <= 1.0:
            t = t2
        if 0.0 <= t1 <= 1.0:
            # t1도 유효하면, p2에 가까운(더 큰) 쪽 선택
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
            ld: lookahead distance.

        Returns:
            곡률 (1/m). 양수=좌회전, 음수=우회전.
        """
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        actual_dist = math.hypot(dx, dy)

        if actual_dist < 1e-6:
            return 0.0

        # 차량 좌표계에서의 각도
        target_angle = math.atan2(dy, dx)
        alpha = _normalize_angle(target_angle - pose.yaw)

        # Pure Pursuit 곡률 공식
        curvature = 2.0 * math.sin(alpha) / max(actual_dist, 0.1)
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
        # 곡률에 따른 감속: 곡률 1.0 이상이면 최대 속도의 30%까지 감속
        curvature_factor = 1.0 / (1.0 + 2.0 * abs(curvature))
        return max_speed * max(0.3, curvature_factor)

    def _twist_to_tracks(
        self, linear: float, angular: float
    ) -> Tuple[float, float]:
        """(linear, angular) 속도를 좌/우 트랙 속도로 변환한다.

        차동 구동 모델: v_left  = v - omega * W / 2
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
        if max_abs > max_speed:
            scale = max_speed / max_abs
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


# ====================================================================== #
#  Phase 2: 슬립 보상 적응형 Pure Pursuit
# ====================================================================== #


class TerrainType(Enum):
    """슬립 프로파일에서 사용하는 지형 유형.

    terrain_classifier.py의 TerrainType과 동일한 이름을 사용하되,
    이 모듈에서는 슬립 파라미터 조회 전용으로 사용한다.
    외부에서 terrain_classifier.TerrainType 값을 문자열로 변환하여
    매핑할 수 있다.
    """

    PAVED = "paved"
    DIRT_ROAD = "dirt_road"
    GRASS = "grass"
    GRAVEL = "gravel"
    CROP_FIELD = "crop_field"
    MUD = "mud"


@dataclass
class TerrainSlipProfile:
    """지형별 슬립 특성 프로파일.

    각 지형 유형에 대한 기본 종방향/횡방향 슬립 비율과
    슬립 변동성(불확실성) 파라미터를 정의한다.

    Attributes:
        longitudinal_slip: 종방향(전후) 기본 슬립 비율 (0.0 ~ 1.0).
            0.0이면 슬립 없음, 1.0이면 완전 슬립(공전).
        lateral_slip: 횡방향(좌우) 기본 슬립 비율 (0.0 ~ 1.0).
            스키드 스티어 차량에서 회전 시 측면 미끄러짐 정도.
        slip_variability: 슬립 변동성 계수 (0.0 ~ 1.0).
            값이 클수록 슬립이 예측 불가능하여 더 보수적인 제어 필요.
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

    terrain_classifier.TerrainType.name 등 외부 문자열을
    이 모듈의 TerrainType으로 변환할 때 사용한다.

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

    추정 방식:
        1. 매 업데이트마다 오도메트리 예측 변위와 실제 변위를 비교
        2. 종방향 슬립 = 1 - (실제 전방 변위 / 명령 전방 변위)
        3. 횡방향 슬립 = |실제 측면 변위| / |실제 전방 변위|
        4. EMA(지수 이동 평균)로 스무딩하여 노이즈 억제

    Attributes:
        longitudinal_slip: 현재 추정 종방향 슬립 비율 (0.0 ~ 1.0).
        lateral_slip: 현재 추정 횡방향 슬립 비율 (0.0 ~ 1.0).
        slip_confidence: 슬립 추정 신뢰도 (0.0 ~ 1.0). 데이터가 축적될수록 증가.
    """

    def __init__(
        self,
        ema_alpha: float = 0.15,
        min_displacement: float = 0.01,
        confidence_rate: float = 0.05,
    ) -> None:
        """초기화.

        Args:
            ema_alpha: EMA 스무딩 계수 (0 < alpha <= 1).
                값이 클수록 최신 측정에 가중치 부여. 기본값 0.15.
            min_displacement: 슬립 계산을 수행하기 위한 최소 변위 (m).
                너무 작은 변위에서는 노이즈가 지배적이므로 무시한다.
            confidence_rate: 업데이트마다 신뢰도가 증가하는 비율.
        """
        self._ema_alpha: float = max(0.01, min(1.0, ema_alpha))
        self._min_displacement: float = min_displacement
        self._confidence_rate: float = confidence_rate

        # 추정 결과
        self.longitudinal_slip: float = 0.0
        self.lateral_slip: float = 0.0
        self.slip_confidence: float = 0.0

        # 이전 상태 저장
        self._prev_odom_pose: Optional[Pose2D] = None
        self._prev_actual_pose: Optional[Pose2D] = None
        self._prev_time: Optional[float] = None

        # 지형 기반 사전 정보 (prior)
        self._terrain_prior: Optional[TerrainSlipProfile] = None

    @property
    def total_slip_magnitude(self) -> float:
        """종/횡방향 슬립의 결합 크기를 반환한다.

        Returns:
            유클리드 노름 기반의 전체 슬립 크기 (0.0 ~ ~1.41).
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

        실측 데이터가 부족할 때 지형 기반 기본값으로 초기 추정을
        보조한다. 실측 데이터가 축적되면 점차 사전 정보의 영향이 감소한다.

        Args:
            terrain: 현재 지형 유형.
        """
        self._terrain_prior = get_terrain_slip_profile(terrain)

        # 신뢰도가 낮을 때 지형 사전 정보를 초기값으로 사용
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

        매 사이클마다 호출하여 슬립 추정을 갱신한다.

        Args:
            odom_pose: 오도메트리(명령 적분) 기반 예측 위치.
            actual_pose: 로컬라이제이션(GPS/SLAM 등) 기반 실측 위치.
            current_time: 현재 시각 (초). None이면 time.monotonic() 사용.

        Returns:
            (longitudinal_slip, lateral_slip) 현재 추정 슬립 비율.
        """
        if current_time is None:
            current_time = time.monotonic()

        # 첫 호출: 초기 상태만 저장하고 반환
        if self._prev_odom_pose is None or self._prev_actual_pose is None:
            self._prev_odom_pose = Pose2D(
                x=odom_pose.x, y=odom_pose.y, yaw=odom_pose.yaw)
            self._prev_actual_pose = Pose2D(
                x=actual_pose.x, y=actual_pose.y, yaw=actual_pose.yaw)
            self._prev_time = current_time
            return self.longitudinal_slip, self.lateral_slip

        # 오도메트리 변위 (명령 기반 예측)
        odom_dx = odom_pose.x - self._prev_odom_pose.x
        odom_dy = odom_pose.y - self._prev_odom_pose.y
        odom_disp = math.hypot(odom_dx, odom_dy)

        # 실측 변위
        actual_dx = actual_pose.x - self._prev_actual_pose.x
        actual_dy = actual_pose.y - self._prev_actual_pose.y
        actual_disp = math.hypot(actual_dx, actual_dy)

        # 변위가 너무 작으면 노이즈 지배적이므로 스킵
        if odom_disp < self._min_displacement:
            self._prev_odom_pose = Pose2D(
                x=odom_pose.x, y=odom_pose.y, yaw=odom_pose.yaw)
            self._prev_actual_pose = Pose2D(
                x=actual_pose.x, y=actual_pose.y, yaw=actual_pose.yaw)
            self._prev_time = current_time
            return self.longitudinal_slip, self.lateral_slip

        # 차량 좌표계에서의 분해
        # 이전 실측 yaw를 기준으로 전방/측방 성분 분리
        ref_yaw = self._prev_actual_pose.yaw
        cos_yaw = math.cos(ref_yaw)
        sin_yaw = math.sin(ref_yaw)

        # 오도메트리 변위를 차량 좌표계로 투영
        odom_forward = odom_dx * cos_yaw + odom_dy * sin_yaw
        odom_lateral = -odom_dx * sin_yaw + odom_dy * cos_yaw

        # 실측 변위를 차량 좌표계로 투영
        actual_forward = actual_dx * cos_yaw + actual_dy * sin_yaw
        actual_lateral = -actual_dx * sin_yaw + actual_dy * cos_yaw

        # 종방향 슬립 계산: 1 - (실제 전방 / 명령 전방)
        if abs(odom_forward) > self._min_displacement:
            raw_long_slip = 1.0 - (actual_forward / odom_forward)
            raw_long_slip = max(0.0, min(1.0, raw_long_slip))
        else:
            raw_long_slip = self.longitudinal_slip  # 유지

        # 횡방향 슬립 계산: |실측 측방 편차| / max(|실제 전방|, min)
        denom = max(abs(actual_forward), self._min_displacement)
        raw_lat_slip = abs(actual_lateral - odom_lateral) / denom
        raw_lat_slip = max(0.0, min(1.0, raw_lat_slip))

        # EMA 스무딩
        alpha = self._ema_alpha
        self.longitudinal_slip = (
            alpha * raw_long_slip
            + (1.0 - alpha) * self.longitudinal_slip
        )
        self.lateral_slip = (
            alpha * raw_lat_slip
            + (1.0 - alpha) * self.lateral_slip
        )

        # 신뢰도 증가 (최대 1.0)
        self.slip_confidence = min(
            1.0, self.slip_confidence + self._confidence_rate)

        # 상태 갱신
        self._prev_odom_pose = Pose2D(
            x=odom_pose.x, y=odom_pose.y, yaw=odom_pose.yaw)
        self._prev_actual_pose = Pose2D(
            x=actual_pose.x, y=actual_pose.y, yaw=actual_pose.yaw)
        self._prev_time = current_time

        return self.longitudinal_slip, self.lateral_slip


@dataclass
class SlipCompensatedPurePursuitConfig(PurePursuitConfig):
    """슬립 보상 Pure Pursuit 파라미터 설정.

    PurePursuitConfig를 상속하며 슬립 보상 관련 추가 파라미터를 포함한다.
    """

    slip_lookahead_gain: float = 2.0
    """슬립에 의한 lookahead 증가 계수. lookahead += gain * total_slip * base_ld."""

    max_slip_lookahead_ratio: float = 2.0
    """슬립으로 인한 lookahead 최대 증가 비율. 원래 ld의 N배까지 허용."""

    longitudinal_compensation_gain: float = 1.0
    """종방향 슬립 보상 게인. 1.0이면 추정 슬립 전량 보상."""

    lateral_compensation_gain: float = 0.8
    """횡방향 슬립 보상 게인. 1.0이면 추정 슬립 전량 보상."""

    slip_speed_reduction_gain: float = 0.5
    """슬립이 클 때 속도 감소 게인. 속도 *= 1 - gain * total_slip."""

    min_speed_ratio: float = 0.2
    """슬립에 의한 최소 속도 비율. 최대 속도의 이 비율 이하로는 감속하지 않음."""

    ema_alpha: float = 0.15
    """슬립 추정기 EMA 스무딩 계수."""

    enable_terrain_adaptation: bool = True
    """지형 적응 활성화 여부. True이면 지형 프로파일 사전 정보를 사용."""


class SlipCompensatedPurePursuit(PurePursuitTracker):
    """슬립 보상 적응형 Pure Pursuit 경로 추종 제어기.

    PurePursuitTracker를 상속하며 다음 기능을 추가한다:

    1. 슬립 추정: SlipEstimator를 내장하여 실시간 종/횡방향 슬립 비율 추정
    2. 적응형 lookahead: 슬립이 클수록 lookahead 거리를 자동 증가
    3. 슬립 보상 조향: 추정된 슬립을 반영하여 트랙 속도 명령 보정
    4. 지형별 적응: 지형 종류에 따른 사전 슬립 파라미터 활용

    기존 PurePursuitTracker와 완전히 호환되는 인터페이스를 유지한다.
    compute(), compute_track_velocities(), set_path() 등의 기본 API를
    그대로 사용할 수 있으며, 슬립 보상은 내부적으로 투명하게 적용된다.

    사용 예시::

        config = SlipCompensatedPurePursuitConfig(
            lookahead_gain=0.8,
            min_lookahead=1.0,
            max_lookahead=5.0,
            slip_lookahead_gain=2.0,
        )
        tracker = SlipCompensatedPurePursuit(config=config)
        tracker.set_path(waypoints)

        # 매 사이클
        tracker.update_slip(odom_pose, actual_pose)
        tracker.set_terrain(TerrainType.MUD)
        linear, angular = tracker.compute(current_pose, speed)
        v_left, v_right = tracker.compute_track_velocities(current_pose, speed)

    Attributes:
        slip_estimator: 내장 슬립 추정기.
        current_terrain: 현재 설정된 지형 유형.
        slip_config: 슬립 보상 전용 파라미터 (SlipCompensatedPurePursuitConfig).
    """

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

        # 슬립 보상 전용 설정에 대한 참조 (타입 명시)
        self.slip_config: SlipCompensatedPurePursuitConfig = config

        # 슬립 추정기
        self.slip_estimator: SlipEstimator = SlipEstimator(
            ema_alpha=config.ema_alpha,
        )

        # 현재 지형
        self.current_terrain: Optional[TerrainType] = None
        self._current_terrain_profile: TerrainSlipProfile = DEFAULT_SLIP_PROFILE

    # ------------------------------------------------------------------ #
    #  공개 API (기존 인터페이스 확장)
    # ------------------------------------------------------------------ #

    def set_path(self, waypoints) -> None:
        """경로를 설정하고 슬립 추정기를 리셋한다.

        새 경로 시작 시 이전 슬립 데이터를 초기화하여
        새로운 환경에서 처음부터 추정을 시작한다.

        Args:
            waypoints: 경로 데이터. PurePursuitTracker.set_path() 참조.
        """
        super().set_path(waypoints)
        self.slip_estimator.reset()

        # 지형이 설정되어 있으면 사전 정보로 초기화
        if self.current_terrain is not None:
            self.slip_estimator.set_terrain_prior(self.current_terrain)

    def set_terrain(self, terrain: TerrainType) -> None:
        """현재 주행 지형을 설정한다.

        지형 변경 시 슬립 추정기에 사전 정보를 제공하고,
        지형 프로파일을 갱신한다.

        Args:
            terrain: 현재 지형 유형.
        """
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
        """오도메트리와 실측 위치를 제공하여 슬립을 갱신한다.

        compute() 호출 전에 매 사이클 호출해야 한다.

        Args:
            odom_pose: 오도메트리 기반 예측 위치.
            actual_pose: 실측(GPS/SLAM) 위치.
            current_time: 현재 시각 (초). None이면 자동.

        Returns:
            (longitudinal_slip, lateral_slip) 현재 추정값.
        """
        return self.slip_estimator.update(
            odom_pose, actual_pose, current_time)

    def get_slip_info(self) -> dict:
        """현재 슬립 관련 진단 정보를 반환한다.

        로깅/모니터링에 사용할 수 있는 딕셔너리를 반환한다.

        Returns:
            슬립 추정 관련 정보 딕셔너리.
        """
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
            "terrain_base_long_slip":
                self._current_terrain_profile.longitudinal_slip,
            "terrain_base_lat_slip":
                self._current_terrain_profile.lateral_slip,
        }

    # ------------------------------------------------------------------ #
    #  오버라이드: 슬립 보상 적용
    # ------------------------------------------------------------------ #

    def _compute_lookahead_distance(self, speed: float) -> float:
        """슬립 적응형 lookahead distance를 계산한다.

        기본 속도 비례 lookahead에 슬립 크기에 비례하는 추가분을 더한다.
        슬립이 클수록 lookahead을 늘려 경로 추종 안정성을 확보한다.

        공식:
            base_ld = lookahead_gain * |speed|
            slip_addition = slip_lookahead_gain * total_slip * base_ld
            ld = base_ld + slip_addition
            ld = clamp(ld, min_lookahead, max_lookahead * max_slip_lookahead_ratio)

        Args:
            speed: 현재 속도 (m/s).

        Returns:
            슬립 보상된 lookahead distance (m).
        """
        # 기본 lookahead (속도 비례)
        base_ld = self.config.lookahead_gain * abs(speed)
        base_ld = max(self.config.min_lookahead, base_ld)

        # 슬립에 의한 추가분
        total_slip = self.slip_estimator.total_slip_magnitude
        slip_addition = (
            self.slip_config.slip_lookahead_gain * total_slip * base_ld
        )

        ld = base_ld + slip_addition

        # 상한을 슬립에 따라 확장 (최대 max_slip_lookahead_ratio배)
        effective_max = (
            self.config.max_lookahead
            * self.slip_config.max_slip_lookahead_ratio
        )
        ld = max(self.config.min_lookahead, min(ld, effective_max))

        return ld

    def _compute_speed(self, curvature: float) -> float:
        """슬립을 반영하여 안전한 속도를 계산한다.

        기본 곡률 기반 감속에 추가로 슬립 크기에 따른 감속을 적용한다.
        지형의 최대 안전 속도도 고려한다.

        Args:
            curvature: 현재 곡률 (1/m).

        Returns:
            슬립 보상된 목표 선속도 (m/s).
        """
        # 기본 속도 (부모 클래스 곡률 기반)
        base_speed = super()._compute_speed(curvature)

        # 슬립에 의한 감속
        total_slip = self.slip_estimator.total_slip_magnitude
        slip_factor = 1.0 - (
            self.slip_config.slip_speed_reduction_gain * total_slip
        )
        slip_factor = max(self.slip_config.min_speed_ratio, slip_factor)

        speed = base_speed * slip_factor

        # 지형 최대 안전 속도 제한
        speed = min(speed, self._current_terrain_profile.max_safe_speed)

        return speed

    def _twist_to_tracks(
        self, linear: float, angular: float
    ) -> Tuple[float, float]:
        """슬립 보상된 좌/우 트랙 속도를 계산한다.

        추정된 슬립을 보상하기 위해 트랙 속도를 사전 보정(pre-compensate)한다.
        종방향 슬립이 있으면 명령 속도를 증가시켜 실제 속도가 목표에 도달하도록 하고,
        횡방향 슬립이 있으면 회전 방향의 트랙 속도를 추가 보정한다.

        Args:
            linear: 선속도 (m/s).
            angular: 각속도 (rad/s).

        Returns:
            (v_left, v_right) 슬립 보상된 트랙 속도 (m/s).
        """
        # 종방향 슬립 보상: 슬립으로 손실되는 만큼 속도를 올림
        long_slip = self.slip_estimator.longitudinal_slip
        long_gain = self.slip_config.longitudinal_compensation_gain
        # 보상 계수: 1 / (1 - slip) 근사. slip이 1.0에 가까우면 상한 제한.
        long_comp = 1.0 + long_gain * long_slip
        long_comp = min(long_comp, 1.5)  # 최대 50% 보상

        compensated_linear = linear * long_comp

        # 횡방향 슬립 보상: 회전 시 측면 미끄러짐을 보정
        lat_slip = self.slip_estimator.lateral_slip
        lat_gain = self.slip_config.lateral_compensation_gain
        # 횡방향 슬립이 크면 각속도를 보강하여 실제 회전이 목표에 도달하도록 함
        lat_comp = 1.0 + lat_gain * lat_slip
        lat_comp = min(lat_comp, 1.5)  # 최대 50% 보상

        compensated_angular = angular * lat_comp

        # 기본 차동 구동 변환
        half_width = self.config.track_width / 2.0
        v_left = compensated_linear - compensated_angular * half_width
        v_right = compensated_linear + compensated_angular * half_width

        # 최대 속도 포화 처리 (비율 유지)
        max_abs = max(abs(v_left), abs(v_right))
        max_speed = self.config.max_linear_speed
        if max_abs > max_speed:
            scale = max_speed / max_abs
            v_left *= scale
            v_right *= scale

        return v_left, v_right
