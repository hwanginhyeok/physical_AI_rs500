"""Pure Pursuit 경로 추종 알고리즘 모듈.

속도 비례 lookahead distance를 사용하여 목표 경로를 추종하는
Pure Pursuit 제어기를 구현한다. 궤도차량(스키드 스티어) 특성을 고려하여
cmd_vel을 좌/우 트랙 속도로 변환하는 기능을 포함한다.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


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
