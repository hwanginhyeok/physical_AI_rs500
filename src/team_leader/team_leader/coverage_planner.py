"""Fields2Cover 스타일 농경지 커버리지 경로 계획 모듈.

Fields2Cover 라이브러리의 4단계 파이프라인(headland -> swath -> route -> path)을
순수 Python으로 구현하여, 외부 C++ 의존성 없이 ROS2 환경에서 바로 사용 가능하다.

지원 패턴:
    - Boustrophedon (왕복 직선): 기본 패턴, 직선 위주로 궤도차량에 최적
    - Spiral (나선형): 경계부터 중심으로 수렴, 회전 최소화
    - Racetrack (경주로): 왕복 + 양쪽 끝 U턴 곡선 연결

참고:
    - Fields2Cover Paper: Mier et al., IEEE RA-L, 2023
    - Boustrophedon: Choset, "Coverage of Known Spaces", 2000
    - 선행연구: docs/research/path_planning_and_slam_literature_review.md
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional


# ---------------------------------------------------------------------------
# 데이터 구조
# ---------------------------------------------------------------------------

@dataclass
class FieldBoundary:
    """필드(농경지) 경계 다각형.

    Attributes:
        vertices: 경계 꼭짓점 좌표 리스트 [(x, y), ...].
            반시계 방향(CCW) 정렬을 권장한다. 최소 3개 이상.
    """
    vertices: List[Tuple[float, float]]

    def __post_init__(self):
        if len(self.vertices) < 3:
            raise ValueError(
                f'FieldBoundary는 최소 3개의 꼭짓점이 필요합니다. '
                f'(입력: {len(self.vertices)}개)'
            )


class CoveragePattern(Enum):
    """커버리지 주행 패턴."""
    BOUSTROPHEDON = auto()  # 왕복 직선 (소가 밭을 가는 방식)
    SPIRAL = auto()         # 나선형 (경계 -> 중심)
    RACETRACK = auto()      # 경주로 (왕복 + U턴 곡선)


@dataclass
class CoverageConfig:
    """커버리지 경로 생성 설정.

    Attributes:
        swath_width: 작업 폭 (m). 인접 스와스 간 간격.
        overlap_ratio: 오버랩 비율 (0.0 ~ 0.5). 인접 스와스 간 중첩 비율.
            예: 0.1이면 작업폭의 10%가 겹침.
        headland_width: 헤드랜드 폭 (m). 필드 경계에서 내부로 들어온 영역.
            0이면 헤드랜드 없이 직접 스와스 생성.
        swath_angle_deg: 스와스 각도 (도). None이면 자동 최적화.
        pattern: 커버리지 패턴 (기본: BOUSTROPHEDON).
    """
    swath_width: float = 2.0
    overlap_ratio: float = 0.0
    headland_width: float = 0.0
    swath_angle_deg: Optional[float] = None
    pattern: CoveragePattern = CoveragePattern.BOUSTROPHEDON


# ---------------------------------------------------------------------------
# 기하 유틸리티
# ---------------------------------------------------------------------------

def _polygon_area_signed(vertices: List[Tuple[float, float]]) -> float:
    """다각형의 부호 있는 넓이를 계산 (Shoelace 공식).

    양수면 반시계 방향(CCW), 음수면 시계 방향(CW).
    """
    n = len(vertices)
    area = 0.0
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return area / 2.0


def _ensure_ccw(vertices: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """다각형 꼭짓점을 반시계 방향(CCW)으로 정렬."""
    if _polygon_area_signed(vertices) < 0:
        return list(reversed(vertices))
    return list(vertices)


def _polygon_centroid(vertices: List[Tuple[float, float]]) -> Tuple[float, float]:
    """다각형 무게중심 계산."""
    n = len(vertices)
    cx, cy = 0.0, 0.0
    signed_area = _polygon_area_signed(vertices)
    if abs(signed_area) < 1e-12:
        # 퇴화 다각형 -> 단순 평균
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        return (sum(xs) / n, sum(ys) / n)

    for i in range(n):
        x0, y0 = vertices[i]
        x1, y1 = vertices[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    factor = 1.0 / (6.0 * signed_area)
    return (cx * factor, cy * factor)


def _offset_polygon(
    vertices: List[Tuple[float, float]],
    offset: float,
) -> List[Tuple[float, float]]:
    """다각형을 안쪽으로 오프셋 (버퍼 축소).

    단순 법선 기반 오프셋으로, 볼록 다각형에서 정확하다.
    비볼록 다각형에서도 합리적인 근사를 제공한다.

    Args:
        vertices: CCW 정렬된 다각형 꼭짓점
        offset: 오프셋 거리 (양수 = 안쪽)

    Returns:
        오프셋된 다각형 꼭짓점 리스트
    """
    n = len(vertices)
    if n < 3 or offset <= 0.0:
        return list(vertices)

    result = []
    for i in range(n):
        # 이전, 현재, 다음 꼭짓점
        p_prev = vertices[(i - 1) % n]
        p_curr = vertices[i]
        p_next = vertices[(i + 1) % n]

        # 이전 변의 법선 (안쪽 방향)
        dx1 = p_curr[0] - p_prev[0]
        dy1 = p_curr[1] - p_prev[1]
        len1 = math.hypot(dx1, dy1)
        if len1 < 1e-12:
            continue
        nx1 = -dy1 / len1  # CCW에서 안쪽 법선
        ny1 = dx1 / len1

        # 다음 변의 법선 (안쪽 방향)
        dx2 = p_next[0] - p_curr[0]
        dy2 = p_next[1] - p_curr[1]
        len2 = math.hypot(dx2, dy2)
        if len2 < 1e-12:
            continue
        nx2 = -dy2 / len2
        ny2 = dx2 / len2

        # 이등분선 방향
        bx = nx1 + nx2
        by = ny1 + ny2
        b_len = math.hypot(bx, by)
        if b_len < 1e-12:
            # 평행한 변 -> 법선 방향으로 단순 오프셋
            result.append((p_curr[0] + nx1 * offset, p_curr[1] + ny1 * offset))
            continue

        bx /= b_len
        by /= b_len

        # 이등분선의 반각(half-angle) 사인 값
        sin_half = nx1 * by - ny1 * bx  # cross product
        cos_half = nx1 * bx + ny1 * by  # dot product

        if abs(sin_half) < 1e-12:
            # 거의 평행 -> 법선 방향 단순 오프셋
            d = offset
        else:
            # miter 오프셋
            d = offset / abs(cos_half) if abs(cos_half) > 1e-6 else offset

        # 최대 miter 거리 제한 (offset의 3배)
        d = min(d, offset * 3.0)

        result.append((p_curr[0] + bx * d, p_curr[1] + by * d))

    return result


def _rotate_point(
    x: float, y: float,
    cx: float, cy: float,
    angle_rad: float,
) -> Tuple[float, float]:
    """점 (x, y)를 중심 (cx, cy) 기준으로 angle_rad 만큼 회전."""
    dx = x - cx
    dy = y - cy
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (cx + dx * cos_a - dy * sin_a, cy + dx * sin_a + dy * cos_a)


def _line_segment_intersect_y(
    polygon: List[Tuple[float, float]],
    y: float,
) -> List[float]:
    """다각형의 변과 수평선(y=const)의 교차 X좌표 리스트.

    레이캐스팅 방식으로 교차점을 찾는다.
    """
    intersections: List[float] = []
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            if abs(y2 - y1) > 1e-10:
                t = (y - y1) / (y2 - y1)
                intersections.append(x1 + t * (x2 - x1))
    return sorted(intersections)


def _point_distance(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
) -> float:
    """두 점 사이의 유클리드 거리."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def _path_total_distance(
    waypoints: List[Tuple[float, float]],
) -> float:
    """웨이포인트 경로의 총 거리."""
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += _point_distance(waypoints[i], waypoints[i + 1])
    return total


# ---------------------------------------------------------------------------
# CoveragePlanner
# ---------------------------------------------------------------------------

class CoveragePlanner:
    """Fields2Cover 스타일 농경지 커버리지 경로 계획기.

    4단계 파이프라인:
        1. Headland 생성 - 필드 경계를 따르는 헤드랜드 경로
        2. Swath 생성 - 내부 영역의 작업 라인 생성
        3. Route 계획 - 스와스 순서 최적화
        4. Path 조합 - headland + 내부 스와스 경로 통합

    사용 예시::

        planner = CoveragePlanner()
        boundary = FieldBoundary(vertices=[(0,0), (100,0), (100,50), (0,50)])
        config = CoverageConfig(swath_width=3.0, headland_width=6.0)
        waypoints = planner.plan_coverage(boundary, config)
    """

    def __init__(self):
        self._field: Optional[FieldBoundary] = None
        self._field_ccw: List[Tuple[float, float]] = []

    # ----- 필드 설정 -----

    def set_field(self, boundary: FieldBoundary) -> None:
        """필드 경계를 설정한다.

        Args:
            boundary: 필드 경계 다각형
        """
        self._field = boundary
        self._field_ccw = _ensure_ccw(boundary.vertices)

    # ----- 스와스 생성 -----

    def generate_swaths(
        self,
        angle_deg: float = 0.0,
        swath_width: float = 2.0,
        overlap_ratio: float = 0.0,
        work_area: Optional[List[Tuple[float, float]]] = None,
    ) -> List[List[Tuple[float, float]]]:
        """지정 각도로 스와스(작업 라인)를 생성한다.

        영역을 angle_deg 방향으로 회전시킨 후 수평 스캔라인을 적용하여
        스와스를 생성하고, 다시 역회전하여 원래 좌표계로 복원한다.

        Args:
            angle_deg: 스와스 방향 각도 (도). 0=Y축 방향 스캔.
            swath_width: 작업 폭 (m)
            overlap_ratio: 오버랩 비율 (0.0 ~ 0.5)
            work_area: 작업 영역 다각형. None이면 set_field()의 영역 사용.

        Returns:
            스와스 리스트. 각 스와스는 [(x_start, y_start), (x_end, y_end)].
        """
        if work_area is None:
            if self._field is None:
                raise RuntimeError('set_field()로 필드를 먼저 설정하세요.')
            work_area = list(self._field_ccw)

        if len(work_area) < 3:
            return []

        # 유효 스와스 간격 (오버랩 적용)
        effective_width = swath_width * (1.0 - overlap_ratio)
        if effective_width <= 0.0:
            raise ValueError(
                f'유효 스와스 간격이 0 이하입니다. '
                f'swath_width={swath_width}, overlap_ratio={overlap_ratio}'
            )

        # 회전 중심: 작업 영역 무게중심
        cx, cy = _polygon_centroid(work_area)
        angle_rad = math.radians(angle_deg)

        # 작업 영역을 -angle_rad 만큼 회전 (수평 스캔 적용을 위해)
        rotated = [
            _rotate_point(x, y, cx, cy, -angle_rad)
            for x, y in work_area
        ]

        # 회전된 영역의 Y 바운딩 박스
        ys = [p[1] for p in rotated]
        y_min, y_max = min(ys), max(ys)

        # 수평 스캔라인으로 스와스 생성
        swaths_rotated: List[List[Tuple[float, float]]] = []
        y = y_min + effective_width / 2.0

        while y <= y_max - effective_width / 2.0 + 1e-6:
            x_intersections = _line_segment_intersect_y(rotated, y)
            # 교차점을 쌍으로 묶어 스와스 생성
            for j in range(0, len(x_intersections) - 1, 2):
                swaths_rotated.append([
                    (x_intersections[j], y),
                    (x_intersections[j + 1], y),
                ])
            y += effective_width

        # 역회전하여 원래 좌표계로 복원
        swaths: List[List[Tuple[float, float]]] = []
        for swath in swaths_rotated:
            restored = [
                _rotate_point(x, y, cx, cy, angle_rad)
                for x, y in swath
            ]
            swaths.append(restored)

        return swaths

    # ----- 최적 각도 계산 -----

    def optimize_angle(
        self,
        swath_width: float = 2.0,
        overlap_ratio: float = 0.0,
        work_area: Optional[List[Tuple[float, float]]] = None,
        angle_step_deg: float = 5.0,
    ) -> float:
        """최소 스와스 횟수(= 최소 회전 횟수)를 달성하는 최적 각도를 계산한다.

        0도부터 180도까지 angle_step_deg 간격으로 탐색하여
        스와스 횟수가 최소가 되는 각도를 반환한다.

        Args:
            swath_width: 작업 폭 (m)
            overlap_ratio: 오버랩 비율
            work_area: 작업 영역 (None이면 필드 영역 사용)
            angle_step_deg: 탐색 각도 간격 (도)

        Returns:
            최적 스와스 각도 (도)
        """
        best_angle = 0.0
        min_swath_count = float('inf')

        angle = 0.0
        while angle < 180.0:
            try:
                swaths = self.generate_swaths(
                    angle_deg=angle,
                    swath_width=swath_width,
                    overlap_ratio=overlap_ratio,
                    work_area=work_area,
                )
                count = len(swaths)
                if 0 < count < min_swath_count:
                    min_swath_count = count
                    best_angle = angle
            except (RuntimeError, ValueError):
                pass
            angle += angle_step_deg

        return best_angle

    # ----- 헤드랜드 경로 -----

    def plan_headland_path(
        self,
        boundary: FieldBoundary,
        width: float,
        num_passes: int = 0,
    ) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
        """헤드랜드(경계 따라) 경로를 생성한다.

        필드 경계로부터 폭(width)만큼 안쪽에 헤드랜드 경로를 만들고,
        헤드랜드 내부의 남은 작업 영역 경계도 함께 반환한다.

        Args:
            boundary: 필드 경계 다각형
            width: 헤드랜드 폭 (m). 0이면 빈 경로 반환.
            num_passes: 헤드랜드 주회 횟수. 0이면 자동 계산
                (width / swath_width 기반, 기본 1회).

        Returns:
            (headland_waypoints, inner_boundary) 튜플.
                headland_waypoints: 헤드랜드 경로 웨이포인트 리스트
                inner_boundary: 헤드랜드 제거 후 내부 경계 꼭짓점 리스트
        """
        vertices_ccw = _ensure_ccw(boundary.vertices)

        if width <= 0.0:
            return ([], list(vertices_ccw))

        # 헤드랜드 주회 횟수 결정
        if num_passes <= 0:
            num_passes = max(1, int(math.ceil(width / max(width, 1.0))))

        pass_width = width / num_passes

        headland_waypoints: List[Tuple[float, float]] = []
        current_boundary = list(vertices_ccw)

        for p in range(num_passes):
            # 현재 경계의 중간선 (pass_width / 2 오프셋)
            offset_mid = pass_width * (p + 0.5)
            mid_poly = _offset_polygon(vertices_ccw, offset_mid)

            if len(mid_poly) < 3:
                break

            # 경계를 따라 한 바퀴 + 시작점으로 복귀
            for pt in mid_poly:
                headland_waypoints.append(pt)
            # 루프 닫기
            headland_waypoints.append(mid_poly[0])

        # 내부 경계: 전체 헤드랜드 폭만큼 오프셋
        inner_boundary = _offset_polygon(vertices_ccw, width)
        if len(inner_boundary) < 3:
            inner_boundary = []

        return (headland_waypoints, inner_boundary)

    # ----- 스와스 순서 최적화 -----

    def reorder_swaths(
        self,
        swaths: List[List[Tuple[float, float]]],
    ) -> List[List[Tuple[float, float]]]:
        """스와스 순서를 최적화하여 이동 거리를 최소화한다.

        그리디(nearest-neighbor) 알고리즘으로 다음 스와스를 선택한다.
        각 스와스는 시작/끝점을 뒤집을 수 있으므로,
        현재 위치에서 가장 가까운 스와스 끝점으로 이동한다.

        Args:
            swaths: 스와스 리스트. 각 스와스는 [(x1,y1), (x2,y2)].

        Returns:
            재정렬된 스와스 리스트
        """
        if len(swaths) <= 1:
            return list(swaths)

        remaining = list(range(len(swaths)))
        ordered: List[List[Tuple[float, float]]] = []

        # 첫 번째 스와스는 원래 순서의 첫 번째
        current_idx = 0
        ordered.append(list(swaths[current_idx]))
        remaining.remove(current_idx)
        current_pos = swaths[current_idx][-1]  # 첫 스와스의 끝점

        while remaining:
            best_idx = -1
            best_dist = float('inf')
            best_reversed = False

            for idx in remaining:
                swath = swaths[idx]
                # 스와스 시작점까지 거리
                d_start = _point_distance(current_pos, swath[0])
                # 스와스 끝점까지 거리 (뒤집어서 접근)
                d_end = _point_distance(current_pos, swath[-1])

                if d_start < best_dist:
                    best_dist = d_start
                    best_idx = idx
                    best_reversed = False

                if d_end < best_dist:
                    best_dist = d_end
                    best_idx = idx
                    best_reversed = True

            if best_idx < 0:
                break

            swath_copy = list(swaths[best_idx])
            if best_reversed:
                swath_copy.reverse()

            ordered.append(swath_copy)
            current_pos = swath_copy[-1]
            remaining.remove(best_idx)

        return ordered

    # ----- 패턴별 경로 생성 -----

    def _generate_boustrophedon(
        self,
        work_area: List[Tuple[float, float]],
        config: CoverageConfig,
        angle_deg: float,
    ) -> List[Tuple[float, float]]:
        """Boustrophedon(왕복 직선) 패턴 경로 생성.

        스와스를 생성하고, 순서를 최적화한 후,
        인접 스와스를 왕복 방향으로 연결한다.
        """
        swaths = self.generate_swaths(
            angle_deg=angle_deg,
            swath_width=config.swath_width,
            overlap_ratio=config.overlap_ratio,
            work_area=work_area,
        )

        if not swaths:
            return []

        # 스와스 순서 최적화
        ordered = self.reorder_swaths(swaths)

        # 왕복 연결: 짝수 스와스는 정방향, 홀수는 역방향
        waypoints: List[Tuple[float, float]] = []
        for i, swath in enumerate(ordered):
            if i % 2 == 0:
                waypoints.extend(swath)
            else:
                waypoints.extend(reversed(swath))

        return waypoints

    def _generate_spiral(
        self,
        work_area: List[Tuple[float, float]],
        config: CoverageConfig,
    ) -> List[Tuple[float, float]]:
        """Spiral(나선형) 패턴 경로 생성.

        필드 경계부터 중심 방향으로 점점 안쪽의 오프셋 다각형을 따라
        나선 형태의 커버리지 경로를 생성한다.
        """
        if len(work_area) < 3:
            return []

        effective_width = config.swath_width * (1.0 - config.overlap_ratio)
        if effective_width <= 0.0:
            return []

        vertices_ccw = _ensure_ccw(work_area)
        waypoints: List[Tuple[float, float]] = []

        current_poly = list(vertices_ccw)
        offset = effective_width / 2.0  # 첫 패스: 경계로부터 반폭 안쪽

        while len(current_poly) >= 3:
            # 현재 경계의 오프셋 경로
            path_poly = _offset_polygon(vertices_ccw, offset)
            if len(path_poly) < 3:
                break

            # 경계를 따라 한 바퀴
            for pt in path_poly:
                waypoints.append(pt)

            offset += effective_width
            # 다음 루프를 위해 영역 축소 확인
            inner = _offset_polygon(vertices_ccw, offset + effective_width / 2.0)
            if len(inner) < 3:
                # 마지막 루프 - 남은 중심 영역 한 바퀴
                last_poly = _offset_polygon(vertices_ccw, offset)
                if len(last_poly) >= 3:
                    for pt in last_poly:
                        waypoints.append(pt)
                break

        return waypoints

    def _generate_racetrack(
        self,
        work_area: List[Tuple[float, float]],
        config: CoverageConfig,
        angle_deg: float,
    ) -> List[Tuple[float, float]]:
        """Racetrack(경주로) 패턴 경로 생성.

        Boustrophedon과 유사하지만, 각 스와스 전환 시
        U턴 곡선(반원)을 삽입하여 부드러운 경로를 생성한다.
        궤도차량은 제자리 회전이 가능하므로 U턴 반경은 작게 설정한다.
        """
        swaths = self.generate_swaths(
            angle_deg=angle_deg,
            swath_width=config.swath_width,
            overlap_ratio=config.overlap_ratio,
            work_area=work_area,
        )

        if not swaths:
            return []

        ordered = self.reorder_swaths(swaths)

        waypoints: List[Tuple[float, float]] = []
        effective_width = config.swath_width * (1.0 - config.overlap_ratio)
        turn_radius = effective_width / 2.0
        num_turn_points = 8  # U턴 보간 점 개수

        for i, swath in enumerate(ordered):
            if i % 2 == 0:
                waypoints.extend(swath)
            else:
                waypoints.extend(reversed(swath))

            # 다음 스와스가 있으면 U턴 곡선 삽입
            if i < len(ordered) - 1:
                p_end = waypoints[-1]
                next_swath = ordered[i + 1]

                # 다음 스와스 시작/끝 중 가까운 쪽 결정
                if (i + 1) % 2 == 0:
                    p_next = next_swath[0]
                else:
                    p_next = next_swath[-1]

                # U턴 중심점
                mid_x = (p_end[0] + p_next[0]) / 2.0
                mid_y = (p_end[1] + p_next[1]) / 2.0

                # 반원 보간 (시작 -> 끝)
                dx = p_next[0] - p_end[0]
                dy = p_next[1] - p_end[1]
                half_dist = math.hypot(dx, dy) / 2.0

                if half_dist > 1e-6:
                    # U턴 방향 결정
                    start_angle = math.atan2(
                        p_end[1] - mid_y, p_end[0] - mid_x
                    )
                    for k in range(1, num_turn_points):
                        t = k / num_turn_points
                        angle = start_angle + math.pi * t
                        ux = mid_x + half_dist * math.cos(angle)
                        uy = mid_y + half_dist * math.sin(angle)
                        waypoints.append((ux, uy))

        return waypoints

    # ----- 전체 커버리지 경로 -----

    def plan_coverage(
        self,
        field: FieldBoundary,
        config: CoverageConfig,
    ) -> List[Tuple[float, float]]:
        """전체 커버리지 경로를 생성한다.

        Fields2Cover 4단계 파이프라인:
            1. 헤드랜드 경로 생성 (경계 따라 주행)
            2. 내부 영역 스와스 생성
            3. 스와스 순서 최적화
            4. 경로 조합 (headland -> 내부 스와스)

        Args:
            field: 필드 경계 다각형
            config: 커버리지 설정

        Returns:
            전체 커버리지 웨이포인트 리스트 [(x, y), ...]
        """
        self.set_field(field)

        # 1단계: 헤드랜드 경로
        headland_wps: List[Tuple[float, float]] = []
        if config.headland_width > 0.0:
            headland_wps, inner_boundary = self.plan_headland_path(
                field, config.headland_width
            )
        else:
            inner_boundary = list(self._field_ccw)

        if not inner_boundary or len(inner_boundary) < 3:
            # 헤드랜드로 전체 영역이 소진됨 -> 헤드랜드만 반환
            return headland_wps

        # 2단계: 스와스 각도 결정
        if config.swath_angle_deg is not None:
            angle_deg = config.swath_angle_deg
        else:
            angle_deg = self.optimize_angle(
                swath_width=config.swath_width,
                overlap_ratio=config.overlap_ratio,
                work_area=inner_boundary,
            )

        # 3단계: 패턴별 내부 커버리지 경로 생성
        if config.pattern == CoveragePattern.BOUSTROPHEDON:
            inner_wps = self._generate_boustrophedon(
                inner_boundary, config, angle_deg
            )
        elif config.pattern == CoveragePattern.SPIRAL:
            inner_wps = self._generate_spiral(inner_boundary, config)
        elif config.pattern == CoveragePattern.RACETRACK:
            inner_wps = self._generate_racetrack(
                inner_boundary, config, angle_deg
            )
        else:
            inner_wps = self._generate_boustrophedon(
                inner_boundary, config, angle_deg
            )

        # 4단계: 경로 조합 (headland 먼저, 그 후 내부)
        all_waypoints = headland_wps + inner_wps

        return all_waypoints

    # ----- 기존 PathPlanner 호환 인터페이스 -----

    def generate_coverage_path_compat(
        self,
        field_boundary: List[Tuple[float, float]],
        swath_width: float,
        pattern: CoveragePattern = CoveragePattern.BOUSTROPHEDON,
        overlap_ratio: float = 0.0,
        headland_width: float = 0.0,
        swath_angle_deg: Optional[float] = None,
    ) -> List[Tuple[float, float]]:
        """기존 PathPlanner.generate_coverage_path() 호환 인터페이스.

        기존의 (field_boundary, swath_width) -> waypoints 시그니처를
        유지하면서 CoveragePlanner의 확장 기능을 사용할 수 있다.

        Args:
            field_boundary: 영역 경계 좌표 리스트 [(x1,y1), ...]
            swath_width: 작업 폭 (m)
            pattern: 커버리지 패턴 (기본: BOUSTROPHEDON)
            overlap_ratio: 오버랩 비율
            headland_width: 헤드랜드 폭 (m)
            swath_angle_deg: 스와스 각도 (None이면 자동)

        Returns:
            커버리지 웨이포인트 리스트 [(x, y), ...]
        """
        if len(field_boundary) < 3:
            return []

        boundary = FieldBoundary(vertices=field_boundary)
        config = CoverageConfig(
            swath_width=swath_width,
            overlap_ratio=overlap_ratio,
            headland_width=headland_width,
            swath_angle_deg=swath_angle_deg,
            pattern=pattern,
        )
        return self.plan_coverage(boundary, config)
