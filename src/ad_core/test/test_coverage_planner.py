"""CoveragePlanner 단위 테스트.

시뮬레이터 없이 순수 Python 로직만 테스트한다.
"""

import math
import pytest

from ad_core.coverage_planner import (
    CoverageConfig,
    CoveragePattern,
    CoveragePlanner,
    FieldBoundary,
    _ensure_ccw,
    _polygon_area_signed,
    _polygon_centroid,
    _point_distance,
    _rotate_point,
)


# ── 테스트 픽스처 ──────────────────────────────────────────────

SQUARE_FIELD = [(0, 0), (100, 0), (100, 50), (0, 50)]
RECT_FIELD = [(0, 0), (200, 0), (200, 100), (0, 100)]
TRIANGLE_FIELD = [(0, 0), (100, 0), (50, 80)]


@pytest.fixture
def planner():
    return CoveragePlanner()


@pytest.fixture
def square_boundary():
    return FieldBoundary(vertices=SQUARE_FIELD)


@pytest.fixture
def rect_boundary():
    return FieldBoundary(vertices=RECT_FIELD)


# ── FieldBoundary 데이터 구조 ──────────────────────────────────

class TestFieldBoundary:
    def test_valid_boundary(self):
        fb = FieldBoundary(vertices=SQUARE_FIELD)
        assert len(fb.vertices) == 4

    def test_minimum_vertices(self):
        fb = FieldBoundary(vertices=TRIANGLE_FIELD)
        assert len(fb.vertices) == 3

    def test_too_few_vertices_raises(self):
        with pytest.raises(ValueError, match="최소 3개"):
            FieldBoundary(vertices=[(0, 0), (1, 1)])

    def test_empty_vertices_raises(self):
        with pytest.raises(ValueError):
            FieldBoundary(vertices=[])


# ── 기하 유틸리티 ──────────────────────────────────────────────

class TestGeometryUtils:
    def test_polygon_area_signed_ccw(self):
        """반시계 방향 다각형은 양수 넓이."""
        area = _polygon_area_signed(SQUARE_FIELD)
        assert area > 0
        assert abs(area - 5000.0) < 1e-6  # 100 * 50

    def test_polygon_area_signed_cw(self):
        """시계 방향 다각형은 음수 넓이."""
        cw = list(reversed(SQUARE_FIELD))
        area = _polygon_area_signed(cw)
        assert area < 0

    def test_ensure_ccw_already_ccw(self):
        result = _ensure_ccw(SQUARE_FIELD)
        assert _polygon_area_signed(result) > 0

    def test_ensure_ccw_converts_cw(self):
        cw = list(reversed(SQUARE_FIELD))
        result = _ensure_ccw(cw)
        assert _polygon_area_signed(result) > 0

    def test_polygon_centroid_square(self):
        cx, cy = _polygon_centroid(SQUARE_FIELD)
        assert abs(cx - 50.0) < 1e-6
        assert abs(cy - 25.0) < 1e-6

    def test_rotate_point_90deg(self):
        rx, ry = _rotate_point(1.0, 0.0, 0.0, 0.0, math.pi / 2)
        assert abs(rx - 0.0) < 1e-6
        assert abs(ry - 1.0) < 1e-6

    def test_rotate_point_360deg_identity(self):
        rx, ry = _rotate_point(3.0, 4.0, 1.0, 2.0, 2 * math.pi)
        assert abs(rx - 3.0) < 1e-6
        assert abs(ry - 4.0) < 1e-6

    def test_point_distance(self):
        d = _point_distance((0, 0), (3, 4))
        assert abs(d - 5.0) < 1e-6


# ── CoveragePlanner 핵심 기능 ──────────────────────────────────

class TestCoveragePlannerSwaths:
    def test_generate_swaths_basic(self, planner, square_boundary):
        planner.set_field(square_boundary)
        swaths = planner.generate_swaths(angle_deg=0, swath_width=10.0)
        assert len(swaths) > 0
        for swath in swaths:
            assert len(swath) == 2  # 각 스와스는 시작/끝 2점

    def test_generate_swaths_no_field_raises(self, planner):
        with pytest.raises(RuntimeError, match="set_field"):
            planner.generate_swaths()

    def test_generate_swaths_invalid_overlap_raises(self, planner, square_boundary):
        planner.set_field(square_boundary)
        with pytest.raises(ValueError, match="유효 스와스 간격"):
            planner.generate_swaths(swath_width=2.0, overlap_ratio=1.0)

    def test_swath_count_decreases_with_wider_width(self, planner, rect_boundary):
        planner.set_field(rect_boundary)
        narrow = planner.generate_swaths(swath_width=5.0)
        wide = planner.generate_swaths(swath_width=20.0)
        assert len(narrow) > len(wide)

    def test_swath_count_increases_with_overlap(self, planner, square_boundary):
        planner.set_field(square_boundary)
        no_overlap = planner.generate_swaths(swath_width=10.0, overlap_ratio=0.0)
        with_overlap = planner.generate_swaths(swath_width=10.0, overlap_ratio=0.3)
        assert len(with_overlap) >= len(no_overlap)


class TestCoveragePlannerOptimize:
    def test_optimize_angle_returns_float(self, planner, square_boundary):
        planner.set_field(square_boundary)
        angle = planner.optimize_angle(swath_width=5.0)
        assert isinstance(angle, float)
        assert 0.0 <= angle < 180.0

    def test_optimize_angle_rectangular_prefers_long_axis(self, planner):
        """긴 직사각형은 긴 축 방향(0도)이 최적일 가능성이 높다."""
        field = FieldBoundary(vertices=[(0, 0), (200, 0), (200, 20), (0, 20)])
        planner.set_field(field)
        angle = planner.optimize_angle(swath_width=5.0, angle_step_deg=5.0)
        # 0도 또는 180도에 가까워야 함 (긴 축 방향)
        assert angle < 30.0 or angle > 150.0


class TestCoveragePlannerHeadland:
    def test_headland_zero_width(self, planner, square_boundary):
        wps, inner = planner.plan_headland_path(square_boundary, width=0.0)
        assert len(wps) == 0
        assert len(inner) == 4  # 원래 경계 그대로

    def test_headland_creates_waypoints(self, planner, square_boundary):
        wps, inner = planner.plan_headland_path(square_boundary, width=5.0)
        assert len(wps) > 0
        assert len(inner) >= 3  # 내부 경계가 남아있어야 함

    def test_headland_inner_is_smaller(self, planner, square_boundary):
        _, inner = planner.plan_headland_path(square_boundary, width=5.0)
        inner_area = abs(_polygon_area_signed(inner))
        original_area = abs(_polygon_area_signed(SQUARE_FIELD))
        assert inner_area < original_area


class TestCoveragePlannerReorder:
    def test_reorder_single_swath(self, planner):
        swaths = [[(0, 0), (10, 0)]]
        result = planner.reorder_swaths(swaths)
        assert len(result) == 1

    def test_reorder_empty(self, planner):
        result = planner.reorder_swaths([])
        assert len(result) == 0

    def test_reorder_preserves_count(self, planner):
        swaths = [
            [(0, 0), (10, 0)],
            [(0, 5), (10, 5)],
            [(0, 10), (10, 10)],
        ]
        result = planner.reorder_swaths(swaths)
        assert len(result) == len(swaths)


# ── 전체 커버리지 경로 생성 ────────────────────────────────────

class TestPlanCoverage:
    def test_boustrophedon_generates_path(self, planner, square_boundary):
        config = CoverageConfig(
            swath_width=10.0,
            pattern=CoveragePattern.BOUSTROPHEDON,
        )
        wps = planner.plan_coverage(square_boundary, config)
        assert len(wps) > 0

    def test_spiral_generates_path(self, planner, square_boundary):
        config = CoverageConfig(
            swath_width=5.0,
            pattern=CoveragePattern.SPIRAL,
        )
        wps = planner.plan_coverage(square_boundary, config)
        assert len(wps) > 0

    def test_racetrack_generates_path(self, planner, square_boundary):
        config = CoverageConfig(
            swath_width=10.0,
            pattern=CoveragePattern.RACETRACK,
        )
        wps = planner.plan_coverage(square_boundary, config)
        assert len(wps) > 0

    def test_with_headland(self, planner, square_boundary):
        config = CoverageConfig(
            swath_width=5.0,
            headland_width=10.0,
            pattern=CoveragePattern.BOUSTROPHEDON,
        )
        wps = planner.plan_coverage(square_boundary, config)
        assert len(wps) > 0

    def test_with_explicit_angle(self, planner, square_boundary):
        config = CoverageConfig(
            swath_width=10.0,
            swath_angle_deg=45.0,
        )
        wps = planner.plan_coverage(square_boundary, config)
        assert len(wps) > 0

    def test_triangle_field(self, planner):
        boundary = FieldBoundary(vertices=TRIANGLE_FIELD)
        config = CoverageConfig(swath_width=10.0)
        wps = planner.plan_coverage(boundary, config)
        assert len(wps) > 0

    def test_compat_interface(self, planner):
        wps = planner.generate_coverage_path_compat(
            field_boundary=SQUARE_FIELD,
            swath_width=10.0,
        )
        assert len(wps) > 0

    def test_compat_too_few_vertices(self, planner):
        wps = planner.generate_coverage_path_compat(
            field_boundary=[(0, 0), (1, 1)],
            swath_width=5.0,
        )
        assert wps == []

    def test_all_waypoints_are_tuples(self, planner, square_boundary):
        config = CoverageConfig(swath_width=10.0)
        wps = planner.plan_coverage(square_boundary, config)
        for wp in wps:
            assert len(wp) == 2
            assert isinstance(wp[0], (int, float))
            assert isinstance(wp[1], (int, float))
