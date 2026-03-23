"""PlanningModule / PathPlanner Mock 노드 테스트 - ROS2 의존 없이 판단 로직 검증."""

import math
from unittest.mock import MagicMock

import pytest

from ad_core.coverage_planner import CoveragePattern


# ── Mock Node ─────────────────────────────────────────────────


class MockParameter:
    def __init__(self, value):
        self.value = value


class MockTimeMsg:
    sec = 0
    nanosec = 0


class MockTime:
    def to_msg(self):
        return MockTimeMsg()


class MockClock:
    def now(self):
        return MockTime()


def make_mock_node(params=None):
    defaults = {'max_speed': 3.0, 'safe_distance': 5.0}
    if params:
        defaults.update(params)
    node = MagicMock()
    node.get_parameter = lambda name: MockParameter(defaults.get(name, None))
    node.get_clock = MagicMock(return_value=MockClock())
    return node


from ad_planning.planning_node import DrivingMode, PathPlanner, PlanningModule


# ── PathPlanner ───────────────────────────────────────────────


class TestPathPlanner:
    @pytest.fixture
    def planner(self):
        return PathPlanner(make_mock_node())

    def test_coverage_boustrophedon(self, planner):
        boundary = [(0, 0), (10, 0), (10, 10), (0, 10)]
        assert len(planner.generate_coverage_path(
            boundary, swath_width=2.0, pattern=CoveragePattern.BOUSTROPHEDON)) > 0

    def test_coverage_spiral(self, planner):
        boundary = [(0, 0), (10, 0), (10, 10), (0, 10)]
        assert len(planner.generate_coverage_path(
            boundary, swath_width=2.0, pattern=CoveragePattern.SPIRAL)) > 0

    def test_too_few_boundary_points(self, planner):
        assert planner.generate_coverage_path([(0, 0), (1, 1)], swath_width=2.0) == []

    def test_zero_swath_width(self, planner):
        assert planner.generate_coverage_path(
            [(0, 0), (10, 0), (10, 10), (0, 10)], swath_width=0.0) == []

    def test_negative_swath_width(self, planner):
        assert planner.generate_coverage_path(
            [(0, 0), (10, 0), (10, 10), (0, 10)], swath_width=-1.0) == []

    def test_generate_waypoint_path(self, planner):
        path = planner.generate_waypoint_path([(0, 0), (5, 0), (5, 5)])
        assert len(path.poses) == 3

    def test_single_waypoint_path(self, planner):
        assert len(planner.generate_waypoint_path([(3, 4)]).poses) == 1

    def test_waypoint_orientation_east(self, planner):
        pose = planner.generate_waypoint_path([(0, 0), (1, 0)]).poses[0]
        assert abs(pose.pose.orientation.z) < 0.01
        assert abs(pose.pose.orientation.w - 1.0) < 0.01

    def test_find_x_intersections(self, planner):
        polygon = [(0, 0), (10, 0), (10, 10), (0, 10)]
        xs = planner._find_x_intersections(polygon, 5.0)
        assert len(xs) == 2
        assert xs[0] == pytest.approx(0.0, abs=0.01)
        assert xs[1] == pytest.approx(10.0, abs=0.01)


# ── PlanningModule 초기화 ────────────────────────────────────


class TestPlanningModuleInit:
    def test_init_defaults(self):
        pm = PlanningModule(make_mock_node())
        assert pm.current_mode == DrivingMode.IDLE
        assert pm.target_speed == 0.0
        assert pm.max_speed == 3.0
        assert pm.safe_distance == 5.0

    def test_custom_params(self):
        pm = PlanningModule(make_mock_node({'max_speed': 5.0, 'safe_distance': 3.0}))
        assert pm.max_speed == 5.0
        assert pm.safe_distance == 3.0


# ── update (인지 결과 기반 판단) ───────────────────────────────


class TestPlanningUpdate:
    @pytest.fixture
    def pm(self):
        return PlanningModule(make_mock_node())

    def test_no_obstacles_lane_keeping(self, pm):
        pm.update({'obstacles': []})
        assert pm.current_mode == DrivingMode.LANE_KEEPING
        assert pm.target_speed == pm.max_speed

    def test_lane_keeping_with_offset(self, pm):
        pm.update({'obstacles': [], 'lane_info': {'offset': 2.0}})
        assert pm.target_steering == pytest.approx(-0.2, abs=1e-6)

    def test_emergency_stop(self, pm):
        pm.update({'obstacles': [{'distance': 1.0}]})
        assert pm.current_mode == DrivingMode.EMERGENCY_STOP
        assert pm.target_speed == 0.0

    def test_obstacle_avoidance(self, pm):
        pm.update({'obstacles': [{'distance': 3.5}]})
        assert pm.current_mode == DrivingMode.OBSTACLE_AVOIDANCE
        assert pm.target_speed == pm.max_speed * 0.5


# ── plan (모드별 계획) ─────────────────────────────────────────


class TestPlanningPlan:
    @pytest.fixture
    def pm(self):
        return PlanningModule(make_mock_node())

    def test_plan_coverage(self, pm):
        pm.plan(DrivingMode.COVERAGE,
                field_boundary=[(0, 0), (20, 0), (20, 20), (0, 20)],
                swath_width=2.0)
        assert pm.current_mode == DrivingMode.COVERAGE
        assert pm.target_speed > 0.0
        assert pm.get_nav_path() is not None

    def test_plan_coverage_invalid(self, pm):
        pm.plan(DrivingMode.COVERAGE, field_boundary=[(0, 0)], swath_width=2.0)
        assert pm.current_mode == DrivingMode.IDLE

    def test_plan_waypoint_nav(self, pm):
        pm.plan(DrivingMode.WAYPOINT_NAV, waypoints=[(0, 0), (5, 5), (10, 0)])
        assert pm.current_mode == DrivingMode.WAYPOINT_NAV
        assert pm.target_speed == pm.max_speed

    def test_plan_waypoint_nav_empty(self, pm):
        pm.plan(DrivingMode.WAYPOINT_NAV, waypoints=[])
        assert pm.current_mode == DrivingMode.IDLE

    def test_plan_emergency_stop(self, pm):
        pm.plan(DrivingMode.EMERGENCY_STOP)
        assert pm.target_speed == 0.0

    def test_plan_idle(self, pm):
        pm.target_speed = 5.0
        pm.plan(DrivingMode.IDLE)
        assert pm.target_speed == 0.0

    def test_plan_lane_keeping(self, pm):
        pm.plan(DrivingMode.LANE_KEEPING, lane_info={'offset': 1.0})
        assert pm.target_steering == pytest.approx(-0.1, abs=1e-6)

    def test_plan_coverage_pattern_string(self, pm):
        pm.plan(DrivingMode.COVERAGE,
                field_boundary=[(0, 0), (20, 0), (20, 20), (0, 20)],
                swath_width=2.0, pattern='RACETRACK')
        assert pm.current_mode == DrivingMode.COVERAGE

    def test_plan_coverage_invalid_pattern(self, pm):
        pm.plan(DrivingMode.COVERAGE,
                field_boundary=[(0, 0), (20, 0), (20, 20), (0, 20)],
                swath_width=2.0, pattern='UNKNOWN_PATTERN')
        assert pm.current_mode == DrivingMode.COVERAGE


# ── 웨이포인트 진행 ───────────────────────────────────────────


class TestAdvanceWaypoint:
    @pytest.fixture
    def pm(self):
        pm = PlanningModule(make_mock_node())
        pm.plan(DrivingMode.WAYPOINT_NAV,
                waypoints=[(0, 0), (5, 0), (10, 0)])
        return pm

    def test_not_done_initially(self, pm):
        assert pm.advance_waypoint((0, 0), tolerance=1.0) is False

    def test_advance_through_all(self, pm):
        pm.advance_waypoint((0, 0), tolerance=1.0)
        pm.advance_waypoint((5, 0), tolerance=1.0)
        assert pm.advance_waypoint((10, 0), tolerance=1.0) is True
        assert pm.current_mode == DrivingMode.IDLE

    def test_empty_waypoints_done(self):
        pm = PlanningModule(make_mock_node())
        assert pm.advance_waypoint((0, 0)) is True

    def test_get_current_target(self, pm):
        assert pm.get_current_target_waypoint() == (0, 0)
        pm.advance_waypoint((0, 0), tolerance=1.0)
        assert pm.get_current_target_waypoint() == (5, 0)


# ── get_plan_result ───────────────────────────────────────────


class TestGetPlanResult:
    def test_idle_result(self):
        result = PlanningModule(make_mock_node()).get_plan_result()
        assert result['mode'] == 'IDLE'

    def test_coverage_result(self):
        pm = PlanningModule(make_mock_node())
        pm.plan(DrivingMode.COVERAGE,
                field_boundary=[(0, 0), (20, 0), (20, 20), (0, 20)],
                swath_width=2.0)
        result = pm.get_plan_result()
        assert result['mode'] == 'COVERAGE'
        assert result['total_waypoints'] > 0
        assert result['has_nav_path'] is True
