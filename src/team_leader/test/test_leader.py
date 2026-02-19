"""team_leader 패키지 기본 테스트."""

import pytest
from team_leader.planning import DrivingMode, PlanningModule


class TestDrivingMode:
    """DrivingMode enum 테스트."""

    def test_modes_exist(self):
        assert DrivingMode.IDLE is not None
        assert DrivingMode.LANE_KEEPING is not None
        assert DrivingMode.LANE_CHANGE is not None
        assert DrivingMode.OBSTACLE_AVOIDANCE is not None
        assert DrivingMode.EMERGENCY_STOP is not None

    def test_mode_names(self):
        assert DrivingMode.IDLE.name == 'IDLE'
        assert DrivingMode.EMERGENCY_STOP.name == 'EMERGENCY_STOP'
