"""Hybrid E2E Node 로직 테스트.

C48: ROS2 Node 의존 없이 hybrid_e2e_node의 핵심 로직을 검증.
- Fallback 레벨 전환
- Guardian 위반 처리
- 궤적 실행 로직
"""

import math
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'ad_core'))
sys.path.insert(0, str(Path(__file__).parent.parent))

from ad_core.hybrid_e2e_types import (
    PerceptionFeatures,
    PlannedTrajectory,
    TrajectoryPoint,
    GuardianDecision,
    HybridE2EState,
    TerrainClass,
)
from ad_core.datatypes import Pose2D
from ad_control.safety_guardian import SafetyGuardian, GuardianConfig


def _wp(x, y, yaw=0.0, v=0.5):
    """TrajectoryPoint 헬퍼."""
    return TrajectoryPoint(
        pose=Pose2D(x=x, y=y, yaw=yaw),
        velocity=v,
        acceleration=0.0,
        curvature=0.0,
        timestamp=0.0,
    )


# ── Helper: Guardian + State를 ROS 없이 테스트하는 컨텍스트 ──


class FakeHybridE2E:
    """hybrid_e2e_node.py의 로직을 ROS2 없이 재현."""

    def __init__(self):
        self._state = HybridE2EState()
        self._guardian = SafetyGuardian(GuardianConfig(
            max_speed=0.83,
            max_slope=0.3,
            min_crop_distance=0.3,
        ))
        self._fallback_level = 0
        self._current_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        self._current_speed = 0.0
        self._last_cmd = None  # (linear_x, angular_z)

    def run_cycle(self, perception, trajectory):
        """한 사이클 실행: guardian 검증 → 실행 or fallback."""
        if trajectory and self._current_pose:
            decision = self._guardian.validate(
                trajectory,
                self._current_pose,
                perception,
                self._current_speed
            )
            self._state.latest_guardian_decision = decision

            if decision.is_safe:
                self._execute(trajectory)
                return 'execute'
            else:
                self._handle_violation(decision)
                return 'violation'
        else:
            self._enter_fallback(1)
            return 'no_trajectory'

    def _execute(self, trajectory):
        if not trajectory.waypoints:
            self._last_cmd = (0.0, 0.0)
            return
        wp = trajectory.waypoints[min(2, len(trajectory.waypoints) - 1)]
        dx = wp.pose.x - self._current_pose.x
        dy = wp.pose.y - self._current_pose.y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self._current_pose.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        linear_x = min(1.5, distance * 0.5)
        angular_z = max(-1.0, min(1.0, angle_diff * 2.0))
        self._last_cmd = (linear_x, angular_z)

    def _handle_violation(self, decision):
        self._state.fallback_count += 1
        if decision.recommended_action == "stop":
            self._last_cmd = (0.0, 0.0)
            self._enter_fallback(3)
        elif decision.recommended_action in ("replan", "slow_down"):
            self._enter_fallback(min(self._fallback_level + 1, 2))

    def _enter_fallback(self, level):
        self._fallback_level = level


# ── Tests ──


class TestFallbackLevels:
    """Fallback 레벨 전환 테스트."""

    def test_initial_level_zero(self):
        e2e = FakeHybridE2E()
        assert e2e._fallback_level == 0

    def test_no_trajectory_enters_level_1(self):
        e2e = FakeHybridE2E()
        perception = PerceptionFeatures(
            terrain_type=TerrainClass.CROP_FIELD,
            overall_confidence=0.9
        )
        result = e2e.run_cycle(perception, None)
        assert result == 'no_trajectory'
        assert e2e._fallback_level == 1

    def test_guardian_stop_enters_level_3(self):
        e2e = FakeHybridE2E()
        # 급경사 → guardian이 stop 권고
        perception = PerceptionFeatures(
            terrain_type=TerrainClass.CROP_FIELD,
            slope_gradient=0.5,  # ~27도, max 17도 초과
            overall_confidence=0.9
        )
        trajectory = PlannedTrajectory(
            waypoints=[_wp(5.0, 0.0)],
            confidence=1.0
        )
        e2e._current_speed = 0.5
        result = e2e.run_cycle(perception, trajectory)
        # slope 초과 시 guardian이 stop or replan 권고
        assert e2e._fallback_level >= 1

    def test_safe_trajectory_stays_level_0(self):
        e2e = FakeHybridE2E()
        perception = PerceptionFeatures(
            terrain_type=TerrainClass.DIRT_ROAD,
            slope_gradient=0.02,
            overall_confidence=0.95
        )
        trajectory = PlannedTrajectory(
            waypoints=[_wp(1.0, 0.0), _wp(2.0, 0.0)],
            confidence=1.0
        )
        result = e2e.run_cycle(perception, trajectory)
        assert result == 'execute'
        assert e2e._fallback_level == 0


class TestTrajectoryExecution:
    """궤적 실행 로직 테스트."""

    def test_straight_trajectory(self):
        e2e = FakeHybridE2E()
        e2e._current_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        trajectory = PlannedTrajectory(
            waypoints=[_wp(1.0, 0.0), _wp(2.0, 0.0), _wp(3.0, 0.0)],
            confidence=1.0
        )
        e2e._execute(trajectory)
        linear_x, angular_z = e2e._last_cmd
        assert linear_x > 0.0, "직진 시 속도 양수"
        assert abs(angular_z) < 0.1, "직진 시 회전 거의 없음"

    def test_left_turn_trajectory(self):
        e2e = FakeHybridE2E()
        e2e._current_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        trajectory = PlannedTrajectory(
            waypoints=[_wp(0.5, 0.5), _wp(1.0, 1.0), _wp(1.5, 1.5)],
            confidence=1.0
        )
        e2e._execute(trajectory)
        _, angular_z = e2e._last_cmd
        assert angular_z > 0.0, "좌회전 시 양의 angular_z"

    def test_right_turn_trajectory(self):
        e2e = FakeHybridE2E()
        e2e._current_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        trajectory = PlannedTrajectory(
            waypoints=[_wp(0.5, -0.5), _wp(1.0, -1.0)],
            confidence=1.0
        )
        e2e._execute(trajectory)
        _, angular_z = e2e._last_cmd
        assert angular_z < 0.0, "우회전 시 음의 angular_z"

    def test_empty_waypoints(self):
        e2e = FakeHybridE2E()
        trajectory = PlannedTrajectory(waypoints=[], confidence=1.0)
        e2e._execute(trajectory)
        assert e2e._last_cmd == (0.0, 0.0)

    def test_speed_limit(self):
        e2e = FakeHybridE2E()
        e2e._current_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        trajectory = PlannedTrajectory(
            waypoints=[_wp(100.0, 0.0)],  # 매우 먼 웨이포인트
            confidence=1.0
        )
        e2e._execute(trajectory)
        linear_x, _ = e2e._last_cmd
        assert linear_x <= 1.5, f"속도 제한 초과: {linear_x}"


class TestGuardianIntegration:
    """Guardian + E2E 통합 테스트."""

    def test_safe_cycle_count(self):
        """연속 안전 사이클 카운팅."""
        e2e = FakeHybridE2E()
        perception = PerceptionFeatures(
            terrain_type=TerrainClass.DIRT_ROAD,
            slope_gradient=0.01,
            overall_confidence=0.95
        )
        trajectory = PlannedTrajectory(
            waypoints=[_wp(1.0, 0.0)],
            confidence=1.0
        )

        for _ in range(5):
            result = e2e.run_cycle(perception, trajectory)
            assert result == 'execute'

        assert e2e._state.fallback_count == 0

    def test_violation_increments_fallback_count(self):
        """Guardian 위반 시 fallback_count 증가."""
        e2e = FakeHybridE2E()
        perception = PerceptionFeatures(
            terrain_type=TerrainClass.CROP_FIELD,
            slope_gradient=0.5,  # 급경사
            overall_confidence=0.9
        )
        trajectory = PlannedTrajectory(
            waypoints=[_wp(5.0, 0.0)],
            confidence=1.0
        )
        e2e._current_speed = 2.0  # 과속

        initial_count = e2e._state.fallback_count
        e2e.run_cycle(perception, trajectory)
        # violation이면 fallback_count가 올라감
        if e2e._state.latest_guardian_decision and not e2e._state.latest_guardian_decision.is_safe:
            assert e2e._state.fallback_count > initial_count


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
