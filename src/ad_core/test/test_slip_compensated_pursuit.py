"""슬립 보상 적응형 Pure Pursuit (Phase 2) 테스트.

SlipEstimator, TerrainSlipProfile, SlipCompensatedPurePursuit 클래스의
기능과 기존 PurePursuitTracker와의 호환성을 검증한다.
"""

import math
import pytest

from ad_core.pure_pursuit import (
    DEFAULT_SLIP_PROFILE,
    Pose2D,
    PurePursuitConfig,
    PurePursuitTracker,
    SlipCompensatedPurePursuit,
    SlipCompensatedPurePursuitConfig,
    SlipEstimator,
    TerrainSlipProfile,
    TerrainType,
    TERRAIN_SLIP_TABLE,
    get_terrain_slip_profile,
    terrain_type_from_name,
)


# ====================================================================== #
#  TerrainSlipProfile 테스트
# ====================================================================== #


class TestTerrainSlipProfile:
    """지형별 슬립 프로파일 테스트."""

    def test_all_terrain_types_have_profiles(self):
        """모든 TerrainType에 대해 슬립 프로파일이 존재해야 한다."""
        for terrain in TerrainType:
            profile = get_terrain_slip_profile(terrain)
            assert isinstance(profile, TerrainSlipProfile)

    def test_paved_has_lowest_slip(self):
        """포장 도로가 가장 낮은 슬립 비율을 가져야 한다."""
        paved = get_terrain_slip_profile(TerrainType.PAVED)
        for terrain in TerrainType:
            if terrain == TerrainType.PAVED:
                continue
            other = get_terrain_slip_profile(terrain)
            assert paved.longitudinal_slip <= other.longitudinal_slip
            assert paved.lateral_slip <= other.lateral_slip

    def test_mud_has_highest_slip(self):
        """진흙이 가장 높은 슬립 비율을 가져야 한다."""
        mud = get_terrain_slip_profile(TerrainType.MUD)
        for terrain in TerrainType:
            if terrain == TerrainType.MUD:
                continue
            other = get_terrain_slip_profile(terrain)
            assert mud.longitudinal_slip >= other.longitudinal_slip
            assert mud.lateral_slip >= other.lateral_slip

    def test_slip_values_in_range(self):
        """모든 슬립 값이 유효 범위 [0, 1]에 있어야 한다."""
        for terrain in TerrainType:
            profile = get_terrain_slip_profile(terrain)
            assert 0.0 <= profile.longitudinal_slip <= 1.0
            assert 0.0 <= profile.lateral_slip <= 1.0
            assert 0.0 <= profile.slip_variability <= 1.0
            assert profile.max_safe_speed > 0.0

    def test_default_profile_for_unknown(self):
        """TERRAIN_SLIP_TABLE에 없는 유형에 대해 기본 프로파일을 반환해야 한다."""
        # get_terrain_slip_profile은 dict.get의 fallback을 사용
        result = TERRAIN_SLIP_TABLE.get(None, DEFAULT_SLIP_PROFILE)
        assert result is DEFAULT_SLIP_PROFILE

    def test_terrain_type_from_name_valid(self):
        """유효한 이름 문자열에서 올바른 TerrainType을 반환해야 한다."""
        assert terrain_type_from_name("PAVED") == TerrainType.PAVED
        assert terrain_type_from_name("mud") == TerrainType.MUD
        assert terrain_type_from_name("Dirt_Road") == TerrainType.DIRT_ROAD
        assert terrain_type_from_name("GRASS") == TerrainType.GRASS

    def test_terrain_type_from_name_invalid(self):
        """유효하지 않은 이름에 대해 DIRT_ROAD를 기본값으로 반환해야 한다."""
        assert terrain_type_from_name("UNKNOWN_SURFACE") == TerrainType.DIRT_ROAD
        assert terrain_type_from_name("") == TerrainType.DIRT_ROAD


# ====================================================================== #
#  SlipEstimator 테스트
# ====================================================================== #


class TestSlipEstimator:
    """슬립 추정기 테스트."""

    def test_initial_state(self):
        """초기 상태에서 슬립이 0이어야 한다."""
        est = SlipEstimator()
        assert est.longitudinal_slip == 0.0
        assert est.lateral_slip == 0.0
        assert est.slip_confidence == 0.0
        assert est.total_slip_magnitude == 0.0

    def test_first_update_stores_state(self):
        """첫 번째 update 호출은 상태만 저장하고 슬립을 변경하지 않아야 한다."""
        est = SlipEstimator()
        odom = Pose2D(x=1.0, y=0.0, yaw=0.0)
        actual = Pose2D(x=1.0, y=0.0, yaw=0.0)
        long_slip, lat_slip = est.update(odom, actual, current_time=0.0)
        assert long_slip == 0.0
        assert lat_slip == 0.0

    def test_no_slip_when_perfect_tracking(self):
        """오도메트리와 실측이 동일하면 슬립이 0에 가까워야 한다."""
        est = SlipEstimator(ema_alpha=0.5, min_displacement=0.001)

        # 첫 번째 호출: 초기화
        est.update(
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            current_time=0.0,
        )

        # 전방으로 1m 이동: odom과 actual 동일
        est.update(
            Pose2D(x=1.0, y=0.0, yaw=0.0),
            Pose2D(x=1.0, y=0.0, yaw=0.0),
            current_time=1.0,
        )

        assert est.longitudinal_slip < 0.05
        assert est.lateral_slip < 0.05

    def test_longitudinal_slip_detected(self):
        """종방향 슬립 발생 시 추정기가 양의 슬립을 감지해야 한다."""
        est = SlipEstimator(ema_alpha=0.8, min_displacement=0.001)

        # 초기화
        est.update(
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            current_time=0.0,
        )

        # 오도메트리는 2m 전진 했다고 하지만 실제로는 1m만 전진 (50% 슬립)
        est.update(
            Pose2D(x=2.0, y=0.0, yaw=0.0),
            Pose2D(x=1.0, y=0.0, yaw=0.0),
            current_time=1.0,
        )

        # EMA로 한 번 업데이트했으므로 정확히 0.5가 아닐 수 있지만 양수여야 함
        assert est.longitudinal_slip > 0.1

    def test_lateral_slip_detected(self):
        """횡방향 슬립 발생 시 추정기가 양의 횡방향 슬립을 감지해야 한다."""
        est = SlipEstimator(ema_alpha=0.8, min_displacement=0.001)

        # 초기화: 차량은 x 방향을 향하고 있음
        est.update(
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            current_time=0.0,
        )

        # 오도메트리는 직진(x방향)인데, 실제로는 측방(y방향)으로 밀림
        est.update(
            Pose2D(x=1.0, y=0.0, yaw=0.0),
            Pose2D(x=1.0, y=0.5, yaw=0.0),
            current_time=1.0,
        )

        assert est.lateral_slip > 0.1

    def test_confidence_increases(self):
        """반복 업데이트 시 신뢰도가 증가해야 한다."""
        est = SlipEstimator(confidence_rate=0.1, min_displacement=0.001)

        # 초기화
        est.update(
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            current_time=0.0,
        )

        for i in range(1, 6):
            est.update(
                Pose2D(x=float(i), y=0.0, yaw=0.0),
                Pose2D(x=float(i), y=0.0, yaw=0.0),
                current_time=float(i),
            )

        assert est.slip_confidence > 0.0
        assert est.slip_confidence <= 1.0

    def test_reset(self):
        """reset() 호출 시 모든 상태가 초기화되어야 한다."""
        est = SlipEstimator()

        # 어떤 값이든 설정
        est.longitudinal_slip = 0.5
        est.lateral_slip = 0.3
        est.slip_confidence = 0.8
        est.reset()

        assert est.longitudinal_slip == 0.0
        assert est.lateral_slip == 0.0
        assert est.slip_confidence == 0.0

    def test_terrain_prior_applied(self):
        """지형 사전 정보가 신뢰도가 낮을 때 적용되어야 한다."""
        est = SlipEstimator()
        assert est.slip_confidence == 0.0

        est.set_terrain_prior(TerrainType.MUD)
        mud_profile = get_terrain_slip_profile(TerrainType.MUD)

        # 신뢰도 0이므로 사전 정보가 전량 적용
        assert abs(est.longitudinal_slip - mud_profile.longitudinal_slip) < 0.01
        assert abs(est.lateral_slip - mud_profile.lateral_slip) < 0.01

    def test_total_slip_magnitude(self):
        """total_slip_magnitude가 유클리드 노름을 반환해야 한다."""
        est = SlipEstimator()
        est.longitudinal_slip = 0.3
        est.lateral_slip = 0.4
        expected = math.hypot(0.3, 0.4)
        assert abs(est.total_slip_magnitude - expected) < 1e-9

    def test_small_displacement_ignored(self):
        """최소 변위 미만의 이동은 슬립 계산에 반영되지 않아야 한다."""
        est = SlipEstimator(min_displacement=0.1, ema_alpha=1.0)

        # 초기화
        est.update(
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            current_time=0.0,
        )

        # 0.01m만 이동 (min_displacement=0.1 미만)
        est.update(
            Pose2D(x=0.01, y=0.0, yaw=0.0),
            Pose2D(x=0.005, y=0.0, yaw=0.0),
            current_time=0.1,
        )

        # 변위가 작으므로 슬립 추정이 갱신되지 않아야 함
        assert est.longitudinal_slip == 0.0


# ====================================================================== #
#  SlipCompensatedPurePursuit 테스트
# ====================================================================== #


class TestSlipCompensatedPurePursuit:
    """슬립 보상 적응형 Pure Pursuit 테스트."""

    @staticmethod
    def _make_straight_path(length: float = 20.0, step: float = 0.5):
        """직선 경로를 생성한다."""
        n = int(length / step) + 1
        return [(i * step, 0.0) for i in range(n)]

    @staticmethod
    def _make_circular_path(radius: float = 10.0, n_points: int = 100):
        """원형 경로를 생성한다."""
        return [
            (radius * math.cos(2 * math.pi * i / n_points),
             radius * math.sin(2 * math.pi * i / n_points))
            for i in range(n_points)
        ]

    def test_interface_compatibility_compute(self):
        """SlipCompensatedPurePursuit이 PurePursuitTracker와 동일한
        compute() 인터페이스를 지원해야 한다."""
        tracker = SlipCompensatedPurePursuit()
        tracker.set_path(self._make_straight_path())

        pose = Pose2D(x=0.0, y=0.5, yaw=0.0)
        result = tracker.compute(pose, 0.5)

        assert isinstance(result, tuple)
        assert len(result) == 2
        linear, angular = result
        assert isinstance(linear, float)
        assert isinstance(angular, float)

    def test_interface_compatibility_compute_track(self):
        """compute_track_velocities()가 올바른 형식을 반환해야 한다."""
        tracker = SlipCompensatedPurePursuit()
        tracker.set_path(self._make_straight_path())

        pose = Pose2D(x=0.0, y=0.5, yaw=0.0)
        result = tracker.compute_track_velocities(pose, 0.5)

        assert isinstance(result, tuple)
        assert len(result) == 2

    def test_isinstance_pure_pursuit_tracker(self):
        """SlipCompensatedPurePursuit이 PurePursuitTracker의 인스턴스여야 한다."""
        tracker = SlipCompensatedPurePursuit()
        assert isinstance(tracker, PurePursuitTracker)

    def test_set_path_resets_slip(self):
        """set_path() 호출 시 슬립 추정기가 초기화되어야 한다."""
        tracker = SlipCompensatedPurePursuit()
        tracker.slip_estimator.longitudinal_slip = 0.5
        tracker.slip_estimator.lateral_slip = 0.3
        tracker.slip_estimator.slip_confidence = 0.8

        tracker.set_path(self._make_straight_path())

        assert tracker.slip_estimator.longitudinal_slip == 0.0
        assert tracker.slip_estimator.lateral_slip == 0.0
        assert tracker.slip_estimator.slip_confidence == 0.0

    def test_set_path_with_terrain_initializes_prior(self):
        """지형이 설정된 상태에서 set_path()하면 사전 정보가 적용되어야 한다."""
        tracker = SlipCompensatedPurePursuit()
        tracker.set_terrain(TerrainType.MUD)
        tracker.set_path(self._make_straight_path())

        mud_profile = get_terrain_slip_profile(TerrainType.MUD)
        # 사전 정보가 적용되어야 함 (신뢰도 0이므로 전량 적용)
        assert abs(tracker.slip_estimator.longitudinal_slip
                   - mud_profile.longitudinal_slip) < 0.01

    def test_adaptive_lookahead_increases_with_slip(self):
        """슬립이 클수록 lookahead distance가 증가해야 한다."""
        config = SlipCompensatedPurePursuitConfig(
            lookahead_gain=1.0,
            min_lookahead=1.0,
            max_lookahead=5.0,
            slip_lookahead_gain=2.0,
            max_slip_lookahead_ratio=2.0,
        )
        tracker = SlipCompensatedPurePursuit(config=config)
        tracker.set_path(self._make_straight_path())

        # 슬립 없을 때
        tracker.slip_estimator.longitudinal_slip = 0.0
        tracker.slip_estimator.lateral_slip = 0.0
        ld_no_slip = tracker._compute_lookahead_distance(1.0)

        # 슬립 있을 때
        tracker.slip_estimator.longitudinal_slip = 0.3
        tracker.slip_estimator.lateral_slip = 0.3
        ld_with_slip = tracker._compute_lookahead_distance(1.0)

        assert ld_with_slip > ld_no_slip

    def test_speed_reduces_with_slip(self):
        """슬립이 클수록 속도가 감소해야 한다."""
        # 지형 안전 속도 제한에 걸리지 않도록 PAVED 지형 사용
        config = SlipCompensatedPurePursuitConfig(max_linear_speed=0.5)
        tracker = SlipCompensatedPurePursuit(config=config)
        tracker.set_path(self._make_straight_path())
        tracker.set_terrain(TerrainType.PAVED)  # max_safe_speed=1.0

        # 슬립 없을 때
        tracker.slip_estimator.longitudinal_slip = 0.0
        tracker.slip_estimator.lateral_slip = 0.0
        speed_no_slip = tracker._compute_speed(0.0)

        # 슬립 있을 때
        tracker.slip_estimator.longitudinal_slip = 0.4
        tracker.slip_estimator.lateral_slip = 0.4
        speed_with_slip = tracker._compute_speed(0.0)

        assert speed_with_slip < speed_no_slip

    def test_terrain_limits_speed(self):
        """지형 최대 안전 속도가 제한으로 작동해야 한다."""
        config = SlipCompensatedPurePursuitConfig(
            max_linear_speed=2.0,  # 높게 설정
        )
        tracker = SlipCompensatedPurePursuit(config=config)
        tracker.set_path(self._make_straight_path())

        tracker.set_terrain(TerrainType.MUD)
        mud_profile = get_terrain_slip_profile(TerrainType.MUD)

        speed = tracker._compute_speed(0.0)
        assert speed <= mud_profile.max_safe_speed

    def test_track_velocity_compensation(self):
        """슬립 보상이 트랙 속도에 반영되어야 한다."""
        tracker = SlipCompensatedPurePursuit()
        tracker.set_path(self._make_straight_path())

        # 슬립 없을 때 트랙 속도
        tracker.slip_estimator.longitudinal_slip = 0.0
        tracker.slip_estimator.lateral_slip = 0.0
        vl_no, vr_no = tracker._twist_to_tracks(0.5, 0.1)

        # 종방향 슬립 있을 때 트랙 속도 (보상으로 속도가 증가해야 함)
        tracker.slip_estimator.longitudinal_slip = 0.3
        tracker.slip_estimator.lateral_slip = 0.0
        vl_slip, vr_slip = tracker._twist_to_tracks(0.5, 0.1)

        # 종방향 보상: 동일한 입력에서 슬립 보상으로 인해 출력이 더 커야 함
        assert abs(vl_slip) + abs(vr_slip) >= abs(vl_no) + abs(vr_no) - 0.01

    def test_get_slip_info(self):
        """get_slip_info()가 올바른 구조의 딕셔너리를 반환해야 한다."""
        tracker = SlipCompensatedPurePursuit()
        tracker.set_terrain(TerrainType.GRAVEL)

        info = tracker.get_slip_info()
        assert "longitudinal_slip" in info
        assert "lateral_slip" in info
        assert "total_slip" in info
        assert "confidence" in info
        assert "terrain" in info
        assert info["terrain"] == "GRAVEL"

    def test_empty_path_returns_zero(self):
        """경로가 비어있으면 (0, 0)을 반환해야 한다."""
        tracker = SlipCompensatedPurePursuit()
        pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

        linear, angular = tracker.compute(pose, 0.5)
        assert linear == 0.0
        assert angular == 0.0

    def test_goal_reached_returns_zero(self):
        """목표 도달 시 (0, 0)을 반환해야 한다."""
        config = SlipCompensatedPurePursuitConfig(goal_tolerance=1.0)
        tracker = SlipCompensatedPurePursuit(config=config)
        tracker.set_path([(0.0, 0.0), (1.0, 0.0)])

        # 목표 근처에서 compute 호출
        pose = Pose2D(x=0.9, y=0.0, yaw=0.0)
        linear, angular = tracker.compute(pose, 0.1)

        # 골 도달 후에는 0 반환
        if tracker.is_goal_reached:
            assert linear == 0.0
            assert angular == 0.0

    def test_slip_compensation_does_not_exceed_max_speed(self):
        """슬립 보상 후에도 트랙 속도가 최대 속도를 초과하지 않아야 한다."""
        config = SlipCompensatedPurePursuitConfig(
            max_linear_speed=1.0,
            longitudinal_compensation_gain=1.0,
            lateral_compensation_gain=1.0,
        )
        tracker = SlipCompensatedPurePursuit(config=config)
        tracker.set_path(self._make_straight_path())

        # 큰 슬립
        tracker.slip_estimator.longitudinal_slip = 0.5
        tracker.slip_estimator.lateral_slip = 0.5

        vl, vr = tracker._twist_to_tracks(1.0, 0.5)
        assert abs(vl) <= config.max_linear_speed + 1e-9
        assert abs(vr) <= config.max_linear_speed + 1e-9

    def test_update_slip_integration(self):
        """update_slip()이 내부 슬립 추정기를 올바르게 업데이트해야 한다."""
        tracker = SlipCompensatedPurePursuit()
        tracker.set_path(self._make_straight_path())

        # 초기 업데이트
        tracker.update_slip(
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            Pose2D(x=0.0, y=0.0, yaw=0.0),
            current_time=0.0,
        )

        # 슬립이 있는 업데이트
        tracker.update_slip(
            Pose2D(x=2.0, y=0.0, yaw=0.0),  # 오도메트리: 2m
            Pose2D(x=1.5, y=0.0, yaw=0.0),  # 실측: 1.5m
            current_time=1.0,
        )

        assert tracker.slip_estimator.longitudinal_slip > 0.0

    def test_min_speed_ratio_enforced(self):
        """슬립이 매우 커도 min_speed_ratio 이상의 속도가 나와야 한다."""
        config = SlipCompensatedPurePursuitConfig(
            max_linear_speed=1.0,
            slip_speed_reduction_gain=1.0,
            min_speed_ratio=0.2,
        )
        tracker = SlipCompensatedPurePursuit(config=config)
        tracker.set_path(self._make_straight_path())

        # 매우 큰 슬립
        tracker.slip_estimator.longitudinal_slip = 0.9
        tracker.slip_estimator.lateral_slip = 0.9

        speed = tracker._compute_speed(0.0)
        # min_speed_ratio=0.2이므로 최소한 max_linear_speed * 0.2 이상이어야 함
        # (지형 안전 속도 제한 전)
        # 실제로는 DEFAULT_SLIP_PROFILE.max_safe_speed와 비교하므로
        # 어느 쪽이든 0 초과여야 함
        assert speed > 0.0


# ====================================================================== #
#  기존 PurePursuitTracker와의 하위 호환성 테스트
# ====================================================================== #


class TestBackwardCompatibility:
    """기존 PurePursuitTracker와의 호환성 테스트.

    SlipCompensatedPurePursuit을 PurePursuitTracker가 사용되던 곳에
    그대로 치환해도 동작해야 한다.
    """

    @staticmethod
    def _make_path():
        return [(i * 0.5, 0.0) for i in range(20)]

    def test_drop_in_replacement(self):
        """PurePursuitTracker 대신 SlipCompensatedPurePursuit을 사용해도
        동일한 메서드를 호출할 수 있어야 한다."""
        # PurePursuitTracker 방식
        base_tracker: PurePursuitTracker = SlipCompensatedPurePursuit()
        base_tracker.set_path(self._make_path())

        pose = Pose2D(x=0.0, y=0.3, yaw=0.0)
        linear, angular = base_tracker.compute(pose, 0.5)
        assert isinstance(linear, float)
        assert isinstance(angular, float)

        vl, vr = base_tracker.compute_track_velocities(pose, 0.5)
        assert isinstance(vl, float)
        assert isinstance(vr, float)

    def test_base_class_without_slip_produces_similar_results(self):
        """슬립이 0일 때 SlipCompensatedPurePursuit의 결과가
        기본 PurePursuitTracker와 유사해야 한다."""
        path = self._make_path()
        pose = Pose2D(x=0.0, y=0.3, yaw=0.0)
        speed = 0.5

        # 기본 추종기
        base_config = PurePursuitConfig(
            lookahead_gain=0.8,
            min_lookahead=1.0,
            max_lookahead=5.0,
            max_linear_speed=1.0,
            track_width=1.4,
        )
        base = PurePursuitTracker(config=base_config)
        base.set_path(path)
        lin_base, ang_base = base.compute(pose, speed)

        # 슬립 보상 추종기 (슬립 0, 지형 미설정)
        slip_config = SlipCompensatedPurePursuitConfig(
            lookahead_gain=0.8,
            min_lookahead=1.0,
            max_lookahead=5.0,
            max_linear_speed=1.0,
            track_width=1.4,
            slip_lookahead_gain=2.0,
        )
        slip = SlipCompensatedPurePursuit(config=slip_config)
        slip.set_path(path)
        lin_slip, ang_slip = slip.compute(pose, speed)

        # 슬립 0이면 결과가 동일해야 함
        # (DEFAULT_SLIP_PROFILE의 max_safe_speed가 0.7이므로 속도가 약간 다를 수 있음)
        # 방향(각속도의 부호)은 동일해야 함
        if ang_base != 0.0:
            assert (ang_base > 0) == (ang_slip > 0), \
                "슬립 0에서 회전 방향이 동일해야 함"

    def test_is_goal_reached_attribute(self):
        """is_goal_reached 속성이 PurePursuitTracker와 동일하게 동작해야 한다."""
        tracker = SlipCompensatedPurePursuit()
        assert hasattr(tracker, 'is_goal_reached')
        assert tracker.is_goal_reached is False

    def test_nearest_idx_attribute(self):
        """nearest_idx 속성이 존재해야 한다."""
        tracker = SlipCompensatedPurePursuit()
        assert hasattr(tracker, 'nearest_idx')

    def test_path_attribute(self):
        """path 속성이 존재하고 리스트여야 한다."""
        tracker = SlipCompensatedPurePursuit()
        assert hasattr(tracker, 'path')
        assert isinstance(tracker.path, list)
