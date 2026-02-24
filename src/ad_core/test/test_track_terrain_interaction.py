"""TrackTerrainInteraction 단위 테스트."""

import math
import pytest

from ad_core.track_terrain_interaction import (
    TerrainConfig,
    TerrainType,
    TrackTerrainInteraction,
    get_terrain_config,
    TERRAIN_PARAMS,
)


class TestTerrainType:
    def test_all_types_have_params(self):
        """모든 TerrainType에 디폴트 파라미터가 있다."""
        for t in TerrainType:
            cfg = get_terrain_config(t)
            assert isinstance(cfg, TerrainConfig)

    def test_new_paddy_types_exist(self):
        """논/밭 확장 지형이 존재한다."""
        assert TerrainType.PADDY_WET.value == "paddy_wet"
        assert TerrainType.PADDY_DRY.value == "paddy_dry"
        assert TerrainType.FIELD_SOFT.value == "field_soft"
        assert TerrainType.FIELD_HARD.value == "field_hard"


class TestTrackTerrainInteraction:
    @pytest.fixture
    def paved(self):
        return TrackTerrainInteraction(
            terrain=TerrainType.PAVED, track_width=0.3, track_length=1.2
        )

    @pytest.fixture
    def paddy_wet(self):
        return TrackTerrainInteraction(
            terrain=TerrainType.PADDY_WET, track_width=0.3, track_length=1.2
        )

    # 침하 테스트
    def test_sinkage_zero_no_force(self, paved):
        """수직력 0이면 침하 0."""
        assert paved.compute_sinkage(0.0) == 0.0

    def test_sinkage_increases_with_force(self, paddy_wet):
        """수직력이 클수록 침하 증가."""
        s1 = paddy_wet.compute_sinkage(500.0)
        s2 = paddy_wet.compute_sinkage(1000.0)
        assert s2 > s1 > 0

    def test_paddy_sinkage_greater_than_paved(self, paved, paddy_wet):
        """젖은 논이 포장도로보다 침하가 크다."""
        force = 800.0
        s_paved = paved.compute_sinkage(force)
        s_paddy = paddy_wet.compute_sinkage(force)
        assert s_paddy > s_paved

    def test_sinkage_bounded(self, paddy_wet):
        """침하량 상한 0.5m."""
        s = paddy_wet.compute_sinkage(100000.0)
        assert s <= 0.5

    # 견인력 테스트
    def test_traction_zero_no_slip(self, paved):
        """슬립비 0이면 견인력 0."""
        t = paved.compute_traction(1000.0, 0.0)
        assert abs(t) < 1e-6

    def test_traction_increases_with_slip(self, paved):
        """슬립비가 증가하면 견인력 증가 (일정 범위까지)."""
        t1 = paved.compute_traction(1000.0, 0.05)
        t2 = paved.compute_traction(1000.0, 0.15)
        assert t2 > t1

    def test_traction_paved_vs_mud(self):
        """포장도로가 진흙보다 견인력이 크다 (동일 수직력/슬립)."""
        paved = TrackTerrainInteraction(TerrainType.PAVED)
        mud = TrackTerrainInteraction(TerrainType.MUD)
        force = 1000.0
        slip = 0.1
        t_paved = paved.compute_traction(force, slip)
        t_mud = mud.compute_traction(force, slip)
        assert t_paved > t_mud

    def test_traction_zero_no_force(self, paved):
        """수직력 0이면 견인력 0."""
        assert paved.compute_traction(0.0, 0.1) == 0.0

    # 선회 저항 테스트
    def test_turning_resistance_zero_straight(self, paved):
        """직진 시 선회저항 0."""
        r = paved.compute_turning_resistance(0.0, 1000.0)
        assert abs(r) < 1e-6

    def test_turning_resistance_increases_with_yaw_rate(self, paddy_wet):
        """각속도 증가 시 선회저항 증가."""
        r1 = paddy_wet.compute_turning_resistance(0.1, 1000.0)
        r2 = paddy_wet.compute_turning_resistance(0.5, 1000.0)
        assert r2 > r1

    def test_turning_resistance_symmetry(self, paved):
        """좌/우 회전의 선회저항이 동일."""
        r_left = paved.compute_turning_resistance(0.5, 1000.0)
        r_right = paved.compute_turning_resistance(-0.5, 1000.0)
        assert abs(r_left - r_right) < 1e-6

    # 구름 저항 테스트
    def test_rolling_resistance_positive(self, paved):
        assert paved.compute_rolling_resistance(1000.0) > 0

    def test_rolling_resistance_zero_no_force(self, paved):
        assert paved.compute_rolling_resistance(0.0) == 0.0

    # 경사면 힘 분해 테스트
    def test_slope_forces_flat(self):
        along, normal, lateral = TrackTerrainInteraction.compute_slope_forces(
            200.0, 0.0, 0.0
        )
        assert abs(along) < 1e-6
        assert abs(normal - 200.0 * 9.81) < 0.1
        assert abs(lateral) < 1e-6

    def test_slope_forces_uphill(self):
        along, normal, lateral = TrackTerrainInteraction.compute_slope_forces(
            200.0, math.radians(30), 0.0
        )
        # 오르막: along > 0 (저항 방향)
        assert along > 0
        # 수직항력 < 중량
        assert normal < 200.0 * 9.81

    # 통합 계산 테스트
    def test_compute_effective_force(self, paddy_wet):
        traction, rolling, turning, sinkage = paddy_wet.compute_effective_force(
            800.0, 0.1, 0.3
        )
        assert traction > 0
        assert rolling > 0
        assert turning > 0
        assert sinkage > 0

    # 지형 변경 테스트
    def test_set_terrain(self, paved):
        s_paved = paved.compute_sinkage(800.0)
        paved.set_terrain(TerrainType.MUD)
        s_mud = paved.compute_sinkage(800.0)
        assert s_mud > s_paved
