"""LidarProcessor 단위 테스트."""

import math
import pytest

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False

from ad_core.lidar_processor import LidarProcessor, Obstacle

pytestmark = pytest.mark.skipif(not _HAS_NUMPY, reason="numpy 필요")


# ── Obstacle 데이터 클래스 ────────────────────────────────────

class TestObstacle:
    def test_distance_auto_calculated(self):
        obs = Obstacle(
            center_x=3.0, center_y=4.0, center_z=0.0,
            width=1.0, height=1.0, depth=1.0, num_points=10,
        )
        assert abs(obs.distance - 5.0) < 1e-6

    def test_zero_distance(self):
        obs = Obstacle(
            center_x=0.0, center_y=0.0, center_z=0.0,
            width=1.0, height=1.0, depth=1.0, num_points=5,
        )
        assert abs(obs.distance) < 1e-6


# ── LidarProcessor ────────────────────────────────────────────

class TestLidarProcessor:
    @pytest.fixture
    def processor(self):
        return LidarProcessor(
            ground_height_threshold=0.15,
            cluster_distance=0.5,
            min_cluster_size=3,
            max_cluster_size=500,
        )

    def test_ground_filter_separates_ground(self, processor):
        """지면 점과 장애물 점을 분리한다."""
        # 지면 점들 (z ≈ 0)
        ground_pts = np.random.uniform(-10, 10, (100, 3))
        ground_pts[:, 2] = np.random.uniform(-0.05, 0.05, 100)

        # 장애물 점들 (z > 0.3)
        obstacle_pts = np.random.uniform(2, 5, (30, 3))
        obstacle_pts[:, 2] = np.random.uniform(0.5, 2.0, 30)

        all_pts = np.vstack([ground_pts, obstacle_pts])
        ground, non_ground = processor.ground_filter(all_pts)

        assert len(ground) > 0
        assert len(non_ground) > 0
        # 대부분의 지면 점이 ground로 분류되어야 함
        assert len(ground) > 50

    def test_ground_filter_empty_input(self, processor):
        """빈 입력 처리."""
        empty = np.empty((0, 3))
        ground, non_ground = processor.ground_filter(empty)
        assert len(ground) == 0
        assert len(non_ground) == 0

    def test_cluster_obstacles_finds_clusters(self, processor):
        """가까운 점들이 하나의 클러스터로 묶인다."""
        # 클러스터 1: (5, 0, 1) 근처
        c1 = np.random.normal(loc=[5, 0, 1], scale=0.1, size=(20, 3))
        # 클러스터 2: (10, 3, 1) 근처
        c2 = np.random.normal(loc=[10, 3, 1], scale=0.1, size=(15, 3))

        points = np.vstack([c1, c2])
        obstacles = processor.cluster_obstacles(points)

        assert len(obstacles) == 2

    def test_cluster_obstacles_ignores_small_clusters(self, processor):
        """min_cluster_size 미만 점은 무시."""
        # 2개 점만 (min=3이므로 무시)
        points = np.array([[1, 0, 1], [1.1, 0, 1]])
        obstacles = processor.cluster_obstacles(points)
        assert len(obstacles) == 0

    def test_cluster_obstacles_empty_input(self, processor):
        """빈 입력 → 빈 결과."""
        empty = np.empty((0, 3))
        obstacles = processor.cluster_obstacles(empty)
        assert len(obstacles) == 0

    def test_process_full_pipeline(self, processor):
        """전체 파이프라인: 지면 필터 + 클러스터링."""
        # 지면
        ground = np.random.uniform(-10, 10, (200, 3))
        ground[:, 2] = np.random.uniform(-0.05, 0.05, 200)

        # 장애물 1
        obs1 = np.random.normal(loc=[5, 0, 1], scale=0.1, size=(30, 3))
        # 장애물 2
        obs2 = np.random.normal(loc=[8, -3, 0.8], scale=0.1, size=(25, 3))

        all_pts = np.vstack([ground, obs1, obs2])
        obstacles = processor.process(all_pts)

        assert len(obstacles) >= 1  # 최소 1개 장애물 감지
        for obs in obstacles:
            assert isinstance(obs, Obstacle)
            assert obs.num_points >= 3
            assert obs.distance >= 0

    def test_obstacle_sizes_reasonable(self, processor):
        """감지된 장애물의 크기가 합리적."""
        pts = np.random.normal(loc=[5, 0, 1], scale=0.2, size=(50, 3))
        obstacles = processor.cluster_obstacles(pts)

        assert len(obstacles) == 1
        obs = obstacles[0]
        assert obs.width < 3.0  # 합리적 크기
        assert obs.height < 3.0
        assert obs.depth < 3.0
        assert abs(obs.center_x - 5.0) < 1.0
