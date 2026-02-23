"""LiDAR 포인트클라우드 처리 모듈.

3D 포인트클라우드에서 지면을 분리하고, 장애물을 클러스터링하여
개별 장애물 객체 목록을 생성하는 인지 파이프라인을 제공한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False


@dataclass
class Obstacle:
    """감지된 장애물 정보를 담는 데이터 클래스.

    Attributes:
        center_x: 장애물 중심 X 좌표 (m).
        center_y: 장애물 중심 Y 좌표 (m).
        center_z: 장애물 중심 Z 좌표 (m).
        width: X축 방향 크기 (m).
        height: Z축 방향 크기 (m).
        depth: Y축 방향 크기 (m).
        num_points: 장애물을 구성하는 포인트 수.
        distance: 원점에서 장애물 중심까지의 수평 거리 (m).
    """

    center_x: float = 0.0
    center_y: float = 0.0
    center_z: float = 0.0
    width: float = 0.0
    height: float = 0.0
    depth: float = 0.0
    num_points: int = 0
    distance: float = field(init=False)

    def __post_init__(self) -> None:
        """수평 거리를 자동 계산한다."""
        self.distance = math.sqrt(self.center_x ** 2 + self.center_y ** 2)


class LidarProcessor:
    """LiDAR 포인트클라우드 기반 장애물 감지 파이프라인.

    Ray Ground Filter로 지면을 분리한 뒤, Euclidean Clustering으로
    비지면 포인트를 개별 장애물로 그룹화한다.
    """

    def __init__(
        self,
        ground_height_threshold: float = 0.15,
        ground_angle_threshold: float = 0.17,
        cluster_distance: float = 0.5,
        min_cluster_size: int = 5,
        max_cluster_size: int = 500,
    ) -> None:
        """LidarProcessor를 초기화한다.

        Args:
            ground_height_threshold: 지면 분리 높이 임계값 (m).
            ground_angle_threshold: 지면 분리 수직 각도 임계값 (rad).
            cluster_distance: 클러스터링 기본 거리 임계값 (m).
            min_cluster_size: 클러스터 최소 포인트 수.
            max_cluster_size: 클러스터 최대 포인트 수.
        """
        self._ground_height_threshold = ground_height_threshold
        self._ground_angle_threshold = ground_angle_threshold
        self._cluster_distance = cluster_distance
        self._min_cluster_size = min_cluster_size
        self._max_cluster_size = max_cluster_size

    # ------------------------------------------------------------------
    # 지면 분리
    # ------------------------------------------------------------------

    def ground_filter(
        self,
        points: "np.ndarray",
        height_threshold: float | None = None,
        angle_threshold: float | None = None,
    ) -> Tuple["np.ndarray", "np.ndarray"]:
        """Ray Ground Filter를 이용하여 지면과 장애물 포인트를 분리한다.

        각 포인트의 수평 거리와 높이로부터 수직 각도를 계산하고,
        인접 포인트 간 각도 변화가 임계값 이하이면 지면으로 판정한다.

        Args:
            points: (N, 3) 형태의 numpy 배열 (x, y, z).
            height_threshold: 지면 높이 임계값 (m). None이면 초기값 사용.
            angle_threshold: 수직 각도 임계값 (rad). None이면 초기값 사용.

        Returns:
            (ground_points, obstacle_points) 튜플. 각각 (M, 3), (K, 3) numpy 배열.
        """
        if not _HAS_NUMPY:
            raise RuntimeError("numpy가 설치되어 있지 않습니다.")

        if points.shape[0] == 0:
            empty = np.empty((0, 3), dtype=np.float64)
            return empty.copy(), empty.copy()

        h_thresh = height_threshold if height_threshold is not None else self._ground_height_threshold
        a_thresh = angle_threshold if angle_threshold is not None else self._ground_angle_threshold

        # 수평 거리 계산
        xy_dist = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
        z_vals = points[:, 2]

        # 빔별 방위각 기준으로 섹터(빔) 분할 (360도를 1도 단위 360섹터)
        azimuth = np.arctan2(points[:, 1], points[:, 0])  # -pi ~ pi
        sector_idx = ((np.degrees(azimuth) + 180.0) % 360.0).astype(int)
        sector_idx = np.clip(sector_idx, 0, 359)

        is_ground = np.zeros(points.shape[0], dtype=bool)

        for sector in range(360):
            mask = sector_idx == sector
            if not np.any(mask):
                continue

            indices = np.where(mask)[0]
            dists = xy_dist[indices]
            heights = z_vals[indices]

            # 수평 거리 기준 정렬
            order = np.argsort(dists)
            sorted_indices = indices[order]
            sorted_dists = dists[order]
            sorted_heights = heights[order]

            # 첫 포인트: 높이 임계값 이하이면 지면
            if sorted_heights[0] < h_thresh:
                is_ground[sorted_indices[0]] = True

            # 인접 포인트 간 수직 각도 변화 비교
            for i in range(1, len(sorted_indices)):
                d_dist = sorted_dists[i] - sorted_dists[i - 1]
                d_height = sorted_heights[i] - sorted_heights[i - 1]

                if d_dist < 1e-6:
                    # 거리가 거의 같으면 이전 판정 따름
                    is_ground[sorted_indices[i]] = is_ground[sorted_indices[i - 1]]
                    continue

                angle = math.atan2(abs(d_height), d_dist)

                if angle < a_thresh and abs(sorted_heights[i]) < h_thresh + 0.3:
                    is_ground[sorted_indices[i]] = True

        ground_points = points[is_ground]
        obstacle_points = points[~is_ground]
        return ground_points, obstacle_points

    # ------------------------------------------------------------------
    # 클러스터링
    # ------------------------------------------------------------------

    def cluster_obstacles(
        self,
        points: "np.ndarray",
        distance_threshold: float | None = None,
        min_cluster_size: int | None = None,
        max_cluster_size: int | None = None,
    ) -> List[Obstacle]:
        """Euclidean Clustering으로 장애물 포인트를 그룹화한다.

        센서에서의 거리에 따라 클러스터링 반경을 적응적으로 조절한다.
        가까운 포인트는 좁은 반경, 먼 포인트는 넓은 반경을 사용한다.

        Args:
            points: (N, 3) 형태의 장애물 포인트 배열.
            distance_threshold: 기본 클러스터링 거리 (m). None이면 초기값 사용.
            min_cluster_size: 최소 클러스터 크기. None이면 초기값 사용.
            max_cluster_size: 최대 클러스터 크기. None이면 초기값 사용.

        Returns:
            감지된 Obstacle 객체 리스트.
        """
        if not _HAS_NUMPY:
            raise RuntimeError("numpy가 설치되어 있지 않습니다.")

        d_thresh = distance_threshold if distance_threshold is not None else self._cluster_distance
        min_sz = min_cluster_size if min_cluster_size is not None else self._min_cluster_size
        max_sz = max_cluster_size if max_cluster_size is not None else self._max_cluster_size

        n = points.shape[0]
        if n == 0:
            return []

        # 각 포인트의 센서 원점 기준 수평 거리 (적응형 반경 계산용)
        point_dists = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)

        visited = np.zeros(n, dtype=bool)
        clusters: List[List[int]] = []

        for i in range(n):
            if visited[i]:
                continue

            # BFS로 클러스터 확장
            queue = [i]
            visited[i] = True
            cluster: List[int] = []

            while queue:
                idx = queue.pop(0)
                cluster.append(idx)

                if len(cluster) > max_sz:
                    break

                # 적응형 거리: 기본 반경 * (1 + 원점거리 / 20)
                adaptive_dist = d_thresh * (1.0 + point_dists[idx] / 20.0)

                # 현재 포인트와의 유클리드 거리 계산
                diff = points - points[idx]
                dists_sq = diff[:, 0] ** 2 + diff[:, 1] ** 2 + diff[:, 2] ** 2
                neighbors = np.where((dists_sq < adaptive_dist ** 2) & (~visited))[0]

                for nb in neighbors:
                    visited[nb] = True
                    queue.append(nb)

            if min_sz <= len(cluster) <= max_sz:
                clusters.append(cluster)

        # 클러스터 → Obstacle 변환
        obstacles: List[Obstacle] = []
        for cluster in clusters:
            cluster_points = points[cluster]
            min_pt = cluster_points.min(axis=0)
            max_pt = cluster_points.max(axis=0)
            center = (min_pt + max_pt) / 2.0

            obstacles.append(Obstacle(
                center_x=float(center[0]),
                center_y=float(center[1]),
                center_z=float(center[2]),
                width=float(max_pt[0] - min_pt[0]),
                height=float(max_pt[2] - min_pt[2]),
                depth=float(max_pt[1] - min_pt[1]),
                num_points=len(cluster),
            ))

        # 거리 기준 정렬 (가까운 장애물 우선)
        obstacles.sort(key=lambda o: o.distance)
        return obstacles

    # ------------------------------------------------------------------
    # 전체 파이프라인
    # ------------------------------------------------------------------

    def process(self, points: "np.ndarray") -> List[Obstacle]:
        """지면 분리 후 클러스터링을 수행하는 전체 인지 파이프라인.

        Args:
            points: (N, 3) 형태의 포인트클라우드 배열 (x, y, z).

        Returns:
            감지된 Obstacle 객체 리스트 (거리순 정렬).
        """
        if not _HAS_NUMPY:
            raise RuntimeError("numpy가 설치되어 있지 않습니다.")

        if points.shape[0] == 0:
            return []

        _ground, obstacle_points = self.ground_filter(points)
        obstacles = self.cluster_obstacles(obstacle_points)
        return obstacles
