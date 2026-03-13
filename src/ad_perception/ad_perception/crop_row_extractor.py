"""작물 행 추출 모듈.

세그멘테이션 마스크에서 작물 행을 라인으로 추출.
Hough Transform + RANSAC 기반 라인 검출.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class CropRow:
    """검출된 작물 행."""
    # 3D 공간의 선 (지면 좌표계)
    start_3d: Tuple[float, float, float]  # (x, y, z)
    end_3d: Tuple[float, float, float]
    
    # 이미지 좌표의 선
    start_2d: Tuple[float, float]  # (u, v)
    end_2d: Tuple[float, float]
    
    # 속성
    confidence: float = 0.0
    width_meters: float = 0.0  # 행 너비
    num_plants: int = 0  # 추정 식물 수
    
    def get_direction_vector(self) -> Tuple[float, float, float]:
        """행 방향 벡터."""
        dx = self.end_3d[0] - self.start_3d[0]
        dy = self.end_3d[1] - self.start_3d[1]
        dz = self.end_3d[2] - self.start_3d[2]
        length = np.sqrt(dx**2 + dy**2 + dz**2)
        if length < 1e-6:
            return (0, 0, 1)
        return (dx/length, dy/length, dz/length)
    
    def get_midpoint(self) -> Tuple[float, float, float]:
        """행 중점."""
        return (
            (self.start_3d[0] + self.end_3d[0]) / 2,
            (self.start_3d[1] + self.end_3d[1]) / 2,
            (self.start_3d[2] + self.end_3d[2]) / 2
        )
    
    def distance_to_point(self, x: float, z: float) -> float:
        """점과 행 사이의 수직 거리 (지면 투영)."""
        # 2D 라인 거리 (x-z 평면)
        x1, z1 = self.start_3d[0], self.start_3d[2]
        x2, z2 = self.end_3d[0], self.end_3d[2]
        
        # 점과 선 사이 거리 공식
        num = abs((z2 - z1) * x - (x2 - x1) * z + x2 * z1 - z2 * x1)
        den = np.sqrt((z2 - z1)**2 + (x2 - x1)**2)
        
        return num / den if den > 1e-6 else float('inf')


class CropRowExtractor:
    """작물 행 추출기.
    
    작물 필드 세그멘테이션 마스크에서 행 구조를 추출.
    """
    
    def __init__(
        self,
        camera_matrix: Optional[np.ndarray] = None,
        camera_height: float = 1.2,
        expected_row_spacing: float = 0.75,  # m (보통 75cm)
        min_row_length: float = 3.0,  # m
    ):
        """초기화.
        
        Args:
            camera_matrix: 3x3 카 메라 행렬
            camera_height: 카 메라 높이
            expected_row_spacing: 예상 행 간격
            min_row_length: 최소 행 길이
        """
        self.camera_height = camera_height
        self.expected_row_spacing = expected_row_spacing
        self.min_row_length = min_row_length
        
        if camera_matrix is None:
            self.camera_matrix = np.array([
                [500, 0, 320],
                [0, 500, 240],
                [0, 0, 1]
            ], dtype=np.float32)
        else:
            self.camera_matrix = camera_matrix
        
        self._prev_rows: List[CropRow] = []
    
    def extract_rows(
        self,
        crop_mask: np.ndarray,
        image: Optional[np.ndarray] = None
    ) -> List[CropRow]:
        """작물 마스크에서 행 추출.
        
        Args:
            crop_mask: HxW binary mask (작물 영역=True)
            image: 원본 이미지 (디버깅용)
            
        Returns:
            CropRow 리스트
        """
        if crop_mask.sum() < 100:  # 충분한 작물 영역 확인
            return []
        
        # 1. 전처리
        processed = self._preprocess_mask(crop_mask)
        
        # 2. Hough Transform으로 라인 검출
        lines_2d = self._detect_lines_hough(processed)
        
        if len(lines_2d) == 0:
            return []
        
        # 3. 클러스터링으로 병행 행 그룹화
        line_clusters = self._cluster_parallel_lines(lines_2d)
        
        # 4. 각 클러스터에서 대표 행 추출
        rows = []
        for cluster in line_clusters:
            row = self._extract_row_from_cluster(cluster, crop_mask.shape)
            if row is not None:
                rows.append(row)
        
        # 5. 거리 기반 필터링 (이상치 제거)
        rows = self._filter_rows_by_spacing(rows)
        
        # 6. 중심 행 정렬 (좌우 순서)
        rows.sort(key=lambda r: r.get_midpoint()[0])
        
        self._prev_rows = rows
        return rows
    
    def _preprocess_mask(self, mask: np.ndarray) -> np.ndarray:
        """마스크 전처리."""
        import cv2
        
        # uint8 변환
        if mask.dtype != np.uint8:
            mask = (mask > 0).astype(np.uint8) * 255
        
        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
    
    def _detect_lines_hough(
        self,
        mask: np.ndarray,
        min_line_length: int = 50,
        max_line_gap: int = 20
    ) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
        """Hough Transform으로 라인 검출."""
        import cv2
        
        # 엣지 검출
        edges = cv2.Canny(mask, 50, 150)
        
        # Hough Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap
        )
        
        if lines is None:
            return []
        
        # 리스트 변환
        line_list = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            line_list.append(((x1, y1), (x2, y2)))
        
        return line_list
    
    def _cluster_parallel_lines(
        self,
        lines: List[Tuple[Tuple[int, int], Tuple[int, int]]],
        angle_threshold: float = 15.0  # degrees
    ) -> List[List[Tuple[Tuple[int, int], Tuple[int, int]]]]:
        """유사 각도의 라인들을 클러스터링."""
        if len(lines) == 0:
            return []
        
        # 각도 계산
        angles = []
        for (x1, y1), (x2, y2) in lines:
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            # 수직 방향 선호 (작물 행은 보통 수직)
            if abs(angle) > 90:
                angle -= 180
            angles.append(angle)
        
        # 클러스터링 (간단한 그룹화)
        clusters = []
        used = set()
        
        for i, angle_i in enumerate(angles):
            if i in used:
                continue
            
            cluster = [lines[i]]
            used.add(i)
            
            for j, angle_j in enumerate(angles[i+1:], start=i+1):
                if j in used:
                    continue
                
                angle_diff = abs(angle_i - angle_j)
                angle_diff = min(angle_diff, 180 - angle_diff)
                
                if angle_diff < angle_threshold:
                    cluster.append(lines[j])
                    used.add(j)
            
            if len(cluster) >= 2:  # 최소 2개 라인
                clusters.append(cluster)
        
        return clusters
    
    def _extract_row_from_cluster(
        self,
        cluster: List[Tuple[Tuple[int, int], Tuple[int, int]]],
        image_shape: Tuple[int, int]
    ) -> Optional[CropRow]:
        """라인 클러스터에서 대표 행 추출."""
        # 모든 포인트 수집
        points_2d = []
        for (x1, y1), (x2, y2) in cluster:
            points_2d.extend([(x1, y1), (x2, y2)])
        
        if len(points_2d) < 4:
            return None
        
        # RANSAC 또는 최소제곱으로 라인 피팅
        pts = np.array(points_2d)
        
        # 주성분 분석으로 방향 결정
        mean = np.mean(pts, axis=0)
        centered = pts - mean
        _, _, vh = np.linalg.svd(centered)
        direction = vh[0]  # 주 방향
        
        # 이미지 경계까지 확장
        h, w = image_shape[:2]
        
        # 중심과 방향으로 라인 파라미터화
        dx, dy = direction
        
        # 이미지 경계 교차 계산
        # 단순화: y 방향으로 이미지 전체
        t_top = -mean[1] / dy if dy != 0 else 0
        t_bottom = (h - mean[1]) / dy if dy != 0 else 0
        
        start_2d = (mean[0] + t_top * dx, mean[1] + t_top * dy)
        end_2d = (mean[0] + t_bottom * dx, mean[1] + t_bottom * dy)
        
        # 3D 좌표 변환 (단순 가정)
        start_3d = self._image_to_ground(start_2d[0], start_2d[1], h)
        end_3d = self._image_to_ground(end_2d[0], end_2d[1], h)
        
        if start_3d is None or end_3d is None:
            return None
        
        # 행 길이 확인
        length = np.linalg.norm(np.array(end_3d) - np.array(start_3d))
        if length < self.min_row_length:
            return None
        
        return CropRow(
            start_3d=start_3d,
            end_3d=end_3d,
            start_2d=tuple(start_2d),
            end_2d=tuple(end_2d),
            confidence=min(1.0, len(cluster) / 10),
            width_meters=self.expected_row_spacing * 0.8
        )
    
    def _filter_rows_by_spacing(
        self,
        rows: List[CropRow],
        tolerance: float = 0.3
    ) -> List[CropRow]:
        """일정한 간격을 갖는 행만 필터링."""
        if len(rows) < 2:
            return rows
        
        # 인접 행 간 거리 계산
        distances = []
        for i in range(len(rows) - 1):
            mid1 = rows[i].get_midpoint()
            mid2 = rows[i + 1].get_midpoint()
            dist = abs(mid2[0] - mid1[0])  # x 방향 거리
            distances.append(dist)
        
        if not distances:
            return rows
        
        # 중앙값 기준 필터링
        median_spacing = np.median(distances)
        
        valid_indices = set()
        for i, dist in enumerate(distances):
            expected = self.expected_row_spacing
            if abs(dist - expected) < expected * tolerance:
                valid_indices.add(i)
                valid_indices.add(i + 1)
        
        # 연속된 행 그룹 찾기
        return [rows[i] for i in sorted(valid_indices)]
    
    def _image_to_ground(
        self,
        u: float,
        v: float,
        image_height: int
    ) -> Optional[Tuple[float, float, float]]:
        """이미지 좌표를 지면 좌표로 변환."""
        # 단순한 역투영 (가정: 평평한 지면)
        
        # 픽셀을 정규화 좌표로
        x_norm = (u - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
        y_norm = (v - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
        
        # 지면 가정: Y = camera_height
        # Z = camera_height / y_norm
        if y_norm >= 0:  # 수평선 아래
            Z = self.camera_height / (y_norm + 1e-6)
            X = x_norm * Z
            Y = 0  # 지면
            return (X, Y, Z)
        
        return None
    
    def get_center_row(self, rows: List[CropRow]) -> Optional[CropRow]:
        """중앙 행 반환 (차량 바로 앞)."""
        if not rows:
            return None
        
        # 가장 가까운 행 (Z값이 작은)
        return min(rows, key=lambda r: r.get_midpoint()[2])
    
    def get_navigation_target(
        self,
        rows: List[CropRow],
        look_ahead_distance: float = 3.0
    ) -> Optional[Tuple[float, float]]:
        """항법 목표점 반환 (x, z)."""
        if not rows:
            return None
        
        # 중앙 행 찾기
        center_row = self.get_center_row(rows)
        if center_row is None:
            return None
        
        # look_ahead_distance 만큼 앞의 점
        direction = center_row.get_direction_vector()
        start = center_row.start_3d
        
        target = (
            start[0] + direction[0] * look_ahead_distance,
            start[2] + direction[2] * look_ahead_distance
        )
        
        return target
