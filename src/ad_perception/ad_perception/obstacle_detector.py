"""장애물 탐지 모듈.

YOLOv8n 기반 실시간 장애물 탐지.
- 정적/동적 장애물 분류
- 3D 위치 추정 (가정된 카 메라 파라미터 사용)
"""

import numpy as np
from typing import List, Optional, Tuple
from dataclasses import dataclass


@dataclass
class DetectedObstacle:
    """탐지된 장애물."""
    # 2D 바운 박스 (픽셀 좌표)
    bbox_2d: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    
    # 3D 위치 추정 (칵메라 좌표계, m)
    position_3d: Optional[Tuple[float, float, float]] = None
    
    # 분류
    class_label: str = "unknown"
    class_id: int = -1
    confidence: float = 0.0
    
    # 동적 정보
    is_moving: bool = False
    velocity: Optional[Tuple[float, float, float]] = None
    
    # 이미지 내 위치
    center_2d: Tuple[float, float] = (0.0, 0.0)  # (x, y)
    
    def get_distance(self) -> Optional[float]:
        """추정 거리 반환."""
        if self.position_3d is None:
            return None
        return np.linalg.norm(self.position_3d)


class ObstacleDetector:
    """YOLOv8 기반 장애물 탐지.
    
    농업 환경 특화:
    - 사람, 동물, 장애물 우선 탐지
    - 3D 위치 추정 (단안 카 메라 기반)
    """
    
    # 농업 환경에서 중요한 클래스
    PRIORITY_CLASSES = {
        0: "person",      # 사람
        16: "dog",        # 개
        17: "horse",      # 말
        18: "sheep",      # 양
        19: "cow",        # 소
        20: "elephant",   # 코끼리 (배제)
        21: "bear",       # 곰 (배제)
    }
    
    # 장애물로 간주할 COCO 클래스
    OBSTACLE_CLASSES = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,  # 사람/탈것/구조물
        14, 15, 16, 17, 18, 19, 20, 21, 22, 23,         # 동물
        24, 25, 26, 27, 28, 56, 57, 58, 59, 60, 61,     # 소지품/가구
        62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72,
    }
    
    def __init__(
        self,
        model_manager,
        model_name: str = "yolov8n",
        conf_threshold: float = 0.4,
        nms_threshold: float = 0.5,
        camera_matrix: Optional[np.ndarray] = None,
        camera_height: float = 1.2  # 카 메라 설치 높이 (m)
    ):
        """초기화.
        
        Args:
            model_manager: ModelManager 인스턴스
            model_name: 사용할 모델
            conf_threshold: 신뢰도 임계값
            nms_threshold: NMS IoU 임계값
            camera_matrix: 3x3 카 메라 내재 행렬
            camera_height: 카 메라 지면으로부터 높이
        """
        self.model_manager = model_manager
        self.model_name = model_name
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.camera_height = camera_height
        
        # 기본 카 메라 행렬 (640x480 가정)
        if camera_matrix is None:
            self.camera_matrix = np.array([
                [500, 0, 320],
                [0, 500, 240],
                [0, 0, 1]
            ], dtype=np.float32)
        else:
            self.camera_matrix = camera_matrix
        
        self._model = None
        self._prev_detections: List[DetectedObstacle] = []
    
    def _get_model(self):
        """Lazy loading."""
        if self._model is None:
            self._model = self.model_manager.load_model(self.model_name)
        return self._model
    
    def detect(self, image: np.ndarray) -> List[DetectedObstacle]:
        """장애물 탐지.
        
        Args:
            image: BGR 이미지
            
        Returns:
            DetectedObstacle 리스트
        """
        model = self._get_model()
        
        # YOLO 추론
        results = model(
            image,
            verbose=False,
            conf=self.conf_threshold,
            iou=self.nms_threshold
        )
        result = results[0]
        
        obstacles = []
        
        if result.boxes is None or len(result.boxes) == 0:
            return obstacles
        
        # 탐지 결과 처리
        for box in result.boxes:
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            
            # 장애물 클래스만 필터링
            if cls_id not in self.OBSTACLE_CLASSES:
                continue
            
            # 바운 박스
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, xyxy)
            
            # 클래스 이름
            class_name = result.names.get(cls_id, f"class_{cls_id}")
            
            # 중심점
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            
            # 3D 위치 추정
            pos_3d = self._estimate_3d_position(
                bbox=(x1, y1, x2, y2),
                image_shape=image.shape
            )
            
            obstacle = DetectedObstacle(
                bbox_2d=(x1, y1, x2, y2),
                position_3d=pos_3d,
                class_label=class_name,
                class_id=cls_id,
                confidence=conf,
                center_2d=(cx, cy)
            )
            obstacles.append(obstacle)
        
        # 우선순위 정렬 (거리 가까운 순, 사람/동물 우선)
        obstacles.sort(key=self._priority_score)
        
        self._prev_detections = obstacles
        return obstacles
    
    def _priority_score(self, obs: DetectedObstacle) -> float:
        """장애물 우선순위 점수 (낮을수록 우선)."""
        # 거리 점수 (가까울수록 우선)
        dist = obs.get_distance()
        dist_score = dist if dist is not None else 100.0
        
        # 클래스 우선순위
        priority = 0 if obs.class_id in self.PRIORITY_CLASSES else 1
        
        # 신뢰도 가중치
        conf_weight = 1.0 - obs.confidence
        
        return priority * 100 + dist_score + conf_weight * 10
    
    def _estimate_3d_position(
        self,
        bbox: Tuple[int, int, int, int],
        image_shape: Tuple[int, ...],
        assumed_height: float = 1.7  # 가정된 장애물 높이 (m)
    ) -> Optional[Tuple[float, float, float]]:
        """단안 카 메라로 3D 위치 추정 (간단한 방법).
        
        바운 박스 높이와 가정된 실제 높이로 거리 추정.
        """
        try:
            x1, y1, x2, y2 = bbox
            h, w = image_shape[:2]
            
            # 바운 박스 높이 (픽셀)
            bbox_height = y2 - y1
            if bbox_height < 5:
                return None
            
            # f_y (초점 거리 y)
            fy = self.camera_matrix[1, 1]
            
            # 거리 추정: Z = (f_y * H_real) / h_pixel
            # 카 메라 높이를 고려한 수정
            distance = (fy * assumed_height) / bbox_height
            
            # x, y 좌표 (지면 투영)
            cx = (x1 + x2) / 2
            cy = y2  # 바닥점 사용
            
            # 정규화된 좌표
            x_norm = (cx - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
            y_norm = (cy - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
            
            # 3D 좌표 (카 메라 좌표계)
            X = x_norm * distance
            Y = self.camera_height  # 지면 높이
            Z = distance
            
            return (X, Y, Z)
            
        except Exception as e:
            return None
    
    def get_closest_obstacle(
        self,
        obstacles: List[DetectedObstacle],
        max_distance: float = 10.0
    ) -> Optional[DetectedObstacle]:
        """가장 가까운 장애물 반환."""
        valid_obs = [
            obs for obs in obstacles
            if obs.get_distance() is not None and obs.get_distance() < max_distance
        ]
        
        if not valid_obs:
            return None
        
        return min(valid_obs, key=lambda o: o.get_distance())
    
    def filter_by_distance(
        self,
        obstacles: List[DetectedObstacle],
        min_dist: float = 0.0,
        max_dist: float = 20.0
    ) -> List[DetectedObstacle]:
        """거리로 장애물 필터링."""
        return [
            obs for obs in obstacles
            if obs.get_distance() is not None
            and min_dist <= obs.get_distance() <= max_dist
        ]
    
    def get_obstacle_grid(
        self,
        obstacles: List[DetectedObstacle],
        grid_size: Tuple[int, int] = (20, 20),  # (forward, lateral)
        cell_size: float = 0.5  # m
    ) -> np.ndarray:
        """장애물을 2D 그리드로 변환.
        
        Returns:
            occupancy grid (HxW), 0=free, 1=occupied
        """
        grid = np.zeros(grid_size, dtype=np.float32)
        
        for obs in obstacles:
            pos = obs.position_3d
            if pos is None:
                continue
            
            x, y, z = pos  # x: 좌우, y: 높이, z: 전방
            
            # 그리드 좌표 변환
            forward_idx = int(z / cell_size)
            lateral_idx = int((x + grid_size[1] * cell_size / 2) / cell_size)
            
            if 0 <= forward_idx < grid_size[0] and 0 <= lateral_idx < grid_size[1]:
                grid[forward_idx, lateral_idx] = max(
                    grid[forward_idx, lateral_idx],
                    obs.confidence
                )
        
        return grid
