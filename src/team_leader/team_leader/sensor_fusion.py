"""Camera + LiDAR Late Fusion 모듈.

LiDAR 장애물 감지 결과와 카메라 객체 탐지 결과를 후기 융합(late fusion)하여
통합된 인지 결과를 생성한다.

핀홀 카메라 모델을 사용하여 LiDAR 3D 좌표를 카메라 이미지 평면에 투영한 후,
투영된 위치와 카메라 바운딩 박스 간의 매칭을 수행한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False

from team_leader.lidar_processor import Obstacle
from team_leader.camera_detector import Detection


# ======================================================================
# 데이터 클래스
# ======================================================================

@dataclass
class FusedObject:
    """융합된 객체 정보를 담는 데이터 클래스.

    LiDAR 장애물과 카메라 탐지 결과를 결합하여
    클래스, 3D 위치, 크기, 신뢰도, 소스 정보를 제공한다.

    Attributes:
        class_name: 객체 클래스 이름 (예: "person", "car", "unknown").
        x: 객체 중심 X 좌표 (m, LiDAR 좌표계).
        y: 객체 중심 Y 좌표 (m, LiDAR 좌표계).
        z: 객체 중심 Z 좌표 (m, LiDAR 좌표계).
        width: X축 방향 크기 (m).
        height: Z축 방향 크기 (m).
        depth: Y축 방향 크기 (m).
        confidence: 융합된 신뢰도 (0.0 ~ 1.0).
        source: 데이터 소스 ("lidar", "camera", "fused").
        distance: 원점에서 객체 중심까지의 수평 거리 (m).
        lidar_obstacle: 원본 LiDAR 장애물 (있는 경우).
        camera_detection: 원본 카메라 탐지 결과 (있는 경우).
    """

    class_name: str = "unknown"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    width: float = 0.0
    height: float = 0.0
    depth: float = 0.0
    confidence: float = 0.0
    source: str = "lidar"  # "lidar" | "camera" | "fused"
    distance: float = field(init=False)
    lidar_obstacle: Optional[Obstacle] = field(default=None, repr=False)
    camera_detection: Optional[Detection] = field(default=None, repr=False)

    def __post_init__(self) -> None:
        """수평 거리를 자동 계산한다."""
        self.distance = math.sqrt(self.x ** 2 + self.y ** 2)


# ======================================================================
# 카메라 파라미터
# ======================================================================

@dataclass
class CameraIntrinsics:
    """카메라 내부 파라미터 (핀홀 카메라 모델).

    Attributes:
        fx: X축 초점 거리 (픽셀).
        fy: Y축 초점 거리 (픽셀).
        cx: 주점(principal point) X 좌표 (픽셀).
        cy: 주점(principal point) Y 좌표 (픽셀).
        image_width: 이미지 너비 (픽셀).
        image_height: 이미지 높이 (픽셀).
    """

    fx: float = 640.0
    fy: float = 640.0
    cx: float = 640.0
    cy: float = 360.0
    image_width: int = 1280
    image_height: int = 720

    def to_matrix(self) -> "np.ndarray":
        """3x3 카메라 내부 파라미터 행렬을 반환한다.

        Returns:
            3x3 numpy 배열 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
        """
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)


@dataclass
class CameraExtrinsics:
    """카메라 외부 파라미터 (LiDAR -> 카메라 변환).

    LiDAR 좌표계에서 카메라 좌표계로의 강체 변환을 정의한다.
    회전은 3x3 행렬, 이동은 3D 벡터로 표현한다.

    기본값은 LiDAR가 차량 상단, 카메라가 전방을 향하는 일반적 배치를 가정한다.
    - LiDAR 좌표계: X=전방, Y=좌측, Z=상방
    - 카메라 좌표계: X=우측, Y=하방, Z=전방

    Attributes:
        rotation: 3x3 회전 행렬 (LiDAR -> 카메라).
        translation: 3D 이동 벡터 [tx, ty, tz] (m).
    """

    rotation: Optional["np.ndarray"] = None
    translation: Optional["np.ndarray"] = None

    def __post_init__(self) -> None:
        """기본 외부 파라미터를 설정한다."""
        if _HAS_NUMPY:
            if self.rotation is None:
                # LiDAR(X=전방, Y=좌측, Z=상방) -> Camera(X=우측, Y=하방, Z=전방)
                self.rotation = np.array([
                    [0.0, -1.0,  0.0],
                    [0.0,  0.0, -1.0],
                    [1.0,  0.0,  0.0],
                ], dtype=np.float64)
            if self.translation is None:
                # LiDAR와 카메라 간 상대 위치 (기본: 동일 위치)
                self.translation = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    def to_matrix(self) -> "np.ndarray":
        """4x4 외부 파라미터 행렬 (동차 변환 행렬)을 반환한다.

        Returns:
            4x4 numpy 배열 [[R, t], [0, 0, 0, 1]].
        """
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = self.rotation
        mat[:3, 3] = self.translation
        return mat


# ======================================================================
# 센서 융합
# ======================================================================

class SensorFusion:
    """Camera + LiDAR Late Fusion을 수행하는 센서 융합 클래스.

    LiDAR의 정확한 3D 위치 정보와 카메라의 풍부한 의미론적(semantic) 분류를
    결합하여 더 신뢰성 높은 인지 결과를 생성한다.

    융합 과정:
        1. LiDAR 장애물의 3D 중심을 카메라 이미지 평면에 투영
        2. 투영된 2D 좌표와 카메라 바운딩 박스 간 매칭
        3. 매칭된 쌍: LiDAR 거리 + 카메라 클래스 결합, 신뢰도 상승
        4. 미매칭 LiDAR: "unknown" 클래스로 유지
        5. 미매칭 카메라: 거리 추정 없이 camera-only로 유지
    """

    def __init__(
        self,
        intrinsics: Optional[CameraIntrinsics] = None,
        extrinsics: Optional[CameraExtrinsics] = None,
        matching_iou_threshold: float = 0.1,
        matching_distance_threshold: float = 80.0,
        fused_confidence_bonus: float = 0.15,
    ) -> None:
        """SensorFusion을 초기화한다.

        Args:
            intrinsics: 카메라 내부 파라미터. None이면 기본값 사용.
            extrinsics: 카메라 외부 파라미터. None이면 기본값 사용.
            matching_iou_threshold: IoU 기반 매칭 최소 임계값.
            matching_distance_threshold: 픽셀 거리 기반 매칭 최대 임계값 (px).
            fused_confidence_bonus: 융합 시 신뢰도 보너스 (0.0 ~ 1.0).
        """
        self._intrinsics = intrinsics if intrinsics is not None else CameraIntrinsics()
        self._extrinsics = extrinsics if extrinsics is not None else CameraExtrinsics()
        self._iou_threshold = matching_iou_threshold
        self._distance_threshold = matching_distance_threshold
        self._fused_bonus = fused_confidence_bonus

    # ------------------------------------------------------------------
    # 속성
    # ------------------------------------------------------------------

    @property
    def intrinsics(self) -> CameraIntrinsics:
        """카메라 내부 파라미터를 반환한다."""
        return self._intrinsics

    @intrinsics.setter
    def intrinsics(self, value: CameraIntrinsics) -> None:
        """카메라 내부 파라미터를 설정한다."""
        self._intrinsics = value

    @property
    def extrinsics(self) -> CameraExtrinsics:
        """카메라 외부 파라미터를 반환한다."""
        return self._extrinsics

    @extrinsics.setter
    def extrinsics(self, value: CameraExtrinsics) -> None:
        """카메라 외부 파라미터를 설정한다."""
        self._extrinsics = value

    # ------------------------------------------------------------------
    # 3D -> 2D 투영
    # ------------------------------------------------------------------

    def project_to_image(
        self,
        point_3d: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float]]:
        """LiDAR 좌표계의 3D 점을 카메라 이미지 평면에 투영한다.

        핀홀 카메라 모델을 사용하여 3D 점을 2D 픽셀 좌표로 변환한다.
        카메라 뒤쪽에 있는 점이나 이미지 범위를 벗어나는 점은 None을 반환한다.

        Args:
            point_3d: LiDAR 좌표계의 (x, y, z) 좌표 (m).

        Returns:
            이미지 평면의 (u, v) 픽셀 좌표, 또는 투영 불가 시 None.
        """
        if not _HAS_NUMPY:
            return None

        # LiDAR 좌표를 카메라 좌표로 변환
        p_lidar = np.array([point_3d[0], point_3d[1], point_3d[2]], dtype=np.float64)
        R = self._extrinsics.rotation
        t = self._extrinsics.translation
        p_camera = R @ p_lidar + t

        # 카메라 앞에 있는지 확인 (Z_camera > 0)
        if p_camera[2] <= 0.0:
            return None

        # 핀홀 모델: u = fx * X/Z + cx, v = fy * Y/Z + cy
        u = self._intrinsics.fx * (p_camera[0] / p_camera[2]) + self._intrinsics.cx
        v = self._intrinsics.fy * (p_camera[1] / p_camera[2]) + self._intrinsics.cy

        # 이미지 범위 확인 (약간의 마진 허용)
        margin = 50.0
        if (u < -margin or u > self._intrinsics.image_width + margin
                or v < -margin or v > self._intrinsics.image_height + margin):
            return None

        return (float(u), float(v))

    def project_obstacle_bbox(
        self,
        obstacle: Obstacle,
    ) -> Optional[Tuple[float, float, float, float]]:
        """LiDAR 장애물의 3D 바운딩 박스를 이미지 평면에 투영한다.

        장애물의 8개 꼭짓점을 모두 투영하여 2D 바운딩 박스를 생성한다.

        Args:
            obstacle: LiDAR 장애물 객체.

        Returns:
            이미지 평면의 (x1, y1, x2, y2) 바운딩 박스, 또는 투영 불가 시 None.
        """
        if not _HAS_NUMPY:
            return None

        # 장애물의 8개 꼭짓점 계산
        half_w = obstacle.width / 2.0
        half_h = obstacle.height / 2.0
        half_d = obstacle.depth / 2.0

        corners = [
            (obstacle.center_x + dx, obstacle.center_y + dy, obstacle.center_z + dz)
            for dx in (-half_w, half_w)
            for dy in (-half_d, half_d)
            for dz in (-half_h, half_h)
        ]

        # 각 꼭짓점을 이미지 평면에 투영
        projected_points: List[Tuple[float, float]] = []
        for corner in corners:
            proj = self.project_to_image(corner)
            if proj is not None:
                projected_points.append(proj)

        if len(projected_points) < 2:
            # 최소 2개 점이 투영되어야 bbox 생성 가능
            return None

        us = [p[0] for p in projected_points]
        vs = [p[1] for p in projected_points]

        x1 = max(0.0, min(us))
        y1 = max(0.0, min(vs))
        x2 = min(float(self._intrinsics.image_width), max(us))
        y2 = min(float(self._intrinsics.image_height), max(vs))

        # 유효한 bbox인지 확인
        if x2 <= x1 or y2 <= y1:
            return None

        return (x1, y1, x2, y2)

    # ------------------------------------------------------------------
    # 매칭
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_iou(
        box_a: Tuple[float, float, float, float],
        box_b: Tuple[float, float, float, float],
    ) -> float:
        """두 바운딩 박스 간 IoU(Intersection over Union)를 계산한다.

        Args:
            box_a: (x1, y1, x2, y2) 바운딩 박스.
            box_b: (x1, y1, x2, y2) 바운딩 박스.

        Returns:
            IoU 값 (0.0 ~ 1.0).
        """
        # 교차 영역 계산
        inter_x1 = max(box_a[0], box_b[0])
        inter_y1 = max(box_a[1], box_b[1])
        inter_x2 = min(box_a[2], box_b[2])
        inter_y2 = min(box_a[3], box_b[3])

        inter_area = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)

        if inter_area == 0.0:
            return 0.0

        # 합집합 영역 계산
        area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
        union_area = area_a + area_b - inter_area

        if union_area <= 0.0:
            return 0.0

        return inter_area / union_area

    @staticmethod
    def _compute_center_distance(
        box_a: Tuple[float, float, float, float],
        box_b: Tuple[float, float, float, float],
    ) -> float:
        """두 바운딩 박스 중심 간 유클리드 거리를 계산한다.

        Args:
            box_a: (x1, y1, x2, y2) 바운딩 박스.
            box_b: (x1, y1, x2, y2) 바운딩 박스.

        Returns:
            중심 간 유클리드 거리 (픽셀).
        """
        cx_a = (box_a[0] + box_a[2]) / 2.0
        cy_a = (box_a[1] + box_a[3]) / 2.0
        cx_b = (box_b[0] + box_b[2]) / 2.0
        cy_b = (box_b[1] + box_b[3]) / 2.0
        return math.sqrt((cx_a - cx_b) ** 2 + (cy_a - cy_b) ** 2)

    def _match_obstacle_to_detection(
        self,
        lidar_bbox: Tuple[float, float, float, float],
        detections: List[Detection],
        used_detections: set,
    ) -> Optional[int]:
        """투영된 LiDAR bbox와 가장 잘 매칭되는 카메라 탐지를 찾는다.

        IoU 기반 매칭을 우선 시도하고, IoU가 임계값 이하이면
        중심 거리 기반 매칭으로 폴백(fallback)한다.

        Args:
            lidar_bbox: 투영된 LiDAR 장애물 bbox (x1, y1, x2, y2).
            detections: 카메라 탐지 결과 리스트.
            used_detections: 이미 매칭된 탐지 인덱스 집합.

        Returns:
            최적 매칭 탐지 인덱스, 또는 매칭 실패 시 None.
        """
        best_idx: Optional[int] = None
        best_iou = 0.0
        best_dist = float('inf')

        for i, det in enumerate(detections):
            if i in used_detections:
                continue

            det_bbox = (det.bbox_x1, det.bbox_y1, det.bbox_x2, det.bbox_y2)

            # IoU 계산
            iou = self._compute_iou(lidar_bbox, det_bbox)
            if iou > best_iou:
                best_iou = iou
                best_idx = i

            # 중심 거리 계산 (IoU 폴백용)
            dist = self._compute_center_distance(lidar_bbox, det_bbox)
            if dist < best_dist:
                best_dist = dist
                if best_iou < self._iou_threshold:
                    # IoU 매칭이 안 되면 거리 기반으로 후보 갱신
                    best_idx = i

        # 매칭 판정
        if best_iou >= self._iou_threshold:
            return best_idx

        if best_dist <= self._distance_threshold:
            return best_idx

        return None

    # ------------------------------------------------------------------
    # 융합
    # ------------------------------------------------------------------

    def fuse(
        self,
        obstacles: List[Obstacle],
        detections: List[Detection],
        camera_info: Optional[Dict] = None,
    ) -> List[FusedObject]:
        """LiDAR 장애물과 카메라 탐지 결과를 융합한다.

        융합 전략:
            1. 각 LiDAR 장애물을 이미지 평면에 투영
            2. 투영된 bbox와 카메라 탐지 bbox 간 IoU/거리 기반 매칭
            3. 매칭 성공: LiDAR 3D 위치 + 카메라 클래스 결합, 신뢰도 보너스 적용
            4. 미매칭 LiDAR: "unknown" 클래스로 LiDAR-only 결과 생성
            5. 미매칭 카메라: 거리 정보 없이 camera-only 결과 생성

        Args:
            obstacles: LiDAR 장애물 리스트 (Obstacle 객체).
            detections: 카메라 탐지 리스트 (Detection 객체).
            camera_info: 런타임 카메라 정보 딕셔너리 (선택).
                         "image_width", "image_height" 키를 통해
                         내부 파라미터를 동적으로 업데이트할 수 있다.

        Returns:
            FusedObject 리스트 (거리순 정렬).
        """
        # 런타임 카메라 정보 적용
        if camera_info is not None:
            if "image_width" in camera_info:
                self._intrinsics.image_width = int(camera_info["image_width"])
            if "image_height" in camera_info:
                self._intrinsics.image_height = int(camera_info["image_height"])

        fused_objects: List[FusedObject] = []
        used_detections: set = set()

        # --- 단계 1: LiDAR 장애물을 투영하여 카메라 탐지와 매칭 ---
        for obstacle in obstacles:
            lidar_bbox = self.project_obstacle_bbox(obstacle)

            if lidar_bbox is not None and len(detections) > 0:
                match_idx = self._match_obstacle_to_detection(
                    lidar_bbox, detections, used_detections,
                )
            else:
                match_idx = None

            if match_idx is not None:
                # --- 매칭 성공: LiDAR + Camera 융합 ---
                det = detections[match_idx]
                used_detections.add(match_idx)

                # 신뢰도: 카메라 탐지 신뢰도에 융합 보너스 적용
                fused_conf = min(1.0, det.confidence + self._fused_bonus)

                fused_objects.append(FusedObject(
                    class_name=det.class_name,
                    x=obstacle.center_x,
                    y=obstacle.center_y,
                    z=obstacle.center_z,
                    width=obstacle.width,
                    height=obstacle.height,
                    depth=obstacle.depth,
                    confidence=fused_conf,
                    source="fused",
                    lidar_obstacle=obstacle,
                    camera_detection=det,
                ))
            else:
                # --- 미매칭 LiDAR: unknown 클래스로 유지 ---
                # 포인트 수 기반 간이 신뢰도 (5~500 포인트 범위)
                lidar_conf = min(0.7, max(0.2, obstacle.num_points / 200.0))

                fused_objects.append(FusedObject(
                    class_name="unknown",
                    x=obstacle.center_x,
                    y=obstacle.center_y,
                    z=obstacle.center_z,
                    width=obstacle.width,
                    height=obstacle.height,
                    depth=obstacle.depth,
                    confidence=lidar_conf,
                    source="lidar",
                    lidar_obstacle=obstacle,
                    camera_detection=None,
                ))

        # --- 단계 2: 미매칭 카메라 탐지를 camera-only로 추가 ---
        for i, det in enumerate(detections):
            if i in used_detections:
                continue

            # 카메라 전용: 3D 위치/크기 정보 없음 (0으로 유지)
            fused_objects.append(FusedObject(
                class_name=det.class_name,
                x=0.0,
                y=0.0,
                z=0.0,
                width=0.0,
                height=0.0,
                depth=0.0,
                confidence=det.confidence,
                source="camera",
                lidar_obstacle=None,
                camera_detection=det,
            ))

        # 거리순 정렬 (fused/lidar 우선, camera-only는 뒤로)
        fused_objects.sort(key=lambda o: (o.source == "camera", o.distance))

        return fused_objects
