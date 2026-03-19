"""인지 시스템 통합 관리자.

두 가지 인지 파이프라인을 선택적으로 사용할 수 있는 매니저:
- Learned: YOLOv8 기반 (카메라 전용) — 지형 세그멘테이션 + 장애물 탐지 + 작물 행 추출
- Traditional: LiDAR + Camera + SensorFusion + SemanticSegmenter

HybridE2ENode에 임베딩되어 런타임 파라미터로 모드 전환 가능.
"""

import time
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, Imu, NavSatFix

from ad_core.hybrid_e2e_types import (
    PerceptionFeatures,
    TerrainClass,
    Line3D,
    Object3D,
)
from ad_core.datatypes import Pose2D

# Traditional perception (Camera-only since C64: LiDAR removed)
from ad_core.camera_detector import CameraDetector, Detection
from ad_core.semantic_segmenter import SemanticSegmenter, SegmentationClass

# Learned perception
from .terrain_segmentation import TerrainSegmenter, TerrainLabel
from .obstacle_detector import ObstacleDetector, DetectedObstacle
from .crop_row_extractor import CropRowExtractor
from .model_manager import ModelManager


class PerceptionMode(Enum):
    """인지 모드."""
    LEARNED = "learned"          # YOLOv8 (카메라 전용)
    TRADITIONAL = "traditional"  # LiDAR + Camera + Fusion
    AUTO = "auto"                # learned 시도 → 실패 시 traditional fallback


# TerrainLabel → TerrainClass 매핑 (Learned용)
_TERRAIN_LABEL_MAP = {
    TerrainLabel.CROP_FIELD: TerrainClass.CROP_FIELD,
    TerrainLabel.GRASS: TerrainClass.GRASS,
    TerrainLabel.DIRT_ROAD: TerrainClass.DIRT_ROAD,
    TerrainLabel.PAVED: TerrainClass.PAVED,
    TerrainLabel.MUD: TerrainClass.MUD,
    TerrainLabel.OBSTACLE: TerrainClass.OBSTACLE,
    TerrainLabel.UNKNOWN: TerrainClass.CROP_FIELD,
}

# SegmentationClass → TerrainClass 매핑 (Traditional용)
_SEG_CLASS_MAP = {
    SegmentationClass.BACKGROUND: TerrainClass.CROP_FIELD,
    SegmentationClass.ROAD: TerrainClass.PAVED,
    SegmentationClass.DIRT_PATH: TerrainClass.DIRT_ROAD,
    SegmentationClass.GRASS: TerrainClass.GRASS,
    SegmentationClass.CROP: TerrainClass.CROP_FIELD,
    SegmentationClass.OBSTACLE: TerrainClass.OBSTACLE,
    SegmentationClass.WATER: TerrainClass.MUD,
    SegmentationClass.MUD: TerrainClass.MUD,
}


class PerceptionManager:
    """두 인지 시스템을 통합 관리.

    HybridE2ENode의 _control_loop에서 process()를 호출하면
    현재 모드에 따라 적절한 인지 파이프라인을 실행하고
    PerceptionFeatures를 반환한다.
    """

    def __init__(
        self,
        node: Node,
        mode: PerceptionMode = PerceptionMode.LEARNED,
        model_dir: Optional[str] = None,
        device: Optional[str] = None,
        conf_threshold: float = 0.3,
        camera_height: float = 1.2,
        row_spacing: float = 0.75,
    ):
        self._node = node
        self._mode = mode

        # ── 센서 데이터 버퍼 ──
        self._latest_image: Optional[np.ndarray] = None
        self._latest_imu_orientation: Optional[dict] = None
        self._latest_gps: Optional[dict] = None
        self._image_stamp = None

        # ── Learned 인지 모듈 ──
        self._model_manager = ModelManager(
            model_dir=model_dir,
            device=device,
        )
        self._terrain_segmenter = TerrainSegmenter(
            self._model_manager,
            model_name="yolov8n-seg",
            conf_threshold=conf_threshold,
        )
        self._obstacle_detector_learned = ObstacleDetector(
            self._model_manager,
            model_name="yolov8n",
            conf_threshold=conf_threshold,
            camera_height=camera_height,
        )
        self._crop_row_extractor = CropRowExtractor(
            camera_height=camera_height,
            expected_row_spacing=row_spacing,
        )

        # ── Traditional 인지 모듈 (Camera-only since C64) ──
        self._camera_detector = CameraDetector()
        self._semantic_segmenter = SemanticSegmenter()

        # ── QoS (센서용 BEST_EFFORT) ──
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 센서 구독 ──
        self._camera_sub = node.create_subscription(
            Image, '/camera/image_raw', self._camera_cb, sensor_qos)
        # LiDAR 구독 제거됨 (C64: Camera-Only 전환)
        self._imu_sub = node.create_subscription(
            Imu, '/sensor/imu', self._imu_cb, sensor_qos)
        self._gps_sub = node.create_subscription(
            NavSatFix, '/sensor/gps', self._gps_cb, sensor_qos)

        # ── 처리 통계 ──
        self._process_count = 0
        self._total_time_ms = 0.0
        self._last_features: Optional[PerceptionFeatures] = None

        node.get_logger().info(
            f'[PerceptionManager] 초기화 완료 — mode={mode.value}'
        )

    # ================================================================
    # Properties
    # ================================================================

    @property
    def mode(self) -> PerceptionMode:
        return self._mode

    @mode.setter
    def mode(self, value: PerceptionMode):
        if value != self._mode:
            self._node.get_logger().info(
                f'[PerceptionManager] 모드 전환: {self._mode.value} → {value.value}'
            )
            self._mode = value

    @property
    def has_image(self) -> bool:
        return self._latest_image is not None

    @property
    def avg_latency_ms(self) -> float:
        if self._process_count == 0:
            return 0.0
        return self._total_time_ms / self._process_count

    # ================================================================
    # Sensor callbacks
    # ================================================================

    def _camera_cb(self, msg: Image):
        """카메라 이미지 수신."""
        try:
            if msg.height == 0 or msg.width == 0:
                return
            image = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if msg.encoding in ('rgb8', 'bgr8'):
                image = image.reshape((msg.height, msg.width, 3))
            elif msg.encoding in ('rgba8', 'bgra8'):
                image = image.reshape((msg.height, msg.width, 4))[:, :, :3]
            elif msg.encoding == 'mono8':
                image = image.reshape((msg.height, msg.width))
                image = np.stack([image] * 3, axis=-1)
            else:
                return
            self._latest_image = image
            self._image_stamp = msg.header.stamp
        except Exception as e:
            self._node.get_logger().debug(f'Image conversion error: {e}')

    def _imu_cb(self, msg: Imu):
        self._latest_imu_orientation = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w,
        }

    def _gps_cb(self, msg: NavSatFix):
        self._latest_gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
        }

    # ================================================================
    # Main processing
    # ================================================================

    def process(self) -> PerceptionFeatures:
        """현재 모드에 따른 인지 처리 실행.

        Returns:
            PerceptionFeatures — 통합된 인지 결과
        """
        start = time.time()

        if self._mode == PerceptionMode.LEARNED:
            features = self._process_learned()
        elif self._mode == PerceptionMode.TRADITIONAL:
            features = self._process_traditional()
        else:  # AUTO
            features = self._process_auto()

        elapsed_ms = (time.time() - start) * 1000
        features.processing_time_ms = elapsed_ms
        features.timestamp = time.time()

        self._process_count += 1
        self._total_time_ms += elapsed_ms
        self._last_features = features

        return features

    # ================================================================
    # Learned pipeline (YOLOv8, camera only)
    # ================================================================

    def _process_learned(self) -> PerceptionFeatures:
        """YOLOv8 기반 인지 파이프라인."""
        if self._latest_image is None:
            return self._empty_features("learned: no image")

        image = self._latest_image

        # 1. 지형 세그멘테이션
        terrain_segments = self._terrain_segmenter.segment(image)
        dominant = self._terrain_segmenter.get_dominant_terrain(terrain_segments)
        terrain_conf = self._terrain_segmenter.get_terrain_confidence(
            terrain_segments, dominant
        )
        terrain_type = _TERRAIN_LABEL_MAP.get(dominant, TerrainClass.CROP_FIELD)

        # 2. 장애물 탐지
        detected = self._obstacle_detector_learned.detect(image)
        obstacles = [
            Object3D(
                position=d.position_3d or (0.0, 0.0, 0.0),
                size=(0.5, 0.5, 0.5),
                class_label=d.class_label,
                confidence=d.confidence,
            )
            for d in detected
        ]

        # 3. 작물 행 추출
        crop_mask = self._get_crop_mask(terrain_segments, image.shape[:2])
        crop_rows_raw = self._crop_row_extractor.extract_rows(crop_mask, image)
        crop_rows = [
            Line3D(start=r.start_3d, end=r.end_3d, confidence=r.confidence)
            for r in crop_rows_raw
        ]

        return PerceptionFeatures(
            crop_rows=crop_rows,
            crop_row_confidence=0.8 if crop_rows else 0.0,
            terrain_type=terrain_type,
            terrain_confidence=terrain_conf,
            obstacles=obstacles,
            slope_gradient=0.0,
            overall_confidence=terrain_conf * 0.85,
        )

    def _get_crop_mask(self, segments, image_shape) -> np.ndarray:
        """세그멘테이션 결과에서 작물 마스크 추출."""
        import cv2
        mask = np.zeros(image_shape[:2], dtype=np.uint8)
        for seg in segments:
            if seg.label == TerrainLabel.CROP_FIELD:
                h, w = seg.mask.shape
                if h != image_shape[0] or w != image_shape[1]:
                    seg_mask = cv2.resize(
                        seg.mask.astype(np.uint8),
                        (image_shape[1], image_shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                else:
                    seg_mask = seg.mask
                mask = np.maximum(mask, seg_mask)
        return mask

    # ================================================================
    # Traditional pipeline (LiDAR + Camera + Fusion)
    # ================================================================

    def _process_traditional(self) -> PerceptionFeatures:
        """카메라 기반 모듈형 인지 파이프라인 (C64: LiDAR 제거)."""
        obstacles_3d: List[Object3D] = []
        terrain_type = TerrainClass.CROP_FIELD
        terrain_conf = 0.5
        slope = 0.0

        # 1. 카메라 탐지
        camera_detections: List[Detection] = []
        if self._latest_image is not None:
            camera_detections = self._camera_detector.detect(self._latest_image)
            for det in camera_detections:
                obstacles_3d.append(Object3D(
                    position=(det.x, det.y, 0.0) if hasattr(det, 'x') else (0.0, 0.0, 0.0),
                    size=(0.5, 0.5, 0.5),
                    class_label=det.class_name if hasattr(det, 'class_name') else 'unknown',
                    confidence=det.confidence if hasattr(det, 'confidence') else 0.5,
                ))

        # 2. 시맨틱 세그멘테이션
        if self._latest_image is not None:
            seg_result = self._semantic_segmenter.segment(self._latest_image)
            if seg_result is not None:
                dominant_cls = self._get_dominant_seg_class(seg_result)
                terrain_type = _SEG_CLASS_MAP.get(
                    dominant_cls, TerrainClass.CROP_FIELD
                )
                terrain_conf = 0.7

        return PerceptionFeatures(
            crop_rows=[],  # Traditional 파이프라인은 작물 행 미지원
            crop_row_confidence=0.0,
            terrain_type=terrain_type,
            terrain_confidence=terrain_conf,
            obstacles=obstacles_3d,
            slope_gradient=slope,
            overall_confidence=0.7 if camera_detections else 0.3,
        )

    def _get_dominant_seg_class(self, seg_result) -> SegmentationClass:
        """세그멘테이션 결과에서 가장 넓은 클래스 반환."""
        try:
            if hasattr(seg_result, 'class_areas'):
                areas = seg_result.class_areas
                if areas:
                    return max(areas, key=areas.get)
            if hasattr(seg_result, 'mask') and seg_result.mask is not None:
                unique, counts = np.unique(seg_result.mask, return_counts=True)
                dominant_idx = counts.argmax()
                return SegmentationClass(unique[dominant_idx])
        except Exception:
            pass
        return SegmentationClass.BACKGROUND

    # ================================================================
    # Auto pipeline
    # ================================================================

    def _process_auto(self) -> PerceptionFeatures:
        """Learned 시도 → 실패 시 Traditional fallback."""
        try:
            if self._latest_image is not None:
                features = self._process_learned()
                if features.overall_confidence > 0.3:
                    return features
        except Exception as e:
            self._node.get_logger().warn(
                f'[PerceptionManager] Learned 실패, Traditional fallback: {e}'
            )

        return self._process_traditional()

    # ================================================================
    # Helpers
    # ================================================================

    def _empty_features(self, reason: str = "") -> PerceptionFeatures:
        """센서 데이터 부재 시 빈 결과 반환."""
        self._node.get_logger().debug(
            f'[PerceptionManager] Empty features: {reason}'
        )
        return PerceptionFeatures(
            terrain_type=TerrainClass.CROP_FIELD,
            overall_confidence=0.0,
        )

    def get_status(self) -> dict:
        """현재 상태 반환 (모니터링용)."""
        return {
            'mode': self._mode.value,
            'has_image': self.has_image,
            'process_count': self._process_count,
            'avg_latency_ms': round(self.avg_latency_ms, 1),
        }
