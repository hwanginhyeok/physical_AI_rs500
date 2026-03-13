"""Learned Perception ROS2 Node.

YOLOv8 기반 실시간 인지 통합:
- 지형 세그멘테이션
- 장애물 탐지  
- 작물 행 추출
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import numpy as np

# ROS2 메시지
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from typing import Tuple

# 커스텀 메시지 (ad_core에서)
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'ad_core'))

from ad_core.hybrid_e2e_types import (
    PerceptionFeatures,
    TerrainClass,
    Line3D,
    Object3D,
)
from ad_core.datatypes import Pose2D

# Perception 모듈
from .model_manager import ModelManager
from .terrain_segmentation import TerrainSegmenter, TerrainLabel
from .obstacle_detector import ObstacleDetector
from .crop_row_extractor import CropRowExtractor


class LearnedPerceptionNode(Node):
    """Learned Perception 통합 노드.
    
    Subscribers:
        - /camera/image_raw (sensor_msgs/Image)
        - /camera/camera_info (sensor_msgs/CameraInfo)
    
    Publishers:
        - /perception/features (PerceptionFeatures - custom)
        - /perception/debug_image (sensor_msgs/Image)
    """
    
    def __init__(self):
        super().__init__('learned_perception_node')
        
        # 파라미터 선언
        self.declare_parameter('model_dir', '')
        self.declare_parameter('device', '')  # auto if empty
        self.declare_parameter('inference_rate', 10.0)  # Hz
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('camera_height', 1.2)
        self.declare_parameter('row_spacing', 0.75)
        
        # 파라미터 읽기
        model_dir = self.get_parameter('model_dir').value
        device = self.get_parameter('device').value
        self.inference_rate = self.get_parameter('inference_rate').value
        conf_threshold = self.get_parameter('conf_threshold').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        camera_height = self.get_parameter('camera_height').value
        row_spacing = self.get_parameter('row_spacing').value
        
        self.get_logger().info(f'LearnedPerception initializing...')
        self.get_logger().info(f'  Device: {device or "auto"}')
        self.get_logger().info(f'  Inference rate: {self.inference_rate} Hz')
        
        # CvBridge
        self.bridge = CvBridge()
        
        # Model Manager
        self.model_manager = ModelManager(
            model_dir=model_dir if model_dir else None,
            device=device if device else None
        )
        
        # Perception 모듈들
        self.terrain_segmenter = TerrainSegmenter(
            self.model_manager,
            model_name="yolov8n-seg",
            conf_threshold=conf_threshold
        )
        
        self.obstacle_detector = ObstacleDetector(
            self.model_manager,
            model_name="yolov8n",
            conf_threshold=conf_threshold,
            camera_height=camera_height
        )
        
        self.crop_row_extractor = CropRowExtractor(
            camera_height=camera_height,
            expected_row_spacing=row_spacing
        )
        
        # QoS 설정
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self._camera_info_callback,
            qos
        )
        
        # Publishers
        # TODO: custom msg 타입이 필요함. 현재는 std_msgs로 대체
        self.features_pub = self.create_publisher(
            Image,  # Placeholder - should be custom msg
            '/perception/debug/features',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '/perception/debug/image',
            10
        )
        
        # 타이머 (추론 제한)
        self.last_inference_time = self.get_clock().now()
        self.min_inference_period = 1.0 / self.inference_rate
        
        # 상태
        self.camera_matrix = None
        self.latest_image = None
        self.processing = False
        
        self.get_logger().info('LearnedPerception initialized successfully')
    
    def _camera_info_callback(self, msg: CameraInfo):
        """카 메라 정보 수신."""
        # 카 메라 행렬 추출
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        
        # 모듈들에 업데이트
        self.obstacle_detector.camera_matrix = self.camera_matrix
        self.crop_row_extractor.camera_matrix = self.camera_matrix
        
        # 한 번만 받으면 됨
        self.camera_info_sub.destroy()
    
    def _image_callback(self, msg: Image):
        """이미지 수신 및 처리."""
        # 속도 제한
        now = self.get_clock().now()
        elapsed = (now - self.last_inference_time).nanoseconds / 1e9
        
        if elapsed < self.min_inference_period:
            return
        
        if self.processing:
            return
        
        self.processing = True
        self.last_inference_time = now
        
        try:
            # 이미지 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            
            # 추론 실행
            features = self._process_image(cv_image)
            
            # 디버그 이미지 퍼블리시
            if self.publish_debug_image:
                debug_image = self._create_debug_image(cv_image, features)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            
            # TODO: features를 ROS 메시지로 변환하여 퍼블리시
            # 현재는 hybrid_e2e_types 객체 사용
            self._publish_features(features, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
            import traceback
            self.get_logger().debug(traceback.format_exc())
        
        finally:
            self.processing = False
    
    def _process_image(self, image: np.ndarray) -> PerceptionFeatures:
        """이미지 처리 파이프라인."""
        start_time = self.get_clock().now()
        
        # 1. 지형 세그멘테이션
        terrain_segments = self.terrain_segmenter.segment(image)
        
        # 2. 장애물 탐지
        obstacles = self.obstacle_detector.detect(image)
        
        # 3. 작물 행 추출
        crop_mask = self._get_crop_mask(terrain_segments, image.shape[:2])
        crop_rows_2d = self.crop_row_extractor.extract_rows(crop_mask, image)
        
        # 4. PerceptionFeatures 생성
        features = self._create_perception_features(
            terrain_segments,
            obstacles,
            crop_rows_2d,
            image.shape[:2]
        )
        
        # 처리 시간 기록
        processing_time = (self.get_clock().now() - start_time).nanoseconds / 1e6
        features.processing_time_ms = processing_time
        
        return features
    
    def _get_crop_mask(
        self,
        segments: list,
        image_shape: Tuple[int, int]
    ) -> np.ndarray:
        """작물 마스크 추출."""
        mask = np.zeros(image_shape, dtype=np.uint8)
        
        for seg in segments:
            if seg.label == TerrainLabel.CROP_FIELD:
                # 마스크 크기 조정
                seg_h, seg_w = seg.mask.shape
                if seg_h != image_shape[0] or seg_w != image_shape[1]:
                    import cv2
                    seg_mask = cv2.resize(
                        seg.mask.astype(np.uint8),
                        (image_shape[1], image_shape[0]),
                        interpolation=cv2.INTER_NEAREST
                    )
                else:
                    seg_mask = seg.mask
                
                mask = np.maximum(mask, seg_mask)
        
        return mask
    
    def _create_perception_features(
        self,
        terrain_segments: list,
        obstacles: list,
        crop_rows: list,
        image_shape: Tuple[int, int]
    ) -> PerceptionFeatures:
        """PerceptionFeatures 객체 생성."""
        # 지형 분류
        dominant_terrain = self.terrain_segmenter.get_dominant_terrain(terrain_segments)
        terrain_conf = self.terrain_segmenter.get_terrain_confidence(
            terrain_segments, dominant_terrain
        )
        
        # TerrainClass로 변환
        terrain_map = {
            TerrainLabel.CROP_FIELD: TerrainClass.CROP_FIELD,
            TerrainLabel.GRASS: TerrainClass.GRASS,
            TerrainLabel.DIRT_ROAD: TerrainClass.DIRT_ROAD,
            TerrainLabel.PAVED: TerrainClass.PAVED,
            TerrainLabel.MUD: TerrainClass.MUD,
            TerrainLabel.OBSTACLE: TerrainClass.OBSTACLE,
            TerrainLabel.UNKNOWN: TerrainClass.CROP_FIELD,
        }
        terrain_type = terrain_map.get(dominant_terrain, TerrainClass.CROP_FIELD)
        
        # 장애물 변환
        obstacle_list = []
        for obs in obstacles:
            obj = Object3D(
                position=obs.position_3d or (0, 0, 0),
                size=(0.5, 0.5, 0.5),  # 기본 크기
                class_label=obs.class_label,
                confidence=obs.confidence
            )
            obstacle_list.append(obj)
        
        # 작물 행 변환
        crop_row_list = []
        for row in crop_rows:
            line = Line3D(
                start=row.start_3d,
                end=row.end_3d,
                confidence=row.confidence
            )
            crop_row_list.append(line)
        
        features = PerceptionFeatures(
            crop_rows=crop_row_list,
            crop_row_confidence=0.8 if crop_rows else 0.0,
            terrain_type=terrain_type,
            terrain_confidence=terrain_conf,
            obstacles=obstacle_list,
            overall_confidence=0.85,
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        
        return features
    
    def _create_debug_image(
        self,
        image: np.ndarray,
        features: PerceptionFeatures
    ) -> np.ndarray:
        """디버그 시각화 이미지 생성."""
        import cv2
        
        debug = image.copy()
        h, w = debug.shape[:2]
        
        # 지형 정보 표시
        terrain_text = f"Terrain: {features.terrain_type.value} ({features.terrain_confidence:.2f})"
        cv2.putText(debug, terrain_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 장애물 표시
        for i, obs in enumerate(features.obstacles[:5]):  # 최대 5개
            pos = obs.position
            dist = np.linalg.norm(pos)
            text = f"{obs.class_label}: {dist:.1f}m"
            y_pos = 60 + i * 25
            cv2.putText(debug, text, (10, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 작물 행 표시
        num_rows = len(features.crop_rows)
        cv2.putText(debug, f"Crop rows: {num_rows}", (10, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 처리 시간
        cv2.putText(debug, f"{features.processing_time_ms:.1f}ms", (w - 100, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return debug
    
    def _publish_features(self, features: PerceptionFeatures, header: Header):
        """PerceptionFeatures 퍼블리시.
        
        TODO: custom 메시지 타입 정의 필요
        """
        # 현재는 로그로 출력
        self.get_logger().debug(
            f'Perception: terrain={features.terrain_type.value}, '
            f'obstacles={len(features.obstacles)}, '
            f'crop_rows={len(features.crop_rows)}, '
            f'time={features.processing_time_ms:.1f}ms'
        )
    
    def destroy_node(self):
        """노드 종료 정리."""
        self.get_logger().info('Shutting down LearnedPerception...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LearnedPerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
