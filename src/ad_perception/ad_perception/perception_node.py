"""인지 노드 - 센서 데이터 수집 및 처리.

카메라, LiDAR, GPS, IMU 센서 데이터를 구독하고,
LidarProcessor와 CameraDetector를 통해 장애물 및 객체 탐지 결과를 생성한다.
SensorFusion을 통해 LiDAR + Camera Late Fusion 결과를 제공한다.
SemanticSegmenter를 통해 픽셀 단위 시맨틱 세그멘테이션 결과를 제공한다.
CropRowDetectorCV를 통해 과수원 행 인식 및 조향 오프셋을 제공한다.
"""

from __future__ import annotations

import struct
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, Imu
from std_msgs.msg import Float32

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False

from ad_core.lidar_processor import LidarProcessor, Obstacle
from ad_core.camera_detector import CameraDetector, Detection
from ad_core.sensor_fusion import SensorFusion, FusedObject
from ad_core.semantic_segmenter import (
    SemanticSegmenter,
    SegmentationResult,
    SegmentationClass,
)
from ad_core.crop_row_detector import (
    CropRowDetectorCV,
    RowDetectionResult,
    OrchardType,
)


class PerceptionModule:
    """센서 데이터를 구독하고 처리하는 인지 모듈.

    카메라, 라이다, GPS, IMU 등의 센서 데이터를 수집하여
    장애물 목록, 차선 정보, 차량 위치 등의 인지 결과를 제공한다.

    LidarProcessor를 통해 포인트클라우드 기반 장애물 감지를,
    CameraDetector를 통해 이미지 기반 객체 탐지를 수행하며,
    SensorFusion을 통해 두 센서의 결과를 후기 융합(late fusion)한다.
    SemanticSegmenter를 통해 픽셀 단위 시맨틱 세그멘테이션을 수행하여
    주행 가능 영역과 장애물 영역을 분류한다.
    """

    def __init__(self, node: Node):
        self._node = node

        # 인지 결과 저장
        self.obstacles: List[Obstacle] = []       # LiDAR 기반 장애물 목록
        self.detections: List[Detection] = []     # 카메라 기반 탐지 결과
        self.fused_objects: List[FusedObject] = []  # 센서 융합 결과
        self.segmentation: Optional[SegmentationResult] = None  # 세그멘테이션 결과
        self.lane_info = None                     # 차선 정보
        self.vehicle_position = None              # 차량 현재 위치 (GPS)
        self.vehicle_orientation = None           # 차량 방향 (IMU)
        self.crop_row_result: Optional[RowDetectionResult] = None  # 과수원 행 감지
        self._crop_row_frame_count: int = 0       # 5Hz 스로틀 카운터
        self._crop_row_no_detect_count: int = 0   # 행 미감지 연속 횟수

        # YOLO 모델 경로 파라미터 읽기
        yolo_model_path = node.declare_parameter(
            'yolo.model_path', ''
        ).value
        yolo_confidence = node.declare_parameter(
            'yolo.confidence_threshold', 0.5
        ).value

        # 모델 경로가 비어있으면 프로젝트 기본 경로에서 탐색
        if not yolo_model_path:
            import os
            candidate = os.path.join(
                os.path.expanduser('~'),
                'physical_AI_rs500', 'models', 'yolo11n.pt',
            )
            if os.path.exists(candidate):
                yolo_model_path = candidate

        # 과수원 행 감지 파라미터
        orchard_type_str = node.declare_parameter(
            'crop_row.orchard_type', 'PEAR'
        ).value
        self._crop_row_enabled = node.declare_parameter(
            'crop_row.enabled', False
        ).value
        self._crop_row_process_every_n = node.declare_parameter(
            'crop_row.process_every_n_frames', 3
        ).value  # 15Hz 카메라에서 3프레임마다 = 5Hz
        self._crop_row_no_detect_limit = node.declare_parameter(
            'crop_row.no_detect_limit', 10
        ).value  # 5Hz에서 10회 = 2초

        # 인지 서브모듈 초기화
        self._lidar_processor = LidarProcessor()
        self._camera_detector = CameraDetector(
            model_path=yolo_model_path if yolo_model_path else None,
            confidence_threshold=yolo_confidence,
        )
        self._sensor_fusion = SensorFusion()
        self._semantic_segmenter = SemanticSegmenter()

        # 과수원 행 감지 모듈
        try:
            orchard_type = OrchardType[orchard_type_str.upper()]
        except KeyError:
            orchard_type = OrchardType.PEAR
            node.get_logger().warn(
                f'[인지] 미지원 과수원 유형: {orchard_type_str}, PEAR 사용'
            )
        self._crop_row_detector = CropRowDetectorCV(orchard_type=orchard_type)

        # 토픽명 파라미터에서 읽기
        camera_topic = node.get_parameter('topics.camera').value
        lidar_topic = node.get_parameter('topics.lidar').value
        gps_topic = node.get_parameter('topics.gps').value
        imu_topic = node.get_parameter('topics.imu').value

        # 센서 구독 설정
        self._camera_sub = node.create_subscription(
            Image, camera_topic, self._camera_callback, 10)
        self._lidar_sub = node.create_subscription(
            PointCloud2, lidar_topic, self._lidar_callback, 10)
        self._gps_sub = node.create_subscription(
            NavSatFix, gps_topic, self._gps_callback, 10)
        self._imu_sub = node.create_subscription(
            Imu, imu_topic, self._imu_callback, 10)

        node.get_logger().info('[인지] 모듈 초기화 완료')
        if self._camera_detector.is_dummy_mode:
            node.get_logger().warn('[인지] 카메라 탐지기: 더미 모드 (YOLO 모델 미로드)')
        if self._semantic_segmenter.is_fallback_mode:
            node.get_logger().warn(
                '[인지] 세그멘터: 색상 규칙 fallback 모드 (PyTorch 미설치)'
            )

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _camera_callback(self, msg: Image) -> None:
        """카메라 이미지 수신 콜백. 객체 탐지, 세그멘테이션, 센서 융합, 행 감지 수행."""
        self._node.get_logger().debug('[인지] 카메라 이미지 수신')
        self.process_camera(msg)
        self._run_segmentation(msg)
        self._run_fusion()
        self._run_crop_row_detection(msg)

    def _lidar_callback(self, msg: PointCloud2) -> None:
        """라이다 포인트클라우드 수신 콜백. 장애물 감지 후 센서 융합 수행."""
        self._node.get_logger().debug('[인지] 라이다 데이터 수신')
        self.process_lidar(msg)
        self._run_fusion()

    def _gps_callback(self, msg: NavSatFix) -> None:
        """GPS 수신 콜백. 차량 위치 업데이트."""
        self.vehicle_position = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
        }

    def _imu_callback(self, msg: Imu) -> None:
        """IMU 수신 콜백. 차량 방향/가속도 업데이트."""
        self.vehicle_orientation = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w,
        }

    # ------------------------------------------------------------------
    # LiDAR 처리
    # ------------------------------------------------------------------

    def process_lidar(self, msg: PointCloud2) -> List[Obstacle]:
        """PointCloud2 메시지를 numpy 배열로 변환 후 장애물 감지를 수행한다."""
        if not _HAS_NUMPY:
            self._node.get_logger().warn('[인지] numpy 미설치 - LiDAR 처리 건너뜀')
            return []

        points = self._pointcloud2_to_numpy(msg)
        if points.shape[0] == 0:
            self.obstacles = []
            return []

        self.obstacles = self._lidar_processor.process(points)
        self._node.get_logger().debug(
            f'[인지] LiDAR 장애물 {len(self.obstacles)}개 감지')
        return self.obstacles

    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> "np.ndarray":
        """PointCloud2 메시지를 (N, 3) numpy 배열로 변환한다."""
        field_map = {f.name: f for f in msg.fields}
        if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
            self._node.get_logger().warn('[인지] PointCloud2에 x/y/z 필드 없음')
            return np.empty((0, 3), dtype=np.float32)

        x_offset = field_map['x'].offset
        y_offset = field_map['y'].offset
        z_offset = field_map['z'].offset
        point_step = msg.point_step

        n_points = msg.width * msg.height
        if n_points == 0 or len(msg.data) == 0:
            return np.empty((0, 3), dtype=np.float32)

        data = bytes(msg.data)
        points = np.zeros((n_points, 3), dtype=np.float32)

        for i in range(n_points):
            base = i * point_step
            if base + max(x_offset, y_offset, z_offset) + 4 > len(data):
                points = points[:i]
                break
            points[i, 0] = struct.unpack_from('f', data, base + x_offset)[0]
            points[i, 1] = struct.unpack_from('f', data, base + y_offset)[0]
            points[i, 2] = struct.unpack_from('f', data, base + z_offset)[0]

        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    # ------------------------------------------------------------------
    # 카메라 처리
    # ------------------------------------------------------------------

    def process_camera(self, msg: Image) -> List[Detection]:
        """Image 메시지를 numpy 배열로 변환 후 객체 탐지를 수행한다."""
        if not _HAS_NUMPY:
            self._node.get_logger().warn('[인지] numpy 미설치 - 카메라 처리 건너뜀')
            return []

        image = self._image_to_numpy(msg)
        if image is None:
            self.detections = []
            return []

        self.detections = self._camera_detector.detect(image)
        self._node.get_logger().debug(
            f'[인지] 카메라 탐지 {len(self.detections)}개 객체')
        return self.detections

    def _image_to_numpy(self, msg: Image) -> Optional["np.ndarray"]:
        """Image 메시지를 (H, W, 3) numpy 배열로 변환한다."""
        if msg.height == 0 or msg.width == 0:
            return None

        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                image = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                image = image.reshape((msg.height, msg.width, 3))
            elif msg.encoding == 'rgba8' or msg.encoding == 'bgra8':
                image = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                image = image.reshape((msg.height, msg.width, 4))
                image = image[:, :, :3]
            elif msg.encoding == 'mono8':
                image = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                image = image.reshape((msg.height, msg.width))
                image = np.stack([image, image, image], axis=-1)
            else:
                self._node.get_logger().warn(
                    f'[인지] 미지원 이미지 인코딩: {msg.encoding}')
                return None
            return image
        except (ValueError, Exception) as e:
            self._node.get_logger().warn(f'[인지] 이미지 변환 실패: {e}')
            return None

    # ------------------------------------------------------------------
    # 시맨틱 세그멘테이션
    # ------------------------------------------------------------------

    def _run_segmentation(self, msg: Image) -> Optional[SegmentationResult]:
        """카메라 이미지에 대해 시맨틱 세그멘테이션을 수행한다."""
        if not _HAS_NUMPY:
            return None

        image = self._image_to_numpy(msg)
        if image is None:
            self.segmentation = None
            return None

        self.segmentation = self._semantic_segmenter.segment(image)
        self._node.get_logger().debug(
            f'[인지] 세그멘테이션 완료: '
            f'{self.segmentation.width}x{self.segmentation.height}, '
            f'{len(self.segmentation.unique_classes)}개 클래스, '
            f'{self.segmentation.processing_time:.3f}초'
        )
        return self.segmentation

    def get_segmentation(self) -> Optional[SegmentationResult]:
        """현재 시맨틱 세그멘테이션 결과를 반환한다."""
        return self.segmentation

    def get_navigable_area(self) -> Optional["np.ndarray"]:
        """현재 세그멘테이션 결과에서 주행 가능 영역 마스크를 반환한다."""
        if self.segmentation is None:
            return None
        return self._semantic_segmenter.get_navigable_area(self.segmentation)

    def get_obstacle_area(self) -> Optional["np.ndarray"]:
        """현재 세그멘테이션 결과에서 장애물 영역 마스크를 반환한다."""
        if self.segmentation is None:
            return None
        return self._semantic_segmenter.get_obstacle_area(self.segmentation)

    # ------------------------------------------------------------------
    # 과수원 행 감지
    # ------------------------------------------------------------------

    def _run_crop_row_detection(self, msg: Image) -> Optional[RowDetectionResult]:
        """카메라 이미지에서 과수원 행을 감지한다. 5Hz 스로틀 적용."""
        if not self._crop_row_enabled or not _HAS_NUMPY:
            return None

        # 프레임 스로틀: process_every_n_frames 마다 실행
        self._crop_row_frame_count += 1
        if self._crop_row_frame_count % self._crop_row_process_every_n != 0:
            return self.crop_row_result

        image = self._image_to_numpy(msg)
        if image is None:
            return None

        result = self._crop_row_detector.detect(image)
        self.crop_row_result = result

        # 행 미감지 카운터 (행 끝 감지용)
        if result.num_rows == 0:
            self._crop_row_no_detect_count += 1
        else:
            self._crop_row_no_detect_count = 0

        self._node.get_logger().debug(
            f'[인지] 행 감지: {result.num_rows}행, '
            f'offset={result.get_steering_offset():.2f}, '
            f'heading_err={result.get_heading_error_deg():.1f}°, '
            f'{result.processing_time_ms:.0f}ms'
        )
        return result

    def get_crop_row_result(self) -> Optional[RowDetectionResult]:
        """현재 과수원 행 감지 결과를 반환한다."""
        return self.crop_row_result

    @property
    def crop_row_end_detected(self) -> bool:
        """행 끝 도달 여부. 연속 N회 행 미감지 시 True."""
        return self._crop_row_no_detect_count >= self._crop_row_no_detect_limit

    # ------------------------------------------------------------------
    # 센서 융합
    # ------------------------------------------------------------------

    def _run_fusion(self) -> List[FusedObject]:
        """현재 LiDAR 장애물과 카메라 탐지 결과를 융합한다."""
        self.fused_objects = self._sensor_fusion.fuse(
            self.obstacles, self.detections,
        )
        fused_count = sum(1 for o in self.fused_objects if o.source == "fused")
        lidar_only = sum(1 for o in self.fused_objects if o.source == "lidar")
        camera_only = sum(1 for o in self.fused_objects if o.source == "camera")
        self._node.get_logger().debug(
            f'[인지] 센서 융합 완료: '
            f'fused={fused_count}, lidar_only={lidar_only}, '
            f'camera_only={camera_only}, total={len(self.fused_objects)}'
        )
        return self.fused_objects

    def get_fused_objects(self) -> List[FusedObject]:
        """현재 센서 융합 결과를 반환한다."""
        return self.fused_objects

    # ------------------------------------------------------------------
    # 결과 조회
    # ------------------------------------------------------------------

    def get_obstacles(self) -> List[Obstacle]:
        """현재 감지된 LiDAR 기반 장애물 목록을 반환한다."""
        return self.obstacles

    def get_detections(self) -> List[Detection]:
        """현재 카메라 탐지 결과를 반환한다."""
        return self.detections

    def get_perception_result(self) -> dict:
        """현재 인지 결과를 반환한다."""
        result = {
            'obstacles': self.obstacles,
            'detections': self.detections,
            'fused_objects': self.fused_objects,
            'segmentation': self.segmentation,
            'lane_info': self.lane_info,
            'position': self.vehicle_position,
            'orientation': self.vehicle_orientation,
        }
        # 과수원 행 감지 결과 추가
        if self._crop_row_enabled and self.crop_row_result is not None:
            result['crop_row'] = self.crop_row_result
            result['crop_row_steering_offset'] = self.crop_row_result.get_steering_offset()
            result['crop_row_heading_error'] = self.crop_row_result.get_heading_error_deg()
            result['crop_row_end_detected'] = self.crop_row_end_detected
        return result


class PerceptionNode(Node):
    """인지 독립 ROS2 노드."""

    def __init__(self):
        super().__init__('perception_node')

        # 파라미터 선언
        self.declare_parameter('topics.camera', '/sensor/camera/front')
        self.declare_parameter('topics.lidar', '/sensor/lidar')
        self.declare_parameter('topics.gps', '/sensor/gps')
        self.declare_parameter('topics.imu', '/sensor/imu')
        self.declare_parameter('crop_row.orchard_type', 'PEAR')
        self.declare_parameter('crop_row.enabled', False)
        self.declare_parameter('crop_row.process_every_n_frames', 3)
        self.declare_parameter('crop_row.no_detect_limit', 10)

        # 인지 모듈 초기화
        self.perception = PerceptionModule(self)

        self.get_logger().info('[인지 노드] 시작')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[인지 노드] 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
