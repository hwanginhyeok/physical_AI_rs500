"""인지 노드 - 멀티카메라 센서 데이터 수집 및 처리.

카메라 3대(전면/좌전방/우전방), GPS, IMU 센서 데이터를 구독하고,
CameraDetector를 통해 객체 탐지,
SemanticSegmenter를 통해 시맨틱 세그멘테이션을 수행한다.

C64: LiDAR 미탑재 → Camera-Only 인지 파이프라인.
     LiDAR 기반 장애물 감지와 Late Fusion 코드 제거.
     Nav2 costmap은 Gazebo rgbd_camera PointCloud2로 직접 동작.
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix, Imu

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False

from ad_core.camera_detector import CameraDetector, Detection
from ad_core.semantic_segmenter import (
    SemanticSegmenter,
    SegmentationResult,
    SegmentationClass,
)


class PerceptionModule:
    """센서 데이터를 구독하고 처리하는 인지 모듈.

    카메라 3대(전면/좌전방/우전방), GPS, IMU 센서 데이터를 수집하여
    객체 탐지, 시맨틱 세그멘테이션, 차량 위치/방향 등의 인지 결과를 제공한다.

    C64: Camera-Only 구성 — LiDAR/SensorFusion 제거.
    """

    def __init__(self, node: Node):
        self._node = node

        # 인지 결과 저장
        self.detections: List[Detection] = []     # 카메라 기반 탐지 결과 (전면 카메라)
        self.segmentation: Optional[SegmentationResult] = None  # 세그멘테이션 결과
        self.lane_info = None                     # 차선 정보
        self.vehicle_position = None              # 차량 현재 위치 (GPS)
        self.vehicle_orientation = None           # 차량 방향 (IMU)

        # 멀티카메라 최신 이미지 캐시
        self._latest_images: dict[str, Optional[Image]] = {
            'front': None,
            'left': None,
            'right': None,
        }

        # 인지 서브모듈 초기화
        self._camera_detector = CameraDetector()
        self._semantic_segmenter = SemanticSegmenter()

        # 토픽명 파라미터에서 읽기
        camera_front_topic = node.get_parameter('topics.camera_front').value
        camera_left_topic = node.get_parameter('topics.camera_left').value
        camera_right_topic = node.get_parameter('topics.camera_right').value
        gps_topic = node.get_parameter('topics.gps').value
        imu_topic = node.get_parameter('topics.imu').value

        # 카메라 3대 구독
        self._camera_front_sub = node.create_subscription(
            Image, camera_front_topic, self._camera_front_callback, 10)
        self._camera_left_sub = node.create_subscription(
            Image, camera_left_topic, self._camera_left_callback, 10)
        self._camera_right_sub = node.create_subscription(
            Image, camera_right_topic, self._camera_right_callback, 10)

        # GPS/IMU 구독
        self._gps_sub = node.create_subscription(
            NavSatFix, gps_topic, self._gps_callback, 10)
        self._imu_sub = node.create_subscription(
            Imu, imu_topic, self._imu_callback, 10)

        node.get_logger().info('[인지] Camera-Only 모듈 초기화 완료')
        if self._camera_detector.is_dummy_mode:
            node.get_logger().warn('[인지] 카메라 탐지기: 더미 모드 (YOLO 모델 미로드)')
        if self._semantic_segmenter.is_fallback_mode:
            node.get_logger().warn(
                '[인지] 세그멘터: 색상 규칙 fallback 모드 (PyTorch 미설치)'
            )

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _camera_front_callback(self, msg: Image) -> None:
        """전면 카메라 이미지 수신. 객체 탐지 + 세그멘테이션 수행."""
        self._node.get_logger().debug('[인지] 전면 카메라 이미지 수신')
        self._latest_images['front'] = msg
        self.process_camera(msg)
        self._run_segmentation(msg)

    def _camera_left_callback(self, msg: Image) -> None:
        """좌전방 카메라 이미지 수신."""
        self._node.get_logger().debug('[인지] 좌전방 카메라 이미지 수신')
        self._latest_images['left'] = msg

    def _camera_right_callback(self, msg: Image) -> None:
        """우전방 카메라 이미지 수신."""
        self._node.get_logger().debug('[인지] 우전방 카메라 이미지 수신')
        self._latest_images['right'] = msg

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
    # 결과 조회
    # ------------------------------------------------------------------

    def get_detections(self) -> List[Detection]:
        """현재 카메라 탐지 결과를 반환한다."""
        return self.detections

    def get_latest_image(self, camera: str = 'front') -> Optional[Image]:
        """지정된 카메라의 최신 이미지를 반환한다."""
        return self._latest_images.get(camera)

    def get_perception_result(self) -> dict:
        """현재 인지 결과를 반환한다."""
        return {
            'detections': self.detections,
            'segmentation': self.segmentation,
            'lane_info': self.lane_info,
            'position': self.vehicle_position,
            'orientation': self.vehicle_orientation,
        }


class PerceptionNode(Node):
    """인지 독립 ROS2 노드."""

    def __init__(self):
        super().__init__('perception_node')

        # 파라미터 선언 — C64: 멀티카메라 + GPS + IMU
        self.declare_parameter('topics.camera_front', '/sensor/camera/front/image')
        self.declare_parameter('topics.camera_left', '/sensor/camera/left/image')
        self.declare_parameter('topics.camera_right', '/sensor/camera/right/image')
        self.declare_parameter('topics.gps', '/sensor/gps')
        self.declare_parameter('topics.imu', '/sensor/imu')

        # 인지 모듈 초기화
        self.perception = PerceptionModule(self)

        self.get_logger().info('[인지 노드] Camera-Only 모드 시작')


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
