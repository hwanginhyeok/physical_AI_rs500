"""인지 모듈 - 센서 데이터 수집 및 처리.

카메라, LiDAR, GPS, IMU 센서 데이터를 구독하고,
LidarProcessor와 CameraDetector를 통해 장애물 및 객체 탐지 결과를 생성한다.
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

from team_leader.lidar_processor import LidarProcessor, Obstacle
from team_leader.camera_detector import CameraDetector, Detection


class PerceptionModule:
    """센서 데이터를 구독하고 처리하는 인지 모듈.

    카메라, 라이다, GPS, IMU 등의 센서 데이터를 수집하여
    장애물 목록, 차선 정보, 차량 위치 등의 인지 결과를 제공한다.

    LidarProcessor를 통해 포인트클라우드 기반 장애물 감지를,
    CameraDetector를 통해 이미지 기반 객체 탐지를 수행한다.
    """

    def __init__(self, node: Node):
        self._node = node

        # 인지 결과 저장
        self.obstacles: List[Obstacle] = []       # LiDAR 기반 장애물 목록
        self.detections: List[Detection] = []     # 카메라 기반 탐지 결과
        self.lane_info = None                     # 차선 정보
        self.vehicle_position = None              # 차량 현재 위치 (GPS)
        self.vehicle_orientation = None           # 차량 방향 (IMU)

        # 인지 서브모듈 초기화
        self._lidar_processor = LidarProcessor()
        self._camera_detector = CameraDetector()

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

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _camera_callback(self, msg: Image) -> None:
        """카메라 이미지 수신 콜백. 객체 탐지 수행."""
        self._node.get_logger().debug('[인지] 카메라 이미지 수신')
        self.process_camera(msg)

    def _lidar_callback(self, msg: PointCloud2) -> None:
        """라이다 포인트클라우드 수신 콜백. 장애물 감지 수행."""
        self._node.get_logger().debug('[인지] 라이다 데이터 수신')
        self.process_lidar(msg)

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
        """PointCloud2 메시지를 numpy 배열로 변환 후 장애물 감지를 수행한다.

        Args:
            msg: ROS2 PointCloud2 메시지.

        Returns:
            감지된 Obstacle 객체 리스트.
        """
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
        """PointCloud2 메시지를 (N, 3) numpy 배열로 변환한다.

        x, y, z 필드를 추출하여 numpy 배열을 생성한다.

        Args:
            msg: ROS2 PointCloud2 메시지.

        Returns:
            (N, 3) 형태의 numpy 배열 (x, y, z).
        """
        # 필드 오프셋 탐색
        field_map = {f.name: f for f in msg.fields}
        if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
            self._node.get_logger().warn('[인지] PointCloud2에 x/y/z 필드 없음')
            return np.empty((0, 3), dtype=np.float32)

        x_offset = field_map['x'].offset
        y_offset = field_map['y'].offset
        z_offset = field_map['z'].offset
        point_step = msg.point_step

        # 총 포인트 수 계산
        n_points = msg.width * msg.height
        if n_points == 0 or len(msg.data) == 0:
            return np.empty((0, 3), dtype=np.float32)

        # 바이트 데이터에서 float32 추출
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

        # NaN / Inf 제거
        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    # ------------------------------------------------------------------
    # 카메라 처리
    # ------------------------------------------------------------------

    def process_camera(self, msg: Image) -> List[Detection]:
        """Image 메시지를 numpy 배열로 변환 후 객체 탐지를 수행한다.

        Args:
            msg: ROS2 Image 메시지.

        Returns:
            Detection 객체 리스트.
        """
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
        """Image 메시지를 (H, W, 3) numpy 배열로 변환한다.

        Args:
            msg: ROS2 Image 메시지.

        Returns:
            (H, W, 3) numpy 배열 또는 None (변환 실패 시).
        """
        if msg.height == 0 or msg.width == 0:
            return None

        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                image = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                image = image.reshape((msg.height, msg.width, 3))
            elif msg.encoding == 'rgba8' or msg.encoding == 'bgra8':
                image = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                image = image.reshape((msg.height, msg.width, 4))
                image = image[:, :, :3]  # 알파 채널 제거
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
    # 결과 조회
    # ------------------------------------------------------------------

    def get_obstacles(self) -> List[Obstacle]:
        """현재 감지된 LiDAR 기반 장애물 목록을 반환한다.

        Returns:
            Obstacle 객체 리스트 (거리순 정렬).
        """
        return self.obstacles

    def get_detections(self) -> List[Detection]:
        """현재 카메라 탐지 결과를 반환한다.

        Returns:
            Detection 객체 리스트.
        """
        return self.detections

    def get_perception_result(self) -> dict:
        """현재 인지 결과를 반환한다.

        Returns:
            장애물, 탐지 결과, 차선 정보, 위치, 방향을 포함하는 딕셔너리.
        """
        return {
            'obstacles': self.obstacles,
            'detections': self.detections,
            'lane_info': self.lane_info,
            'position': self.vehicle_position,
            'orientation': self.vehicle_orientation,
        }
