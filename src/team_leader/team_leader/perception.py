"""인지 모듈 - 센서 데이터 수집 및 처리."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from std_msgs.msg import Float32


class PerceptionModule:
    """센서 데이터를 구독하고 처리하는 인지 모듈.

    카메라, 라이다, GPS, IMU 등의 센서 데이터를 수집하여
    장애물 목록, 차선 정보, 차량 위치 등의 인지 결과를 제공한다.
    """

    def __init__(self, node: Node):
        self._node = node

        # 인지 결과 저장
        self.obstacles = []       # 감지된 장애물 목록
        self.lane_info = None     # 차선 정보
        self.vehicle_position = None  # 차량 현재 위치 (GPS)
        self.vehicle_orientation = None  # 차량 방향 (IMU)

        # 센서 구독 설정
        self._camera_sub = node.create_subscription(
            Image, '/sensor/camera/front', self._camera_callback, 10)
        self._lidar_sub = node.create_subscription(
            PointCloud2, '/sensor/lidar', self._lidar_callback, 10)
        self._gps_sub = node.create_subscription(
            NavSatFix, '/sensor/gps', self._gps_callback, 10)
        self._imu_sub = node.create_subscription(
            Imu, '/sensor/imu', self._imu_callback, 10)

        node.get_logger().info('[인지] 모듈 초기화 완료')

    def _camera_callback(self, msg: Image):
        """카메라 이미지 수신 콜백. 차선 및 객체 감지 수행."""
        # TODO: 실제 이미지 처리 파이프라인 연결
        self._node.get_logger().debug('[인지] 카메라 이미지 수신')

    def _lidar_callback(self, msg: PointCloud2):
        """라이다 포인트클라우드 수신 콜백. 장애물 감지 수행."""
        # TODO: 포인트클라우드 기반 장애물 감지
        self._node.get_logger().debug('[인지] 라이다 데이터 수신')

    def _gps_callback(self, msg: NavSatFix):
        """GPS 수신 콜백. 차량 위치 업데이트."""
        self.vehicle_position = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
        }

    def _imu_callback(self, msg: Imu):
        """IMU 수신 콜백. 차량 방향/가속도 업데이트."""
        self.vehicle_orientation = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w,
        }

    def get_perception_result(self) -> dict:
        """현재 인지 결과를 반환."""
        return {
            'obstacles': self.obstacles,
            'lane_info': self.lane_info,
            'position': self.vehicle_position,
            'orientation': self.vehicle_orientation,
        }
