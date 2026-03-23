"""PerceptionModule Mock 노드 테스트 - ROS2 의존 없이 인지 로직 검증."""

import struct
from unittest.mock import MagicMock

import pytest
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField, Image, NavSatFix, Imu


# ── Mock Node ─────────────────────────────────────────────────


class MockParameter:
    def __init__(self, value):
        self.value = value


def make_mock_node():
    params = {
        'topics.camera': '/sensor/camera/front',
        'topics.lidar': '/sensor/lidar',
        'topics.gps': '/sensor/gps',
        'topics.imu': '/sensor/imu',
    }
    node = MagicMock()
    node.get_parameter = lambda name: MockParameter(params.get(name, ''))
    node.create_subscription = MagicMock(return_value=MagicMock())
    return node


from ad_perception.perception_node import PerceptionModule


# ── Helper ────────────────────────────────────────────────────


def make_pointcloud2(points):
    msg = PointCloud2()
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.width = len(points)
    msg.height = 1
    data = b''
    for x, y, z in points:
        data += struct.pack('fff', x, y, z)
    msg.data = list(data)
    return msg


def make_image(height, width, encoding='rgb8'):
    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    if encoding in ('rgb8', 'bgr8'):
        msg.data = list(bytes([128] * (height * width * 3)))
    elif encoding == 'mono8':
        msg.data = list(bytes([128] * (height * width)))
    elif encoding in ('rgba8', 'bgra8'):
        msg.data = list(bytes([128] * (height * width * 4)))
    return msg


# ── 초기화 ────────────────────────────────────────────────────


class TestPerceptionModuleInit:
    def test_init(self):
        pm = PerceptionModule(make_mock_node())
        assert pm.obstacles == []
        assert pm.detections == []
        assert pm.fused_objects == []
        assert pm.segmentation is None
        assert pm.vehicle_position is None

    def test_subscriptions_created(self):
        node = make_mock_node()
        PerceptionModule(node)
        assert node.create_subscription.call_count == 4


# ── GPS / IMU 콜백 ────────────────────────────────────────────


class TestGPSIMUCallbacks:
    @pytest.fixture
    def pm(self):
        return PerceptionModule(make_mock_node())

    def test_gps_callback(self, pm):
        msg = NavSatFix()
        msg.latitude = 37.5665
        msg.longitude = 126.9780
        msg.altitude = 50.0
        pm._gps_callback(msg)
        assert pm.vehicle_position['latitude'] == 37.5665
        assert pm.vehicle_position['longitude'] == 126.9780

    def test_imu_callback(self, pm):
        msg = Imu()
        msg.orientation.z = 0.707
        msg.orientation.w = 0.707
        pm._imu_callback(msg)
        assert pm.vehicle_orientation['z'] == pytest.approx(0.707)
        assert pm.vehicle_orientation['w'] == pytest.approx(0.707)


# ── LiDAR 처리 ────────────────────────────────────────────────


class TestLidarProcessing:
    @pytest.fixture
    def pm(self):
        return PerceptionModule(make_mock_node())

    def test_process_lidar_valid(self, pm):
        points = [(3.0, 0.0, 0.5), (3.1, 0.1, 0.6), (3.0, -0.1, 0.4)]
        obstacles = pm.process_lidar(make_pointcloud2(points))
        assert isinstance(obstacles, list)

    def test_process_lidar_empty(self, pm):
        assert pm.process_lidar(make_pointcloud2([])) == []

    def test_pointcloud2_to_numpy(self, pm):
        arr = pm._pointcloud2_to_numpy(
            make_pointcloud2([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]))
        assert arr.shape == (2, 3)
        assert arr[0, 0] == pytest.approx(1.0)
        assert arr[1, 2] == pytest.approx(6.0)

    def test_pointcloud2_missing_fields(self, pm):
        msg = PointCloud2()
        msg.fields = [PointField(name='intensity', offset=0,
                                 datatype=PointField.FLOAT32, count=1)]
        msg.point_step = 4
        msg.width = 1
        msg.height = 1
        msg.data = list(struct.pack('f', 1.0))
        assert pm._pointcloud2_to_numpy(msg).shape[0] == 0


# ── 카메라 처리 ────────────────────────────────────────────────


class TestCameraProcessing:
    @pytest.fixture
    def pm(self):
        return PerceptionModule(make_mock_node())

    def test_process_camera_rgb8(self, pm):
        assert isinstance(pm.process_camera(make_image(480, 640, 'rgb8')), list)

    def test_process_camera_mono8(self, pm):
        assert isinstance(pm.process_camera(make_image(480, 640, 'mono8')), list)

    def test_process_camera_rgba8(self, pm):
        assert isinstance(pm.process_camera(make_image(480, 640, 'rgba8')), list)

    def test_process_camera_empty(self, pm):
        msg = Image()
        msg.height = 0
        msg.width = 0
        msg.encoding = 'rgb8'
        msg.data = []
        assert pm.process_camera(msg) == []

    def test_image_to_numpy_rgb8(self, pm):
        arr = pm._image_to_numpy(make_image(2, 3, 'rgb8'))
        assert arr is not None
        assert arr.shape == (2, 3, 3)

    def test_image_to_numpy_unsupported(self, pm):
        msg = Image()
        msg.height = 2
        msg.width = 3
        msg.encoding = 'yuv422'
        msg.data = list(bytes([0] * 12))
        assert pm._image_to_numpy(msg) is None


# ── 센서 융합 ─────────────────────────────────────────────────


class TestSensorFusion:
    def test_fusion_runs(self):
        pm = PerceptionModule(make_mock_node())
        pm.process_lidar(make_pointcloud2([(3.0, 0.0, 0.5)]))
        pm.process_camera(make_image(480, 640))
        assert isinstance(pm._run_fusion(), list)

    def test_get_fused_objects_initially_empty(self):
        assert PerceptionModule(make_mock_node()).get_fused_objects() == []


# ── 시맨틱 세그멘테이션 ───────────────────────────────────────


class TestSegmentation:
    @pytest.fixture
    def pm(self):
        return PerceptionModule(make_mock_node())

    def test_segmentation_runs(self, pm):
        result = pm._run_segmentation(make_image(100, 100, 'rgb8'))
        assert result is not None
        assert result.width == 100
        assert result.height == 100

    def test_segmentation_empty_image(self, pm):
        msg = Image()
        msg.height = 0
        msg.width = 0
        msg.encoding = 'rgb8'
        msg.data = []
        assert pm._run_segmentation(msg) is None

    def test_get_navigable_area_none(self, pm):
        assert pm.get_navigable_area() is None

    def test_get_navigable_area_after_seg(self, pm):
        pm._run_segmentation(make_image(100, 100, 'rgb8'))
        assert pm.get_navigable_area() is not None

    def test_get_segmentation(self, pm):
        assert pm.get_segmentation() is None
        pm._run_segmentation(make_image(50, 50, 'rgb8'))
        assert pm.get_segmentation() is not None


# ── get_perception_result ─────────────────────────────────────


class TestGetPerceptionResult:
    def test_result_structure(self):
        result = PerceptionModule(make_mock_node()).get_perception_result()
        for key in ('obstacles', 'detections', 'fused_objects',
                    'segmentation', 'lane_info', 'position', 'orientation'):
            assert key in result
