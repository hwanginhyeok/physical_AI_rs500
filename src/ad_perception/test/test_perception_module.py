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


def make_mock_node(extra_params=None):
    params = {
        'topics.camera': '/sensor/camera/front',
        'topics.lidar': '/sensor/lidar',
        'topics.gps': '/sensor/gps',
        'topics.imu': '/sensor/imu',
        'yolo.model_path': '',
        'yolo.confidence_threshold': 0.5,
        'crop_row.orchard_type': 'PEAR',
        'crop_row.enabled': False,
        'crop_row.process_every_n_frames': 3,
        'crop_row.no_detect_limit': 10,
    }
    if extra_params:
        params.update(extra_params)
    node = MagicMock()
    node.get_parameter = lambda name: MockParameter(params.get(name, ''))
    node.declare_parameter = lambda name, default: MockParameter(params.get(name, default))
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

    def test_result_includes_crop_row_when_enabled(self):
        """crop_row 활성화 + 감지 결과 있을 때 perception_result에 포함."""
        # process_every_n_frames=1: 첫 호출에서 바로 감지 실행
        pm = PerceptionModule(make_mock_node({
            'crop_row.enabled': True,
            'crop_row.process_every_n_frames': 1,
        }))
        # 초록색 세로 줄무늬 이미지 생성 (행 감지 가능)
        img = _make_green_stripe_image(480, 640)
        msg = _numpy_to_image_msg(img)
        pm._run_crop_row_detection(msg)
        result = pm.get_perception_result()
        assert 'crop_row' in result
        assert 'crop_row_steering_offset' in result
        assert 'crop_row_heading_error' in result
        assert 'crop_row_end_detected' in result

    def test_result_no_crop_row_when_disabled(self):
        """crop_row 비활성화 시 perception_result에 미포함."""
        pm = PerceptionModule(make_mock_node({'crop_row.enabled': False}))
        result = pm.get_perception_result()
        assert 'crop_row' not in result


# ── 과수원 행 감지 ─────────────────────────────────────────


def _make_green_stripe_image(h, w):
    """초록색 세로 줄무늬 이미지 생성. 행 감지용."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    # 토양 배경 (갈색)
    img[:, :] = [100, 80, 60]  # BGR
    # 초록 줄무늬 (행 2개: x=160, x=480)
    for cx in [160, 480]:
        x_start = max(0, cx - 40)
        x_end = min(w, cx + 40)
        img[:, x_start:x_end] = [30, 150, 50]  # 초록
    return img


def _numpy_to_image_msg(arr):
    """numpy 배열을 Image 메시지로 변환."""
    msg = Image()
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = 'bgr8'
    msg.data = list(arr.tobytes())
    return msg


class TestCropRowDetection:
    @pytest.fixture
    def pm_enabled(self):
        return PerceptionModule(make_mock_node({'crop_row.enabled': True}))

    @pytest.fixture
    def pm_disabled(self):
        return PerceptionModule(make_mock_node({'crop_row.enabled': False}))

    def test_disabled_returns_none(self, pm_disabled):
        """비활성화 시 감지 안 함."""
        msg = make_image(480, 640, 'rgb8')
        assert pm_disabled._run_crop_row_detection(msg) is None

    def test_enabled_returns_result(self, pm_enabled):
        """활성화 + 첫 프레임(스로틀 통과 안 함) → 이전 결과(None)."""
        msg = make_image(480, 640, 'rgb8')
        # 첫 프레임(count=1)은 process_every_n=3의 배수가 아님
        result = pm_enabled._run_crop_row_detection(msg)
        assert result is None  # 첫 프레임은 스킵

    def test_throttle_every_n_frames(self, pm_enabled):
        """process_every_n_frames=3 → 3번째 프레임에서 실행."""
        msg = _numpy_to_image_msg(_make_green_stripe_image(480, 640))
        # 프레임 1, 2 스킵
        pm_enabled._run_crop_row_detection(msg)
        pm_enabled._run_crop_row_detection(msg)
        # 프레임 3: 실행
        result = pm_enabled._run_crop_row_detection(msg)
        assert result is not None
        assert pm_enabled.crop_row_result is not None

    def test_green_stripe_detects_rows(self, pm_enabled):
        """초록 줄무늬 이미지에서 행을 감지."""
        msg = _numpy_to_image_msg(_make_green_stripe_image(480, 640))
        # 3프레임 돌려서 실행
        for _ in range(3):
            pm_enabled._run_crop_row_detection(msg)
        result = pm_enabled.crop_row_result
        assert result is not None
        assert result.num_rows >= 1

    def test_empty_image_no_rows(self, pm_enabled):
        """빈(검은) 이미지에서 행 미감지."""
        black_img = np.zeros((480, 640, 3), dtype=np.uint8)
        msg = _numpy_to_image_msg(black_img)
        for _ in range(3):
            pm_enabled._run_crop_row_detection(msg)
        result = pm_enabled.crop_row_result
        assert result is not None
        assert result.num_rows == 0

    def test_crop_row_end_detection(self, pm_enabled):
        """연속 N회 미감지 시 행 끝 판정."""
        black_img = np.zeros((480, 640, 3), dtype=np.uint8)
        msg = _numpy_to_image_msg(black_img)
        assert not pm_enabled.crop_row_end_detected
        # no_detect_limit=10, process_every_n=3 → 30프레임 필요
        for _ in range(30):
            pm_enabled._run_crop_row_detection(msg)
        assert pm_enabled.crop_row_end_detected

    def test_crop_row_end_resets_on_detection(self, pm_enabled):
        """행 감지되면 미감지 카운터 리셋."""
        black = np.zeros((480, 640, 3), dtype=np.uint8)
        green = _make_green_stripe_image(480, 640)
        # 미감지 카운터 올리기
        for _ in range(15):
            pm_enabled._run_crop_row_detection(_numpy_to_image_msg(black))
        count_before = pm_enabled._crop_row_no_detect_count
        assert count_before > 0
        # 행 감지되면 리셋
        for _ in range(3):
            pm_enabled._run_crop_row_detection(_numpy_to_image_msg(green))
        if pm_enabled.crop_row_result and pm_enabled.crop_row_result.num_rows > 0:
            assert pm_enabled._crop_row_no_detect_count == 0
