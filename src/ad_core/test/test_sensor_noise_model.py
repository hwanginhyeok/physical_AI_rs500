"""SensorNoiseModel 단위 테스트."""

import math
import pytest

from ad_core.sensor_noise_model import (
    CameraNoiseConfig,
    CameraNoiseModel,
    GPSNoiseConfig,
    GPSNoiseModel,
    IMUNoiseConfig,
    IMUNoiseModel,
    LiDARNoiseConfig,
    LiDARNoiseModel,
    SensorNoiseBundle,
    SensorNoiseConfig,
)

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False


class TestGPSNoiseModel:
    @pytest.fixture
    def gps(self):
        cfg = GPSNoiseConfig(
            position_stddev=1.0,
            multipath_amplitude=0.2,
            dropout_probability=0.0,  # 드랍아웃 비활성
        )
        model = GPSNoiseModel(cfg)
        model.reset(seed=42)
        return model

    def test_noise_zero_mean(self, gps):
        """노이즈 평균이 0에 가까워야 한다."""
        errors_x = []
        errors_y = []
        for i in range(1000):
            nx, ny, valid = gps.apply(10.0, 10.0, 0.01)
            if valid:
                errors_x.append(nx - 10.0)
                errors_y.append(ny - 10.0)

        mean_x = sum(errors_x) / len(errors_x)
        mean_y = sum(errors_y) / len(errors_y)
        # 멀티패스 바이어스가 있으므로 완전 0은 아님, 하지만 범위 내
        assert abs(mean_x) < 1.0
        assert abs(mean_y) < 1.0

    def test_noise_variance(self, gps):
        """노이즈 분산이 설정된 sigma^2에 근접."""
        errors = []
        for _ in range(2000):
            nx, _, valid = gps.apply(0.0, 0.0, 0.01)
            if valid:
                errors.append(nx)

        variance = sum(e * e for e in errors) / len(errors)
        # sigma=1.0 + multipath → 분산이 1.0 근처
        assert 0.3 < variance < 3.0

    def test_dropout(self):
        """높은 드랍아웃 확률에서 일부가 무효."""
        cfg = GPSNoiseConfig(dropout_probability=0.5)
        gps = GPSNoiseModel(cfg)
        gps.reset(seed=42)

        valid_count = sum(
            1 for _ in range(100)
            if gps.apply(0, 0, 0.01)[2]
        )
        assert 20 < valid_count < 80  # 대략 50%

    def test_rtk_low_noise(self):
        """RTK 모드에서 노이즈가 작다."""
        cfg = GPSNoiseConfig(position_stddev=0.02, multipath_amplitude=0.005)
        gps = GPSNoiseModel(cfg)
        gps.reset(seed=42)

        max_err = 0
        for _ in range(100):
            nx, ny, _ = gps.apply(5.0, 5.0, 0.01)
            err = math.hypot(nx - 5.0, ny - 5.0)
            max_err = max(max_err, err)

        assert max_err < 0.2  # RTK: ~2cm + 작은 노이즈


class TestIMUNoiseModel:
    @pytest.fixture
    def imu(self):
        cfg = IMUNoiseConfig(
            gyro_noise_density=0.01,
            gyro_bias_instability=0.001,
        )
        model = IMUNoiseModel(cfg)
        model.reset(seed=42)
        return model

    def test_gyro_noise_zero_mean(self, imu):
        """자이로 노이즈 평균 ≈ 0."""
        errors = []
        for _ in range(2000):
            noisy = imu.apply_gyro(0.0, 0.01)
            errors.append(noisy)

        mean = sum(errors) / len(errors)
        assert abs(mean) < 0.1  # 바이어스 드리프트 포함해도

    def test_gyro_has_noise(self, imu):
        """자이로에 노이즈가 추가된다."""
        vals = set()
        for _ in range(10):
            vals.add(round(imu.apply_gyro(1.0, 0.01), 6))
        assert len(vals) > 1  # 모두 같지 않아야 함

    def test_bias_drift_accumulates(self, imu):
        """바이어스가 시간에 따라 변화한다."""
        bias_0 = imu.gyro_bias
        for _ in range(1000):
            imu.apply_gyro(0.0, 0.01)
        bias_after = imu.gyro_bias
        # 바이어스가 변했어야 함 (정확히 0은 극히 드뭄)
        assert bias_0 != bias_after

    def test_accel_noise(self, imu):
        """가속도계 노이즈가 추가된다."""
        ax, ay = imu.apply_accel(0.0, 0.0, 0.01)
        # 노이즈로 인해 정확히 0이 아님
        assert abs(ax) > 0 or abs(ay) > 0  # 최소한 하나는 0 아님


class TestLiDARNoiseModel:
    @pytest.fixture
    def lidar(self):
        cfg = LiDARNoiseConfig(
            range_stddev=0.02,
            dropout_probability=0.0,
        )
        model = LiDARNoiseModel(cfg)
        model.reset(seed=42)
        return model

    def test_range_noise_small(self, lidar):
        """거리 노이즈가 sigma 범위 내."""
        errors = []
        for _ in range(1000):
            noisy, valid = lidar.apply_range(10.0)
            if valid:
                errors.append(abs(noisy - 10.0))

        mean_err = sum(errors) / len(errors)
        assert mean_err < 0.1  # sigma=0.02

    def test_out_of_range(self, lidar):
        """범위 밖 거리는 무효."""
        _, valid_near = lidar.apply_range(0.01)  # < min_range
        _, valid_far = lidar.apply_range(100.0)   # > max_range
        assert not valid_near
        assert not valid_far

    def test_rain_increases_dropout(self):
        """비 조건에서 드랍아웃 증가."""
        cfg = LiDARNoiseConfig(dropout_probability=0.01, rain_dropout_extra=0.2)
        lidar = LiDARNoiseModel(cfg)
        lidar.reset(seed=42)

        # 맑은 날
        valid_clear = sum(
            1 for _ in range(500)
            if lidar.apply_range(5.0)[1]
        )
        lidar.reset(seed=42)
        lidar.set_weather(rain=True)
        valid_rain = sum(
            1 for _ in range(500)
            if lidar.apply_range(5.0)[1]
        )
        assert valid_rain < valid_clear

    def test_scan(self, lidar):
        """전체 스캔 처리."""
        ranges = [5.0, 10.0, 15.0, 25.0]
        results = lidar.apply_scan(ranges)
        assert len(results) == 4
        for r, v in results:
            assert isinstance(r, float)
            assert isinstance(v, bool)


@pytest.mark.skipif(not _HAS_NUMPY, reason="numpy required")
class TestCameraNoiseModel:
    def test_noise_added(self):
        cam = CameraNoiseModel(CameraNoiseConfig(pixel_noise_stddev=10.0))
        cam.reset(seed=42)
        img = np.ones((10, 10, 3), dtype=np.uint8) * 128
        noisy = cam.apply(img, vehicle_speed=0.0)
        assert noisy.shape == img.shape
        # 노이즈가 추가되어 원본과 다름
        assert not np.array_equal(noisy, img)


class TestSensorNoiseBundle:
    def test_bundle_creation(self):
        bundle = SensorNoiseBundle()
        assert bundle.gps is not None
        assert bundle.imu is not None
        assert bundle.lidar is not None
        assert bundle.camera is not None

    def test_reset(self):
        bundle = SensorNoiseBundle()
        bundle.reset(seed=42)
        # 리셋 후 정상 동작
        x, y, valid = bundle.gps.apply(1.0, 1.0, 0.01)
        assert isinstance(x, float)
