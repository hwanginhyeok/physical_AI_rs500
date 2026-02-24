"""센서 노이즈 모델 모듈.

GPS, IMU, LiDAR, Camera 각 센서별 현실적 노이즈 모델을 제공한다.
Allan Variance 기반 IMU 노이즈, 멀티패스 GPS 바이어스,
LiDAR 포인트 드랍아웃 등을 포함한다.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False


# ====================================================================== #
#  GPS 노이즈 모델
# ====================================================================== #


@dataclass
class GPSNoiseConfig:
    """GPS 노이즈 파라미터.

    Attributes:
        position_stddev: 위치 표준편차 (m). 단독=2.5, RTK=0.02.
        multipath_amplitude: 멀티패스 바이어스 진폭 (m).
        multipath_period: 멀티패스 변동 주기 (s).
        dropout_probability: 신호 단절 확률 (0~1). 매 호출당.
        update_rate: GPS 갱신 주기 (Hz).
    """

    position_stddev: float = 2.5
    multipath_amplitude: float = 0.5
    multipath_period: float = 30.0
    dropout_probability: float = 0.01
    update_rate: float = 10.0


class GPSNoiseModel:
    """GPS 센서 노이즈 생성기."""

    def __init__(self, config: GPSNoiseConfig | None = None) -> None:
        self.config = config or GPSNoiseConfig()
        self._time: float = 0.0
        self._multipath_bias_x: float = 0.0
        self._multipath_bias_y: float = 0.0
        self._rng = random.Random()
        self._last_valid = True

    def reset(self, seed: int | None = None) -> None:
        """상태를 초기화한다."""
        self._time = 0.0
        self._multipath_bias_x = 0.0
        self._multipath_bias_y = 0.0
        if seed is not None:
            self._rng = random.Random(seed)
        self._last_valid = True

    def apply(
        self, true_x: float, true_y: float, dt: float
    ) -> Tuple[float, float, bool]:
        """참값에 GPS 노이즈를 적용한다.

        Args:
            true_x: 참 x 좌표 (m).
            true_y: 참 y 좌표 (m).
            dt: 시간 간격 (s).

        Returns:
            (noisy_x, noisy_y, is_valid). is_valid=False면 드랍아웃.
        """
        self._time += dt
        cfg = self.config

        # 드랍아웃 체크
        if self._rng.random() < cfg.dropout_probability:
            self._last_valid = False
            return true_x, true_y, False

        self._last_valid = True

        # 멀티패스 바이어스 (저주파 랜덤워크)
        period = max(cfg.multipath_period, 1e-6)
        phase = 2.0 * math.pi * self._time / period
        self._multipath_bias_x = cfg.multipath_amplitude * math.sin(phase + 0.3)
        self._multipath_bias_y = cfg.multipath_amplitude * math.cos(phase + 1.7)

        # 백색 노이즈
        noise_x = self._rng.gauss(0.0, cfg.position_stddev)
        noise_y = self._rng.gauss(0.0, cfg.position_stddev)

        noisy_x = true_x + noise_x + self._multipath_bias_x
        noisy_y = true_y + noise_y + self._multipath_bias_y

        return noisy_x, noisy_y, True


# ====================================================================== #
#  IMU 노이즈 모델
# ====================================================================== #


@dataclass
class IMUNoiseConfig:
    """IMU 노이즈 파라미터 (Allan Variance 기반).

    Attributes:
        gyro_noise_density: 자이로 노이즈 밀도 ARW (rad/s/sqrt(Hz)).
        gyro_bias_instability: 자이로 바이어스 불안정성 (rad/s).
        gyro_bias_time_constant: 바이어스 드리프트 시정수 (s).
        accel_noise_density: 가속도계 노이즈 밀도 VRW (m/s^2/sqrt(Hz)).
        accel_bias_instability: 가속도계 바이어스 불안정성 (m/s^2).
        accel_bias_time_constant: 바이어스 드리프트 시정수 (s).
    """

    gyro_noise_density: float = 0.01      # rad/s/sqrt(Hz)
    gyro_bias_instability: float = 0.001  # rad/s
    gyro_bias_time_constant: float = 100.0  # s
    accel_noise_density: float = 0.02     # m/s^2/sqrt(Hz)
    accel_bias_instability: float = 0.005  # m/s^2
    accel_bias_time_constant: float = 200.0  # s


class IMUNoiseModel:
    """IMU 센서 노이즈 생성기.

    자이로와 가속도계 각각에 대해:
    - White noise (ARW/VRW)
    - 1차 마르코프 프로세스 바이어스 드리프트
    """

    def __init__(self, config: IMUNoiseConfig | None = None) -> None:
        self.config = config or IMUNoiseConfig()
        self._gyro_bias: float = 0.0
        self._accel_bias_x: float = 0.0
        self._accel_bias_y: float = 0.0
        self._rng = random.Random()

    def reset(self, seed: int | None = None) -> None:
        self._gyro_bias = 0.0
        self._accel_bias_x = 0.0
        self._accel_bias_y = 0.0
        if seed is not None:
            self._rng = random.Random(seed)

    def apply_gyro(self, true_yaw_rate: float, dt: float) -> float:
        """자이로 노이즈를 적용한다.

        Args:
            true_yaw_rate: 참 yaw rate (rad/s).
            dt: 시간 간격 (s).

        Returns:
            노이즈가 추가된 yaw rate (rad/s).
        """
        cfg = self.config

        # 바이어스 드리프트 (1차 마르코프 프로세스)
        tau = max(cfg.gyro_bias_time_constant, 1e-6)
        alpha = 1.0 - math.exp(-dt / tau)
        bias_noise = self._rng.gauss(0.0, cfg.gyro_bias_instability)
        self._gyro_bias = (1.0 - alpha) * self._gyro_bias + alpha * bias_noise

        # 백색 노이즈 (ARW)
        # 이산화: sigma = noise_density / sqrt(dt)
        sigma = cfg.gyro_noise_density / max(math.sqrt(dt), 1e-6)
        white_noise = self._rng.gauss(0.0, sigma)

        return true_yaw_rate + white_noise + self._gyro_bias

    def apply_accel(
        self, true_accel_x: float, true_accel_y: float, dt: float
    ) -> Tuple[float, float]:
        """가속도계 노이즈를 적용한다.

        Args:
            true_accel_x: 참 x 가속도 (m/s^2).
            true_accel_y: 참 y 가속도 (m/s^2).
            dt: 시간 간격 (s).

        Returns:
            (noisy_accel_x, noisy_accel_y).
        """
        cfg = self.config

        # 바이어스 드리프트
        tau = max(cfg.accel_bias_time_constant, 1e-6)
        alpha = 1.0 - math.exp(-dt / tau)
        self._accel_bias_x = (1.0 - alpha) * self._accel_bias_x + alpha * self._rng.gauss(
            0.0, cfg.accel_bias_instability
        )
        self._accel_bias_y = (1.0 - alpha) * self._accel_bias_y + alpha * self._rng.gauss(
            0.0, cfg.accel_bias_instability
        )

        # 백색 노이즈 (VRW)
        sigma = cfg.accel_noise_density / max(math.sqrt(dt), 1e-6)
        noise_x = self._rng.gauss(0.0, sigma)
        noise_y = self._rng.gauss(0.0, sigma)

        return (
            true_accel_x + noise_x + self._accel_bias_x,
            true_accel_y + noise_y + self._accel_bias_y,
        )

    @property
    def gyro_bias(self) -> float:
        """현재 자이로 바이어스 (rad/s)."""
        return self._gyro_bias


# ====================================================================== #
#  LiDAR 노이즈 모델
# ====================================================================== #


@dataclass
class LiDARNoiseConfig:
    """LiDAR 노이즈 파라미터.

    Attributes:
        range_stddev: 거리 측정 노이즈 표준편차 (m).
        max_range: 최대 측정 거리 (m).
        min_range: 최소 측정 거리 (m).
        dropout_probability: 포인트 드랍아웃 확률 (0~1).
        rain_dropout_extra: 비 조건에서 추가 드랍아웃 확률.
        dust_dropout_extra: 먼지 조건에서 추가 드랍아웃 확률.
        angular_resolution: 각도 해상도 (deg).
    """

    range_stddev: float = 0.02
    max_range: float = 30.0
    min_range: float = 0.1
    dropout_probability: float = 0.01
    rain_dropout_extra: float = 0.05
    dust_dropout_extra: float = 0.10
    angular_resolution: float = 0.25


class LiDARNoiseModel:
    """LiDAR 센서 노이즈 생성기."""

    def __init__(self, config: LiDARNoiseConfig | None = None) -> None:
        self.config = config or LiDARNoiseConfig()
        self._rng = random.Random()
        self._rain_active: bool = False
        self._dust_active: bool = False

    def reset(self, seed: int | None = None) -> None:
        if seed is not None:
            self._rng = random.Random(seed)
        self._rain_active = False
        self._dust_active = False

    def set_weather(self, rain: bool = False, dust: bool = False) -> None:
        """기상 조건을 설정한다."""
        self._rain_active = rain
        self._dust_active = dust

    def apply_range(self, true_range: float) -> Tuple[float, bool]:
        """단일 거리 측정에 노이즈를 적용한다.

        Args:
            true_range: 참 거리 (m).

        Returns:
            (noisy_range, is_valid). is_valid=False면 드랍아웃.
        """
        cfg = self.config

        # 범위 밖이면 무효
        if true_range < cfg.min_range or true_range > cfg.max_range:
            return 0.0, False

        # 드랍아웃 확률 계산
        dropout = cfg.dropout_probability
        if self._rain_active:
            dropout += cfg.rain_dropout_extra
        if self._dust_active:
            dropout += cfg.dust_dropout_extra

        if self._rng.random() < dropout:
            return 0.0, False

        # 거리 노이즈 (가우시안)
        noise = self._rng.gauss(0.0, cfg.range_stddev)
        noisy_range = true_range + noise

        # 반사율에 의한 최대 거리 감소 (먼 거리일수록 드랍 확률 증가)
        distance_dropout = (true_range / cfg.max_range) ** 2 * 0.05
        if self._rng.random() < distance_dropout:
            return 0.0, False

        return max(cfg.min_range, noisy_range), True

    def apply_scan(self, true_ranges: List[float]) -> List[Tuple[float, bool]]:
        """전체 스캔에 노이즈를 적용한다.

        Args:
            true_ranges: 참 거리값 리스트.

        Returns:
            [(noisy_range, is_valid), ...] 리스트.
        """
        return [self.apply_range(r) for r in true_ranges]


# ====================================================================== #
#  Camera 노이즈 모델 (선택적)
# ====================================================================== #


@dataclass
class CameraNoiseConfig:
    """카메라 노이즈 파라미터.

    Attributes:
        pixel_noise_stddev: 픽셀 가우시안 노이즈 표준편차 (0~255 스케일).
        motion_blur_coeff: 모션 블러 계수. 속도에 비례하여 블러 커널 크기 결정.
        exposure_variation: 노출 변동 (0~1). 밝기 스케일링 변동.
    """

    pixel_noise_stddev: float = 5.0
    motion_blur_coeff: float = 2.0
    exposure_variation: float = 0.05


class CameraNoiseModel:
    """카메라 센서 노이즈 생성기. numpy가 필요하다."""

    def __init__(self, config: CameraNoiseConfig | None = None) -> None:
        self.config = config or CameraNoiseConfig()
        self._rng = random.Random()

    def reset(self, seed: int | None = None) -> None:
        if seed is not None:
            self._rng = random.Random(seed)

    def apply(
        self,
        image: "np.ndarray",
        vehicle_speed: float = 0.0,
    ) -> "np.ndarray":
        """이미지에 카메라 노이즈를 적용한다.

        Args:
            image: (H, W, 3) uint8 이미지.
            vehicle_speed: 차량 속도 (m/s). 모션 블러에 사용.

        Returns:
            노이즈가 추가된 이미지.
        """
        if not _HAS_NUMPY:
            return image

        cfg = self.config
        noisy = image.astype(np.float32)

        # 1. 가우시안 픽셀 노이즈
        noise = np.random.normal(0.0, cfg.pixel_noise_stddev, noisy.shape)
        noisy += noise

        # 2. 노출 변동
        exposure_scale = 1.0 + self._rng.gauss(0.0, cfg.exposure_variation)
        noisy *= max(0.5, min(1.5, exposure_scale))

        # 3. 모션 블러 (간소화: 수평 방향 평균 필터)
        if vehicle_speed > 0.1 and cfg.motion_blur_coeff > 0:
            kernel_size = max(1, int(cfg.motion_blur_coeff * vehicle_speed))
            if kernel_size > 1:
                # 수평 이동 평균
                kernel = np.ones(kernel_size) / kernel_size
                for c in range(min(3, noisy.shape[2] if noisy.ndim > 2 else 1)):
                    if noisy.ndim > 2:
                        for row in range(noisy.shape[0]):
                            noisy[row, :, c] = np.convolve(
                                noisy[row, :, c], kernel, mode='same'
                            )

        # 클램프 & 형변환
        noisy = np.clip(noisy, 0, 255).astype(np.uint8)
        return noisy


# ====================================================================== #
#  통합 노이즈 설정
# ====================================================================== #


@dataclass
class SensorNoiseConfig:
    """전체 센서 노이즈 통합 설정."""

    gps: GPSNoiseConfig = field(default_factory=GPSNoiseConfig)
    imu: IMUNoiseConfig = field(default_factory=IMUNoiseConfig)
    lidar: LiDARNoiseConfig = field(default_factory=LiDARNoiseConfig)
    camera: CameraNoiseConfig = field(default_factory=CameraNoiseConfig)
    enabled: bool = True


class SensorNoiseBundle:
    """모든 센서 노이즈 모델을 관리하는 번들."""

    def __init__(self, config: SensorNoiseConfig | None = None) -> None:
        self.config = config or SensorNoiseConfig()
        self.gps = GPSNoiseModel(self.config.gps)
        self.imu = IMUNoiseModel(self.config.imu)
        self.lidar = LiDARNoiseModel(self.config.lidar)
        self.camera = CameraNoiseModel(self.config.camera)

    def reset(self, seed: int | None = None) -> None:
        """모든 센서를 초기화한다."""
        self.gps.reset(seed)
        self.imu.reset(seed + 1 if seed is not None else None)
        self.lidar.reset(seed + 2 if seed is not None else None)
        self.camera.reset(seed + 3 if seed is not None else None)
