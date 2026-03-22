"""과수원 작물 행 감지 모듈.

카메라 이미지에서 과수 행(나무 열)을 감지하고,
행간 주행 경로를 생성한다.

Phase 1: Classical CV (색상 세그멘테이션 + Hough 라인 + 행 피팅)
Phase 2: DL 모델 교체 예정 (인터페이스 동일)

지원 과수: 사과, 배, 감 등 (프로파일 기반 확장)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Protocol, Tuple

import numpy as np


# ══════════════════════════════════════════════════════════════
# 데이터 타입
# ══════════════════════════════════════════════════════════════


class OrchardType(Enum):
    """과수원 종류."""
    APPLE = auto()       # 사과
    PEAR = auto()        # 배
    PERSIMMON = auto()   # 감
    PEACH = auto()       # 복숭아
    GRAPE = auto()       # 포도
    GENERIC = auto()     # 범용


@dataclass
class ColorRange:
    """HSV 색상 범위."""
    h_min: int = 0
    h_max: int = 180
    s_min: int = 0
    s_max: int = 255
    v_min: int = 0
    v_max: int = 255

    def as_lower(self) -> np.ndarray:
        return np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)

    def as_upper(self) -> np.ndarray:
        return np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)


@dataclass
class OrchardProfile:
    """과수원 유형별 감지 파라미터."""
    orchard_type: OrchardType
    canopy_color: List[ColorRange] = field(default_factory=list)
    trunk_color: List[ColorRange] = field(default_factory=list)
    soil_color: List[ColorRange] = field(default_factory=list)
    expected_row_spacing_m: float = 4.0
    expected_tree_spacing_m: float = 2.5
    min_canopy_ratio: float = 0.05
    description: str = ""


@dataclass
class CropRow:
    """감지된 작물 행."""
    x_center_px: float         # 이미지에서 행 중심 X 좌표 (px)
    angle_deg: float           # 행 각도 (deg, 수직=0)
    width_px: float            # 행 폭 (px)
    confidence: float          # 신뢰도 (0~1)
    row_index: int = 0         # 행 번호

    @property
    def angle_rad(self) -> float:
        return math.radians(self.angle_deg)


@dataclass
class RowDetectionResult:
    """행 감지 결과."""
    rows: List[CropRow]
    nav_path_px: List[Tuple[float, float]]  # 행간 주행 경로 (px 좌표)
    vegetation_mask: Optional[np.ndarray]    # 초목 마스크
    image_width: int
    image_height: int
    orchard_type: OrchardType
    processing_time_ms: float = 0.0

    @property
    def num_rows(self) -> int:
        return len(self.rows)

    @property
    def has_nav_path(self) -> bool:
        return len(self.nav_path_px) > 0

    def get_steering_offset(self) -> float:
        """이미지 중심 대비 주행 경로 오프셋 (-1~1, 좌=-1, 우=+1)."""
        if not self.nav_path_px:
            return 0.0
        center_x = self.image_width / 2.0
        path_xs = [p[0] for p in self.nav_path_px]
        avg_x = sum(path_xs) / len(path_xs)
        return max(-1.0, min(1.0, (avg_x - center_x) / center_x))

    def get_heading_error_deg(self) -> float:
        """행 기준 차량 방향 오차 (deg). 행이 수직이면 0."""
        if not self.rows:
            return 0.0
        # 신뢰도 가중 평균 각도
        total_w = sum(r.confidence for r in self.rows)
        if total_w < 1e-6:
            return 0.0
        avg_angle = sum(r.angle_deg * r.confidence for r in self.rows) / total_w
        return avg_angle


# ══════════════════════════════════════════════════════════════
# 과수원 프로파일
# ══════════════════════════════════════════════════════════════


def get_default_profiles() -> dict[OrchardType, OrchardProfile]:
    """기본 과수원 프로파일 반환."""
    return {
        OrchardType.APPLE: OrchardProfile(
            orchard_type=OrchardType.APPLE,
            canopy_color=[
                ColorRange(h_min=25, h_max=90, s_min=30, s_max=255, v_min=30, v_max=255),
            ],
            trunk_color=[
                ColorRange(h_min=5, h_max=25, s_min=20, s_max=180, v_min=30, v_max=180),
            ],
            soil_color=[
                ColorRange(h_min=5, h_max=30, s_min=10, s_max=150, v_min=40, v_max=200),
            ],
            expected_row_spacing_m=4.0,
            expected_tree_spacing_m=2.5,
            description="사과 과수원 (왜성대목, 행간 3.5~4.5m)",
        ),
        OrchardType.PEAR: OrchardProfile(
            orchard_type=OrchardType.PEAR,
            canopy_color=[
                ColorRange(h_min=25, h_max=85, s_min=25, s_max=255, v_min=35, v_max=255),
            ],
            trunk_color=[
                ColorRange(h_min=5, h_max=25, s_min=15, s_max=160, v_min=30, v_max=170),
            ],
            soil_color=[
                ColorRange(h_min=5, h_max=30, s_min=10, s_max=150, v_min=40, v_max=200),
            ],
            expected_row_spacing_m=5.0,
            expected_tree_spacing_m=3.0,
            description="배 과수원 (행간 4.5~5.5m)",
        ),
        OrchardType.PERSIMMON: OrchardProfile(
            orchard_type=OrchardType.PERSIMMON,
            canopy_color=[
                ColorRange(h_min=20, h_max=85, s_min=25, s_max=255, v_min=30, v_max=255),
            ],
            trunk_color=[
                ColorRange(h_min=5, h_max=25, s_min=15, s_max=150, v_min=25, v_max=160),
            ],
            soil_color=[
                ColorRange(h_min=5, h_max=30, s_min=10, s_max=150, v_min=40, v_max=200),
            ],
            expected_row_spacing_m=5.5,
            expected_tree_spacing_m=4.0,
            description="감 과수원 (행간 5~6m)",
        ),
        OrchardType.PEACH: OrchardProfile(
            orchard_type=OrchardType.PEACH,
            canopy_color=[
                ColorRange(h_min=25, h_max=90, s_min=25, s_max=255, v_min=30, v_max=255),
            ],
            trunk_color=[
                ColorRange(h_min=5, h_max=25, s_min=15, s_max=160, v_min=25, v_max=170),
            ],
            soil_color=[
                ColorRange(h_min=5, h_max=30, s_min=10, s_max=150, v_min=40, v_max=200),
            ],
            expected_row_spacing_m=4.5,
            expected_tree_spacing_m=3.0,
            description="복숭아 과수원 (행간 4~5m)",
        ),
        OrchardType.GRAPE: OrchardProfile(
            orchard_type=OrchardType.GRAPE,
            canopy_color=[
                ColorRange(h_min=25, h_max=95, s_min=30, s_max=255, v_min=25, v_max=255),
            ],
            trunk_color=[
                ColorRange(h_min=5, h_max=25, s_min=10, s_max=140, v_min=20, v_max=150),
            ],
            soil_color=[
                ColorRange(h_min=5, h_max=30, s_min=10, s_max=150, v_min=40, v_max=200),
            ],
            expected_row_spacing_m=3.0,
            expected_tree_spacing_m=2.0,
            description="포도 과수원 (덕식, 행간 2.5~3.5m)",
        ),
        OrchardType.GENERIC: OrchardProfile(
            orchard_type=OrchardType.GENERIC,
            canopy_color=[
                ColorRange(h_min=20, h_max=95, s_min=20, s_max=255, v_min=25, v_max=255),
            ],
            trunk_color=[
                ColorRange(h_min=5, h_max=30, s_min=10, s_max=180, v_min=20, v_max=180),
            ],
            soil_color=[
                ColorRange(h_min=5, h_max=35, s_min=10, s_max=160, v_min=35, v_max=210),
            ],
            expected_row_spacing_m=4.0,
            expected_tree_spacing_m=3.0,
            description="범용 과수원 프로파일",
        ),
    }


# ══════════════════════════════════════════════════════════════
# 감지 인터페이스 (Protocol)
# ══════════════════════════════════════════════════════════════


class CropRowDetectorInterface(Protocol):
    """작물 행 감지기 인터페이스. DL 모델 교체 시 이 인터페이스를 구현."""

    def detect(self, image: np.ndarray) -> RowDetectionResult:
        ...

    def set_orchard_type(self, orchard_type: OrchardType) -> None:
        ...


# ══════════════════════════════════════════════════════════════
# Classical CV 감지기
# ══════════════════════════════════════════════════════════════


@dataclass
class CVDetectorConfig:
    """Classical CV 감지기 설정."""
    morph_kernel_size: int = 7
    min_row_height_ratio: float = 0.3
    hough_threshold: int = 50
    hough_min_line_ratio: float = 0.2
    hough_max_gap_ratio: float = 0.05
    angle_tolerance_deg: float = 30.0
    min_peak_distance_px: int = 30
    min_peak_prominence: float = 0.1
    roi_top_ratio: float = 0.2
    roi_bottom_ratio: float = 0.95


class CropRowDetectorCV:
    """Classical CV 기반 과수원 작물 행 감지기.

    파이프라인:
    1. HSV 변환 + 초목 마스크 생성 (프로파일 기반)
    2. 모폴로지 연산으로 노이즈 제거
    3. 수직 투영 히스토그램으로 행 후보 탐색
    4. 피크 검출로 행 중심 결정
    5. Hough 라인으로 행 각도 보정
    6. 행간 중심선 계산 → 주행 경로 생성
    """

    def __init__(
        self,
        orchard_type: OrchardType = OrchardType.GENERIC,
        config: CVDetectorConfig | None = None,
        profiles: dict[OrchardType, OrchardProfile] | None = None,
    ) -> None:
        self._config = config or CVDetectorConfig()
        self._profiles = profiles or get_default_profiles()
        self._orchard_type = orchard_type
        self._profile = self._profiles.get(orchard_type, self._profiles[OrchardType.GENERIC])

    @property
    def orchard_type(self) -> OrchardType:
        return self._orchard_type

    @property
    def profile(self) -> OrchardProfile:
        return self._profile

    def set_orchard_type(self, orchard_type: OrchardType) -> None:
        self._orchard_type = orchard_type
        self._profile = self._profiles.get(orchard_type, self._profiles[OrchardType.GENERIC])

    def detect(self, image: np.ndarray) -> RowDetectionResult:
        """이미지에서 작물 행을 감지한다.

        Args:
            image: (H, W, 3) BGR 또는 RGB 이미지.

        Returns:
            RowDetectionResult.
        """
        import time
        t0 = time.monotonic()

        h, w = image.shape[:2]
        cfg = self._config

        # 1. ROI 추출 (상단 하늘/하단 차체 제외)
        roi_top = int(h * cfg.roi_top_ratio)
        roi_bottom = int(h * cfg.roi_bottom_ratio)
        roi = image[roi_top:roi_bottom, :]
        roi_h = roi_bottom - roi_top

        # 2. 초목 마스크 생성
        veg_mask = self._create_vegetation_mask(roi)

        # 3. 모폴로지 정리
        kernel = np.ones((cfg.morph_kernel_size, cfg.morph_kernel_size), np.uint8)
        veg_mask = self._morphology_close(veg_mask, kernel)
        veg_mask = self._morphology_open(veg_mask, kernel)

        # 4. 수직 투영 히스토그램
        projection = veg_mask.sum(axis=0).astype(np.float64)
        if projection.max() > 0:
            projection = projection / projection.max()

        # 5. 피크 검출 → 행 후보
        peaks = self._find_peaks(
            projection,
            min_distance=cfg.min_peak_distance_px,
            min_prominence=cfg.min_peak_prominence,
        )

        # 6. 행 각도 추정 (Hough 라인)
        avg_angle = self._estimate_row_angle(veg_mask, cfg)

        # 7. CropRow 객체 생성
        rows: List[CropRow] = []
        for i, peak_x in enumerate(peaks):
            # 피크 주변 폭 계산
            width = self._estimate_row_width(projection, peak_x)
            confidence = float(projection[peak_x]) if peak_x < len(projection) else 0.0

            rows.append(CropRow(
                x_center_px=float(peak_x),
                angle_deg=avg_angle,
                width_px=float(width),
                confidence=confidence,
                row_index=i,
            ))

        # 신뢰도 순 정렬
        rows.sort(key=lambda r: r.x_center_px)

        # 8. 행간 주행 경로 생성
        nav_path = self._generate_nav_path(rows, w, roi_top, roi_bottom)

        # 전체 이미지 크기 마스크로 복원
        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[roi_top:roi_bottom, :] = veg_mask

        elapsed = (time.monotonic() - t0) * 1000.0

        return RowDetectionResult(
            rows=rows,
            nav_path_px=nav_path,
            vegetation_mask=full_mask,
            image_width=w,
            image_height=h,
            orchard_type=self._orchard_type,
            processing_time_ms=elapsed,
        )

    # ── 내부 메서드 ───────────────────────────────────────────

    def _create_vegetation_mask(self, image: np.ndarray) -> np.ndarray:
        """HSV 색상 기반 초목 마스크 생성."""
        # ExG (Excess Green) 지수 기반 빠른 초목 감지
        img_f = image.astype(np.float32)
        if img_f.shape[2] == 3:
            r, g, b = img_f[:, :, 0], img_f[:, :, 1], img_f[:, :, 2]
            total = r + g + b + 1e-6
            exg = 2.0 * g / total - r / total - b / total
            mask_exg = (exg > 0.05).astype(np.uint8) * 255
        else:
            mask_exg = np.zeros(image.shape[:2], dtype=np.uint8)

        # HSV 색상 범위 마스크
        hsv = self._rgb_to_hsv(image)
        mask_hsv = np.zeros(image.shape[:2], dtype=np.uint8)
        for color_range in self._profile.canopy_color:
            lower = color_range.as_lower()
            upper = color_range.as_upper()
            in_range = (
                (hsv[:, :, 0] >= lower[0]) & (hsv[:, :, 0] <= upper[0]) &
                (hsv[:, :, 1] >= lower[1]) & (hsv[:, :, 1] <= upper[1]) &
                (hsv[:, :, 2] >= lower[2]) & (hsv[:, :, 2] <= upper[2])
            )
            mask_hsv = mask_hsv | (in_range.astype(np.uint8) * 255)

        # ExG와 HSV 마스크 결합 (OR)
        combined = np.maximum(mask_exg, mask_hsv)
        return combined

    @staticmethod
    def _rgb_to_hsv(image: np.ndarray) -> np.ndarray:
        """RGB/BGR → HSV 변환 (OpenCV 없이 numpy로 구현)."""
        img_f = image.astype(np.float32) / 255.0
        r, g, b = img_f[:, :, 0], img_f[:, :, 1], img_f[:, :, 2]

        cmax = np.maximum(np.maximum(r, g), b)
        cmin = np.minimum(np.minimum(r, g), b)
        delta = cmax - cmin

        # Hue
        h = np.zeros_like(delta)
        mask_r = (cmax == r) & (delta > 1e-6)
        mask_g = (cmax == g) & (delta > 1e-6)
        mask_b = (cmax == b) & (delta > 1e-6)

        h[mask_r] = 60.0 * (((g[mask_r] - b[mask_r]) / delta[mask_r]) % 6)
        h[mask_g] = 60.0 * (((b[mask_g] - r[mask_g]) / delta[mask_g]) + 2)
        h[mask_b] = 60.0 * (((r[mask_b] - g[mask_b]) / delta[mask_b]) + 4)
        h = h / 2.0  # OpenCV 스타일 (0~180)

        # Saturation
        s = np.zeros_like(delta)
        nonzero = cmax > 1e-6
        s[nonzero] = (delta[nonzero] / cmax[nonzero]) * 255.0

        # Value
        v = cmax * 255.0

        hsv = np.stack([h, s, v], axis=-1).astype(np.uint8)
        return hsv

    @staticmethod
    def _morphology_close(mask: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        """모폴로지 닫기 (팽창 → 침식). OpenCV 없이 구현."""
        dilated = CropRowDetectorCV._dilate(mask, kernel)
        eroded = CropRowDetectorCV._erode(dilated, kernel)
        return eroded

    @staticmethod
    def _morphology_open(mask: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        """모폴로지 열기 (침식 → 팽창)."""
        eroded = CropRowDetectorCV._erode(mask, kernel)
        dilated = CropRowDetectorCV._dilate(eroded, kernel)
        return dilated

    @staticmethod
    def _dilate(mask: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        """간단한 팽창 연산."""
        kh, kw = kernel.shape
        ph, pw = kh // 2, kw // 2
        h, w = mask.shape
        # 패딩
        padded = np.zeros((h + 2 * ph, w + 2 * pw), dtype=mask.dtype)
        padded[ph:ph + h, pw:pw + w] = mask
        result = np.zeros_like(mask)
        # 슬라이딩 윈도우 최대값 (행 방향 → 열 방향 분리)
        # 성능을 위해 행/열 분리 적용
        temp = np.zeros_like(padded)
        for dy in range(-ph, ph + 1):
            shifted = np.roll(padded, dy, axis=0)
            temp = np.maximum(temp, shifted)
        result_padded = np.zeros_like(padded)
        for dx in range(-pw, pw + 1):
            shifted = np.roll(temp, dx, axis=1)
            result_padded = np.maximum(result_padded, shifted)
        result = result_padded[ph:ph + h, pw:pw + w]
        return result

    @staticmethod
    def _erode(mask: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        """간단한 침식 연산."""
        kh, kw = kernel.shape
        ph, pw = kh // 2, kw // 2
        h, w = mask.shape
        padded = np.full((h + 2 * ph, w + 2 * pw), 255, dtype=mask.dtype)
        padded[ph:ph + h, pw:pw + w] = mask
        temp = np.full_like(padded, 255)
        for dy in range(-ph, ph + 1):
            shifted = np.roll(padded, dy, axis=0)
            temp = np.minimum(temp, shifted)
        result_padded = np.full_like(padded, 255)
        for dx in range(-pw, pw + 1):
            shifted = np.roll(temp, dx, axis=1)
            result_padded = np.minimum(result_padded, shifted)
        return result_padded[ph:ph + h, pw:pw + w]

    @staticmethod
    def _find_peaks(
        signal: np.ndarray,
        min_distance: int = 30,
        min_prominence: float = 0.1,
    ) -> List[int]:
        """1D 신호에서 피크를 검출한다.

        플랫탑(연속 동일값) 구간도 지원: 상승→평탄→하강 패턴의
        평탄 구간 중심을 피크로 인식한다.
        """
        n = len(signal)
        if n < 3:
            return []

        # 연속 구간(run) 기반 피크 검출
        # 각 run: (start, end, value) — signal[start:end+1] 이 모두 동일 값
        peaks = []
        i = 0
        while i < n:
            j = i
            while j < n - 1 and abs(signal[j + 1] - signal[i]) < 1e-9:
                j += 1
            # run: [i, j], value: signal[i]
            val = signal[i]
            if val >= min_prominence:
                # 좌측 이웃이 더 작고, 우측 이웃이 더 작으면 피크
                left_ok = (i == 0) or (signal[i - 1] < val)
                right_ok = (j == n - 1) or (signal[j + 1] < val)
                if left_ok and right_ok and not (i == 0 and j == n - 1):
                    center = (i + j) // 2
                    peaks.append(center)
            i = j + 1

        # 최소 거리 필터
        if min_distance > 1 and len(peaks) > 1:
            filtered = [peaks[0]]
            for p in peaks[1:]:
                if p - filtered[-1] >= min_distance:
                    filtered.append(p)
                elif signal[p] > signal[filtered[-1]]:
                    filtered[-1] = p
            peaks = filtered

        return peaks

    @staticmethod
    def _estimate_row_width(projection: np.ndarray, peak_x: int) -> float:
        """피크 주변에서 행 폭을 추정한다 (반치전폭)."""
        n = len(projection)
        if peak_x >= n:
            return 0.0

        half_max = projection[peak_x] * 0.5
        left = peak_x
        while left > 0 and projection[left] > half_max:
            left -= 1
        right = peak_x
        while right < n - 1 and projection[right] > half_max:
            right += 1

        return float(right - left)

    def _estimate_row_angle(
        self, mask: np.ndarray, cfg: CVDetectorConfig
    ) -> float:
        """Hough 라인 기반 행 각도 추정."""
        h, w = mask.shape
        min_line_len = int(h * cfg.hough_min_line_ratio)

        # 간단한 Hough 변환: 수직에 가까운 선분만 탐색
        # 에지 검출 (간단한 Sobel X)
        edge = np.zeros_like(mask)
        edge[:, 1:] = np.abs(
            mask[:, 1:].astype(np.int16) - mask[:, :-1].astype(np.int16)
        ).astype(np.uint8)

        # 열별 연속 에지 포인트 수직 라인 탐색
        angles = []
        for x in range(0, w, max(1, w // 20)):
            col = edge[:, x]
            runs = self._find_vertical_runs(col, min_length=min_line_len)
            for y_start, y_end in runs:
                angles.append(0.0)  # 수직 라인 → 0도

        # 경사진 라인 탐색 (±angle_tolerance 범위)
        # 대각선 방향으로 에지 연속성 확인
        for angle_deg in range(-int(cfg.angle_tolerance_deg), int(cfg.angle_tolerance_deg) + 1, 5):
            if angle_deg == 0:
                continue
            dx_per_row = math.tan(math.radians(angle_deg))
            count = 0
            for x_start in range(0, w, max(1, w // 10)):
                length = 0
                for y in range(h):
                    x = int(x_start + y * dx_per_row)
                    if 0 <= x < w and edge[y, x] > 0:
                        length += 1
                if length >= min_line_len:
                    angles.append(float(angle_deg))
                    count += 1

        if not angles:
            return 0.0
        return float(np.median(angles))

    @staticmethod
    def _find_vertical_runs(
        col: np.ndarray, min_length: int
    ) -> List[Tuple[int, int]]:
        """열에서 연속된 에지 포인트 구간을 찾는다."""
        runs = []
        n = len(col)
        i = 0
        while i < n:
            if col[i] > 0:
                start = i
                while i < n and col[i] > 0:
                    i += 1
                if i - start >= min_length:
                    runs.append((start, i - 1))
            else:
                i += 1
        return runs

    def _generate_nav_path(
        self,
        rows: List[CropRow],
        image_width: int,
        roi_top: int,
        roi_bottom: int,
    ) -> List[Tuple[float, float]]:
        """감지된 행 사이의 주행 경로를 생성한다."""
        if len(rows) < 2:
            # 행이 1개면 이미지 중심과 행 사이 경로
            if len(rows) == 1:
                cx = rows[0].x_center_px
                img_cx = image_width / 2.0
                mid_x = (cx + img_cx) / 2.0
                return [
                    (mid_x, float(roi_top)),
                    (mid_x, float(roi_bottom)),
                ]
            return []

        # 가장 가까운 두 행 사이 경로 (이미지 중심에 가장 가까운 간격)
        center_x = image_width / 2.0
        best_gap = None
        best_dist = float('inf')

        for i in range(len(rows) - 1):
            mid = (rows[i].x_center_px + rows[i + 1].x_center_px) / 2.0
            dist = abs(mid - center_x)
            if dist < best_dist:
                best_dist = dist
                best_gap = (rows[i], rows[i + 1])

        if best_gap is None:
            return []

        r1, r2 = best_gap
        mid_x = (r1.x_center_px + r2.x_center_px) / 2.0

        # 행 각도 반영
        avg_angle = (r1.angle_deg + r2.angle_deg) / 2.0
        dx = math.tan(math.radians(avg_angle))

        n_points = 5
        path = []
        for i in range(n_points):
            t = i / (n_points - 1)
            y = roi_top + t * (roi_bottom - roi_top)
            x = mid_x + (y - (roi_top + roi_bottom) / 2.0) * dx
            path.append((float(x), float(y)))

        return path
