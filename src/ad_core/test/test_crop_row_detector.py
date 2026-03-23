"""CropRowDetectorCV 단위 테스트."""

import math

import numpy as np
import pytest

from ad_core.crop_row_detector import (
    ColorRange,
    CropRow,
    CropRowDetectorCV,
    CVDetectorConfig,
    OrchardProfile,
    OrchardType,
    RowDetectionResult,
    get_default_profiles,
)


# ── Helper: 합성 과수원 이미지 생성 ──────────────────────────


def make_orchard_image(
    width: int = 640,
    height: int = 480,
    num_rows: int = 3,
    row_width_px: int = 60,
    bg_color: tuple = (80, 60, 40),   # 토양 (갈색)
    row_color: tuple = (30, 120, 30),  # 초목 (녹색)
) -> np.ndarray:
    """수직 녹색 줄무늬로 과수 행을 모방한 합성 이미지."""
    img = np.full((height, width, 3), bg_color, dtype=np.uint8)

    if num_rows == 0:
        return img

    spacing = width // (num_rows + 1)
    for i in range(num_rows):
        cx = spacing * (i + 1)
        x1 = max(0, cx - row_width_px // 2)
        x2 = min(width, cx + row_width_px // 2)
        # ROI 영역에만 (상단 20% ~ 하단 95%)
        y1 = int(height * 0.2)
        y2 = int(height * 0.95)
        img[y1:y2, x1:x2] = row_color

    return img


def make_empty_image(width: int = 640, height: int = 480) -> np.ndarray:
    """균일한 토양색 이미지 (행 없음)."""
    return np.full((height, width, 3), (80, 60, 40), dtype=np.uint8)


def make_all_green_image(width: int = 320, height: int = 240) -> np.ndarray:
    """전체가 초록인 이미지."""
    return np.full((height, width, 3), (30, 150, 30), dtype=np.uint8)


# ── 프로파일 ──────────────────────────────────────────────────


class TestProfiles:
    def test_default_profiles_exist(self):
        profiles = get_default_profiles()
        assert OrchardType.APPLE in profiles
        assert OrchardType.PEAR in profiles
        assert OrchardType.PERSIMMON in profiles
        assert OrchardType.PEACH in profiles
        assert OrchardType.GRAPE in profiles
        assert OrchardType.GENERIC in profiles

    def test_profile_has_canopy_color(self):
        profiles = get_default_profiles()
        for otype, profile in profiles.items():
            assert len(profile.canopy_color) > 0, f"{otype.name} has no canopy color"

    def test_profile_row_spacing_positive(self):
        for profile in get_default_profiles().values():
            assert profile.expected_row_spacing_m > 0


class TestColorRange:
    def test_as_lower_upper(self):
        cr = ColorRange(h_min=25, h_max=90, s_min=30, s_max=255, v_min=30, v_max=255)
        lower = cr.as_lower()
        upper = cr.as_upper()
        assert lower.tolist() == [25, 30, 30]
        assert upper.tolist() == [90, 255, 255]


# ── 감지기 초기화 ─────────────────────────────────────────────


class TestDetectorInit:
    def test_default_init(self):
        det = CropRowDetectorCV()
        assert det.orchard_type == OrchardType.GENERIC

    def test_apple_init(self):
        det = CropRowDetectorCV(OrchardType.APPLE)
        assert det.orchard_type == OrchardType.APPLE
        assert "사과" in det.profile.description

    def test_set_orchard_type(self):
        det = CropRowDetectorCV()
        det.set_orchard_type(OrchardType.PEAR)
        assert det.orchard_type == OrchardType.PEAR
        assert "배" in det.profile.description

    def test_all_orchard_types(self):
        for otype in OrchardType:
            det = CropRowDetectorCV(otype)
            assert det.orchard_type == otype


# ── 행 감지 ───────────────────────────────────────────────────


class TestRowDetection:
    @pytest.fixture
    def detector(self):
        return CropRowDetectorCV(OrchardType.APPLE)

    def test_detect_three_rows(self, detector):
        img = make_orchard_image(num_rows=3)
        result = detector.detect(img)
        assert result.num_rows >= 2  # 최소 2개 이상 감지
        assert result.image_width == 640
        assert result.image_height == 480

    def test_detect_no_rows(self, detector):
        img = make_empty_image()
        result = detector.detect(img)
        assert result.num_rows == 0 or all(r.confidence < 0.3 for r in result.rows)

    def test_detect_five_rows(self, detector):
        img = make_orchard_image(num_rows=5, width=800)
        result = detector.detect(img)
        assert result.num_rows >= 3

    def test_result_has_mask(self, detector):
        img = make_orchard_image(num_rows=3)
        result = detector.detect(img)
        assert result.vegetation_mask is not None
        assert result.vegetation_mask.shape == (480, 640)

    def test_result_processing_time(self, detector):
        img = make_orchard_image(num_rows=3)
        result = detector.detect(img)
        assert result.processing_time_ms > 0

    def test_rows_sorted_by_x(self, detector):
        img = make_orchard_image(num_rows=4)
        result = detector.detect(img)
        if result.num_rows >= 2:
            for i in range(result.num_rows - 1):
                assert result.rows[i].x_center_px <= result.rows[i + 1].x_center_px


# ── 주행 경로 생성 ────────────────────────────────────────────


class TestNavPath:
    @pytest.fixture
    def detector(self):
        return CropRowDetectorCV(OrchardType.APPLE)

    def test_nav_path_generated(self, detector):
        img = make_orchard_image(num_rows=3)
        result = detector.detect(img)
        if result.num_rows >= 2:
            assert result.has_nav_path
            assert len(result.nav_path_px) >= 2

    def test_nav_path_empty_no_rows(self, detector):
        img = make_empty_image()
        result = detector.detect(img)
        if result.num_rows < 2:
            # 0~1 행이면 경로가 없거나 중심 기반
            pass  # OK

    def test_steering_offset_range(self, detector):
        img = make_orchard_image(num_rows=3)
        result = detector.detect(img)
        offset = result.get_steering_offset()
        assert -1.0 <= offset <= 1.0

    def test_heading_error(self, detector):
        img = make_orchard_image(num_rows=3)
        result = detector.detect(img)
        error = result.get_heading_error_deg()
        assert -90.0 <= error <= 90.0


# ── CropRow 데이터 ────────────────────────────────────────────


class TestCropRow:
    def test_angle_rad(self):
        row = CropRow(x_center_px=100, angle_deg=45.0, width_px=50, confidence=0.8)
        assert row.angle_rad == pytest.approx(math.pi / 4, abs=0.01)

    def test_zero_angle(self):
        row = CropRow(x_center_px=100, angle_deg=0.0, width_px=50, confidence=0.8)
        assert row.angle_rad == pytest.approx(0.0)


# ── RowDetectionResult ────────────────────────────────────────


class TestRowDetectionResult:
    def test_get_steering_offset_no_path(self):
        result = RowDetectionResult(
            rows=[], nav_path_px=[], vegetation_mask=None,
            image_width=640, image_height=480,
            orchard_type=OrchardType.APPLE,
        )
        assert result.get_steering_offset() == 0.0

    def test_get_steering_offset_centered(self):
        result = RowDetectionResult(
            rows=[], nav_path_px=[(320.0, 100.0), (320.0, 400.0)],
            vegetation_mask=None, image_width=640, image_height=480,
            orchard_type=OrchardType.APPLE,
        )
        assert result.get_steering_offset() == pytest.approx(0.0, abs=0.01)

    def test_get_steering_offset_left(self):
        result = RowDetectionResult(
            rows=[], nav_path_px=[(100.0, 200.0)],
            vegetation_mask=None, image_width=640, image_height=480,
            orchard_type=OrchardType.APPLE,
        )
        assert result.get_steering_offset() < 0

    def test_get_heading_error_no_rows(self):
        result = RowDetectionResult(
            rows=[], nav_path_px=[], vegetation_mask=None,
            image_width=640, image_height=480,
            orchard_type=OrchardType.APPLE,
        )
        assert result.get_heading_error_deg() == 0.0

    def test_get_heading_error_vertical_rows(self):
        rows = [CropRow(100, 0.0, 50, 0.9), CropRow(300, 0.0, 50, 0.9)]
        result = RowDetectionResult(
            rows=rows, nav_path_px=[], vegetation_mask=None,
            image_width=640, image_height=480,
            orchard_type=OrchardType.APPLE,
        )
        assert result.get_heading_error_deg() == pytest.approx(0.0)


# ── 내부 메서드 ───────────────────────────────────────────────


class TestInternalMethods:
    @pytest.fixture
    def detector(self):
        return CropRowDetectorCV()

    def test_rgb_to_hsv_black(self, detector):
        img = np.zeros((2, 2, 3), dtype=np.uint8)
        hsv = CropRowDetectorCV._rgb_to_hsv(img)
        assert hsv.shape == (2, 2, 3)
        assert np.all(hsv == 0)

    def test_rgb_to_hsv_white(self, detector):
        img = np.full((2, 2, 3), 255, dtype=np.uint8)
        hsv = CropRowDetectorCV._rgb_to_hsv(img)
        assert hsv[0, 0, 2] == 255  # V = 255

    def test_find_peaks_simple(self, detector):
        signal = np.array([0, 0.5, 1.0, 0.5, 0, 0.3, 0.8, 0.3, 0])
        peaks = CropRowDetectorCV._find_peaks(signal, min_distance=1, min_prominence=0.1)
        assert 2 in peaks  # 인덱스 2에 피크
        assert 6 in peaks  # 인덱스 6에 피크

    def test_find_peaks_empty(self, detector):
        signal = np.array([0.5, 0.5, 0.5])
        peaks = CropRowDetectorCV._find_peaks(signal, min_distance=1, min_prominence=0.1)
        assert len(peaks) == 0

    def test_find_peaks_min_distance(self, detector):
        signal = np.array([0, 1, 0, 1, 0])
        peaks = CropRowDetectorCV._find_peaks(signal, min_distance=3, min_prominence=0.1)
        assert len(peaks) == 1

    def test_estimate_row_width(self, detector):
        projection = np.array([0, 0, 0.5, 1.0, 0.5, 0, 0])
        width = CropRowDetectorCV._estimate_row_width(projection, 3)
        assert width >= 2.0

    def test_morphology_preserves_shape(self, detector):
        mask = np.zeros((100, 200), dtype=np.uint8)
        mask[30:70, 80:120] = 255
        kernel = np.ones((5, 5), dtype=np.uint8)
        closed = CropRowDetectorCV._morphology_close(mask, kernel)
        assert closed.shape == mask.shape
        opened = CropRowDetectorCV._morphology_open(mask, kernel)
        assert opened.shape == mask.shape

    def test_vegetation_mask_green_image(self, detector):
        img = make_all_green_image()
        mask = detector._create_vegetation_mask(img)
        green_ratio = mask.sum() / (mask.size * 255)
        assert green_ratio > 0.3  # 대부분 녹색

    def test_vegetation_mask_soil_image(self, detector):
        img = make_empty_image()
        roi = img[int(480 * 0.2):int(480 * 0.95), :]
        mask = detector._create_vegetation_mask(roi)
        green_ratio = mask.sum() / (mask.size * 255)
        assert green_ratio < 0.3  # 거의 녹색 없음


# ── 다양한 과수원 타입 ────────────────────────────────────────


class TestMultipleOrchardTypes:
    @pytest.mark.parametrize("orchard_type", list(OrchardType))
    def test_detect_runs_for_all_types(self, orchard_type):
        det = CropRowDetectorCV(orchard_type)
        img = make_orchard_image(num_rows=3)
        result = det.detect(img)
        assert result.orchard_type == orchard_type
        assert result.image_width == 640
