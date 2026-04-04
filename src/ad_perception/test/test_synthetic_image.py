"""합성 이미지 퍼블리셔 + crop_row_detector 통합 검증 테스트."""

import numpy as np
import pytest

from ad_core.crop_row_detector import CropRowDetectorCV, OrchardType
from ad_perception.synthetic_image_publisher import (
    generate_orchard_image,
    SCENARIOS,
    IMG_W,
    IMG_H,
)


@pytest.fixture
def detector():
    return CropRowDetectorCV(orchard_type=OrchardType.PEAR)


class TestImageGeneration:
    """합성 이미지 기본 검증."""

    @pytest.mark.parametrize("scenario", SCENARIOS)
    def test_image_shape(self, scenario):
        img = generate_orchard_image(scenario)
        assert img.shape == (IMG_H, IMG_W, 3)
        assert img.dtype == np.uint8

    @pytest.mark.parametrize("scenario", SCENARIOS)
    def test_image_not_blank(self, scenario):
        """완전 검정/단일색이 아닌지 확인."""
        img = generate_orchard_image(scenario)
        assert img.std() > 5.0

    def test_unknown_scenario_fallback(self, detector):
        """알 수 없는 시나리오 → center 기본값."""
        img = generate_orchard_image("unknown_xyz")
        result = detector.detect(img)
        assert result.num_rows == 2


class TestCenterScenario:
    """center: 두 행 사이 중앙 → offset ≈ 0."""

    def test_two_rows_detected(self, detector):
        result = detector.detect(generate_orchard_image("center"))
        assert result.num_rows == 2

    def test_steering_near_zero(self, detector):
        result = detector.detect(generate_orchard_image("center"))
        assert abs(result.get_steering_offset()) < 0.1

    def test_has_nav_path(self, detector):
        result = detector.detect(generate_orchard_image("center"))
        assert result.has_nav_path


class TestOffsetLeftScenario:
    """offset_left: 왼쪽 치우침 → 양수 offset (우측 조향)."""

    def test_two_rows(self, detector):
        result = detector.detect(generate_orchard_image("offset_left"))
        assert result.num_rows == 2

    def test_positive_offset(self, detector):
        result = detector.detect(generate_orchard_image("offset_left"))
        assert result.get_steering_offset() > 0.2


class TestOffsetRightScenario:
    """offset_right: 오른쪽 치우침 → 음수 offset (좌측 조향)."""

    def test_two_rows(self, detector):
        result = detector.detect(generate_orchard_image("offset_right"))
        assert result.num_rows == 2

    def test_negative_offset(self, detector):
        result = detector.detect(generate_orchard_image("offset_right"))
        assert result.get_steering_offset() < -0.2


class TestNoRowsScenario:
    """no_rows: 빈 필드 → 행 미감지."""

    def test_zero_rows(self, detector):
        result = detector.detect(generate_orchard_image("no_rows"))
        assert result.num_rows == 0

    def test_no_nav_path(self, detector):
        result = detector.detect(generate_orchard_image("no_rows"))
        assert not result.has_nav_path

    def test_zero_offset(self, detector):
        result = detector.detect(generate_orchard_image("no_rows"))
        assert result.get_steering_offset() == 0.0


class TestEndOfRowScenario:
    """end_of_row: 행 끝 → 행 미감지 (end_detected 트리거용)."""

    def test_zero_rows(self, detector):
        result = detector.detect(generate_orchard_image("end_of_row"))
        assert result.num_rows == 0


class TestSingleRowScenario:
    """single_row: 행 1개만 → 경로는 이미지 중심과의 중간."""

    def test_one_row(self, detector):
        result = detector.detect(generate_orchard_image("single_row"))
        assert result.num_rows == 1

    def test_has_nav_path(self, detector):
        result = detector.detect(generate_orchard_image("single_row"))
        assert result.has_nav_path


class TestOffsetSymmetry:
    """offset_left와 offset_right의 대칭성 검증."""

    def test_opposite_signs(self, detector):
        left = detector.detect(generate_orchard_image("offset_left"))
        right = detector.detect(generate_orchard_image("offset_right"))
        assert left.get_steering_offset() > 0
        assert right.get_steering_offset() < 0

    def test_similar_magnitude(self, detector):
        left = detector.detect(generate_orchard_image("offset_left"))
        right = detector.detect(generate_orchard_image("offset_right"))
        diff = abs(abs(left.get_steering_offset()) - abs(right.get_steering_offset()))
        assert diff < 0.15  # 노이즈 허용 범위
