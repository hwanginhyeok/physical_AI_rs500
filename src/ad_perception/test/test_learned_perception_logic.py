"""Learned Perception 로직 테스트.

C48: ROS2 Node 의존 없이 perception 모듈의 핵심 로직을 검증.
- TerrainSegmenter: 입력/출력 형식
- ObstacleDetector: 탐지 결과 형식
- CropRowExtractor: 행 추출 결과 형식
- PerceptionFeatures 생성
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'ad_perception'))
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'ad_core'))

from model_manager import ModelManager
from terrain_segmentation import TerrainSegmenter, TerrainLabel
from obstacle_detector import ObstacleDetector
from crop_row_extractor import CropRowExtractor
from ad_core.hybrid_e2e_types import (
    PerceptionFeatures,
    TerrainClass,
    Line3D,
    Object3D,
)


def _make_test_image(h=480, w=640, color='green'):
    """테스트용 이미지 생성."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    if color == 'green':
        img[:, :, 1] = 120
    elif color == 'brown':
        img[:, :, 0] = 60
        img[:, :, 1] = 80
        img[:, :, 2] = 100
    else:
        img[:] = 128
    return img


class TestModelManager:
    """ModelManager 초기화 테스트."""

    def test_cpu_init(self):
        manager = ModelManager(device='cpu')
        assert manager.device == 'cpu'

    def test_available_models(self):
        manager = ModelManager(device='cpu')
        models = manager.list_available_models()
        assert isinstance(models, (list, dict, set))


class TestTerrainSegmenter:
    """TerrainSegmenter 테스트."""

    def test_init(self):
        manager = ModelManager(device='cpu')
        seg = TerrainSegmenter(manager, model_name="yolov8n-seg")
        assert seg is not None

    def test_segment_returns_list(self):
        manager = ModelManager(device='cpu')
        seg = TerrainSegmenter(manager, model_name="yolov8n-seg")
        img = _make_test_image()
        result = seg.segment(img)
        assert isinstance(result, list)

    def test_dominant_terrain(self):
        manager = ModelManager(device='cpu')
        seg = TerrainSegmenter(manager, model_name="yolov8n-seg")
        img = _make_test_image(color='green')
        segments = seg.segment(img)
        dominant = seg.get_dominant_terrain(segments)
        assert isinstance(dominant, TerrainLabel)


class TestObstacleDetector:
    """ObstacleDetector 테스트."""

    def test_init(self):
        manager = ModelManager(device='cpu')
        det = ObstacleDetector(manager, model_name="yolov8n")
        assert det is not None

    def test_detect_returns_list(self):
        manager = ModelManager(device='cpu')
        det = ObstacleDetector(manager, model_name="yolov8n")
        img = _make_test_image()
        result = det.detect(img)
        assert isinstance(result, list)


class TestCropRowExtractor:
    """CropRowExtractor 테스트."""

    def test_init(self):
        ext = CropRowExtractor(camera_height=1.2, expected_row_spacing=0.75)
        assert ext is not None

    def test_extract_with_blank_mask(self):
        ext = CropRowExtractor(camera_height=1.2, expected_row_spacing=0.75)
        blank_mask = np.zeros((480, 640), dtype=np.uint8)
        img = _make_test_image()
        rows = ext.extract_rows(blank_mask, img)
        assert isinstance(rows, list)
        # 빈 마스크에서는 행이 없어야 함
        assert len(rows) == 0

    def test_extract_with_line_mask(self):
        """수직 줄무늬 마스크 → 행 추출."""
        ext = CropRowExtractor(camera_height=1.2, expected_row_spacing=0.75)
        mask = np.zeros((480, 640), dtype=np.uint8)
        # 수직 줄무늬 3개 추가
        for x in [160, 320, 480]:
            mask[:, max(0, x - 5):min(640, x + 5)] = 255
        img = _make_test_image()
        rows = ext.extract_rows(mask, img)
        assert isinstance(rows, list)


class TestPerceptionFeaturesCreation:
    """PerceptionFeatures 생성 테스트."""

    def test_default_creation(self):
        pf = PerceptionFeatures(
            terrain_type=TerrainClass.CROP_FIELD,
            overall_confidence=0.85
        )
        assert pf.terrain_type == TerrainClass.CROP_FIELD
        assert pf.overall_confidence == 0.85
        assert len(pf.crop_rows) == 0
        assert len(pf.obstacles) == 0

    def test_with_obstacles(self):
        obs = Object3D(
            position=(3.0, 1.0, 0.0),
            size=(0.5, 0.5, 0.5),
            class_label="person",
            confidence=0.92
        )
        pf = PerceptionFeatures(
            terrain_type=TerrainClass.DIRT_ROAD,
            obstacles=[obs],
            overall_confidence=0.9
        )
        assert len(pf.obstacles) == 1
        assert pf.obstacles[0].class_label == "person"

    def test_with_crop_rows(self):
        row = Line3D(
            start=(0.0, -2.0, 0.0),
            end=(10.0, -2.0, 0.0),
            confidence=0.8
        )
        pf = PerceptionFeatures(
            terrain_type=TerrainClass.CROP_FIELD,
            crop_rows=[row],
            overall_confidence=0.88
        )
        assert len(pf.crop_rows) == 1

    def test_terrain_class_values(self):
        """모든 TerrainClass 값이 유효한지."""
        for tc in TerrainClass:
            pf = PerceptionFeatures(
                terrain_type=tc,
                overall_confidence=0.5
            )
            assert pf.terrain_type == tc


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
