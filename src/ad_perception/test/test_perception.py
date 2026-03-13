"""Learned Perception 모듈 테스트.

ARCH-004: 농업용 Hybrid E2E - Learned Perception 검증
"""

import sys
from pathlib import Path
import numpy as np

# 경로 설정
sys.path.insert(0, str(Path(__file__).parent.parent / 'ad_perception'))
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'ad_core'))

from model_manager import ModelManager
from terrain_segmentation import TerrainSegmenter, TerrainLabel
from obstacle_detector import ObstacleDetector
from crop_row_extractor import CropRowExtractor


def create_test_image(height=480, width=640, pattern='random'):
    """테스트용 이미지 생성."""
    if pattern == 'random':
        return np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
    elif pattern == 'green':
        # 작물 필드 같은 초록색 이미지
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:, :, 1] = 128 + np.random.randint(-20, 20, (height, width))
        return img
    else:
        return np.ones((height, width, 3), dtype=np.uint8) * 128


def test_model_manager():
    """Model Manager 테스트."""
    print("\n=== Model Manager 테스트 ===")
    
    # CPU 모드로 테스트 (CI/CD 호환)
    manager = ModelManager(device='cpu')
    
    # 사용 가능한 모델 확인
    models = manager.list_available_models()
    print(f"✅ Available models: {models}")
    
    # 정보 확인
    info = manager.get_model_info('yolov8n')
    print(f"✅ yolov8n info: {info}")
    
    # 메모리 사용량
    mem = manager.get_memory_usage()
    print(f"✅ Memory: {mem}")
    
    print("✅ Model Manager 테스트 통과")


def test_terrain_segmentation():
    """지형 세그멘테이션 테스트."""
    print("\n=== Terrain Segmentation 테스트 ===")
    
    # 모델 매니저 (CPU)
    manager = ModelManager(device='cpu')
    
    # 세그멘테이션 (모델 없이 로직 테스트)
    segmenter = TerrainSegmenter(manager, model_name='yolov8n-seg')
    
    # 클래스 매핑 확인
    print(f"✅ Class map size: {len(segmenter.class_map)}")
    
    # TerrainLabel 확인
    assert TerrainLabel.CROP_FIELD.value == 1
    assert TerrainLabel.OBSTACLE.value == 6
    print("✅ TerrainLabel enum 확인")
    
    print("✅ Terrain Segmentation 테스트 통과")


def test_obstacle_detector():
    """장애물 탐지 테스트."""
    print("\n=== Obstacle Detector 테스트 ===")
    
    manager = ModelManager(device='cpu')
    detector = ObstacleDetector(manager, model_name='yolov8n')
    
    # 설정 확인
    assert detector.conf_threshold == 0.4
    print(f"✅ Conf threshold: {detector.conf_threshold}")
    
    # 클래스 필터링 확인
    assert 0 in detector.OBSTACLE_CLASSES  # person
    assert 16 in detector.OBSTACLE_CLASSES  # dog
    print(f"✅ Obstacle classes count: {len(detector.OBSTACLE_CLASSES)}")
    
    # 우선순위 클래스 확인
    assert detector.PRIORITY_CLASSES[0] == 'person'
    print("✅ Priority classes 확인")
    
    print("✅ Obstacle Detector 테스트 통과")


def test_crop_row_extractor():
    """작물 행 추출 테스트."""
    print("\n=== Crop Row Extractor 테스트 ===")
    
    extractor = CropRowExtractor(
        camera_height=1.2,
        expected_row_spacing=0.75
    )
    
    assert extractor.camera_height == 1.2
    assert extractor.expected_row_spacing == 0.75
    print(f"✅ Config: height={extractor.camera_height}, spacing={extractor.expected_row_spacing}")
    
    # 테스트 마스크 (간단한 세로 선)
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[:, 300:340] = 255  # 중앙 세로 선
    
    # 추출 시도 (실제로는 Hough transform 결과 없을 수 있음)
    rows = extractor.extract_rows(mask)
    print(f"✅ Extracted {len(rows)} rows from test mask")
    
    print("✅ Crop Row Extractor 테스트 통과")


def test_integration():
    """통합 테스트 (모델 로드 없이)."""
    print("\n=== 통합 테스트 ===")
    
    # 모듈들 인스턴스화
    manager = ModelManager(device='cpu')
    
    segmenter = TerrainSegmenter(manager)
    detector = ObstacleDetector(manager)
    extractor = CropRowExtractor()
    
    print("✅ All modules instantiated")
    
    # 테스트 이미지
    test_img = create_test_image(pattern='green')
    print(f"✅ Test image shape: {test_img.shape}")
    
    print("✅ 통합 테스트 통과 (모델 로드 없이)")


if __name__ == '__main__':
    print("=" * 60)
    print("Learned Perception 통합 테스트")
    print("=" * 60)
    
    try:
        test_model_manager()
        test_terrain_segmentation()
        test_obstacle_detector()
        test_crop_row_extractor()
        test_integration()
        
        print("\n" + "=" * 60)
        print("🎉 모든 테스트 통과!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ 테스트 실패: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
