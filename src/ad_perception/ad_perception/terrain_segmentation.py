"""지형 세그멘테이션 모듈.

YOLOv8n-seg 기반 실시간 지형 분류.
- crop_field, grass, dirt_road, paved, mud, obstacle 분류
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum


class TerrainLabel(Enum):
    """지형 라벨 (COCO + 커스텀)."""
    BACKGROUND = 0
    CROP_FIELD = 1
    GRASS = 2
    DIRT_ROAD = 3
    PAVED = 4
    MUD = 5
    OBSTACLE = 6
    UNKNOWN = 99


@dataclass
class TerrainSegment:
    """세그멘테이션 결과."""
    label: TerrainLabel
    confidence: float
    mask: np.ndarray  # HxW boolean
    area_pixels: int
    centroid: Tuple[float, float]  # (x, y) in image coords


class TerrainSegmenter:
    """지형 세그멘테이션.
    
    YOLOv8n-seg를 사용하여 픽셀 단위 지형 분류 수행.
    """
    
    # YOLO 기본 클래스를 지형으로 매핑
    DEFAULT_CLASS_MAP = {
        0: TerrainLabel.OBSTACLE,   # person
        1: TerrainLabel.OBSTACLE,   # bicycle
        2: TerrainLabel.OBSTACLE,   # car
        3: TerrainLabel.OBSTACLE,   # motorcycle
        4: TerrainLabel.OBSTACLE,   # airplane
        5: TerrainLabel.OBSTACLE,   # bus
        6: TerrainLabel.OBSTACLE,   # train
        7: TerrainLabel.OBSTACLE,   # truck
        8: TerrainLabel.OBSTACLE,   # boat
        9: TerrainLabel.OBSTACLE,   # traffic light
        10: TerrainLabel.GRASS,     # fire hydrant -> grass (approx)
        11: TerrainLabel.OBSTACLE,  # stop sign
        12: TerrainLabel.OBSTACLE,  # parking meter
        13: TerrainLabel.OBSTACLE,  # bench
        14: TerrainLabel.OBSTACLE,  # bird
        15: TerrainLabel.OBSTACLE,  # cat
        16: TerrainLabel.OBSTACLE,  # dog
        17: TerrainLabel.OBSTACLE,  # horse
        18: TerrainLabel.OBSTACLE,  # sheep
        19: TerrainLabel.OBSTACLE,  # cow
        20: TerrainLabel.OBSTACLE,  # elephant
        21: TerrainLabel.OBSTACLE,  # bear
        22: TerrainLabel.OBSTACLE,  # zebra
        23: TerrainLabel.OBSTACLE,  # giraffe
        24: TerrainLabel.UNKNOWN,   # backpack
        25: TerrainLabel.UNKNOWN,   # umbrella
        26: TerrainLabel.UNKNOWN,   # handbag
        27: TerrainLabel.UNKNOWN,   # tie
        28: TerrainLabel.UNKNOWN,   # suitcase
        29: TerrainLabel.OBSTACLE,  # frisbee
        30: TerrainLabel.UNKNOWN,   # skis
        31: TerrainLabel.UNKNOWN,   # snowboard
        32: TerrainLabel.UNKNOWN,   # sports ball
        33: TerrainLabel.UNKNOWN,   # kite
        34: TerrainLabel.UNKNOWN,   # baseball bat
        35: TerrainLabel.UNKNOWN,   # baseball glove
        36: TerrainLabel.UNKNOWN,   # skateboard
        37: TerrainLabel.UNKNOWN,   # surfboard
        38: TerrainLabel.UNKNOWN,   # tennis racket
        39: TerrainLabel.OBSTACLE,  # bottle
        40: TerrainLabel.UNKNOWN,   # wine glass
        41: TerrainLabel.UNKNOWN,   # cup
        42: TerrainLabel.UNKNOWN,   # fork
        43: TerrainLabel.UNKNOWN,   # knife
        44: TerrainLabel.UNKNOWN,   # spoon
        45: TerrainLabel.UNKNOWN,   # bowl
        46: TerrainLabel.OBSTACLE,  # banana
        47: TerrainLabel.OBSTACLE,  # apple
        48: TerrainLabel.OBSTACLE,  # sandwich
        49: TerrainLabel.OBSTACLE,  # orange
        50: TerrainLabel.OBSTACLE,  # broccoli -> crop_field
        51: TerrainLabel.OBSTACLE,  # carrot
        52: TerrainLabel.UNKNOWN,   # hot dog
        53: TerrainLabel.UNKNOWN,   # pizza
        54: TerrainLabel.UNKNOWN,   # donut
        55: TerrainLabel.UNKNOWN,   # cake
        56: TerrainLabel.OBSTACLE,  # chair
        57: TerrainLabel.OBSTACLE,  # couch
        58: TerrainLabel.UNKNOWN,   # potted plant
        59: TerrainLabel.OBSTACLE,  # bed
        60: TerrainLabel.UNKNOWN,   # dining table
        61: TerrainLabel.UNKNOWN,   # toilet
        62: TerrainLabel.UNKNOWN,   # tv
        63: TerrainLabel.UNKNOWN,   # laptop
        64: TerrainLabel.UNKNOWN,   # mouse
        65: TerrainLabel.UNKNOWN,   # remote
        66: TerrainLabel.UNKNOWN,   # keyboard
        67: TerrainLabel.UNKNOWN,   # cell phone
        68: TerrainLabel.UNKNOWN,   # microwave
        69: TerrainLabel.OBSTACLE,  # oven
        70: TerrainLabel.OBSTACLE,  # toaster
        71: TerrainLabel.UNKNOWN,   # sink
        72: TerrainLabel.UNKNOWN,   # refrigerator
        73: TerrainLabel.UNKNOWN,   # book
        74: TerrainLabel.UNKNOWN,   # clock
        75: TerrainLabel.UNKNOWN,   # vase
        76: TerrainLabel.UNKNOWN,   # scissors
        77: TerrainLabel.UNKNOWN,   # teddy bear
        78: TerrainLabel.UNKNOWN,   # hair drier
        79: TerrainLabel.UNKNOWN,   # toothbrush
    }
    
    def __init__(
        self,
        model_manager,
        model_name: str = "yolov8n-seg",
        conf_threshold: float = 0.3,
        class_map: Optional[Dict[int, TerrainLabel]] = None
    ):
        """초기화.
        
        Args:
            model_manager: ModelManager 인스턴스
            model_name: 사용할 모델 이름
            conf_threshold: 신뢰도 임계값
            class_map: YOLO 클래스 ID -> TerrainLabel 매핑
        """
        self.model_manager = model_manager
        self.model_name = model_name
        self.conf_threshold = conf_threshold
        self.class_map = class_map or self.DEFAULT_CLASS_MAP
        
        self._model = None
        self._last_inference_time = 0.0
    
    def _get_model(self):
        """Lazy loading."""
        if self._model is None:
            self._model = self.model_manager.load_model(self.model_name)
        return self._model
    
    def segment(self, image: np.ndarray) -> List[TerrainSegment]:
        """이미지 세그멘테이션.
        
        Args:
            image: BGR 이미지 (OpenCV format)
            
        Returns:
            TerrainSegment 리스트
        """
        model = self._get_model()
        
        # YOLO 추론
        results = model(image, verbose=False, conf=self.conf_threshold)
        result = results[0]
        
        segments = []
        
        if result.boxes is None or len(result.boxes) == 0:
            return segments
        
        # 마스크 추출
        if hasattr(result, 'masks') and result.masks is not None:
            masks = result.masks.data.cpu().numpy()  # NxHxW
            
            for i, (box, mask) in enumerate(zip(result.boxes, result.masks.data)):
                cls_id = int(box.cls.item())
                conf = float(box.conf.item())
                
                # 지형 라벨로 매핑
                terrain_label = self.class_map.get(cls_id, TerrainLabel.UNKNOWN)
                
                if terrain_label == TerrainLabel.UNKNOWN:
                    continue
                
                # 마스크 처리
                mask_np = mask.cpu().numpy() if hasattr(mask, 'cpu') else mask
                mask_binary = (mask_np > 0.5).astype(np.uint8)
                
                # 중심점 계산
                ys, xs = np.where(mask_binary > 0)
                if len(xs) == 0:
                    continue
                    
                centroid = (float(xs.mean()), float(ys.mean()))
                
                segment = TerrainSegment(
                    label=terrain_label,
                    confidence=conf,
                    mask=mask_binary,
                    area_pixels=int(mask_binary.sum()),
                    centroid=centroid
                )
                segments.append(segment)
        
        # 도미넌트 지형 결정
        self._last_result = segments
        return segments
    
    def get_dominant_terrain(self, segments: List[TerrainSegment]) -> TerrainLabel:
        """가장 넓은 영역의 지형 반환."""
        if not segments:
            return TerrainLabel.UNKNOWN
        
        # 면적 기준 정렬
        sorted_segs = sorted(segments, key=lambda s: s.area_pixels, reverse=True)
        return sorted_segs[0].label
    
    def get_terrain_confidence(self, segments: List[TerrainSegment], 
                                target: TerrainLabel) -> float:
        """특정 지형의 신뢰도 (해당 클래스 면적 비율)."""
        if not segments:
            return 0.0
        
        target_area = sum(s.area_pixels for s in segments if s.label == target)
        total_area = sum(s.area_pixels for s in segments)
        
        return target_area / total_area if total_area > 0 else 0.0
    
    def create_terrain_mask(
        self,
        segments: List[TerrainSegment],
        image_shape: Tuple[int, int]
    ) -> Dict[TerrainLabel, np.ndarray]:
        """지형별 마스크 생성.
        
        Returns:
            {TerrainLabel: HxW boolean mask} 딕셔너리
        """
        h, w = image_shape[:2]
        masks = {}
        
        for segment in segments:
            if segment.label not in masks:
                masks[segment.label] = np.zeros((h, w), dtype=np.uint8)
            
            # 마스크 크기 조정
            mask_resized = cv2.resize(
                segment.mask.astype(np.uint8),
                (w, h),
                interpolation=cv2.INTER_NEAREST
            )
            masks[segment.label] = np.maximum(masks[segment.label], mask_resized)
        
        return masks


# OpenCV import
import cv2
