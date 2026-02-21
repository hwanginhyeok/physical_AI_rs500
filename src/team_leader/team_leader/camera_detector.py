"""카메라 객체 탐지 래퍼 모듈.

YOLO 기반 객체 탐지를 수행하며, 모델이 없는 환경에서도
더미 모드로 동작하여 개발 및 테스트를 지원한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False

# ultralytics (YOLO) 라이브러리 — 설치되어 있지 않으면 더미 모드로 전환
try:
    from ultralytics import YOLO  # type: ignore[import-untyped]
    _HAS_YOLO = True
except ImportError:
    YOLO = None  # type: ignore[assignment, misc]
    _HAS_YOLO = False


# 농경지 자율주행에 관련된 관심 클래스 목록
DEFAULT_CLASSES: List[str] = [
    "person", "car", "truck", "tree", "rock", "animal",
]


@dataclass
class Detection:
    """단일 객체 탐지 결과를 담는 데이터 클래스.

    Attributes:
        class_name: 탐지된 객체의 클래스 이름.
        confidence: 탐지 신뢰도 (0.0 ~ 1.0).
        bbox_x1: 바운딩 박스 좌상단 X 좌표 (픽셀).
        bbox_y1: 바운딩 박스 좌상단 Y 좌표 (픽셀).
        bbox_x2: 바운딩 박스 우하단 X 좌표 (픽셀).
        bbox_y2: 바운딩 박스 우하단 Y 좌표 (픽셀).
    """

    class_name: str
    confidence: float
    bbox_x1: float
    bbox_y1: float
    bbox_x2: float
    bbox_y2: float

    @property
    def bbox_width(self) -> float:
        """바운딩 박스 너비 (픽셀)."""
        return self.bbox_x2 - self.bbox_x1

    @property
    def bbox_height(self) -> float:
        """바운딩 박스 높이 (픽셀)."""
        return self.bbox_y2 - self.bbox_y1

    @property
    def bbox_center(self) -> tuple[float, float]:
        """바운딩 박스 중심 좌표 (cx, cy)."""
        return (
            (self.bbox_x1 + self.bbox_x2) / 2.0,
            (self.bbox_y1 + self.bbox_y2) / 2.0,
        )


class CameraDetector:
    """카메라 이미지 기반 객체 탐지기.

    YOLO 모델이 있으면 실제 탐지를 수행하고,
    없으면 더미 모드(빈 결과)로 동작한다.
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        confidence_threshold: float = 0.5,
        classes: Optional[Sequence[str]] = None,
    ) -> None:
        """CameraDetector를 초기화한다.

        Args:
            model_path: YOLO 가중치 파일 경로. None이면 더미 모드.
            confidence_threshold: 탐지 최소 신뢰도 (0.0 ~ 1.0).
            classes: 관심 클래스 이름 목록. None이면 기본 농경지 클래스 사용.
        """
        self._confidence_threshold = confidence_threshold
        self._classes = list(classes) if classes is not None else DEFAULT_CLASSES
        self._model = None
        self._dummy_mode = True

        if model_path is not None and _HAS_YOLO:
            try:
                self._model = YOLO(model_path)
                self._dummy_mode = False
            except Exception:
                # 모델 로드 실패 시 더미 모드 유지
                self._model = None
                self._dummy_mode = True

    @property
    def is_dummy_mode(self) -> bool:
        """더미 모드 여부를 반환한다."""
        return self._dummy_mode

    # ------------------------------------------------------------------
    # 탐지
    # ------------------------------------------------------------------

    def detect(self, image: "np.ndarray") -> List[Detection]:
        """입력 이미지에서 객체를 탐지한다.

        YOLO 모델이 로드되어 있으면 실제 추론을 수행하고,
        없으면 더미 모드로 빈 리스트를 반환한다.

        Args:
            image: (H, W, 3) 형태의 BGR/RGB numpy 배열.

        Returns:
            Detection 객체 리스트.
        """
        if self._dummy_mode or self._model is None:
            return self.detect_dummy(image)

        return self._detect_yolo(image)

    def detect_dummy(self, image: "np.ndarray") -> List[Detection]:
        """더미 탐지 — 모델 없이 빈 결과를 반환한다.

        개발 및 테스트 환경에서 파이프라인 동작 확인용으로 사용한다.

        Args:
            image: (H, W, 3) 형태의 numpy 배열 (사용하지 않음).

        Returns:
            빈 Detection 리스트.
        """
        return []

    def _detect_yolo(self, image: "np.ndarray") -> List[Detection]:
        """YOLO 모델로 실제 객체 탐지를 수행한다.

        Args:
            image: (H, W, 3) 형태의 numpy 배열.

        Returns:
            필터링된 Detection 객체 리스트.
        """
        results = self._model(image, verbose=False)  # type: ignore[union-attr]

        detections: List[Detection] = []
        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self._confidence_threshold:
                    continue

                cls_id = int(box.cls[0])
                cls_name = result.names.get(cls_id, f"class_{cls_id}")

                # 관심 클래스 필터링
                if self._classes and cls_name not in self._classes:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detections.append(Detection(
                    class_name=cls_name,
                    confidence=conf,
                    bbox_x1=float(x1),
                    bbox_y1=float(y1),
                    bbox_x2=float(x2),
                    bbox_y2=float(y2),
                ))

        return detections
