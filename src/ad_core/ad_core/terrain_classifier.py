"""지형 분류 모듈.

카메라 이미지 패치의 색상 정보를 기반으로 지형 유형을 추정하고,
각 지형에 대한 주행 가능성(traversability) 비용을 제공한다.
향후 딥러닝 모델로 교체할 수 있도록 추상적 인터페이스를 유지한다.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional, Tuple

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False


class TerrainType(Enum):
    """지형 유형과 기본 주행 비용을 정의하는 열거형.

    값은 해당 지형의 기본 주행 비용(traversability cost)을 나타낸다.
    0.0에 가까울수록 주행이 용이하고, 1.0에 가까울수록 주행이 어렵다.
    """

    PAVED = 0.1        # 포장 도로
    DIRT_ROAD = 0.2    # 비포장 도로
    GRASS = 0.3        # 잔디/풀밭
    GRAVEL = 0.25      # 자갈길
    CROP_FIELD = 0.4   # 농작물 밭
    MUD = 0.6          # 진흙
    OBSTACLE = 1.0     # 장애물 (통행 불가)
    PADDY_WET = 0.7    # 젖은 논
    PADDY_DRY = 0.45   # 마른 논
    FIELD_SOFT = 0.55  # 부드러운 밭
    FIELD_HARD = 0.35  # 단단한 밭


class TerrainClassifier:
    """지형 분류기.

    이미지 패치의 RGB 평균값을 기반으로 간이 지형 분류를 수행한다.
    실제 운용 시에는 딥러닝 모델 기반 분류기로 교체할 수 있다.
    """

    def __init__(self, model_path: Optional[str] = None) -> None:
        """TerrainClassifier를 초기화한다.

        Args:
            model_path: 딥러닝 모델 가중치 경로. None이면 색상 기반 규칙 사용.
                        향후 학습된 모델 로드 시 이 경로를 사용한다.
        """
        self._model_path = model_path
        self._model = None

        if model_path is not None:
            self._load_model(model_path)

    def _load_model(self, model_path: str) -> None:
        """딥러닝 모델을 로드한다 (향후 구현용 플레이스홀더).

        Args:
            model_path: 모델 가중치 파일 경로.
        """
        # TODO: 향후 딥러닝 모델 로드 구현
        #   예: self._model = torch.load(model_path)
        pass

    # ------------------------------------------------------------------
    # 분류
    # ------------------------------------------------------------------

    def classify(self, image_patch: "np.ndarray") -> TerrainType:
        """이미지 패치에서 지형 유형을 분류한다.

        딥러닝 모델이 로드되어 있으면 모델 추론을,
        없으면 색상 기반 규칙 분류를 수행한다.

        Args:
            image_patch: (H, W, 3) 형태의 RGB numpy 배열.

        Returns:
            추정된 TerrainType.
        """
        if self._model is not None:
            return self._classify_model(image_patch)
        return self.classify_from_color(image_patch)

    def classify_from_color(self, image_patch: "np.ndarray") -> TerrainType:
        """RGB 평균값 기반의 간이 지형 분류를 수행한다.

        색상 공간에서의 휴리스틱 규칙을 사용하여 지형을 추정한다.
        정확도는 제한적이나, 모델 없이도 기본적인 분류가 가능하다.

        Args:
            image_patch: (H, W, 3) 형태의 RGB numpy 배열.

        Returns:
            추정된 TerrainType.
        """
        if not _HAS_NUMPY:
            return TerrainType.DIRT_ROAD  # numpy 없으면 기본값

        if image_patch.size == 0:
            return TerrainType.DIRT_ROAD

        # RGB 채널 평균값 계산
        mean_color = image_patch.mean(axis=(0, 1))  # (R, G, B)
        r, g, b = float(mean_color[0]), float(mean_color[1]), float(mean_color[2])

        # 밝기 및 색상 비율 계산
        brightness = (r + g + b) / 3.0
        total = r + g + b + 1e-6  # 0 나눗셈 방지

        r_ratio = r / total
        g_ratio = g / total
        b_ratio = b / total

        # 규칙 기반 분류 (RGB 0~255 기준)
        # 1. 포장 도로: 회색 계열 (R~G~B, 중간 밝기)
        if self._is_gray(r, g, b) and 80 < brightness < 180:
            return TerrainType.PAVED

        # 2. 진흙: 어두운 갈색 (R > G > B, 낮은 밝기)
        if r > g > b and brightness < 80 and r_ratio > 0.38:
            return TerrainType.MUD

        # 3. 잔디/풀밭: 녹색 우세
        if g_ratio > 0.40 and g > r and g > b:
            return TerrainType.GRASS

        # 4. 농작물 밭: 연한 녹색~황록색
        if g_ratio > 0.36 and r_ratio > 0.30 and b_ratio < 0.28:
            return TerrainType.CROP_FIELD

        # 5. 자갈길: 밝은 회색~갈색
        if self._is_gray(r, g, b) and brightness >= 180:
            return TerrainType.GRAVEL

        # 6. 비포장 도로: 갈색 계열 (R > G > B)
        if r > g > b and r_ratio > 0.36:
            return TerrainType.DIRT_ROAD

        # 기본값
        return TerrainType.DIRT_ROAD

    def _classify_model(self, image_patch: "np.ndarray") -> TerrainType:
        """딥러닝 모델 기반 분류 (향후 구현용 플레이스홀더).

        Args:
            image_patch: (H, W, 3) 형태의 RGB numpy 배열.

        Returns:
            추정된 TerrainType.
        """
        # TODO: 모델 추론 결과를 TerrainType으로 매핑
        return self.classify_from_color(image_patch)

    # ------------------------------------------------------------------
    # 주행 비용
    # ------------------------------------------------------------------

    @staticmethod
    def get_traversability_cost(terrain_type: TerrainType) -> float:
        """지형 유형에 해당하는 주행 비용을 반환한다.

        Args:
            terrain_type: 지형 유형.

        Returns:
            주행 비용 (0.0 ~ 1.0). 낮을수록 주행이 용이.
        """
        return terrain_type.value

    @staticmethod
    def get_all_costs() -> dict[TerrainType, float]:
        """모든 지형 유형의 주행 비용을 딕셔너리로 반환한다.

        Returns:
            {TerrainType: cost} 매핑 딕셔너리.
        """
        return {t: t.value for t in TerrainType}

    # ------------------------------------------------------------------
    # 유틸리티
    # ------------------------------------------------------------------

    @staticmethod
    def _is_gray(r: float, g: float, b: float, threshold: float = 20.0) -> bool:
        """RGB 값이 회색 계열인지 판정한다.

        Args:
            r: 빨강 채널 값 (0~255).
            g: 초록 채널 값 (0~255).
            b: 파랑 채널 값 (0~255).
            threshold: 채널 간 최대 차이 허용값.

        Returns:
            회색 계열이면 True.
        """
        return (
            abs(r - g) < threshold
            and abs(g - b) < threshold
            and abs(r - b) < threshold
        )
