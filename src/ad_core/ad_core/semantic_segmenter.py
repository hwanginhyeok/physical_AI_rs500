"""시맨틱 세그멘테이션 모듈.

ENet 또는 BiSeNetV2 기반의 픽셀 단위 시맨틱 세그멘테이션을 수행한다.
농경지 자율주행에 필요한 11개 클래스를 분류하며,
주행 가능 영역과 장애물 영역 마스크를 제공한다.

PyTorch가 설치되지 않은 환경에서는 HSV 색상 규칙 기반의
fallback 세그멘테이션으로 동작한다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore[assignment]
    _HAS_NUMPY = False

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    _HAS_TORCH = True
except ImportError:
    torch = None  # type: ignore[assignment]
    nn = None  # type: ignore[assignment]
    F = None  # type: ignore[assignment]
    _HAS_TORCH = False

from ad_core.terrain_classifier import TerrainType


# ======================================================================
# 세그멘테이션 클래스 정의
# ======================================================================

class SegmentationClass(IntEnum):
    """시맨틱 세그멘테이션 클래스 열거형.

    농경지 자율주행 환경에서 필요한 11개 클래스를 정의한다.
    각 클래스의 정수 값은 세그멘테이션 마스크에서의 레이블 ID로 사용된다.
    """

    BACKGROUND = 0   # 배경 / 미분류
    ROAD = 1         # 포장 도로
    DIRT_PATH = 2    # 비포장 도로 / 흙길
    GRASS = 3        # 잔디 / 풀밭
    CROP = 4         # 농작물
    TREE = 5         # 나무 / 수목
    BUILDING = 6     # 건물 / 구조물
    WATER = 7        # 물 / 웅덩이
    MUD = 8          # 진흙
    OBSTACLE = 9     # 장애물 (바위, 울타리 등)
    SKY = 10         # 하늘


# 세그멘테이션 클래스 수
NUM_CLASSES: int = len(SegmentationClass)

# 클래스별 표시 색상 (RGB, 시각화용)
CLASS_COLORS: Dict[SegmentationClass, Tuple[int, int, int]] = {
    SegmentationClass.BACKGROUND: (0, 0, 0),
    SegmentationClass.ROAD:      (128, 128, 128),
    SegmentationClass.DIRT_PATH: (139, 90, 43),
    SegmentationClass.GRASS:     (0, 200, 0),
    SegmentationClass.CROP:      (154, 205, 50),
    SegmentationClass.TREE:      (0, 100, 0),
    SegmentationClass.BUILDING:  (180, 180, 180),
    SegmentationClass.WATER:     (0, 0, 200),
    SegmentationClass.MUD:       (80, 50, 20),
    SegmentationClass.OBSTACLE:  (255, 0, 0),
    SegmentationClass.SKY:       (135, 206, 235),
}

# 주행 가능 영역에 해당하는 클래스 집합
NAVIGABLE_CLASSES: frozenset = frozenset({
    SegmentationClass.ROAD,
    SegmentationClass.DIRT_PATH,
    SegmentationClass.GRASS,
})

# 장애물 영역에 해당하는 클래스 집합
OBSTACLE_CLASSES: frozenset = frozenset({
    SegmentationClass.OBSTACLE,
    SegmentationClass.BUILDING,
    SegmentationClass.WATER,
    SegmentationClass.TREE,
})

# SegmentationClass -> TerrainType 매핑 테이블
SEGMENTATION_TO_TERRAIN: Dict[SegmentationClass, TerrainType] = {
    SegmentationClass.BACKGROUND: TerrainType.DIRT_ROAD,
    SegmentationClass.ROAD:       TerrainType.PAVED,
    SegmentationClass.DIRT_PATH:  TerrainType.DIRT_ROAD,
    SegmentationClass.GRASS:      TerrainType.GRASS,
    SegmentationClass.CROP:       TerrainType.CROP_FIELD,
    SegmentationClass.TREE:       TerrainType.OBSTACLE,
    SegmentationClass.BUILDING:   TerrainType.OBSTACLE,
    SegmentationClass.WATER:      TerrainType.OBSTACLE,
    SegmentationClass.MUD:        TerrainType.MUD,
    SegmentationClass.OBSTACLE:   TerrainType.OBSTACLE,
    SegmentationClass.SKY:        TerrainType.DIRT_ROAD,
}


# ======================================================================
# 세그멘테이션 결과 데이터 클래스
# ======================================================================

@dataclass
class SegmentationResult:
    """시맨틱 세그멘테이션 결과를 담는 데이터 클래스.

    Attributes:
        mask: (H, W) 정수 배열. 각 픽셀의 SegmentationClass 레이블 ID.
        class_names: 클래스 ID -> 클래스 이름 매핑 딕셔너리.
        confidence_map: (H, W) 실수 배열. 각 픽셀의 분류 신뢰도 (0.0~1.0).
        processing_time: 세그멘테이션 소요 시간 (초).
    """

    mask: "np.ndarray"  # (H, W), dtype=int
    class_names: Dict[int, str]
    confidence_map: "np.ndarray"  # (H, W), dtype=float32
    processing_time: float

    @property
    def height(self) -> int:
        """마스크의 높이 (픽셀)."""
        return int(self.mask.shape[0])

    @property
    def width(self) -> int:
        """마스크의 너비 (픽셀)."""
        return int(self.mask.shape[1])

    @property
    def unique_classes(self) -> List[int]:
        """마스크에 존재하는 고유 클래스 ID 목록."""
        return sorted(int(c) for c in np.unique(self.mask))

    def to_terrain_map(self) -> "np.ndarray":
        """세그멘테이션 마스크를 TerrainType 기반 주행 비용 맵으로 변환한다.

        Returns:
            (H, W) 실수 배열. 각 픽셀의 주행 비용 (0.0~1.0).
        """
        cost_map = np.zeros_like(self.mask, dtype=np.float32)
        for seg_cls, terrain_type in SEGMENTATION_TO_TERRAIN.items():
            cost_map[self.mask == seg_cls.value] = terrain_type.value
        return cost_map

    def to_color_image(self) -> "np.ndarray":
        """세그멘테이션 마스크를 컬러 시각화 이미지로 변환한다.

        Returns:
            (H, W, 3) uint8 RGB 배열.
        """
        color_img = np.zeros((*self.mask.shape, 3), dtype=np.uint8)
        for seg_cls, color in CLASS_COLORS.items():
            color_img[self.mask == seg_cls.value] = color
        return color_img


# ======================================================================
# ENet 경량 인코더-디코더 아키텍처
# ======================================================================

if _HAS_TORCH:

    class _InitialBlock(nn.Module):
        """ENet 초기 블록.

        입력 이미지에 대해 3x3 합성곱과 2x2 맥스 풀링을 병렬 수행한 후
        채널 축으로 결합하여 초기 특징맵을 생성한다.
        """

        def __init__(self, in_channels: int = 3, out_channels: int = 16):
            super().__init__()
            # 합성곱 분기: out_channels - in_channels 채널 생성
            self.conv = nn.Conv2d(
                in_channels, out_channels - in_channels,
                kernel_size=3, stride=2, padding=1, bias=False,
            )
            self.bn = nn.BatchNorm2d(out_channels - in_channels)
            # 맥스 풀링 분기: in_channels 채널 유지
            self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
            self.relu = nn.ReLU(inplace=True)

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            conv_out = self.bn(self.conv(x))
            pool_out = self.pool(x)
            # 채널 축 결합
            out = torch.cat([conv_out, pool_out], dim=1)
            return self.relu(out)

    class _BottleneckBlock(nn.Module):
        """ENet 병목(Bottleneck) 블록.

        1x1 축소 -> 3x3 합성곱 -> 1x1 확장 구조의 잔차 블록.
        다운샘플링, 확장 합성곱(dilated), 비대칭 합성곱을 지원한다.
        """

        def __init__(
            self,
            in_channels: int,
            out_channels: int,
            downsample: bool = False,
            dilation: int = 1,
            asymmetric: bool = False,
            dropout_prob: float = 0.1,
        ):
            super().__init__()
            self.downsample = downsample
            internal_channels = in_channels // 4

            # --- 메인 분기 ---
            # 1x1 축소
            stride = 2 if downsample else 1
            self.conv1 = nn.Conv2d(
                in_channels, internal_channels,
                kernel_size=stride, stride=stride, bias=False,
            )
            self.bn1 = nn.BatchNorm2d(internal_channels)

            # 중앙 합성곱 (일반 / 비대칭 / 확장)
            if asymmetric:
                self.conv2a = nn.Conv2d(
                    internal_channels, internal_channels,
                    kernel_size=(5, 1), padding=(2, 0), bias=False,
                )
                self.conv2b = nn.Conv2d(
                    internal_channels, internal_channels,
                    kernel_size=(1, 5), padding=(0, 2), bias=False,
                )
                self.bn2 = nn.BatchNorm2d(internal_channels)
            else:
                self.conv2a = nn.Conv2d(
                    internal_channels, internal_channels,
                    kernel_size=3, padding=dilation, dilation=dilation, bias=False,
                )
                self.conv2b = None
                self.bn2 = nn.BatchNorm2d(internal_channels)

            # 1x1 확장
            self.conv3 = nn.Conv2d(
                internal_channels, out_channels,
                kernel_size=1, bias=False,
            )
            self.bn3 = nn.BatchNorm2d(out_channels)

            self.dropout = nn.Dropout2d(p=dropout_prob)
            self.relu = nn.ReLU(inplace=True)

            # --- 스킵 분기 ---
            if downsample:
                self.pool = nn.MaxPool2d(kernel_size=2, stride=2, return_indices=True)
                if in_channels != out_channels:
                    self.skip_conv = nn.Conv2d(
                        in_channels, out_channels,
                        kernel_size=1, bias=False,
                    )
                    self.skip_bn = nn.BatchNorm2d(out_channels)
                else:
                    self.skip_conv = None
            elif in_channels != out_channels:
                self.skip_conv = nn.Conv2d(
                    in_channels, out_channels,
                    kernel_size=1, bias=False,
                )
                self.skip_bn = nn.BatchNorm2d(out_channels)
            else:
                self.skip_conv = None

        def forward(
            self, x: "torch.Tensor",
        ) -> "Tuple[torch.Tensor, Optional[torch.Tensor]]":
            """순전파. 풀링 인덱스를 반환한다 (디코더의 언풀링에 사용).

            Returns:
                (출력 텐서, 풀링 인덱스 또는 None).
            """
            # 메인 분기
            main = self.bn1(self.conv1(x))
            main = self.relu(main)

            main = self.conv2a(main)
            if self.conv2b is not None:
                main = self.conv2b(main)
            main = self.bn2(main)
            main = self.relu(main)

            main = self.bn3(self.conv3(main))
            main = self.dropout(main)

            # 스킵 분기
            indices = None
            if self.downsample:
                skip, indices = self.pool(x)
                if self.skip_conv is not None:
                    skip = self.skip_bn(self.skip_conv(skip))
            else:
                skip = x
                if hasattr(self, 'skip_conv') and self.skip_conv is not None:
                    skip = self.skip_bn(self.skip_conv(skip))

            # 크기 맞춤 (다운샘플 후 홀수 크기 차이 보정)
            if main.shape != skip.shape:
                main = F.interpolate(
                    main, size=skip.shape[2:], mode='bilinear', align_corners=False,
                )

            out = self.relu(main + skip)
            return out, indices

    class _UpsamplingBottleneck(nn.Module):
        """ENet 업샘플링 병목 블록 (디코더용).

        맥스 언풀링을 사용하여 공간 해상도를 복원한다.
        """

        def __init__(
            self,
            in_channels: int,
            out_channels: int,
            dropout_prob: float = 0.1,
        ):
            super().__init__()
            internal_channels = in_channels // 4

            # 메인 분기: 전치 합성곱으로 업샘플링
            self.conv1 = nn.Conv2d(
                in_channels, internal_channels,
                kernel_size=1, bias=False,
            )
            self.bn1 = nn.BatchNorm2d(internal_channels)

            self.deconv = nn.ConvTranspose2d(
                internal_channels, internal_channels,
                kernel_size=3, stride=2, padding=1, output_padding=1, bias=False,
            )
            self.bn2 = nn.BatchNorm2d(internal_channels)

            self.conv3 = nn.Conv2d(
                internal_channels, out_channels,
                kernel_size=1, bias=False,
            )
            self.bn3 = nn.BatchNorm2d(out_channels)

            self.dropout = nn.Dropout2d(p=dropout_prob)
            self.relu = nn.ReLU(inplace=True)

            # 스킵 분기: 1x1 합성곱으로 채널 축소 + 맥스 언풀링
            self.skip_conv = nn.Conv2d(
                in_channels, out_channels,
                kernel_size=1, bias=False,
            )
            self.skip_bn = nn.BatchNorm2d(out_channels)
            self.unpool = nn.MaxUnpool2d(kernel_size=2, stride=2)

        def forward(
            self,
            x: "torch.Tensor",
            indices: "torch.Tensor",
            output_size: "torch.Size",
        ) -> "torch.Tensor":
            # 메인 분기
            main = self.bn1(self.conv1(x))
            main = self.relu(main)
            main = self.bn2(self.deconv(main))
            main = self.relu(main)
            main = self.bn3(self.conv3(main))
            main = self.dropout(main)

            # 스킵 분기: 언풀링
            skip = self.skip_bn(self.skip_conv(x))
            skip = self.unpool(skip, indices, output_size=output_size)

            # 크기 맞춤
            if main.shape != skip.shape:
                main = F.interpolate(
                    main, size=skip.shape[2:], mode='bilinear', align_corners=False,
                )

            return self.relu(main + skip)

    class ENetModel(nn.Module):
        """ENet 경량 시맨틱 세그멘테이션 모델.

        Paszke et al. (2016) "ENet: A Deep Neural Network Architecture for
        Real-Time Semantic Segmentation" 기반의 경량 인코더-디코더 구조.

        인코더:
            - Initial Block (3 -> 16ch)
            - Stage 1: 1 downsample + 4 bottleneck (16 -> 64ch)
            - Stage 2: 1 downsample + 8 bottleneck (64 -> 128ch, dilated)
            - Stage 3: 8 bottleneck (128ch, dilated, asymmetric)
        디코더:
            - Stage 4: 1 upsample + 2 bottleneck (128 -> 64ch)
            - Stage 5: 1 upsample + 1 bottleneck (64 -> 16ch)
            - Full-resolution convolution (16 -> num_classes)
        """

        def __init__(self, num_classes: int = NUM_CLASSES):
            super().__init__()
            self.num_classes = num_classes

            # --- 인코더 ---
            self.initial = _InitialBlock(3, 16)

            # Stage 1: 16 -> 64
            self.enc1_0 = _BottleneckBlock(16, 64, downsample=True, dropout_prob=0.01)
            self.enc1_1 = _BottleneckBlock(64, 64, dropout_prob=0.01)
            self.enc1_2 = _BottleneckBlock(64, 64, dropout_prob=0.01)
            self.enc1_3 = _BottleneckBlock(64, 64, dropout_prob=0.01)
            self.enc1_4 = _BottleneckBlock(64, 64, dropout_prob=0.01)

            # Stage 2: 64 -> 128
            self.enc2_0 = _BottleneckBlock(64, 128, downsample=True)
            self.enc2_1 = _BottleneckBlock(128, 128)
            self.enc2_2 = _BottleneckBlock(128, 128, dilation=2)
            self.enc2_3 = _BottleneckBlock(128, 128, asymmetric=True)
            self.enc2_4 = _BottleneckBlock(128, 128, dilation=4)
            self.enc2_5 = _BottleneckBlock(128, 128)
            self.enc2_6 = _BottleneckBlock(128, 128, dilation=8)
            self.enc2_7 = _BottleneckBlock(128, 128, asymmetric=True)
            self.enc2_8 = _BottleneckBlock(128, 128, dilation=16)

            # Stage 3: 128 -> 128 (반복, 다운샘플 없음)
            self.enc3_1 = _BottleneckBlock(128, 128)
            self.enc3_2 = _BottleneckBlock(128, 128, dilation=2)
            self.enc3_3 = _BottleneckBlock(128, 128, asymmetric=True)
            self.enc3_4 = _BottleneckBlock(128, 128, dilation=4)
            self.enc3_5 = _BottleneckBlock(128, 128)
            self.enc3_6 = _BottleneckBlock(128, 128, dilation=8)
            self.enc3_7 = _BottleneckBlock(128, 128, asymmetric=True)
            self.enc3_8 = _BottleneckBlock(128, 128, dilation=16)

            # --- 디코더 ---
            # Stage 4: 128 -> 64
            self.dec4_0 = _UpsamplingBottleneck(128, 64, dropout_prob=0.1)
            self.dec4_1 = _BottleneckBlock(64, 64, dropout_prob=0.1)
            self.dec4_2 = _BottleneckBlock(64, 64, dropout_prob=0.1)

            # Stage 5: 64 -> 16
            self.dec5_0 = _UpsamplingBottleneck(64, 16, dropout_prob=0.1)
            self.dec5_1 = _BottleneckBlock(16, 16, dropout_prob=0.1)

            # 최종 전치 합성곱: 원본 해상도 복원 + 클래스 분류
            self.fullconv = nn.ConvTranspose2d(
                16, num_classes,
                kernel_size=3, stride=2, padding=1, output_padding=1,
            )

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            """순전파.

            Args:
                x: (B, 3, H, W) 입력 이미지 텐서.

            Returns:
                (B, num_classes, H, W) 클래스 로짓 텐서.
            """
            input_size = x.shape[2:]

            # Initial
            x = self.initial(x)
            s1_size = x.shape[2:]

            # Encoder Stage 1
            x, idx1 = self.enc1_0(x)
            s2_size = x.shape[2:]
            x, _ = self.enc1_1(x)
            x, _ = self.enc1_2(x)
            x, _ = self.enc1_3(x)
            x, _ = self.enc1_4(x)

            # Encoder Stage 2
            x, idx2 = self.enc2_0(x)
            s3_size = x.shape[2:]
            x, _ = self.enc2_1(x)
            x, _ = self.enc2_2(x)
            x, _ = self.enc2_3(x)
            x, _ = self.enc2_4(x)
            x, _ = self.enc2_5(x)
            x, _ = self.enc2_6(x)
            x, _ = self.enc2_7(x)
            x, _ = self.enc2_8(x)

            # Encoder Stage 3
            x, _ = self.enc3_1(x)
            x, _ = self.enc3_2(x)
            x, _ = self.enc3_3(x)
            x, _ = self.enc3_4(x)
            x, _ = self.enc3_5(x)
            x, _ = self.enc3_6(x)
            x, _ = self.enc3_7(x)
            x, _ = self.enc3_8(x)

            # Decoder Stage 4
            x = self.dec4_0(x, idx2, output_size=torch.Size([x.shape[0], 64, *s2_size]))
            x, _ = self.dec4_1(x)
            x, _ = self.dec4_2(x)

            # Decoder Stage 5
            x = self.dec5_0(x, idx1, output_size=torch.Size([x.shape[0], 16, *s1_size]))
            x, _ = self.dec5_1(x)

            # Full-resolution 복원
            x = self.fullconv(x)

            # 입력 크기에 정확히 맞춤
            if x.shape[2:] != input_size:
                x = F.interpolate(
                    x, size=input_size, mode='bilinear', align_corners=False,
                )

            return x

    class BiSeNetV2Model(nn.Module):
        """BiSeNetV2 경량 시맨틱 세그멘테이션 모델 (간소화 버전).

        Yu et al. (2021) "BiSeNet V2: Bilateral Network with Guided Aggregation
        for Real-time Semantic Segmentation" 기반.

        Detail Branch (고해상도 공간 정보)와 Semantic Branch (저해상도 의미 정보)를
        Guided Aggregation으로 결합하여 실시간 세그멘테이션을 수행한다.
        """

        def __init__(self, num_classes: int = NUM_CLASSES):
            super().__init__()
            self.num_classes = num_classes

            # --- Detail Branch: 공간 정보 보존 ---
            self.detail_branch = nn.Sequential(
                # Stage 1: 3 -> 64, stride 2
                nn.Conv2d(3, 64, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.Conv2d(64, 64, 3, padding=1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                # Stage 2: 64 -> 64, stride 2
                nn.Conv2d(64, 64, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.Conv2d(64, 64, 3, padding=1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                # Stage 3: 64 -> 128, stride 2
                nn.Conv2d(64, 128, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(128),
                nn.ReLU(inplace=True),
                nn.Conv2d(128, 128, 3, padding=1, bias=False),
                nn.BatchNorm2d(128),
                nn.ReLU(inplace=True),
            )

            # --- Semantic Branch: 의미 정보 추출 ---
            self.semantic_branch = nn.Sequential(
                # Stem: 3 -> 16, stride 4
                nn.Conv2d(3, 16, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(16),
                nn.ReLU(inplace=True),
                nn.Conv2d(16, 16, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(16),
                nn.ReLU(inplace=True),
                # GE (Gather-and-Expansion) 간소화: 16 -> 32, stride 2
                nn.Conv2d(16, 32, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(32),
                nn.ReLU(inplace=True),
                nn.Conv2d(32, 32, 3, padding=1, bias=False),
                nn.BatchNorm2d(32),
                nn.ReLU(inplace=True),
                # GE: 32 -> 64, stride 2
                nn.Conv2d(32, 64, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.Conv2d(64, 64, 3, padding=1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                # CE (Context Embedding): 64 -> 128
                nn.AdaptiveAvgPool2d(1),
                nn.Conv2d(64, 128, 1, bias=False),
                nn.BatchNorm2d(128),
                nn.ReLU(inplace=True),
            )

            # Semantic Branch 업샘플 경로 (CE 출력을 Detail 크기에 맞춤)
            self.semantic_upsample = nn.Sequential(
                nn.Conv2d(128, 128, 3, padding=1, bias=False),
                nn.BatchNorm2d(128),
                nn.ReLU(inplace=True),
            )

            # --- Guided Aggregation ---
            self.agg_detail = nn.Sequential(
                nn.Conv2d(128, 128, 3, padding=1, bias=False),
                nn.BatchNorm2d(128),
            )
            self.agg_semantic = nn.Sequential(
                nn.Conv2d(128, 128, 3, padding=1, bias=False),
                nn.BatchNorm2d(128),
            )
            self.agg_conv = nn.Sequential(
                nn.Conv2d(128, 128, 3, padding=1, bias=False),
                nn.BatchNorm2d(128),
                nn.ReLU(inplace=True),
            )

            # 분류 헤드
            self.head = nn.Conv2d(128, num_classes, 1)

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            """순전파.

            Args:
                x: (B, 3, H, W) 입력 이미지 텐서.

            Returns:
                (B, num_classes, H, W) 클래스 로짓 텐서.
            """
            input_size = x.shape[2:]

            # Detail Branch
            detail = self.detail_branch(x)  # (B, 128, H/8, W/8)

            # Semantic Branch
            semantic = self.semantic_branch(x)  # (B, 128, 1, 1)
            # Detail 크기로 업샘플
            semantic = F.interpolate(
                semantic, size=detail.shape[2:],
                mode='bilinear', align_corners=False,
            )
            semantic = self.semantic_upsample(semantic)  # (B, 128, H/8, W/8)

            # Guided Aggregation
            d = self.agg_detail(detail)
            s = self.agg_semantic(semantic)
            agg = torch.sigmoid(d) * s + torch.sigmoid(s) * d
            agg = self.agg_conv(agg)  # (B, 128, H/8, W/8)

            # 분류 + 원본 해상도 복원
            out = self.head(agg)
            out = F.interpolate(
                out, size=input_size, mode='bilinear', align_corners=False,
            )

            return out


# ======================================================================
# 색상 규칙 기반 Fallback 세그멘테이션
# ======================================================================

def _segment_by_color_rules(image: "np.ndarray") -> Tuple["np.ndarray", "np.ndarray"]:
    """HSV 색상 공간 기반 규칙으로 간이 세그멘테이션을 수행한다.

    terrain_classifier.py의 색상 분류 로직과 유사한 접근 방식을 사용하되,
    패치 단위가 아닌 픽셀 단위로 분류한다.

    Args:
        image: (H, W, 3) RGB numpy 배열, dtype=uint8.

    Returns:
        (mask, confidence_map) 튜플.
            mask: (H, W) int 배열 (SegmentationClass 레이블).
            confidence_map: (H, W) float32 배열 (0.0~1.0).
    """
    h, w = image.shape[:2]
    mask = np.full((h, w), SegmentationClass.BACKGROUND.value, dtype=np.int32)
    confidence = np.full((h, w), 0.3, dtype=np.float32)  # 규칙 기반 기본 신뢰도

    r = image[:, :, 0].astype(np.float32)
    g = image[:, :, 1].astype(np.float32)
    b = image[:, :, 2].astype(np.float32)

    brightness = (r + g + b) / 3.0
    total = r + g + b + 1e-6

    r_ratio = r / total
    g_ratio = g / total
    b_ratio = b / total

    # 채널 간 차이 (회색 판별용)
    rg_diff = np.abs(r - g)
    gb_diff = np.abs(g - b)
    rb_diff = np.abs(r - b)
    gray_mask = (rg_diff < 20) & (gb_diff < 20) & (rb_diff < 20)

    # --- 분류 규칙 (우선순위 순서) ---

    # 1. 하늘: 상단 1/3, 파랑 우세, 높은 밝기
    sky_region = np.zeros((h, w), dtype=bool)
    sky_region[:h // 3, :] = True
    sky_cond = sky_region & (b_ratio > 0.36) & (brightness > 120)
    mask[sky_cond] = SegmentationClass.SKY.value
    confidence[sky_cond] = 0.5

    # 2. 물: 파랑 우세, 중간~낮은 밝기
    water_cond = (b_ratio > 0.40) & (b > r) & (b > g) & (brightness < 150) & ~sky_cond
    mask[water_cond] = SegmentationClass.WATER.value
    confidence[water_cond] = 0.4

    # 3. 잔디/풀밭: 녹색 우세
    grass_cond = (g_ratio > 0.40) & (g > r) & (g > b)
    mask[grass_cond] = SegmentationClass.GRASS.value
    confidence[grass_cond] = 0.45

    # 4. 나무: 짙은 녹색 (잔디보다 어두움)
    tree_cond = grass_cond & (brightness < 80)
    mask[tree_cond] = SegmentationClass.TREE.value
    confidence[tree_cond] = 0.4

    # 5. 농작물: 연한 녹색~황록색
    crop_cond = (g_ratio > 0.36) & (r_ratio > 0.30) & (b_ratio < 0.28) & ~grass_cond
    mask[crop_cond] = SegmentationClass.CROP.value
    confidence[crop_cond] = 0.4

    # 6. 포장 도로: 회색, 중간 밝기
    road_cond = gray_mask & (brightness > 80) & (brightness < 180)
    mask[road_cond] = SegmentationClass.ROAD.value
    confidence[road_cond] = 0.4

    # 7. 건물: 밝은 회색
    building_cond = gray_mask & (brightness >= 180)
    mask[building_cond] = SegmentationClass.BUILDING.value
    confidence[building_cond] = 0.35

    # 8. 진흙: 어두운 갈색
    mud_cond = (r > g) & (g > b) & (brightness < 80) & (r_ratio > 0.38)
    mask[mud_cond] = SegmentationClass.MUD.value
    confidence[mud_cond] = 0.4

    # 9. 비포장 도로: 갈색 계열
    dirt_cond = (r > g) & (g > b) & (r_ratio > 0.36) & ~mud_cond
    mask[dirt_cond] = SegmentationClass.DIRT_PATH.value
    confidence[dirt_cond] = 0.4

    # 10. 장애물: 매우 어두움 (하단 영역)
    obstacle_region = np.zeros((h, w), dtype=bool)
    obstacle_region[h // 2:, :] = True
    obstacle_cond = obstacle_region & (brightness < 40)
    mask[obstacle_cond] = SegmentationClass.OBSTACLE.value
    confidence[obstacle_cond] = 0.35

    return mask, confidence


# ======================================================================
# SemanticSegmenter 메인 클래스
# ======================================================================

class SemanticSegmenter:
    """시맨틱 세그멘테이션 수행 클래스.

    ENet 또는 BiSeNetV2 모델을 사용하여 픽셀 단위 시맨틱 세그멘테이션을 수행한다.
    PyTorch가 설치되지 않은 환경에서는 HSV 색상 규칙 기반의
    fallback 세그멘테이션으로 자동 전환된다.

    사용 예시::

        segmenter = SemanticSegmenter(model_type="enet")
        result = segmenter.segment(image)

        # 주행 가능 영역 추출
        navigable = segmenter.get_navigable_area(result)

        # 특정 클래스 마스크 추출
        road_mask = segmenter.get_class_mask(result, SegmentationClass.ROAD)
    """

    # 지원 모델 유형
    SUPPORTED_MODELS = ("enet", "bisenetv2")

    def __init__(
        self,
        model_type: str = "enet",
        input_size: Tuple[int, int] = (512, 512),
        device: Optional[str] = None,
        weights_path: Optional[str] = None,
    ) -> None:
        """SemanticSegmenter를 초기화한다.

        Args:
            model_type: 모델 유형. "enet" 또는 "bisenetv2".
            input_size: 모델 입력 크기 (height, width).
            device: PyTorch 디바이스 ("cpu", "cuda", "cuda:0" 등).
                    None이면 GPU 가용 시 자동 선택.
            weights_path: 사전 학습된 가중치 파일 경로. None이면 가중치 없이 초기화.
        """
        self._model_type = model_type.lower()
        self._input_size = input_size
        self._weights_path = weights_path
        self._model = None
        self._device = None
        self._fallback_mode = True

        if self._model_type not in self.SUPPORTED_MODELS:
            raise ValueError(
                f"지원하지 않는 모델 유형: '{model_type}'. "
                f"지원 모델: {self.SUPPORTED_MODELS}"
            )

        if _HAS_TORCH and _HAS_NUMPY:
            self._init_model(device, weights_path)
        else:
            # PyTorch 미설치: fallback 모드
            self._fallback_mode = True

    def _init_model(
        self,
        device: Optional[str],
        weights_path: Optional[str],
    ) -> None:
        """PyTorch 모델을 초기화한다.

        Args:
            device: PyTorch 디바이스 문자열.
            weights_path: 가중치 파일 경로.
        """
        # 디바이스 설정
        if device is not None:
            self._device = torch.device(device)
        elif torch.cuda.is_available():
            self._device = torch.device("cuda")
        else:
            self._device = torch.device("cpu")

        # 모델 생성
        try:
            if self._model_type == "enet":
                self._model = ENetModel(num_classes=NUM_CLASSES)
            elif self._model_type == "bisenetv2":
                self._model = BiSeNetV2Model(num_classes=NUM_CLASSES)

            # 가중치 로드
            if weights_path is not None:
                state_dict = torch.load(
                    weights_path, map_location=self._device, weights_only=True,
                )
                self._model.load_state_dict(state_dict)

            self._model = self._model.to(self._device)
            self._model.eval()
            self._fallback_mode = False
        except Exception:
            # 모델 초기화 실패 시 fallback 모드
            self._model = None
            self._fallback_mode = True

    # ------------------------------------------------------------------
    # 속성
    # ------------------------------------------------------------------

    @property
    def is_fallback_mode(self) -> bool:
        """Fallback(색상 규칙) 모드 여부를 반환한다."""
        return self._fallback_mode

    @property
    def model_type(self) -> str:
        """현재 설정된 모델 유형을 반환한다."""
        return self._model_type

    @property
    def input_size(self) -> Tuple[int, int]:
        """모델 입력 크기 (height, width)를 반환한다."""
        return self._input_size

    @property
    def device(self) -> Optional[str]:
        """현재 디바이스 문자열을 반환한다."""
        if self._device is not None:
            return str(self._device)
        return None

    @property
    def num_classes(self) -> int:
        """세그멘테이션 클래스 수를 반환한다."""
        return NUM_CLASSES

    # ------------------------------------------------------------------
    # 세그멘테이션
    # ------------------------------------------------------------------

    def segment(self, image: "np.ndarray") -> SegmentationResult:
        """입력 이미지에 대해 시맨틱 세그멘테이션을 수행한다.

        PyTorch 모델이 로드되어 있으면 딥러닝 추론을,
        없으면 색상 규칙 기반 fallback 세그멘테이션을 수행한다.

        Args:
            image: (H, W, 3) RGB numpy 배열, dtype=uint8.

        Returns:
            SegmentationResult 객체.
        """
        if not _HAS_NUMPY:
            # numpy조차 없으면 빈 결과 반환 (방어 코드)
            return self._empty_result()

        if image.ndim != 3 or image.shape[2] != 3:
            return self._empty_result()

        start_time = time.monotonic()

        if self._fallback_mode or self._model is None:
            mask, confidence_map = _segment_by_color_rules(image)
        else:
            mask, confidence_map = self._segment_model(image)

        elapsed = time.monotonic() - start_time

        class_names = {cls.value: cls.name for cls in SegmentationClass}

        return SegmentationResult(
            mask=mask,
            class_names=class_names,
            confidence_map=confidence_map,
            processing_time=elapsed,
        )

    def _segment_model(
        self, image: "np.ndarray",
    ) -> Tuple["np.ndarray", "np.ndarray"]:
        """PyTorch 모델을 사용하여 세그멘테이션을 수행한다.

        Args:
            image: (H, W, 3) RGB numpy 배열.

        Returns:
            (mask, confidence_map) 튜플.
        """
        orig_h, orig_w = image.shape[:2]

        # 전처리: 리사이즈 + 정규화 + 텐서 변환
        # 간단한 bilinear 리사이즈 (PIL/torchvision 없이)
        inp_h, inp_w = self._input_size
        # numpy 기반 리사이즈 (가장 가까운 이웃 보간)
        row_idx = (np.arange(inp_h) * orig_h / inp_h).astype(int)
        col_idx = (np.arange(inp_w) * orig_w / inp_w).astype(int)
        row_idx = np.clip(row_idx, 0, orig_h - 1)
        col_idx = np.clip(col_idx, 0, orig_w - 1)
        resized = image[np.ix_(row_idx, col_idx)]

        # 정규화 (ImageNet 평균/표준편차)
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        normalized = (resized.astype(np.float32) / 255.0 - mean) / std

        # (H, W, C) -> (1, C, H, W) 텐서 변환
        tensor = torch.from_numpy(normalized.transpose(2, 0, 1)).unsqueeze(0)
        tensor = tensor.to(self._device)

        # 추론
        with torch.no_grad():
            logits = self._model(tensor)  # (1, C, H, W)
            probs = F.softmax(logits, dim=1)  # (1, C, H, W)

        # 결과 추출
        pred_class = logits.argmax(dim=1).squeeze(0).cpu().numpy()  # (H, W)
        pred_conf = probs.max(dim=1).values.squeeze(0).cpu().numpy()  # (H, W)

        # 원본 크기로 복원 (가장 가까운 이웃 보간)
        row_idx_out = (np.arange(orig_h) * inp_h / orig_h).astype(int)
        col_idx_out = (np.arange(orig_w) * inp_w / orig_w).astype(int)
        row_idx_out = np.clip(row_idx_out, 0, inp_h - 1)
        col_idx_out = np.clip(col_idx_out, 0, inp_w - 1)

        mask = pred_class[np.ix_(row_idx_out, col_idx_out)].astype(np.int32)
        confidence_map = pred_conf[np.ix_(row_idx_out, col_idx_out)].astype(np.float32)

        return mask, confidence_map

    def _empty_result(self) -> SegmentationResult:
        """빈 세그멘테이션 결과를 생성한다.

        Returns:
            1x1 크기의 빈 SegmentationResult.
        """
        if _HAS_NUMPY:
            mask = np.zeros((1, 1), dtype=np.int32)
            conf = np.zeros((1, 1), dtype=np.float32)
        else:
            # numpy 없으면 리스트 기반 (비정상 상황)
            mask = [[0]]
            conf = [[0.0]]

        return SegmentationResult(
            mask=mask,
            class_names={cls.value: cls.name for cls in SegmentationClass},
            confidence_map=conf,
            processing_time=0.0,
        )

    # ------------------------------------------------------------------
    # 마스크 추출 유틸리티
    # ------------------------------------------------------------------

    @staticmethod
    def get_class_mask(
        result: SegmentationResult,
        seg_class: SegmentationClass,
    ) -> "np.ndarray":
        """특정 클래스의 바이너리 마스크를 추출한다.

        Args:
            result: 세그멘테이션 결과.
            seg_class: 추출할 SegmentationClass.

        Returns:
            (H, W) bool 배열. 해당 클래스인 픽셀이 True.
        """
        return result.mask == seg_class.value

    @staticmethod
    def get_navigable_area(result: SegmentationResult) -> "np.ndarray":
        """주행 가능 영역의 바이너리 마스크를 추출한다.

        ROAD, DIRT_PATH, GRASS 클래스를 주행 가능 영역으로 정의한다.

        Args:
            result: 세그멘테이션 결과.

        Returns:
            (H, W) bool 배열. 주행 가능 픽셀이 True.
        """
        navigable = np.zeros_like(result.mask, dtype=bool)
        for cls in NAVIGABLE_CLASSES:
            navigable |= (result.mask == cls.value)
        return navigable

    @staticmethod
    def get_obstacle_area(result: SegmentationResult) -> "np.ndarray":
        """장애물 영역의 바이너리 마스크를 추출한다.

        OBSTACLE, BUILDING, WATER, TREE 클래스를 장애물 영역으로 정의한다.

        Args:
            result: 세그멘테이션 결과.

        Returns:
            (H, W) bool 배열. 장애물 픽셀이 True.
        """
        obstacle = np.zeros_like(result.mask, dtype=bool)
        for cls in OBSTACLE_CLASSES:
            obstacle |= (result.mask == cls.value)
        return obstacle

    # ------------------------------------------------------------------
    # TerrainType 매핑
    # ------------------------------------------------------------------

    @staticmethod
    def to_terrain_type(seg_class: SegmentationClass) -> TerrainType:
        """SegmentationClass를 TerrainType으로 변환한다.

        terrain_classifier.py의 TerrainType과 호환되는 매핑을 제공한다.

        Args:
            seg_class: 세그멘테이션 클래스.

        Returns:
            대응하는 TerrainType.
        """
        return SEGMENTATION_TO_TERRAIN.get(seg_class, TerrainType.DIRT_ROAD)

    @staticmethod
    def get_traversability_cost(seg_class: SegmentationClass) -> float:
        """세그멘테이션 클래스의 주행 비용을 반환한다.

        TerrainType 매핑을 통해 주행 비용을 산출한다.

        Args:
            seg_class: 세그멘테이션 클래스.

        Returns:
            주행 비용 (0.0~1.0). 낮을수록 주행 용이.
        """
        terrain = SEGMENTATION_TO_TERRAIN.get(seg_class, TerrainType.DIRT_ROAD)
        return terrain.value

    @staticmethod
    def get_navigable_ratio(result: SegmentationResult) -> float:
        """이미지에서 주행 가능 영역이 차지하는 비율을 반환한다.

        Args:
            result: 세그멘테이션 결과.

        Returns:
            주행 가능 영역 비율 (0.0~1.0).
        """
        navigable = np.zeros_like(result.mask, dtype=bool)
        for cls in NAVIGABLE_CLASSES:
            navigable |= (result.mask == cls.value)
        total_pixels = result.mask.size
        if total_pixels == 0:
            return 0.0
        return float(np.sum(navigable)) / total_pixels

    @staticmethod
    def get_class_distribution(result: SegmentationResult) -> Dict[str, float]:
        """각 클래스의 픽셀 점유율을 반환한다.

        Args:
            result: 세그멘테이션 결과.

        Returns:
            {클래스이름: 점유율} 딕셔너리 (0.0~1.0).
        """
        total_pixels = result.mask.size
        if total_pixels == 0:
            return {}

        distribution: Dict[str, float] = {}
        for cls in SegmentationClass:
            count = int(np.sum(result.mask == cls.value))
            if count > 0:
                distribution[cls.name] = count / total_pixels

        return distribution
