# 인지 및 센서 융합 선행연구 보고서

> **프로젝트**: RS500 궤도차량 자율주행 시스템
> **작성일**: 2026-02-21
> **센서 구성**: 640x480 전방 카메라, 360도 16채널 LiDAR, GPS, IMU
> **운용 환경**: 농경지(Phase 1), 비포장/험지(Phase 2)

---

## 목차

1. [LiDAR 기반 장애물 감지](#1-lidar-기반-장애물-감지)
2. [카메라 기반 인지](#2-카메라-기반-인지)
3. [Camera + LiDAR 센서 융합](#3-camera--lidar-센서-융합)
4. [농경지 특화 인지](#4-농경지-특화-인지)
5. [ROS2 환경 인지 파이프라인 구현 사례](#5-ros2-환경-인지-파이프라인-구현-사례)
6. [비포장/험지 Terrain Classification](#6-비포장험지-terrain-classification)
7. [우리 프로젝트 적용 로드맵](#7-우리-프로젝트-적용-로드맵)

---

## 1. LiDAR 기반 장애물 감지

### 1.1 지면 분리 (Ground Segmentation)

LiDAR 포인트클라우드에서 장애물을 감지하기 위한 첫 단계는 **지면 포인트를 분리**하는 것이다. 16채널 LiDAR(~300,000 pts/sec)의 비교적 희소한 포인트클라우드에서도 정확한 지면 분리가 핵심이다.

#### 주요 알고리즘 비교

| 알고리즘 | 원리 | 장점 | 단점 | 실시간성 |
|----------|------|------|------|----------|
| **RANSAC** | 평면 피팅 반복 | 구현 간단, 평탄한 지면에 효과적 | 경사/불균일 지면에 취약 | 빠름 (~5ms) |
| **Multi-Region RANSAC** | 영역별 분할 후 각각 RANSAC | 경사 지면 대응 가능 | 파라미터 튜닝 필요 | 보통 (~15ms) |
| **Patchwork++** | 동심원 구역(Concentric Zone) 기반 GLE | 비평탄 지면 강건, 적응형 파라미터 | 상대적 높은 연산량 | 보통 (~126ms 원논문 기준, 최적화 시 ~30ms) |
| **CSF (Cloth Simulation Filter)** | 천 시뮬레이션 물리 모델 | 사전지식 불필요, 직관적 파라미터 | 실시간 부적합 (배치 처리) | 느림 |
| **Ray Ground Filter** | 각 빔별 지면 각도 기반 | 매우 빠름, Autoware 기본 탑재 | 급경사 대응 한계 | 매우 빠름 (~3ms) |

**프로젝트 권장**: 16채널 LiDAR의 희소한 데이터 특성 고려 시, **Ray Ground Filter**(1차)와 **Patchwork++**(정밀 필요 시) 조합 추천. Autoware에서 즉시 사용 가능한 Ray Ground Filter로 시작하고, 험지 대응 시 Patchwork++로 전환.

#### 핵심 논문

- **Patchwork++**: "Fast and Robust Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud" (IROS 2022) - [arXiv:2207.11919](https://arxiv.org/pdf/2207.11919)
- **Ground-distance segmentation**: "Ground-distance segmentation of 3D LiDAR point cloud toward autonomous driving" (APSIPA 2024) - [Cambridge Core](https://www.cambridge.org/core/journals/apsipa-transactions-on-signal-and-information-processing/article/grounddistance-segmentation-of-3d-lidar-point-cloud-toward-autonomous-driving/91F72A086131E906889C5918E94B8152)

### 1.2 포인트클라우드 클러스터링

지면 분리 후 남은 비-지면 포인트들을 개별 객체로 그룹핑한다.

#### 주요 알고리즘

| 알고리즘 | 원리 | 특성 |
|----------|------|------|
| **Euclidean Clustering** | 거리 기반 클러스터 분리 (PCL 제공) | 빠르고 직관적, 밀도 불균일에 취약 |
| **DBSCAN** | 밀도 기반 클러스터링 | 노이즈 강건, 비구형 클러스터 지원 |
| **Adaptive DBSCAN** | 거리별 적응형 반경 | 원거리 희소 포인트 대응 |
| **K-means++ + DBSCAN** | K-means 사전 클러스터링 후 DBSCAN 정밀화 | 소형 객체 탐지 성능 향상 |
| **ScanLine Run** | 스캔 라인 기반 실시간 클러스터링 | 매우 빠름, ring 구조 활용 |

**프로젝트 권장**: 16채널 LiDAR는 포인트 밀도가 낮으므로, **거리 적응형 Euclidean Clustering**(가까운 곳은 좁은 반경, 먼 곳은 넓은 반경)으로 시작. 포인트 수가 적어 실시간 처리에 유리하다.

#### 핵심 논문/오픈소스

- **소형 객체 감지**: "A Small-Object-Detection Algorithm Based on LiDAR Point-Cloud Clustering for Autonomous Vehicles" (Sensors, 2024) - [MDPI](https://www.mdpi.com/1424-8220/24/16/5423)
- **클러스터링 서베이**: "Systematic and Comprehensive Review of Clustering and Multi-Target Tracking Techniques for LiDAR Point Clouds" (PMC, 2023) - [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC10346986/)
- **오픈소스**: PCL (Point Cloud Library) - `pcl::EuclideanClusterExtraction`, Autoware `euclidean_cluster` 노드

### 1.3 3D 객체 탐지 (딥러닝 기반)

클러스터링 방식의 한계를 넘어 딥러닝 기반 3D 객체 탐지도 고려할 수 있다.

| 모델 | 입력 표현 | mAP (nuScenes) | 추론 속도 | 특징 |
|------|-----------|----------------|-----------|------|
| **PointPillars** | Pillar (의사 2D) | ~45% | 62 Hz (~16ms) | 매우 빠름, 경량 |
| **CenterPoint** | Voxel + Center | ~60% | ~11 Hz | 높은 정확도, 추적 내장 |
| **PointPainting** | LiDAR + 이미지 시맨틱 | ~58% | ~8 Hz | 카메라 융합 가능 |

**프로젝트 권장**: 16채널 LiDAR의 희소성과 임베디드 제약 고려 시, **PointPillars**가 최적. Autoware Universe에 기본 구현체가 존재하며, TensorRT 가속 지원.

#### 핵심 논문

- **PointPillars**: "Fast Encoders for Object Detection from Point Clouds" (CVPR 2019) - [arXiv:1812.05784](https://arxiv.org/abs/1812.05784)
- **CenterPoint**: "Center-based 3D Object Detection and Tracking" (CVPR 2021)
- **S2*-ODM**: "Dual-Stage Improved PointPillar for 3D Object Detection" (2025) - [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC11902516/)

### 1.4 프로젝트 센서 적용 방안 (16채널 LiDAR)

```
16채널 LiDAR 데이터 (~300K pts/sec)
        │
        ▼
  ┌─────────────┐
  │ Ray Ground  │  ← 1단계: 빠른 지면 분리 (~3ms)
  │   Filter    │
  └──────┬──────┘
         │ (비-지면 포인트)
         ▼
  ┌─────────────┐
  │  Adaptive   │  ← 2단계: 거리 적응형 클러스터링 (~5ms)
  │  Euclidean  │
  │  Clustering │
  └──────┬──────┘
         │ (클러스터 목록)
         ▼
  ┌─────────────┐
  │  Bounding   │  ← 3단계: 바운딩 박스 피팅
  │  Box Fit    │    L-Shape Fitting 또는 Min Area Rect
  └──────┬──────┘
         │
         ▼
   장애물 목록 (위치, 크기, 속도)
   → /perception/obstacles 토픽 발행
```

**처리 시간 예상**: 총 ~10-15ms (실시간 20Hz 충족 가능)

---

## 2. 카메라 기반 인지

### 2.1 객체 탐지 (Object Detection)

640x480 해상도에서의 실시간 객체 탐지 모델 비교.

#### YOLO 시리즈 성능 비교

| 모델 | mAP@50 (COCO) | 추론 시간 (Jetson Orin) | 파라미터 수 | 640x480 적합성 |
|------|---------------|------------------------|-------------|----------------|
| **YOLOv8n** | 37.3% | ~3.3ms (TensorRT FP16) | 3.2M | 매우 적합 |
| **YOLO11n** | 39.5% | ~2.9ms (TensorRT FP16) | 2.6M | 매우 적합 |
| **YOLO26n** | ~41% | ~2.5ms (TensorRT FP16) | 2.4M | 최적 (2025 최신) |
| **YOLOv8s** | 44.9% | ~5.1ms (TensorRT FP16) | 11.2M | 적합 |
| **YOLO11s** | 47.0% | ~4.8ms (TensorRT FP16) | 9.4M | 적합 |

**YOLO26 (2025 최신)**: Distribution Focal Loss 제거, NMS-free 추론, MuSGD 옵티마이저 도입. INT8 양자화에서도 FP32 대비 mAP 손실 최소화.

**프로젝트 권장**: **YOLO11n** 또는 **YOLO26n** (nano 모델). 640x480 입력에서 TensorRT FP16 변환 시 Jetson Orin NX 기준 ~3ms 추론 가능. 실시간 30+ FPS 달성 용이.

#### 핵심 논문/리소스

- **YOLO11**: Ultralytics (2024) - [arXiv:2510.09653](https://arxiv.org/pdf/2510.09653)
- **YOLO26**: "Key Architectural Enhancements and Performance Benchmarking" (2025) - [arXiv:2509.25164](https://arxiv.org/html/2509.25164v2)
- **YOLOv8 Jetson 배포**: [Seeed Studio Wiki](https://wiki.seeedstudio.com/YOLOv8-TRT-Jetson/)
- **AW-YOLO**: "Multi-Object Detection Under All Weather Conditions" (IET 2025) - [Wiley](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/ipr2.70111)
- **Jetson 벤치마크**: "Benchmarking YOLOv8 Variants on Jetson Orin NX" (Computers 2025) - [MDPI](https://www.mdpi.com/2073-431X/15/2/74)

### 2.2 시맨틱 세그멘테이션 (Semantic Segmentation)

픽셀 단위 분류를 통해 주행 가능 영역, 장애물, 지형 타입을 구분한다.

#### 주요 모델 비교

| 모델 | mIoU (Cityscapes) | 추론 속도 (임베디드) | 특징 |
|------|-------------------|---------------------|------|
| **ENet** | 58.3% | ~50 FPS | 초경량, 농업용 연구에 다수 적용 |
| **BiSeNetV2** | 72.6% | ~40 FPS | 2-path 설계, 실시간 특화 |
| **PP-LiteSeg** | 73.9% | ~35 FPS | PaddlePaddle 기반, 경량 |
| **DeepLabV3+ (MobileNetV3)** | 75.2% | ~15 FPS | 정확도/속도 균형 |
| **SegFormer-B0** | 76.2% | ~20 FPS | Transformer 기반 경량 |

**프로젝트 권장**: 농경지 환경에서는 **ENet** 또는 **BiSeNetV2**로 시작(초고속 추론), 정확도 요구 시 **SegFormer-B0**으로 업그레이드. 640x480 해상도에서 실시간 처리 가능.

#### 농업/비포장 특화 세그멘테이션 클래스 정의 (제안)

```
클래스 맵:
  0: 배경/하늘
  1: 주행 가능 영역 (포장도로/다져진 흙길)
  2: 주의 주행 영역 (비포장/풀밭)
  3: 주행 불가 (장애물/수목/구조물)
  4: 작물/식생
  5: 수면/웅덩이
  6: 사람
  7: 차량/기계
```

### 2.3 지형 분류 (Terrain Classification from Camera)

카메라 이미지로부터 지형의 종류(흙, 자갈, 잔디, 아스팔트, 진흙 등)를 분류한다.

| 접근법 | 정확도 | 실시간성 | 특징 |
|--------|--------|----------|------|
| **CNN 분류기 (ResNet-18)** | ~92% | 매우 빠름 | 패치 단위 분류 |
| **시맨틱 세그멘테이션** | ~85% mIoU | 빠름 | 픽셀 단위 지형 맵 |
| **BEVNet** | ~88% | 보통 | BEV 변환 후 분류, 시간적 일관성 |

---

## 3. Camera + LiDAR 센서 융합

### 3.1 융합 전략 비교

센서 융합은 크게 세 가지 레벨로 나뉜다.

#### Early Fusion (데이터 수준)

```
Camera Image ──┐
               ├── 원시 데이터 결합 ── 단일 네트워크 ── 결과
LiDAR Points ──┘
```

- **방식**: LiDAR 포인트를 이미지에 투영하여 RGBD 형태로 결합, 또는 이미지 특징을 포인트클라우드에 페인팅
- **장점**: 센서 간 상호보완 정보 최대 활용
- **단점**: 카메라 미탐지 시 전체 파이프라인에 영향, 정밀한 캘리브레이션 필수
- **대표**: PointPainting, MVXNet
- **640x480 + 16ch LiDAR 적합성**: 보통 (희소 포인트와 저해상도 이미지 결합 효과 제한적)

#### Late Fusion (결정 수준)

```
Camera Image ── 카메라 탐지기 ── 2D 바운딩 박스 ──┐
                                                  ├── 결과 융합 ── 최종 결과
LiDAR Points ── LiDAR 탐지기 ── 3D 바운딩 박스 ──┘
```

- **방식**: 각 센서에서 독립적으로 탐지한 결과를 후처리로 결합
- **장점**: 각 센서 독립 동작 가능(폴백), 구현 간단, 경량
- **단점**: 센서 간 상호보완 효과 제한적
- **대표**: CLOCs, 3D-CVF
- **640x480 + 16ch LiDAR 적합성**: **높음** (각 센서 독립 처리로 유연성 확보)

#### Mid-Level Fusion (특징 수준)

```
Camera Image ── 이미지 백본 ── 이미지 특징 ──┐
                                             ├── 특징 융합 ── 탐지 헤드 ── 결과
LiDAR Points ── 포인트 백본 ── 포인트 특징 ──┘
```

- **방식**: 각 센서의 중간 특징 맵을 공유 표현 공간(BEV 등)에서 결합
- **장점**: 풍부한 정보 활용, 높은 정확도
- **단점**: 높은 연산량, 학습 복잡
- **대표**: BEVFusion, TransFusion, DeepFusion
- **640x480 + 16ch LiDAR 적합성**: 낮음 (연산량 과다, 임베디드 배포 어려움)

### 3.2 주요 융합 모델 상세

#### BEVFusion (MIT HAN Lab, ICRA 2023)

- **핵심**: 카메라와 LiDAR 특징을 Bird's Eye View(BEV) 공간에서 통합
- **성능**: nuScenes 벤치마크에서 mAP 71.7%, mIoU 62.7% (3D 세그멘테이션)
- **혁신**: BEV Pooling 최적화로 지연시간 40배 감소
- **연산량**: 높음 (서버급 GPU 권장)
- **오픈소스**: [github.com/mit-han-lab/bevfusion](https://github.com/mit-han-lab/bevfusion)

#### Attention-Based LiDAR-Camera Fusion (2025)

- **핵심**: 채널별 어텐션 가중치로 LiDAR 기하 정보와 카메라 시맨틱 정보 적응적 융합
- **성능**: 기존 대비 3-5% mAP 향상
- **참조**: [MDPI](https://www.mdpi.com/2032-6653/16/6/306)

#### Cross-dataset Late Fusion (2025)

- **핵심**: 카메라-LiDAR-레이더 모델의 경량 Late Fusion
- **장점**: 각 센서 모델을 독립적으로 학습/교체 가능, 실시간 임베디드 배포 적합
- **참조**: [Nature Scientific Reports](https://www.nature.com/articles/s41598-025-32588-5)

### 3.3 프로젝트 적용 전략

```
[권장: Late Fusion 기반 단계적 구현]

Phase 1 - 독립 파이프라인
├── Camera: YOLO11n → 2D 바운딩 박스 + 클래스
├── LiDAR: Ground Filter → Clustering → 3D 바운딩 박스
└── 결과: 독립 발행 (각각 토픽으로)

Phase 2 - Late Fusion
├── 카메라 2D 박스 + LiDAR 3D 클러스터 매칭
├── Frustum Association: 2D 박스를 3D 공간에 투영하여 대응 클러스터 찾기
└── 결과: 융합된 3D 객체 목록 (클래스 + 위치 + 크기)

Phase 3 - Early Fusion (선택)
├── PointPainting: LiDAR 포인트에 카메라 시맨틱 레이블 부착
└── 결과: 시맨틱이 풍부한 3D 탐지
```

**캘리브레이션 필수**: Camera-LiDAR 외부 파라미터(Extrinsic) 캘리브레이션 수행 필요.
ROS2 패키지 `ros2_camera_lidar_fusion`이 캘리브레이션 기능을 제공한다.
- [github.com/CDonosoK/ros2_camera_lidar_fusion](https://github.com/CDonosoK/ros2_camera_lidar_fusion)

---

## 4. 농경지 특화 인지

### 4.1 작물 행 인식 (Crop Row Detection)

궤도차량이 농경지 내에서 작물 행을 따라 자율주행하기 위한 핵심 기술.

#### 주요 접근법

| 방법 | 원리 | 정확도 | 실시간성 | 적용 조건 |
|------|------|--------|----------|-----------|
| **Hough Transform** | 직선 검출 | 보통 | 매우 빠름 | 직선형 작물 행 |
| **색상 기반 + 형태학** | 녹색 분리 → 스켈레톤화 | 보통 | 빠름 | 성장기 작물 |
| **시맨틱 세그멘테이션** | 픽셀 분류 후 행 추출 | 높음 | 보통 | 다양한 성장 단계 |
| **딥러닝 직접 회귀** | 이미지 → 행 파라미터 | 높음 | 빠름 | 학습 데이터 필요 |
| **LiDAR 기반** | 3D 포인트 높이 차이 | 높음 | 빠름 | 전천후 |

#### 최신 연구 동향

- **CropNav (2024)**: 실제 농장에서의 자율 내비게이션 프레임워크. 시맨틱 세그멘테이션 기반 작물 행 중심선 추출 및 경로 생성. [arXiv:2411.10974](https://arxiv.org/html/2411.10974v1)
- **AgroNav (CVPR Workshop 2023)**: YOLO + 시맨틱 세그멘테이션 결합 농업 로봇 내비게이션 프레임워크. [CVPR](https://openaccess.thecvf.com/content/CVPR2023W/AgriVision/papers/Panda_Agronav_Autonomous_Navigation_Framework_for_Agricultural_Robots_and_Vehicles_Using_CVPRW_2023_paper.pdf)
- **딥러닝 기반 내비게이션 라인 추출 (2025)**: ENet, DeepLabV3+, PSPNet으로 작물 행 세그멘테이션 후 주행 경로 중심선 추출. 다양한 성장 단계, 조명 조건, 잡초 환경에서 정확한 예측. [Springer](https://link.springer.com/article/10.1007/s44163-025-00301-0)
- **합성 데이터 기반 학습 (2024)**: 합성 데이터로 학습하여 실제 농경지에 적용하는 sim-to-real 접근법. [Springer](https://link.springer.com/article/10.1007/s11119-024-10157-6)

#### 작물 행 인식 서베이

"Advances in Crop Row Detection for Agricultural Robots: Methods, Performance Indicators, and Scene Adaptability" (Agriculture, 2025) - 시각 센서, LiDAR 전처리, 3D 특징 추출, 멀티센서 융합 방법을 체계적으로 분류. [MDPI](https://www.mdpi.com/2077-0472/15/20/2151)

### 4.2 경계 탐지 (Field Boundary Detection)

농경지 경계(밭 가장자리, 도랑, 도로 경계) 인식.

| 방법 | 적용 |
|------|------|
| 시맨틱 세그멘테이션 | 작물/비작물 영역 경계 추출 |
| LiDAR 높이 맵 | 밭두렁, 배수로 등 높이 차이로 경계 검출 |
| GPS 기반 지오펜싱 | 사전 정의된 작업 영역 경계 (RTK GPS 필요) |
| 하이브리드 | GPS 대략적 경계 + 시각/LiDAR 정밀 보정 |

**프로젝트 권장**: GPS 기반 대략적 경계 설정 + 카메라 시맨틱 세그멘테이션으로 실시간 경계 보정. 궤도차량 특성상 경계 부근에서의 저속 정밀 주행이 중요.

### 4.3 지형 Traversability 분석

주행 가능 여부를 지형 특성에 따라 판단.

#### Traversability Cost Map 생성

```
센서 데이터 입력
     │
     ├── Camera → 시맨틱 세그멘테이션 → 지형 타입 분류
     │                                    │
     ├── LiDAR → 높이 맵 생성 → 경사도/거칠기 계산
     │                              │
     └── IMU → 차체 기울기/진동 ──────┘
                                     │
                                     ▼
              Traversability Cost Map (2D 그리드)
              ┌───────────────────────┐
              │ 각 셀: cost 0.0 ~ 1.0 │
              │ 0.0 = 완전 주행 가능   │
              │ 1.0 = 주행 불가        │
              └───────────────────────┘
                      │
                      ▼
              경로 계획기 입력 (Nav2 costmap)
```

#### Traversability 비용 매핑 예시

| 지형 타입 | 기본 비용 | 경사도 보정 | 궤도차량 특성 |
|-----------|-----------|-------------|---------------|
| 다져진 흙길 | 0.1 | +0.02/도 | 양호 |
| 잔디/풀밭 | 0.2 | +0.03/도 | 양호 |
| 자갈길 | 0.3 | +0.03/도 | 양호 (궤도 유리) |
| 진흙 | 0.5 | +0.05/도 | 주의 (슬립 가능) |
| 얕은 물웅덩이 | 0.6 | N/A | 주의 |
| 경작지 (부드러운 흙) | 0.4 | +0.04/도 | 보통 (함몰 주의) |
| 급경사 (>25도) | 0.9 | N/A | 위험 |
| 장애물/수목 | 1.0 | N/A | 통행 불가 |

---

## 5. ROS2 환경 인지 파이프라인 구현 사례

### 5.1 Autoware Universe 인지 스택

Autoware는 가장 성숙한 오픈소스 ROS2 자율주행 플랫폼이다.

#### 인지 파이프라인 아키텍처

```
센서 드라이버 (velodyne_driver, usb_cam 등)
        │
        ▼
┌───────────────────────────────────────────┐
│           Sensing Layer                    │
│  - 포인트클라우드 전처리 (crop, downsample)│
│  - 이미지 전처리 (undistort, resize)       │
│  - TF 프레임 관리                          │
└────────────────┬──────────────────────────┘
                 │
                 ▼
┌───────────────────────────────────────────┐
│          Perception Layer                  │
│                                           │
│  LiDAR 경로:                              │
│  ├── ground_segmentation (ray_ground)     │
│  ├── euclidean_cluster_detector           │
│  ├── shape_estimation (L-Shape fitting)    │
│  └── (선택) CenterPoint / PointPillars    │
│                                           │
│  Camera 경로:                              │
│  ├── tensorrt_yolox (2D 객체 탐지)         │
│  ├── traffic_light_classifier             │
│  └── (선택) semantic_segmentation          │
│                                           │
│  Fusion:                                   │
│  ├── roi_cluster_fusion                   │
│  ├── (선택) pointpainting_fusion          │
│  └── multi_object_tracker                 │
└────────────────┬──────────────────────────┘
                 │
                 ▼
         DetectedObjects / TrackedObjects
         → Planning Layer로 전달
```

**참조**: [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_lidar_transfusion/)

#### AWML (2025)

"An Open-Source ML-based Robotics Perception Framework to Deploy for ROS-based Autonomous Driving Software" - ROS 기반 자율주행을 위한 ML 인지 프레임워크. [arXiv:2506.00645](https://arxiv.org/html/2506.00645v1)

### 5.2 ROS2 Camera-LiDAR Fusion 오픈소스 패키지

| 패키지 | 기능 | 특징 |
|--------|------|------|
| **cam_lidar_framework** | 3D/2D 융합, 탐지, 추적 | 모듈식, 전통적+DL 탐지 지원 |
| **ros2_camera_lidar_fusion** | 캘리브레이션 + 융합 | 내재/외재 캘리브레이션 도구 포함 |
| **cam2lidar_fusion** | 프로젝션 기반 융합 | 수동 변환 행렬, 경량 |
| **YOLOv9 + TensorRT + ROS2** | ADAS용 융합 | 2D LiDAR + Camera, TensorRT 최적화 |

#### 핵심 논문 (ROS2 구현)

- **YOLOv9 + LiDAR Fusion in ROS2**: "LiDAR 2D and Camera Fusion for ADAS: A Practical Approach With YOLOv9 and ROS2 Framework" (IEEE, 2024) - [IEEE Xplore](https://ieeexplore.ieee.org/document/11145440)
- **Object Detection and Distance Estimation**: "ROS2 Implementation of Object Detection and Distance Estimation using Camera and 2D LiDAR Fusion" (IEEE, 2024) - [IEEE Xplore](https://ieeexplore.ieee.org/document/10595707/)
- **임베디드 인지 시스템**: "Mobile robot perception system in ROS 2 on embedded computing platforms" (ScienceDirect, 2024) - [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S2405896324005032)
- **LiDAR-Camera Fusion 개선**: "Improving LiDAR-Camera Fusion in ROS 2" - [Blog](https://brianzheng.info/blog/posts/lidar-camera-fusion.html)

### 5.3 프로젝트 ROS2 노드 설계 (제안)

현재 프로젝트의 `perception.py` 스켈레톤을 기반으로 한 확장 설계:

```
ROS2 노드 구조:

/perception/camera_detector
  - 입력: /sensor/camera/front (Image)
  - 출력: /perception/camera/detections (Detection2DArray)
  - 구현: YOLO11n + TensorRT

/perception/lidar_processor
  - 입력: /sensor/lidar (PointCloud2)
  - 출력: /perception/lidar/obstacles (DetectedObjectArray)
  - 구현: Ground Filter → Clustering → BBox Fitting

/perception/terrain_classifier
  - 입력: /sensor/camera/front (Image)
  - 출력: /perception/terrain/costmap (OccupancyGrid)
  - 구현: ENet/BiSeNetV2 → Traversability Map

/perception/fusion_node
  - 입력: camera detections + lidar obstacles
  - 출력: /perception/fused_objects (DetectedObjectArray)
  - 구현: Frustum-based Late Fusion

/perception/localization
  - 입력: /sensor/gps, /sensor/imu, /sensor/lidar
  - 출력: /localization/pose (PoseStamped)
  - 구현: GPS + IMU EKF (Phase 1), LiDAR SLAM (Phase 2)
```

**참고**: 현재 bridge_config.yaml에서 LiDAR가 `LaserScan`으로 설정되어 있는데, 3D 포인트클라우드 처리를 위해서는 `PointCloud2`로 변경 필요. 시뮬레이션 모델(SDF)에서 GPU LiDAR 또는 3D LiDAR 센서를 사용하도록 수정 권장.

---

## 6. 비포장/험지 Terrain Classification

### 6.1 최신 연구 동향

#### 서베이 논문

- **"Advances and Trends in Terrain Classification Methods for Off-Road Perception"** (Journal of Field Robotics, 2025) - 딥러닝 기반 지형 분류의 최신 동향을 체계적으로 정리. 반지도/자기지도 학습, 합성 데이터, 멀티모달 융합 방향 제시. [Wiley](https://onlinelibrary.wiley.com/doi/10.1002/rob.22586?af=R)
- **"Terrain detection and segmentation for autonomous vehicle navigation: A state-of-the-art systematic review"** (Information Fusion, 2024) - [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S1566253524004226)
- **"A comprehensive survey of UGV terrain traversability for unstructured environments"** (2023) - [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S2215098623001350)

### 6.2 자기지도 학습 기반 Traversability 추정

라벨링 없이 주행 경험으로부터 지형의 주행 가능성을 학습하는 접근법.

| 방법 | 핵심 아이디어 | 연도 | 참조 |
|------|-------------|------|------|
| **Follow the Footprints** | 이전 주행 경로(footprint)를 자기지도 신호로 활용, 표면 경사+시맨틱 정보 결합 | 2024 | [arXiv:2402.15363](https://arxiv.org/abs/2402.15363) |
| **V-STRONG** | Vision Foundation Model + 대조 학습, 분포 외 환경에 강건 | 2024 | [arXiv:2312.16016](https://arxiv.org/html/2312.16016v2) |
| **RoadRunner** | 기존 traversability 스택을 사후 교사(hindsight teacher)로 활용하여 엔드투엔드 학습 | 2024 | [arXiv:2402.19341](https://arxiv.org/html/2402.19341v2) |
| **Wild Visual Navigation** | 사전학습 모델 + 온라인 자기지도, 현장 즉시 적응 | 2025 | [Springer](https://link.springer.com/article/10.1007/s10514-025-10202-x) |
| **Adaptive Online Continual Learning** | 지속적 온라인 학습으로 새 지형에 적응 | 2024 | [IEEE Xplore](https://ieeexplore.ieee.org/document/10494895/) |

**프로젝트 적합성**: 궤도차량의 농경지/험지 주행 데이터를 축적하면서 자기지도 학습으로 traversability 모델을 점진적으로 개선할 수 있다. **Wild Visual Navigation**의 온라인 적응 방식이 다양한 현장 조건에 대응하기에 가장 유연.

### 6.3 멀티모달 Terrain Classification 파이프라인

```
┌────────────────────────────────────────────────┐
│         Multi-Modal Terrain Classification      │
│                                                │
│  Camera ──► 시맨틱 세그멘테이션 ──► 지형 타입   │
│              (색상/텍스처 기반)       분류 맵     │
│                                       │        │
│  LiDAR ──► 높이 맵 ──► 기하학적 특징 ──┤        │
│             (경사, 거칠기, 단차)        │        │
│                                       ▼        │
│  IMU ──► 차체 동역학 ──────► Traversability     │
│          (진동, 기울기)        Cost Map          │
│                                  │             │
│                                  ▼             │
│                        Nav2 Costmap Layer       │
│                        (경로 계획 입력)          │
└────────────────────────────────────────────────┘
```

### 6.4 BEVNet: LiDAR 기반 시맨틱 지형 분류

- 희소 LiDAR 입력에서 직접 Bird's Eye View 지형 클래스 맵 예측
- 기하학적 + 시맨틱 정보를 동시 처리
- 학습된 사전(prior)으로 미관측 영역도 예측 가능
- 시간적 일관성(temporal consistency) 확보
- **프로젝트 적합성**: 16채널 LiDAR의 희소 데이터에서도 동작 가능하도록 설계됨

**참조**: "Semantic Terrain Classification for Off-Road Autonomous Driving" - [OpenReview](https://openreview.net/forum?id=AL4FPs84YdQ), CMU AirLab Off-Road Driving - [AirLab](https://theairlab.org/offroad/)

---

## 7. 우리 프로젝트 적용 로드맵

### 7.1 센서 구성 분석

| 센서 | 사양 | 강점 | 제약 |
|------|------|------|------|
| 카메라 (640x480) | 전방 단안 | 색상/텍스처 인지, 객체 분류 | 저해상도, 단안 깊이 한계 |
| LiDAR (16ch, 360도) | ~300K pts/sec, 100m 거리 | 3D 거리 정보, 전방위 | 포인트 희소, 수직 해상도 낮음 |
| GPS | 일반 GPS (RTK 미확인) | 전역 위치 | 수 미터 오차 (RTK 없을 시) |
| IMU | 가속도/자이로 | 자세 추정, 고주파 | 드리프트 |

### 7.2 단계별 구현 계획

#### Stage 1: 기본 인지 (1-2개월)

```
목표: 독립 센서 파이프라인 구축

[LiDAR 파이프라인]
  sensor/lidar (PointCloud2) → Ray Ground Filter → Euclidean Clustering
  → Bounding Box Fitting → /perception/lidar/obstacles

[Camera 파이프라인]
  sensor/camera/front (Image) → YOLO11n (TensorRT) → 2D Detections
  → /perception/camera/detections

[위치 추정]
  sensor/gps + sensor/imu → robot_localization (EKF)
  → /localization/pose

필요 작업:
  - bridge_config.yaml: LiDAR LaserScan → PointCloud2 변경
  - SDF 모델: 3D LiDAR 센서 플러그인 확인/수정
  - YOLO11n 모델 학습 또는 사전학습 가중치 적용
  - pcl_ros, pointcloud_to_laserscan 등 ROS2 패키지 설치
```

#### Stage 2: 센서 융합 + 지형 인지 (2-3개월)

```
목표: Late Fusion + Terrain Classification

[Late Fusion]
  Camera 2D 박스 + LiDAR 3D 클러스터 → Frustum Association
  → /perception/fused_objects

[지형 분류]
  Camera Image → ENet/BiSeNetV2 (세그멘테이션)
  → 지형 타입 맵 → Traversability Cost Map
  → Nav2 costmap layer 연동

[카메라-LiDAR 캘리브레이션]
  ros2_camera_lidar_fusion 패키지로 외재 파라미터 산출

필요 작업:
  - 카메라-LiDAR 캘리브레이션 데이터 수집 (체커보드)
  - 농경지 지형 데이터셋 구축 또는 공개 데이터셋 활용
  - Traversability 비용 파라미터 튜닝 (궤도차량 특성 반영)
```

#### Stage 3: 농경지 특화 + 고도화 (3-4개월)

```
목표: 작물 행 추종, 경계 인식, 정밀 내비게이션

[작물 행 인식]
  Camera → 시맨틱 세그멘테이션 → 작물 행 중심선 추출
  → 경로 생성 → Planning Layer 전달

[자기지도 Traversability]
  주행 데이터 축적 → WVN 기반 온라인 학습
  → 현장 적응형 지형 분류

[PointPillars 3D 탐지 (선택)]
  16ch LiDAR → PointPillars (TensorRT)
  → 정밀 3D 객체 탐지

필요 작업:
  - 농경지 작물 행 데이터셋 구축 (Gazebo 합성 + 실제)
  - 자기지도 학습 파이프라인 구축
  - PointPillars 모델 파인튜닝 (농경지 객체 클래스)
```

### 7.3 필수 변경 사항 (현재 코드베이스)

현재 `bridge_config.yaml`에서 LiDAR 토픽이 `LaserScan`으로 설정되어 있다:

```yaml
# 현재 (2D LaserScan)
- ros_topic_name: /sensor/lidar
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan

# 변경 필요 (3D PointCloud2)
- ros_topic_name: /sensor/lidar
  ros_type_name: sensor_msgs/msg/PointCloud2
  gz_type_name: gz.msgs.PointCloudPacked
```

현재 `perception.py`에서는 이미 `PointCloud2`를 import하고 있어 코드 수정은 최소화됨:

```python
# perception.py (현재)
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
```

### 7.4 추천 개발 환경 및 하드웨어

| 구성 | 개발/시뮬레이션 | 실차 탑재 |
|------|----------------|-----------|
| **컴퓨트** | 데스크탑 (NVIDIA GPU) | Jetson Orin NX (16GB) |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04 (JetPack) |
| **ROS2** | Humble | Humble |
| **DL 프레임워크** | PyTorch + TensorRT | TensorRT (추론 전용) |
| **시뮬레이션** | Gazebo gz-sim | N/A |

### 7.5 핵심 오픈소스 의존성 목록

| 패키지 | 용도 | 라이선스 |
|--------|------|----------|
| `pcl_ros` | 포인트클라우드 처리 (ROS2) | BSD |
| `ultralytics` | YOLO11/YOLO26 학습/추론 | AGPL-3.0 |
| `tensorrt` | DL 모델 최적화/추론 | NVIDIA |
| `robot_localization` | EKF/UKF 기반 위치 추정 | BSD |
| `nav2` | 경로 계획, costmap, 내비게이션 | Apache-2.0 |
| `ros2_camera_lidar_fusion` | 캘리브레이션 + 융합 | MIT |
| `image_transport` | 이미지 압축/전송 | BSD |
| `cv_bridge` | ROS Image <-> OpenCV 변환 | BSD |

---

## 참고 문헌 종합

### LiDAR 처리
- [Lidar_For_AD_references (GitHub)](https://github.com/beedotkiran/Lidar_For_AD_references)
- [Patchwork++ (arXiv)](https://arxiv.org/pdf/2207.11919)
- [LiDAR Clustering Survey (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC10346986/)
- [PointPillars (arXiv)](https://arxiv.org/abs/1812.05784)

### 카메라 인지
- [YOLO Evolution Survey (arXiv)](https://arxiv.org/pdf/2510.09653)
- [YOLO26 (arXiv)](https://arxiv.org/html/2509.25164v2)
- [YOLOv8 Jetson Benchmark (MDPI)](https://www.mdpi.com/2073-431X/15/2/74)
- [Terrain Detection Segmentation Survey (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S1566253524004226)

### 센서 융합
- [BEVFusion (MIT HAN Lab)](https://github.com/mit-han-lab/bevfusion)
- [DeepFusion (arXiv)](https://arxiv.org/abs/2203.08195)
- [Attention-Based Fusion (MDPI)](https://www.mdpi.com/2032-6653/16/6/306)
- [Cross-dataset Late Fusion (Nature)](https://www.nature.com/articles/s41598-025-32588-5)
- [Early vs. Late Fusion Study (Medium)](https://medium.com/@az.tayyebi/early-vs-late-camera-lidar-fusion-in-3d-object-detection-a-performance-study-5fb1688426f9)

### 농업 자율주행
- [CropNav (arXiv)](https://arxiv.org/html/2411.10974v1)
- [AgroNav (CVPR)](https://openaccess.thecvf.com/content/CVPR2023W/AgriVision/papers/Panda_Agronav_Autonomous_Navigation_Framework_for_Agricultural_Robots_and_Vehicles_Using_CVPRW_2023_paper.pdf)
- [Crop Row Detection Survey (MDPI)](https://www.mdpi.com/2077-0472/15/20/2151)
- [DL Navigation Line Extraction (Springer)](https://link.springer.com/article/10.1007/s44163-025-00301-0)

### ROS2 구현
- [cam_lidar_framework (GitHub)](https://github.com/adrian-soch/cam_lidar_framework)
- [ros2_camera_lidar_fusion (GitHub)](https://github.com/CDonosoK/ros2_camera_lidar_fusion)
- [YOLOv9+ROS2 Fusion (IEEE)](https://ieeexplore.ieee.org/document/11145440)
- [Embedded Perception in ROS2 (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S2405896324005032)
- [Autoware Universe](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_lidar_transfusion/)

### Terrain Classification
- [Off-Road Terrain Classification Survey (Wiley, 2025)](https://onlinelibrary.wiley.com/doi/10.1002/rob.22586?af=R)
- [Follow the Footprints (arXiv)](https://arxiv.org/abs/2402.15363)
- [V-STRONG (arXiv)](https://arxiv.org/html/2312.16016v2)
- [RoadRunner (arXiv)](https://arxiv.org/html/2402.19341v2)
- [Wild Visual Navigation (Springer)](https://link.springer.com/article/10.1007/s10514-025-10202-x)
- [CMU AirLab Off-Road Driving](https://theairlab.org/offroad/)
- [BEVNet Semantic Terrain (OpenReview)](https://openreview.net/forum?id=AL4FPs84YdQ)
