# ARCH-004: 농업용 Hybrid E2E 아키텍처

> **상태**: 채택됨  
> **작성일**: 2026-03-04  
> **관련 TASK**: C60 (Hybrid E2E 아키텍처 도입)

## 1. 개요

SS500 궤도차량 농업 자율주행을 위한 Hybrid End-to-End 아키텍처를 도입한다.

- **Tesla 방식**: Monolithic E2E (완전 신경망)
- **Autoware 방식**: Modular E2E (단계적 통합)
- **SS500 방식**: **Hybrid E2E** (Learned Perception + Learned Planning + Rule-based Guardian)

농업 환경의 특수성(작물 보호, 저속 주행, 비구조화 환경)을 고려하여 안전성과 유연성을 모두 확보한다.

## 2. 아키텍처 개요

```
┌─────────────────────────────────────────────────────────────────┐
│                    SS500 Hybrid E2E Pipeline                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────────────────────────────┐   │
│  │   Sensors    │───→│      Learned Perception              │   │
│  │  (Camera,    │    │  (Foundation Model - AutoSeg Style)  │   │
│  │   LiDAR,     │    └──────────────────┬───────────────────┘   │
│  │   IMU, GPS)  │                       │                       │
│  └──────────────┘                       ↓                       │
│                              ┌──────────────────┐              │
│                              │  Feature Fusion  │              │
│                              │  (BEV + Temporal)│              │
│                              └────────┬─────────┘              │
│                                       │                        │
│                                       ↓                        │
│  ┌────────────────────────────────────────────────────────┐   │
│  │              Learned Planning (Diffusion)               │   │
│  │  - 작물 행 추종 (Crop Row Following)                     │   │
│  │  - 커버리지 경로 최적화 (Fields2Cover + Neural)          │   │
│  │  - 장애물 회피 궤적 생성                                 │   │
│  └──────────────────────┬───────────────────────────────────┘   │
│                         │                                       │
│                         ↓                                       │
│  ┌────────────────────────────────────────────────────────┐   │
│  │              Safety Guardian (Rule-Based)               │   │
│  │  - 경사도 검증 (Slope Check)                             │   │
│  │  - 작물 손상 위험 평가 (Crop Damage Risk)                 │   │
│  │  - 비상 정지 조건 (Emergency Stop)                       │   │
│  │  - 속도/조향 한계 검증 (Limits Check)                    │   │
│  └──────────────────────┬───────────────────────────────────┘   │
│                         │                                       │
│                         ↓                                       │
│  ┌────────────────────────────────────────────────────────┐   │
│  │         Traditional Control (Pure Pursuit + MPC)        │   │
│  │  - 슬립 보상 (Skid Steer Compensation)                   │   │
│  │  - 저속 정밀 제어 (Low-speed Precision)                  │   │
│  └────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## 3. 모듈 상세

### 3.1 Learned Perception (`ad_perception/learned/`)

**목적**: Foundation Model 기반 다중 태스크 인지

**입력**: 
- Camera (RGB): 640x480, 30fps
- LiDAR: PointCloud2
- IMU: 가속도, 각속도

**출력**:
```python
PerceptionFeatures:
    - crop_rows: List[Line3D]          # 작물 행 위치
    - terrain_type: TerrainClass       # 지형 분류 (7종)
    - obstacles: List[Object3D]        # 장애물 (나무, 돌 등)
    - free_space: OccupancyGrid        # 주행 가능 영역
    - slope: float                      # 경사도
    - confidence: float                 # 신뢰도
```

**구현**:
- Backbone: EfficientNet-B3 or ResNet-50
- Heads: 
  - Crop Row Detection Head
  - Terrain Classification Head
  - Obstacle Detection Head (CenterPoint)
  - Free Space Segmentation Head

### 3.2 Learned Planning (`ad_planning/learned/`)

**목적**: Diffusion Model 기반 경로 생성

**입력**:
- PerceptionFeatures
- 목표 위치 (GPS or Local)
- 작업 모드 (Plowing, Seeding, Harvesting)

**출력**:
```python
PlannedTrajectory:
    - waypoints: List[Pose2D]          # 경로 점들
    - timestamps: List[float]          # 각 점의 예상 시간
    - confidence: float                # 경로 신뢰도
    - alternative_paths: List[Path]    # 대체 경로들
```

**구현**:
- Base: Fields2Cover (Boustrophedon)
- Refinement: Diffusion Model
- Cost Function: 작물 손상 최소화 + 이동 거리 최소화

### 3.3 Safety Guardian (`ad_control/guardian/`)

**목적**: 딥러닝 출력을 검증하는 규칙 기반 안전 계층

**검증 항목**:
```python
class SafetyGuardian:
    def validate(self, trajectory, vehicle_state, perception):
        # 1. 경사 검증
        if max_slope(trajectory) > MAX_SLOPE:
            return False, "SLOPE_TOO_STEEP"
        
        # 2. 작물 거리 검증
        if min_distance_to_crop(trajectory) < MIN_CROP_DISTANCE:
            return False, "CROP_TOO_CLOSE"
        
        # 3. 속도 한계
        if any(v > MAX_SPEED for v in trajectory.velocities):
            return False, "SPEED_EXCEEDED"
        
        # 4. 급격한 조향 변경
        if max_curvature(trajectory) > MAX_CURVATURE:
            return False, "CURVATURE_TOO_HIGH"
        
        return True, "OK"
```

### 3.4 Traditional Control (`ad_control/`)

기존 Pure Pursuit + Slip Compensation 유지
- 검증된 안정성
- 농업 환경에 최적화됨

## 4. 데이터 흐름

```
Time t:
1. Sensor Data → Perception Model (10Hz)
2. Perception Features → Planning Model (5Hz)
3. Planned Trajectory → Guardian Check (100Hz)
4. Valid Trajectory → Control (20Hz)
5. Control Command → Vehicle

Time t+1:
... (반복)
```

## 5. 학습 데이터 파이프라인

### 5.1 데이터 수집
```
[시뮬레이션] Gazebo + 다양한 농경지 월드
    ↓
[실차 데이터] SS500 실제 주행 데이터
    ↓
[데이터 증강] 시뮬 + 실차 합성
```

### 5.2 레이블링
- 자동: 시뮬레이션에서 GT 생성
- 수동: 실차 데이터에서 전문가 검증
- Semi-supervised: 자동 레이블 + 수동 검증

## 6. fallback 계층

Learned Planning 실패 시 계층적 fallback:

```
Level 0: Learned Planning (Diffusion)
    ↓ 실패
Level 1: Fields2Cover (규칙 기반 커버리지)
    ↓ 실패
Level 2: Pure Pursuit (기존 방식)
    ↓ 실패
Level 3: Emergency Stop (비상 정지)
```

## 7. 성능 목표

| 지표 | 목표 | 현재 | 개선율 |
|------|------|------|--------|
| 작물 손상률 | < 0.1% | 0.5% | 5x |
| 경로 부드러움 | < 0.1 m/s³ jerk | 0.3 m/s³ | 3x |
| 인지 정확도 | > 95% | 85% | 1.12x |
| 주행 효율 | > 98% | 92% | 1.06x |

## 8. 개발 단계

### Phase 1 (2주): 기반 구조
- [ ] `ad_perception/learned/` 패키지 생성
- [ ] `ad_planning/learned/` 패키지 생성
- [ ] `ad_control/guardian/` 모듈 생성
- [ ] 메시지 인터페이스 정의

### Phase 2 (4주): Learned Perception
- [ ] Foundation Model backbone 구현
- [ ] Multi-task heads 구현
- [ ] Training pipeline 구축
- [ ] Gazebo 데이터셋 생성

### Phase 3 (4주): Learned Planning
- [ ] Diffusion Planner 통합
- [ ] Fields2Cover 연동
- [ ] Cost function 설계

### Phase 4 (2주): Safety Guardian
- [ ] 규칙 엔진 구현
- [ ] Fallback 로직 구현
- [ ] 통합 테스트

### Phase 5 (지속): 실차 검증
- [ ] 실제 농경지 테스트
- [ ] 성능 튜닝
- [ ] 데이터 추가 수집

## 9. 관련 모듈

- `ad_core/datatypes.py`: PerceptionFeatures, PlannedTrajectory 추가
- `ad_perception/learned/`: Foundation Model 기반 인지
- `ad_planning/learned/`: Diffusion 기반 경로 계획
- `ad_control/guardian/`: 안전 검증 레이어

## 10. 참고 자료

- Tesla FSD v12: End-to-End Neural Networks
- Autoware.Universe: Modular E2E Roadmap
- Fields2Cover: Agricultural Coverage Planning
- Hydra-MDP: End-to-End Driving at Scale (CVPR 2024)

---

**결정 사항**: 
- Tesla의 Monolithic E2E 대신 Autoware-style Modular E2E 채택
- 농업 특성상 Safety Guardian는 Rule-based로 유지
- Phase별 점진적 도입으로 리스크 최소화
