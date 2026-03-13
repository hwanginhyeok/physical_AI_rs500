# C60: 농업용 Hybrid E2E 아키텍처 구축

> **작업**: C60  
> **분야**: 아키텍처  
> **중요도**: P2  
> **담당**: 그린  
> **상태**: 진행 중  
> **생성일**: 2026-03-04

## 개요

Tesla FSD v12의 End-to-End Neural Network 방식과 Autoware.Universe의 Modular E2E 방식을 결합하여, 농업용 궤도차량(SS500)에 최적화된 Hybrid E2E 아키텍처를 구축한다.

### Hybrid E2E의 정의

```
┌─────────────────────────────────────────────────────────────┐
│                    Tesla (Monolithic)                        │
│   [Camera] → [One Big Neural Net] → [Steering/Brake]        │
│              (300k lines → 2k lines)                        │
└─────────────────────────────────────────────────────────────┘
                              ↓
                    (참고: 개념적 이점)
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                  Autoware (Modular E2E)                      │
│   [Sensor] → [Perception] → [Planning] → [Control]          │
│               (Foundation    (Diffusion    (MPC)            │
│                 Models)       Planner)                      │
└─────────────────────────────────────────────────────────────┘
                              ↓
                    (참고: 구조적 안정성)
                              ↓
┌─────────────────────────────────────────────────────────────┐
│              SS500 Hybrid E2E (제안)                         │
│   [Sensor] → [Learned      → [Learned    → [Guardian]      │
│               Perception]     Planning]    (Rule-Based)     │
│               (AutoSeg)       (Diffusion)  ↓                │
│                                            [Control]        │
│                                            (Pure Pursuit)   │
└─────────────────────────────────────────────────────────────┘
```

**핵심 차별점**:
1. **Safety Guardian**: 딥러닝 출력을 검증하는 명시적 규칙 계층 (농업용 필수)
2. **Fallback Hierarchy**: Learned → Fields2Cover → Pure Pursuit → Emergency Stop
3. **Agriculture-specific**: 작물 보호, 경사 제한, 저속 정밀 제어

## 배경 및 동기

### Tesla FSD v12의 혁신
- 300,000+ 라인 C++ 코드 → 2,000 라인 (2개월 만에 구현)
- End-to-End Neural Network (칼라 입력 → 조향/가속 출력)
- "ChatGPT for Cars" 철학

### Autoware의 Modular E2E 로드맵
- 2024-2025: Learned Planning (Step 1)
- 2025-2026: Deep Perception + Learned Planning (Step 2)
- 2026+: Monolithic E2E (Step 3)
- 2027+: Hybrid E2E with Guardian (Step 4)

### 농업용 특수성
| 특성 | 도로 주행 | 농업 주행 |
|------|-----------|-----------|
| 안전 우선순위 | 보행자/차량 | 작물/토양 |
| 환경 | 구조화 | 비구조화 |
| 속도 | 60-120 km/h | 5-15 km/h |
| 실패 결과 | 사고 | 수확량 감소 |

→ **완전한 E2E는 위험, 명시적 Guardian 필요**

## 설계 결정

### ADR-004-1: Safety Guardian 도입
**결정**: Learned Planning 출력을 Rule-based로 검증하는 Guardian 계층 추가

**이유**:
- 농업에서는 "블랙박스" 신경망 출력을 신뢰할 수 없음
- 작물 손상 방지를 위한 명시적 규칙 필요
- 규제/인증을 위한 해석 가능한 결정 필요

**구현**:
- `ad_control/safety_guardian.py`
- 검증 항목: 경사, 작물 거리, 속도, 곡률, 장애물
- Fallback 트리거 조건 정의

### ADR-004-2: Learned Perception Architecture
**결정**: AutoSeg 스타일의 Foundation Model (HydraNet) 채택

**구조**:
```
Single Backbone (EfficientNet-B3)
    ↓
Multiple Heads:
    ├── Crop Row Detection Head
    ├── Terrain Classification Head
    ├── Obstacle Detection Head (CenterPoint)
    └── Free Space Segmentation Head
```

**장점**:
- Tesla의 HydraNet과 유사한 구조 (개념적 호환)
- 다중 태스크로 효율적인 계산
- Autoware.Universe의 CenterPoint와 통합 가능

### ADR-004-3: Fallback Hierarchy
**결정**: 4단계 Fallback 계층 구조

```
Level 0: Learned Planning (Diffusion Model)
    ↓ 실패 또는 Guardian 거부
Level 1: Fields2Cover (Rule-based Coverage)
    ↓ 실패
Level 2: Pure Pursuit (Traditional)
    ↓ 실패
Level 3: Emergency Stop
```

**복구 전략**:
- 연속 10회 안전 판정 시 상위 레벨로 복귀
- 급격한 레벨 변화 시 히스테리시스 적용

## 변경 내역

### 새로 생성된 파일

| 파일 | 설명 | 라인 |
|------|------|------|
| `docs/arch/ARCH-004_hybrid_e2e_agricultural.md` | 아키텍처 결정 기록 | 200+ |
| `src/ad_core/ad_core/hybrid_e2e_types.py` | Hybrid E2E 데이터 타입 | 250+ |
| `src/ad_control/ad_control/safety_guardian.py` | Safety Guardian 모듈 | 400+ |
| `src/ad_control/ad_control/hybrid_e2e_node.py` | 통합 노드 | 350+ |

### 수정된 파일

| 파일 | 변경 내용 |
|------|-----------|
| `docs/프로젝트/TASK.md` | C60 작업 추가 |

## 검증 방법

### 1. Unit Test (Guardian)
```python
def test_guardian_crop_distance():
    guardian = SafetyGuardian()
    trajectory = create_test_trajectory(distance_to_crop=0.05)  # 5cm
    decision = guardian.validate(trajectory, ...)
    assert not decision.is_safe
    assert "CROP" in decision.violated_constraints[0]
```

### 2. Integration Test (Fallback)
```python
def test_fallback_hierarchy():
    node = HybridE2ENode()
    
    # Level 0 실패 시뮬레이션
    node._fallback_level = 0
    node._run_planning = Mock(side_effect=[None, mock_trajectory])
    
    trajectory = node._run_planning(perception)
    assert node._fallback_level == 1  # Fields2Cover로 fallback
```

### 3. Simulation Test (Gazebo)
- 목표: Gazebo 농경지 월드에서 100m 주행
- 기준: Guardian이 10회 이상 작물 보호 조치
- 측정: fallback 횟수, emergency stop 횟수

## 미결 이슈

### ISSUE-1: Learned Perception 학습 데이터
**문제**: 실제 농경지 데이터 부족
**해결책**: 
- Gazebo에서 합성 데이터 생성 (procedural fields)
- 실차 소규모 데이터 수집 (100장 → 10,000장 증강)
- Transfer Learning: Cityscapes → Agricultural

### ISSUE-2: Diffusion Planner 실시간 성능
**문제**: RTX 2060에서 20Hz 달성 가능?
**해결책**:
- Lightweight Diffusion Model (1-step denoising)
- TensorRT 최적화
- 필요 시 Rule-based로 fallback

### ISSUE-3: Guardian 임계값 튜닝
**문제**: 너무 엄격하면 frequent fallback, 너무 느슨하면 작물 손상
**해결책**:
- 시뮬레이션에서 파라미터 스윕
- 실차로 10시간 이상 주행 데이터 수집
- Adaptive threshold (학습 기반 동적 조정)

## 다음 단계

### Phase 1 (완료): 기반 구조
- [x] 아키텍처 설계 문서
- [x] 데이터 타입 정의
- [x] Safety Guardian 구현
- [x] 통합 노드 구조

### Phase 2 (예정): Learned Perception
- [ ] Foundation Model backbone 구현
- [ ] Multi-task heads 구현
- [ ] Training pipeline 구축
- [ ] Gazebo 데이터셋 생성

### Phase 3 (예정): Learned Planning
- [ ] Diffusion Planner 통합
- [ ] Fields2Cover 연동
- [ ] Cost function 설계

### Phase 4 (예정): 통합 및 검증
- [ ] Gazebo 시뮬레이션 테스트
- [ ] 실차 소규모 테스트
- [ ] 성능 메트릭 측정

## 참고 자료

1. Tesla FSD v12: End-to-End Neural Networks (2024)
2. Autoware.Universe: Modular E2E Roadmap (2025)
3. Fields2Cover v2.0: Non-convex Field Support
4. Hydra-MDP: End-to-End Driving at Scale (CVPR 2024)
5. Safety-Critical E2E: Guardian Concept (Waymo, 2023)

## 결론

농업용 자율주행에 Tesla/Autoware의 E2E 개념을 적용하되, **Safety Guardian**을 통해 농업 특유의 안전성 요구사항을 충족하는 Hybrid E2E 아키텍처를 제안한다.

이 접근법은:
- Tesla의 "학습 기반 유연성"을 채택
- Autoware의 "모듈형 안정성"을 유지
- 농업의 "작물 보호 우선"을 반영

## 관련 문서

- ARCH-004: Hybrid E2E 아키텍처 (본 문서)
- ARCH-001: TF Tree 구조
- ARCH-002: Dual-EKF 센서 퓨전
- ARCH-003: Nav2 스택 구성
