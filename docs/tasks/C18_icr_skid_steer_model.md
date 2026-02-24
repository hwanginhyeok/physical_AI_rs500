# C18: ICR 기반 스키드 스티어 조향 모델

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

궤도차량의 스키드 스티어 운동학 모델 구현. ICR(Instantaneous Center of Rotation) 기반으로 좌/우 궤도 속도차에 의한 조향을 모델링.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/skid_steer_model.py` — 834줄 (C27 보정 모델 포함)

### 핵심 클래스

**`SkidSteerConfig`** (dataclass):
- `track_width` — 궤도 간 거리 (m)
- `max_speed` — 최대 속도 (m/s)
- `steering_efficiency` — 조향 효율 (0~1, 디폴트 0.8)
- `slip_table` — 지면별 슬립 계수 딕셔너리

**`SkidSteerModel`**:
- `twist_to_tracks(linear, angular)` → `(left_speed, right_speed)`
- `tracks_to_twist(left, right)` → `(linear, angular)`
- `predict_pose(pose, left, right, dt)` → `Pose2D` (운동학 적분)
- `compute_icr(left, right)` → ICR 좌표
- `apply_slip(left, right, terrain)` → 슬립 적용된 속도
- `get_turning_radius(left, right)` → 선회 반경

### 설계 결정

- ICR 기반 운동학: 차동 구동과 유사하되 `steering_efficiency` 파라미터로 궤도차량 특성 반영
- 슬립 테이블: 지면 유형별 정적 슬립 계수 (C38에서 동적 모델로 확장)
- `predict_pose()`에서 Euler 적분 사용

## 테스트

- `src/ad_core/test/test_skid_steer_model.py` — 189줄, 20개 테스트
- 직진/선회/제자리 회전/역방향 등 시나리오 검증
