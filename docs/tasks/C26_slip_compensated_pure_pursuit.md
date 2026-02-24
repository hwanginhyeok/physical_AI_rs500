# C26: 슬립 보상 적응형 Pure Pursuit

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

지면 슬립을 실시간 추정하여 Pure Pursuit 경로 추종을 보상하는 적응형 알고리즘. C17 기본 Pure Pursuit를 상속 확장.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/pure_pursuit.py` — 708~1002행

### 핵심 클래스

**`SlipCompensatedPurePursuitConfig`** (dataclass, 708~737행):
```python
slip_lookahead_gain: float = 2.0
max_slip_lookahead_ratio: float = 2.0
longitudinal_compensation_gain: float = 1.0
lateral_compensation_gain: float = 0.8
slip_speed_reduction_gain: float = 0.5
min_speed_ratio: float = 0.2
ema_alpha: float = 0.15
enable_terrain_adaptation: bool = True
```

**`SlipCompensatedPurePursuit`** (740행, PurePursuitTracker 상속):
- `set_terrain(terrain_type)` — 지면 유형 설정, 슬립 프로파일 업데이트
- `update_slip(measured_velocity, commanded_velocity)` — 실시간 슬립 추정
- `_compute_lookahead_distance()` 오버라이드 — 슬립 비례 lookahead 확대 (최대 2배)
- `_compute_speed()` 오버라이드 — 슬립 비례 감속
- `_twist_to_tracks()` 오버라이드 — 종방향/횡방향 슬립 보상 (최대 1.5배)

**`SlipEstimator`** (517행):
- EMA(Exponential Moving Average) 기반 슬립 추정
- 지면 유형별 사전 슬립 프로파일 (`TERRAIN_SLIP_TABLE`)

**`TerrainType`** enum (397행):
- PAVED, DIRT_ROAD, GRASS, GRAVEL, CROP_FIELD, MUD

**`TerrainSlipProfile`** (414행):
- `longitudinal_slip`, `lateral_slip`, `slip_variability`, `max_safe_speed`

### 설계 결정

- 상속 기반 확장: PurePursuitTracker의 3개 핵심 메서드만 오버라이드
- EMA 필터로 노이즈 제거 (alpha=0.15)
- 보상량에 상한 설정 (1.5배) — 과보상 방지
- 지면 유형별 사전 프로파일 + 실시간 측정 융합

## 테스트

- `src/ad_core/test/test_slip_compensated_pursuit.py` — 590줄, 40개 테스트
- 지면별 슬립 보상 검증, 보상 상한 검증, 경로 추종 정확도 비교
