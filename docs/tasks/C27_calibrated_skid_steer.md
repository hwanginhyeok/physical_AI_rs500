# C27: 유효 궤도 폭 보정 + 모터 비대칭 보상

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

실차 운행 중 궤도 폭과 모터 비대칭을 온라인 보정하는 기능. 이론적 궤도 폭과 실제 선회 거동의 차이, 좌/우 모터 출력 차이를 자동 보정.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/skid_steer_model.py` — 240~834행

### 핵심 클래스

**`CalibrationData`** (dataclass, 240~298행):
```python
track_width_correction: float = 1.0    # 유효 궤도 폭 보정 계수
left_motor_gain: float = 1.0           # 좌 모터 게인
right_motor_gain: float = 1.0          # 우 모터 게인
calibration_count: int = 0
track_width_samples: int = 0
asymmetry_samples: int = 0
```

**`CalibratedSkidSteerModel`** (301행, SkidSteerModel 상속):
- `effective_track_width` 프로퍼티 — `track_width * calibration.track_width_correction`
- `twist_to_tracks()` 오버라이드 — 유효 궤도 폭 + 모터 게인 적용
- `tracks_to_twist()` 오버라이드 — 모터 게인 역보정
- `update_track_width_correction()` — 온라인 EMA 보정 (commanded/measured angular 비율)
  - `min_angular_for_track_cal: 0.1` rad/s 이상일 때만 보정
  - `correction_factor_min/max: 0.5~2.0` 범위 제한
- `update_motor_asymmetry()` — 직진 시 편향 감지하여 좌/우 게인 보정
  - 5개 이상 직진 샘플 축적 후 보정
  - `motor_gain_min/max: 0.8~1.2` 범위 제한
- `run_straight_calibration()` — 오프라인 직진 보정 (yaw_rate 샘플 배치)
- `run_rotation_calibration()` — 오프라인 선회 보정 (중앙값 기반)
- `save_calibration()` / `load_calibration()` — JSON 파일 영속화

### 설계 결정

- EMA(alpha=0.05) 기반 온라인 보정: 약 20개 샘플 이동 평균
- 온라인/오프라인 보정 모두 지원
- 보정 범위에 상/하한 설정 — 비정상 값 방지
- JSON 파일로 보정 데이터 저장/복원 — 전원 off 후에도 유지

## 테스트

- `src/ad_core/test/test_skid_steer_model.py` — 189줄, 20개 테스트
- 보정 수렴, 범위 제한, 비대칭 보상 검증
