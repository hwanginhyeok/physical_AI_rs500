# C21: MPPI 지역 경로 계획

- 분야: 알고리즘/설정
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

Nav2의 MPPI(Model Predictive Path Integral) 컨트롤러를 지역 경로 계획기로 설정.

## 변경 사항

### 주요 파일

`src/ad_bringup/config/nav2_params.yaml` — 154~247행

### MPPI Controller 설정

```yaml
plugin: "nav2_mppi_controller::MPPIController"
time_steps: 56
model_dt: 0.05            # 예측 수평선 = 2.8초
batch_size: 1000
temperature: 0.3
motion_model: "DiffDrive"
vx_max: 3.0
vx_min: -1.5
wz_max: 2.0
```

### Critics (비용 함수)

| Critic | 가중치 | 역할 |
|--------|--------|------|
| GoalCritic | 5.0 | 목표점 도달 |
| GoalAngleCritic | 3.0 | 목표 방향 정렬 |
| ObstaclesCritic | 20.0 | 장애물 회피 (최대 가중치) |
| PathFollowCritic | 5.0 | 글로벌 경로 추종 |
| PathAngleCritic | 2.0 | 경로 방향 정렬 |
| PathAlignCritic | 14.0 | 경로 정렬 |
| PreferForwardCritic | 5.0 | 전진 선호 |

### 설계 결정

- DiffDrive 모션 모델: 궤도차량을 차동 구동으로 근사
- batch_size=1000: 충분한 샘플 수로 최적 경로 탐색
- ObstaclesCritic 가중치(20.0)를 가장 높게 설정 — 안전 최우선
- 후진 속도(-1.5m/s)를 전진(3.0m/s)의 절반으로 제한
