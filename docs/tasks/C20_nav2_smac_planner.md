# C20: Nav2 SmacPlannerLattice

- 분야: 알고리즘/설정
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

Nav2 글로벌 경로 계획기로 SmacPlannerLattice를 설정. 궤도차량의 비홀로노믹 제약을 반영하는 격자 기반 경로 계획.

## 변경 사항

### 주요 파일

`src/ad_bringup/config/nav2_params.yaml` — 489줄 (C21 MPPI 포함)

### SmacPlannerLattice 설정 (91~123행)

```yaml
plugin: "nav2_smac_planner::SmacPlannerLattice"
tolerance: 0.25
max_iterations: 1000000
max_planning_time: 5.0
analytic_expansion_ratio: 3.5
reverse_penalty: 2.0
allow_reverse_expansion: true
smooth_path: true
smoother: { w_smooth: 0.3, w_data: 0.2 }
```

### 기타 Nav2 구성

- `global_costmap`: 해상도 0.1m, static + obstacle + inflation 레이어
- `local_costmap`: 해상도 0.05m, 롤링 6×6m, voxel_layer 포함
- `smoother_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`
- `map_server`: 정적 맵 서빙

### 설계 결정

- SmacPlannerLattice 선택: 후진 허용 + 비홀로노믹 차량 지원
- `reverse_penalty: 2.0` — 전진 선호하되 후진 허용
- 별도 Python 구현 없이 Nav2 파라미터 설정으로 구성
