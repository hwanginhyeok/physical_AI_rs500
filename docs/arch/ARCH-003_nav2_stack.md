# ARCH-003: Nav2 플래너/컨트롤러 스택 구성

- 상태: `채택됨`
- 결정일: 2026-02-21 (C20, C21), 2026-03-01 업데이트 (C52)
- 관련 TASK: C20, C21, C52

## 배경

농경지 자율주행에서 Nav2 스택의 글로벌 플래너와 로컬 컨트롤러를 선택해야 한다.
SS500은 스키드 스티어(궤도차량)이므로 회전 반경이 작고 후진이 가능하다.
농경지 특성상 좁은 두렁, 반복 패턴, 드리프트 있는 지형을 고려해야 한다.

## 결정

**글로벌 플래너**: `SmacPlannerLattice` (격자 기반 최적 경로)
**로컬 컨트롤러**: `MPPIController` (샘플링 기반 최적 제어)
**costmap 구성**: global(obstacle+inflation) + local(obstacle+voxel+inflation)

```
Foxglove 클릭 → /goal_pose
                    ↓
         waypoint_manager_node
                    ↓ FollowWaypoints 액션
         Nav2 WaypointFollower
                    ↓ NavigateToPose
         BT Navigator
          ├── SmacPlannerLattice  →  /plan (글로벌 경로)
          └── MPPIController      →  /local_plan → /cmd_vel_smoothed
                                             ↓
                                    VelocitySmoother → /cmd_vel
                                             ↓
                                    CollisionMonitor → /cmd_vel (최종)
                                             ↓
                                         Gazebo / 실물
```

**주요 파라미터 결정:**

| 파라미터 | 값 | 이유 |
|----------|-----|------|
| Lattice `lattice_filepath` | 5cm/0.5m 회전반경 diff | SS500 최소 회전반경 근사 |
| MPPI `batch_size` | 1000 | GPU 없이 CPU에서 실시간 처리 가능한 상한 |
| MPPI `motion_model` | `DiffDrive` | 스키드 스티어 → DiffDrive 근사 (정확도 vs 단순성) |
| MPPI `vx_max` | 3.0 m/s | 농경지 최대 실용 속도 |
| global costmap `rolling_window` | `true` | map_server 없이 동작 가능 (C52) |
| global costmap `width/height` | 100m | 50m 반경 커버 |
| local costmap `resolution` | 0.05m | 정밀 장애물 회피 |

## 근거

**SmacPlannerLattice 선택 이유:**
- 차동 구동 로봇의 **운동학적 제약을 경로에 내재화** (TEB도 가능하나 격자 방식이 장거리 일관성 높음)
- 후진 허용 (`allow_reverse_expansion: true`) → 좁은 두렁에서 유용
- 파일 기반 lattice primitive → 운동학 파라미터 독립적으로 튜닝 가능

**MPPIController 선택 이유:**
- 샘플링 기반이라 복잡한 비용 함수를 쉽게 정의 가능
- 로컬 장애물 회피와 경로 추종을 통합 최적화
- DiffDrive 모션 모델로 스키드 스티어 근사 허용

**global costmap rolling_window 전환 (C52):**
- 원래 static_layer 포함 설계였으나 empty_map.yaml + map_server 구성에서
  lifecycle 순서 문제로 초기화 지연 발생
- rolling_window 모드는 map_server 없이 센서 데이터만으로 동작 가능
- 농경지처럼 사전 맵 없이 탐색하는 시나리오에 더 적합

## 검토한 대안

| 대안 | 기각 이유 |
|------|-----------|
| NavfnPlanner | Dubins/Reeds-Shepp 곡선 미지원, 궤도차 운동학 무시 |
| TEB (Timed Elastic Band) | 파라미터 민감도 높음. 장거리 농경지 경로에서 불안정 사례 |
| DWA Controller | 샘플링 공간 제한적, 장애물 근방 성능 MPPI 대비 열위 |
| Pure Pursuit (직접 구현) | C26에서 구현했으나 Nav2 생태계 통합 및 장애물 회피를 위해 MPPI 병행 |

## 결과 및 트레이드오프

- 긍정: Nav2 표준 구성, Foxglove와 costmap/경로 시각화 자연스럽게 연동
- 부정/제약:
  - MPPI CPU 부하 (batch 1000 × time_steps 56 매 제어 주기)
  - SmacPlannerLattice primitive 파일 교체 없이는 운동학 변경 반영 어려움
  - rolling_window 전환으로 절대 좌표 기반 전역 장애물 지도 없음 (GPS 기반 글로벌 추정 완료 후 재검토)

## 변경 이력

| 날짜 | 변경 내용 |
|------|-----------|
| 2026-02-21 | C20(SmacLattice), C21(MPPI) 최초 구현 |
| 2026-03-01 | C52: global costmap → rolling_window 전환, static_layer 제거 |
