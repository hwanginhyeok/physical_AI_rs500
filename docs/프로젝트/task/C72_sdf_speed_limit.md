# C72: SDF 속도 제한 — 실차 max 0.83m/s + Nav2 동기화

## 배경

현재 시뮬레이션 모델(model.sdf)에 차량 최대 속도 제한이 없다.
HIH_2 VCU 펌웨어(`Dm.h`)에 `MOTOR_MAX_SPD = 30` (3.0 km/h = 0.833 m/s)이 정의되어 있으므로
시뮬레이션과 Nav2 파라미터에 동일하게 반영한다.

## HIH_2 출처

- **파일**: `/home/gint_pcd/projects/HIH_2/1)프로젝트/5)농업/ssvcu_branch_20260105_merge_20260128/SSVCU/Asw/Dm.h`
- **값**:
  - `MOTOR_MAX_SPD = 30` → 3.0 km/h → **0.833 m/s**
  - `MOTOR_MIN_SPD = 5` → 0.5 km/h → **0.139 m/s**

## 변경 내역

### 1. model.sdf — TrackedVehicle 플러그인

TrackedVehicle 플러그인에 속도 제한 파라미터 추가:
- `max_velocity`: 0.833
- `min_velocity`: 0.139

### 2. nav2_params.yaml — 속도 파라미터 동기화

| 파라미터 | 현재값 | 변경값 | 비고 |
|---------|--------|--------|------|
| `controller_server.max_vel_x` | 1.5 (또는 미설정) | 0.83 | 전진 최대 |
| `controller_server.min_vel_x` | -0.5 | -0.83 | 후진 최대 |
| `velocity_smoother.max_velocity` | (확인 필요) | [0.83, 0.0, 0.5] | vx, vy, wz |

### 3. velocity_smoother 가감속 제한

실차 램핑 로직에 맞춰 가감속 제한:
- `max_accel`: [0.5, 0.0, 1.0] (부드러운 가속)
- `max_decel`: [1.0, 0.0, 2.0] (빠른 감속)

## 수정 대상 파일

- `src/ad_bringup/models/ss500/model.sdf`
- `src/ad_bringup/config/nav2_params.yaml`

## 검증

- `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5}}" --once`
  → Gazebo에서 실제 속도가 0.83 m/s 이하로 클램핑되는지 확인
- Nav2 웨이포인트 주행 시 `cmd_vel.linear.x`가 0.83을 초과하지 않는지 Plot 확인
