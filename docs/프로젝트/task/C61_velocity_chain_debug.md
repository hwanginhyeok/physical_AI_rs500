# C61: 차량 물리 동작 검증 — Velocity Chain 디버깅

**작업 ID:** C61
**분야:** 시뮬레이션
**중요도:** P2
**담당:** 그린
**상태:** 진행 (DDS 문제로 실행 검증 대기)
**발견:** C52 검증 중 (2026-03-04)

---

## 배경

C52에서 Nav2 스택 동작까지 확인 완료. controller_server가 cmd_vel_nav를 정상 발행하나
**차량이 실제로 움직이지 않는 문제** 발견.

```
[controller_server]: Received a goal, begin computing control effort. ✅
[controller_server]: cmd_vel_nav 발행 (linear.x: 0.168...) ✅
[controller_server]: Failed to make progress ❌
```

---

## 결정 사항

### 1. Root Cause: FastRTPS SharedMemory 포트 충돌

```
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7011:
  open_and_lock_file failed
```

- lifecycle_manager가 velocity_smoother를 activate 하지 못함
- velocity_smoother가 inactive 상태 → cmd_vel_smoothed 미발행
- 하류 cmd_vel_relay / collision_monitor 모두 입력 없음 → 차량 미이동

### 2. 해결 방안: CycloneDDS 전환 (영구 해결)

| 대안 | 장점 | 단점 |
|------|------|------|
| **CycloneDDS 전환** (채택) | 영구 해결, WSL2 안정적 | 패키지 설치 필요 |
| FastRTPS SharedMemory 수동 정리 | 빠른 | WSL 재시작마다 반복 |
| FastRTPS SharedMemory 비활성화 XML | 중간 | 프로파일 관리 번거로움 |

```bash
# CycloneDDS 전환 방법
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

### 3. collision_monitor 우회 (임시)

cmd_vel_relay 노드를 추가하여 collision_monitor를 바이패스:
```
cmd_vel_smoothed → cmd_vel_relay → cmd_vel → ros_gz_bridge → Gazebo
```
- 파일: `src/ad_bringup/ad_bringup/cmd_vel_relay.py` (신규)
- 정상 동작 확인 후 collision_monitor 복원 예정

---

## Velocity Chain 분석

### 토픽 흐름 (정상 동작 시)

```
controller_server (20Hz)
    │ /cmd_vel_nav
    ▼
velocity_smoother (Rate Limiter, 20Hz)
    │ /cmd_vel_smoothed
    ▼
collision_monitor (FootprintApproach, LiDAR)
    │ /cmd_vel
    ▼
ros_gz_bridge → Gazebo TrackedVehicle plugin
```

### velocity_smoother 알고리즘: Rate Limiter

```
dt = 1/20 = 0.05s
for each axis [vx, vy, wz]:
    a_target = (v_target - v_current) / dt
    a_limited = clamp(a_target, max_decel, max_accel)
    v_new = clamp(v_current + a_limited * dt, min_vel, max_vel)
```

**파라미터 (nav2_params.yaml):**

| 파라미터 | 값 | 물리적 의미 |
|----------|-----|------------|
| max_velocity | [3.0, 0.0, 2.0] | vx_max, vy_max, wz_max |
| min_velocity | [-1.5, 0.0, -2.0] | 후진, 역회전 |
| max_accel | [2.5, 0.0, 3.2] | 0→1m/s: 0.4초 |
| max_decel | [-2.5, 0.0, -3.2] | 1→0m/s: 0.4초 |
| feedback | OPEN_LOOP | odom 미사용 |

### 검증 상태

| 구간 | 상태 | 비고 |
|------|------|------|
| controller_server → cmd_vel_nav | ✅ 발행 확인 | MPPI 정상 동작 |
| velocity_smoother 알고리즘 | ✅ 분석 완료 | Rate Limiter, 파라미터 정상 |
| velocity_smoother lifecycle | ❌ DDS 문제 | activate 실패 |
| velocity_smoother → cmd_vel_smoothed | ❌ 미발행 | lifecycle 문제로 인해 |
| collision_monitor | ⏸️ disabled (임시) | FootprintApproach enabled: false |
| cmd_vel_relay (우회 노드) | ✅ 코드 정상 | 입력(cmd_vel_smoothed) 대기 |
| ros_gz_bridge → Gazebo | ✅ 설정 정상 | cmd_vel 미수신 상태 |

---

## 변경 내역

| 파일 | 변경 |
|------|------|
| `config/nav2_params.yaml` | bt_navigator 타임아웃 1000ms, collision_monitor 비활성화, voxel 높이 조정 |
| `config/bridge_config.yaml` | cmd_vel 브릿지 확인 |
| `config/dual_ekf.yaml` | navsat_transform GPS 프레임 설정 |
| `launch/simulation_launch.py` | cmd_vel_relay 노드 추가, 로그 레벨 조정 |
| `ad_bringup/cmd_vel_relay.py` | 신규 — collision_monitor 우회용 |
| `setup.py` | cmd_vel_relay entry point 추가 |

---

## 해결 후 검증 순서

```bash
# Step 1: DDS 복구
sudo rm -rf /dev/shm/fastrtps*
# 또는 CycloneDDS 전환 후

# Step 2: velocity_smoother 동작 확인
ros2 lifecycle get /velocity_smoother   # → active
ros2 topic hz /cmd_vel_nav              # → 20Hz
ros2 topic hz /cmd_vel_smoothed         # → 20Hz
ros2 topic hz /cmd_vel                  # → 20Hz

# Step 3: Gazebo 차량 이동 (수동 테스트)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" --once

# Step 4: Nav2 End-to-End
# Foxglove에서 waypoint → Nav2 goal → 차량 이동 확인
```

---

## C61 완료 조건

- [ ] CycloneDDS 전환 또는 SharedMemory 정리
- [ ] velocity_smoother lifecycle active 확인
- [ ] cmd_vel_nav → cmd_vel_smoothed → cmd_vel 전체 체인 확인
- [ ] Gazebo에서 차량 실제 이동 확인
- [ ] collision_monitor 복원 여부 결정
- [ ] cmd_vel_relay 제거 여부 결정

---

## 미결 이슈

- collision_monitor 복원 시 LiDAR 데이터 충분한지 확인 필요
- local_costmap "Sensor origin out of bounds" 경고 — voxel_layer 높이 설정 재확인
