# C58: bt_navigator `follow_path` 타임아웃 해결

**작업 ID:** C58  
**분야:** 시뮬레이션  
**중요도:** P1 (긴급)  
**담당:** 그린  
**상태:** ✅ 완료 (2026-03-04)  
**관련 작업:** C52 (Gazebo 시뮬레이션 통합 검증)

---

## 문제 증상

```
[bt_navigator]: Timed out while waiting for action server to acknowledge 
                goal request for follow_path
```

- Waypoint goal(5,0) 수신 ✅
- Nav2 lifecycle active ✅
- bt_navigator → controller_server goal 전송 ❌ (타임아웃)

---

## 원인 분석

### 시뮬레이션 환경에서의 타이밍 문제

| 파라미터 | 원래 값 | 문제 | 새 값 |
|----------|---------|------|-------|
| `default_server_timeout` | 20ms | controller_server 초기화에 시간 부족 | 1000ms |
| `wait_for_service_timeout` | 1000ms | 서비스 대기 시간 부족 | 5000ms |
| `bt_loop_duration` | 10ms | BT 루프 주기 너무 짧음 | 100ms |

### 왜 20ms로는 안 되는가?

1. **controller_server 초기화 순서:**
   - costmap 초기화 (local + global)
   - MPPI controller 초기화 (1000개 샘플 경로)
   - action server 등록
   - lifecycle active 전환

2. **시뮬레이션 랙:**
   - Gazebo RTF(Real Time Factor)가 1.0 미만일 때 시간 지연
   - Dual-EKF 초기화 지연

3. **Nav2 Jazzy 특성:**
   - 첫 goal 전송 시 plugin 동적 로딩
   - behavior tree XML 파싱

---

## 해결책

### 수정 파일: `src/ad_bringup/config/nav2_params.yaml`

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/local
    
    # === C58 수정 ===
    bt_loop_duration: 100            # 10 → 100 ms
    default_server_timeout: 1000     # 20 → 1000 ms  
    wait_for_service_timeout: 5000   # 1000 → 5000 ms
    # ================
    
    action_server_result_timeout: 900.0
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    default_nav_to_pose_bt_xml: ""
    default_nav_through_poses_bt_xml: ""
```

---

## 검증 방법

### 1. 빌드
```bash
cd ~/projects/자율주행프로젝트_work
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ad_bringup
source install/setup.bash
```

### 2. 시뮬레이션 실행
```bash
ros2 launch ad_bringup simulation_launch.py
```

### 3. 상태 확인 (15초 후)
```bash
# Action server 확인
ros2 action info /follow_path

# Lifecycle 상태
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator

# TF 체인
ros2 run tf2_tools view_frames
```

### 4. Goal 전송 테스트
```bash
# 수동 goal 전송
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**기대 출력:**
```
[bt_navigator]: Goal accepted by follow_path action server
[controller_server]: Received goal, starting MPPI controller
[bt_navigator]: Navigation succeeded
```

---

## 사이드 이펙트

| 항목 | 영향 | 설명 |
|------|------|------|
| 반응 속도 | 미미함 | BT 루프 100ms는 여전히 빠른 응답 (10Hz) |
| 초기화 시간 | 개선됨 | Nav2 노드들이 충분히 초기화될 시간 확보 |
| 시뮬레이션 안정성 | 향상됨 | 타임아웃으로 인한 재시도 감소 |

---

## 관련 이슈

- **C52:** Gazebo 시뮬레이션 통합 검증의 하위 이슈
- **C50-C55:** TF 및 localization 관련 이슈 (선결조건)

---

## 변경 이력

| 날짜 | 변경 내용 | 작성자 |
|------|----------|--------|
| 2026-03-04 | 초기 작성: 타임아웃 원인 분석 및 해결 | 그린 |
| 2026-03-04 | nav2_params.yaml 수정 적용 | 그린 |
