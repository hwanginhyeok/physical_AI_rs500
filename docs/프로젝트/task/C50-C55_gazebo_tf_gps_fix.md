# C50-C55: Gazebo 웨이포인트 네비게이션 버그 6종 연쇄 수정

- 분야: 시뮬레이션
- 담당: 그린
- 기간: 2026-03-01 ~ 2026-03-01
- 상태: 완료 (C52 end-to-end 검증 진행 중)

## 배경

Gazebo Harmonic + Nav2 + Dual-EKF 환경에서 웨이포인트 네비게이션을 반복 실행했지만 결과가 개선되지 않음.
시뮬레이션 로그(`/tmp/ad_sim_v2.log`) 분석 및 TF 트리 점검을 통해 6개의 연쇄 버그를 발견.
모든 버그가 Nav2 초기화를 완전 블로킹하는 P1 수준이었음.

좀비 프로세스 2개(PID 49278, 62453 — waypoint_manager)도 발견하여 kill.

---

## 버그 분석 및 결정 사항

### 버그 1 (C50): map 프레임 미발행 → Nav2 전체 블로킹

**증상**: `[planner_server] Timed out waiting for transform from base_link to map` 반복

**근본 원인**: `ekf_global.publish_tf: false` + `amcl.tf_broadcast: true` 구성에서 AMCL이 map→odom을 담당하도록 설계됨. 그러나 `flat_field.sdf`는 특징점 없는 빈 평지라 LiDAR 반사체가 부족 → AMCL 파티클 필터 초기화 실패 → map 프레임 미발행 → Nav2 전체 프레임 트리 불완전.

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| static_transform_publisher로 map→odom identity 고정 | AMCL은 시뮬레이션 목적(웨이포인트 추종 검증)에 불필요. 로봇이 map 원점에서 출발하므로 identity 변환으로 충분. 빠른 unblock이 목적 | ekf_global.publish_tf: true 활성화 → GPS datum 미수렴 시 map→odom이 오염됨. C51(datum 고정) 완료 전에는 불안정하므로 기각 |
| amcl.tf_broadcast: false | AMCL이 실패해도 TF 발행 시도 자체가 없어야 static publisher와 충돌 없음 | AMCL 완전 제거 → Nav2 bringup_launch.py가 AMCL lifecycle을 관리하므로 파라미터 비활성화가 더 안전 |

**미래 전환 계획**: C51 GPS datum 고정 완료 후 `ekf_global.publish_tf: true` + `map_to_odom_static` 노드 제거 → 실제 GPS 기반 글로벌 로컬라이제이션으로 전환.

---

### 버그 2 (C51): GPS datum 0.2초마다 교체 → EKF 글로벌 수렴 불가

**증상**: `[navsat_transform] Datum is (37.51, 127.47)` → `(37.56, 126.29)` 0.2초 간격으로 교체

**근본 원인**: `wait_for_datum: false` 설정 시 navsat_transform이 **첫 GPS 메시지 수신 즉시** datum을 설정. Gazebo NavSat 플러그인이 초기화 과정에서 불안정한 값을 여러 번 발행하여 datum이 수십km씩 점핑. datum이 바뀌면 좌표 원점이 재설정되어 ekf_global이 발산.

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| datum: [37.5665, 126.978, 0.0] 하드코딩 | flat_field.sdf `<spherical_coordinates>`와 동일한 값 → GPS→UTM 변환이 Gazebo 월드 원점과 정확히 일치 | Gazebo NavSat 플러그인 초기화 딜레이 추가 → 근본 원인 해결 아님. GPS 수렴 대기 타임아웃 추가 → 임시방편 |

**참조**: `flat_field.sdf` 35-41번 줄
```xml
<spherical_coordinates>
  <latitude_deg>37.5665</latitude_deg>
  <longitude_deg>126.978</longitude_deg>
  <elevation>0</elevation>
</spherical_coordinates>
```

---

### 버그 3 (C52 → 검증 중): LiDAR 타임스탬프 TF 캐시 불일치

**증상**: sim_time 환경에서 LiDAR 메시지 타임스탬프가 TF 동적 캐시 범위(기본 10초) 벗어남

**현황**: C50+C51 적용 후 end-to-end 재실행으로 재현 여부 확인 중. static TF 변환을 추가했으므로 sensor frame lookup 문제는 C55(아래)에서 해소된 것으로 판단.

---

### 버그 4 (C53): model.sdf 센서 noise SDF 포맷 오류 → 로봇 스폰 실패

**증상**: `Unable to read file` 오류 → 로봇 미스폰 (spawn_robot 노드 즉시 종료)

**근본 원인**: SDF 1.9 표준에서 noise type은 attribute로 표기해야 함.
```xml
<!-- 올바른 SDF 1.9 포맷 -->
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.001</stddev>
</noise>

<!-- 기존 잘못된 포맷 (SDF 1.4 child element 방식) -->
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.001</stddev>
</noise>
```
GPS sensor 파싱 오류가 cascade되어 전체 모델 로딩 실패.

**수정 범위**: `src/ad_bringup/models/ss500/model.sdf`
- IMU noise: 6개 (linear acceleration 3축 + angular velocity 3축)
- LiDAR noise: 1개
- Camera noise: 1개
- GPS noise: 2개 (horizontal + vertical position)
- **합계: 10개 noise 요소 포맷 수정**

---

### 버그 5 (C54): EKF ↔ robot_state_publisher TF 부모 충돌

**증상**: `[ekf_local] Skipping measurement from /odom` 반복, `odom` 프레임이 TF 트리에 없음

**근본 원인**: URDF에서 robot_state_publisher가 `base_footprint → base_link` 변환을 발행. ekf_local은 `base_link_frame: base_link` 설정으로 `odom → base_link`를 발행하려 시도. TF2는 한 노드(프레임)의 부모가 두 곳에서 서로 다르게 선언되면 충돌로 판단하여 ekf_local의 변환을 거부 → odom 프레임 자체가 TF 트리에 미등록.

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| dual_ekf.yaml `base_link_frame: base_footprint` | URDF 루트 링크가 base_footprint이므로 EKF가 `odom → base_footprint`를 발행하는 것이 TF 트리와 정합 | URDF base_footprint 제거 → URDF 구조 대규모 변경 필요, 기각 |

**검증**: `ros2 run tf2_tools view_frames` → `odom → base_footprint → base_link → ...` 체인 확인

---

### 버그 6 (C55): Gazebo scoped sensor frame_id → URDF TF 불일치

**증상**: `[amcl] Message Filter dropping message: frame 'ss500/lidar_link/gpu_lidar'`

**근본 원인**: Gazebo Harmonic의 센서 메시지 frame_id는 **scoped name** 형태로 발행됨.
- LiDAR: `ss500/lidar_link/gpu_lidar`
- IMU: `ss500/imu_link/imu_sensor`

URDF TF 트리에는 `lidar_link`, `imu_link`만 존재 → TF2가 scoped frame을 찾지 못해 모든 센서 메시지 드롭.

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| static_transform_publisher로 identity bridge 추가 (`lidar_link → ss500/lidar_link/gpu_lidar`) | scoped frame을 URDF TF 트리에 연결하는 표준 방법. zero transform이므로 측정 정확도 영향 없음 | SDF의 `<frame_id>` 태그 수정 → ros_gz_bridge가 이 설정을 무시하는 Gazebo Harmonic 8.x 버그 있음 |

---

## 변경 내역

### 수정 파일

| 파일 | 변경 내용 | 해당 버그 |
|------|-----------|-----------|
| `src/ad_bringup/launch/simulation_launch.py` | `map_to_odom_static` 노드 추가 (map→odom identity) | C50 |
| `src/ad_bringup/launch/simulation_launch.py` | `lidar_frame_bridge`, `imu_frame_bridge` identity TF 노드 추가 | C55 |
| `src/ad_bringup/config/nav2_params.yaml:21` | `amcl.tf_broadcast: true → false` | C50 |
| `src/ad_bringup/config/dual_ekf.yaml:106` | `ekf_global.publish_tf` 주석 업데이트 (C51 연계 계획 명시) | C50 |
| `src/ad_bringup/config/dual_ekf.yaml:20,102` | `base_link_frame: base_link → base_footprint` (ekf_local, ekf_global 모두) | C54 |
| `src/ad_bringup/config/dual_ekf.yaml:183-188` | `wait_for_datum: true`, `datum: [37.5665, 126.978, 0.0]` 추가 | C51 |
| `src/ad_bringup/models/ss500/model.sdf` | noise 포맷 10개 수정 (`<type>` child → `type=` attribute) | C53 |

---

## 검증

- 빌드: `colcon build --packages-select ad_bringup` 성공 (1.13s)
- TF 확인: `ros2 run tf2_tools view_frames` → `odom → base_footprint` 동적 TF 발행 확인
- GPS datum: `/rosout` 로그에서 datum 교체 메시지 미발생 확인 예정
- end-to-end: C52 태스크에서 Nav2 lifecycle 활성화 + 웨이포인트 네비게이션 전체 검증 예정

---

## 미결 이슈

- [x] C52: `map → odom` TF 발행 확인 — static publisher 정상 동작 확인 (2026-03-02)
- [x] C52: Nav2 lifecycle 노드들이 모두 `active` 상태로 전환되는지 — active 확인 ✅ (2026-03-02)
- [ ] C52(C58): `bt_navigator` → `controller_server` follow_path 타임아웃 원인 파악. goal(5,0) 수신은 됐으나 action server acknowledge 실패. `/odom`·`/sensor/imu` 데이터 흐름 확인 필요
- [ ] C52: 웨이포인트 네비게이션 end-to-end 완전 동작 확인

**이번 세션(2026-03-02) 추가 수정 사항:**
- `simulation_launch.py`: Gazebo GUI 제거 (RTF 0.006 → 정상화), `gps_frame_bridge` 노드 추가
- `nav2_params.yaml`: global_costmap rolling_window 100m 전환, static_layer 제거, voxel_layer z_voxels 20

- [ ] 장기: C51 GPS datum 고정 → `ekf_global.publish_tf: true`로 전환 + `map_to_odom_static` 노드 제거 (실제 GPS 기반 글로벌 로컬라이제이션으로 업그레이드)
- [ ] 장기: AMCL 완전 제거 고려 (빈 평지 시뮬레이션에서는 완전 불필요, Nav2 리소스 절감 가능)
