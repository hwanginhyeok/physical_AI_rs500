# 시뮬레이션 사전 검증 스킬 (launch-preflight)

> **트리거**: 아래 표현이 나오면 이 스킬을 즉시 실행한다.
> - "런치 검증" / "시뮬 체크" / "프리플라이트"
> - "시뮬레이션 돌리기 전에" / "런치 전 확인"
> - Gazebo 시뮬레이션 실행 직전 자동 제안

---

## 목적

Gazebo 시뮬레이션 실행(30초+ 소요)에 앞서, YAML 문법 오류·파일 누락·TF 불일치 등을 사전에 감지하여 시간 낭비를 방지한다.

---

## 실행 순서 (순서 준수 필수)

### STEP 1 — YAML 문법 검증

다음 설정 파일의 YAML 파싱 가능 여부를 확인한다:

| 파일 | 검증 항목 |
|------|----------|
| `src/ad_bringup/config/nav2_params.yaml` | YAML 문법, 들여쓰기 |
| `src/ad_bringup/config/dual_ekf.yaml` | YAML 문법 |
| `src/ad_bringup/config/bridge_config.yaml` | YAML 문법 |
| `src/ad_bringup/config/lio_sam_params.yaml` | YAML 문법 |

```bash
python3 -c "import yaml; yaml.safe_load(open('파일경로'))" 2>&1
```

### STEP 2 — Nav2 파라미터 필수 키 확인

`nav2_params.yaml`에서 다음 필수 섹션/키가 존재하는지 확인:

- `bt_navigator` → `default_nav_to_pose_bt_xml`, `plugin_lib_names`
- `controller_server` → `FollowPath` 플러그인 설정
- `planner_server` → `GridBased` 플러그인 설정
- `local_costmap` → `resolution`, `plugins`
- `global_costmap` → `resolution`, `plugins`
- `velocity_smoother` → `max_velocity`, `max_accel` (C61 관련)

누락된 키가 있으면 FAIL + 어떤 키가 빠졌는지 명시.

### STEP 3 — EKF 파라미터 일관성 확인

`dual_ekf.yaml`에서:
- `ekf_filter_node_odom` / `ekf_filter_node_map` 두 필터 모두 정의되어 있는지
- `odom_frame`, `base_link_frame`, `world_frame` 이름이 TF 트리와 일치하는지
- 최소 하나의 센서 입력(`odomN`, `imuN`)이 설정되어 있는지

### STEP 4 — Bridge 토픽 매핑 확인

`bridge_config.yaml`에서:
- 각 토픽 매핑의 `ros_type_name`과 `gz_type_name`이 유효한 메시지 타입인지
- 방향(`direction`)이 올바른지 (`BIDIRECTIONAL`, `GZ_TO_ROS`, `ROS_TO_GZ`)
- `/cmd_vel`, `/odom`, `/scan`, `/imu` 필수 토픽이 포함되어 있는지

### STEP 5 — URDF/SDF 일관성 확인

- `src/ad_simulation/urdf/ss500.urdf.xacro` 파일 존재 확인
- `src/ad_simulation/urdf/ss500_sensors.urdf.xacro` 센서 frame_id 목록 추출
- frame_id가 ARCH-001(TF 트리)에 정의된 프레임과 일치하는지 확인
- SDF 모델 파일(`src/ad_simulation/models/` 하위) 존재 확인

### STEP 6 — Launch 파일 참조 확인

`src/ad_bringup/launch/simulation_launch.py`를 읽고:
- 참조하는 설정 파일(`config/*.yaml`)이 모두 존재하는지
- URDF xacro 파일 경로가 올바른지
- 선언된 launch argument와 기본값 확인

### STEP 7 — 결과 요약

체크 결과를 테이블로 출력한다:

```
🛫 런치 프리플라이트 — {날짜}

| # | 항목 | 결과 | 상세 |
|---|------|------|------|
| 1 | nav2_params.yaml YAML 문법 | ✅ PASS | |
| 2 | dual_ekf.yaml YAML 문법 | ✅ PASS | |
| 3 | bridge_config.yaml YAML 문법 | ✅ PASS | |
| 4 | Nav2 필수 파라미터 | ✅ PASS | |
| 5 | EKF 필터 설정 | ⚠️ WARN | odom_frame 불일치 |
| 6 | Bridge 토픽 매핑 | ✅ PASS | |
| 7 | URDF/SDF 일관성 | ✅ PASS | |
| 8 | Launch 파일 참조 | ✅ PASS | |

종합: ✅ 7 PASS / ⚠️ 1 WARN / ❌ 0 FAIL
→ 시뮬레이션 실행 가능 (경고 항목 확인 권장)
```

---

## 판단 규칙

| 결과 | 행동 |
|------|------|
| 전체 PASS | "시뮬레이션 실행 준비 완료" 알림 |
| WARN 존재 | "경고 항목 확인 후 실행 권장" + 상세 설명 |
| FAIL 존재 | "시뮬레이션 실행 불가" + 수정 가이드 제시 |
| FAIL이 YAML 문법 오류 | 오류 위치(행/열) 명시 + 수정 제안 |

---

## 주의사항

- 이 스킬은 읽기 전용이다. 파일을 수정하지 않는다
- YAML 검증은 `pyyaml`의 `safe_load`만 사용 (커스텀 태그 미지원 가능)
- xacro 파일은 텍스트 레벨 검증만 수행 (xacro 매크로 해석은 빌드 시점)
- 프리플라이트 통과가 시뮬레이션 성공을 보장하지 않음 (런타임 오류는 별도)
