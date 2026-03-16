# TF 트리 검증 스킬 (tf-check)

> **트리거**: 아래 표현이 나오면 이 스킬을 즉시 실행한다.
> - "TF 확인" / "TF 트리 검증" / "TF 체크"
> - "프레임 확인" / "좌표계 확인"
> - URDF/xacro/SDF 파일 변경 시 자동 제안

---

## 목적

URDF xacro 수정 후 TF 트리가 깨지면 EKF/Nav2 전체가 오작동한다 (C54에서 경험). 변경 시마다 TF 트리를 검증하여 사전에 문제를 감지한다.

참조: `docs/arch/ARCH-001_tf_tree.md`

---

## URDF 파일 구조

| 파일 | 역할 |
|------|------|
| `src/ad_simulation/urdf/ss500.urdf.xacro` | 메인 URDF (다른 xacro include) |
| `src/ad_simulation/urdf/ss500_base.urdf.xacro` | 차체 + 바퀴 |
| `src/ad_simulation/urdf/ss500_sensors.urdf.xacro` | 센서 마운트 |
| `src/ad_simulation/urdf/ss500_gazebo.urdf.xacro` | Gazebo 플러그인 |

---

## 실행 순서 (순서 준수 필수)

### STEP 1 — URDF 변경 감지

```bash
git diff --name-only HEAD | grep -E "\.(xacro|urdf|sdf)$"
```

변경된 파일이 없으면 → "URDF 변경 없음" 알림. 그래도 검증하고 싶으면 계속.

### STEP 2 — 정적 분석 (xacro 텍스트 레벨)

xacro 파일에서 프레임 정의를 추출한다:

```bash
# link 이름 추출
grep -h '<link name=' src/ad_simulation/urdf/*.xacro | sort

# joint의 parent-child 관계 추출
grep -hA2 '<joint name=' src/ad_simulation/urdf/*.xacro
```

추출된 관계를 테이블로 정리:

```
📐 TF 트리 (정적 분석)

| Joint | Parent | Child | Type |
|-------|--------|-------|------|
| base_footprint_to_base | base_footprint | base_link | fixed |
| ... | ... | ... | ... |
```

### STEP 3 — ARCH-001 대조

`docs/arch/ARCH-001_tf_tree.md`를 읽고, 정적 분석 결과와 비교:

- **추가된 프레임**: ARCH-001에 없는 새 link/joint → 문서 업데이트 필요
- **제거된 프레임**: ARCH-001에 있지만 xacro에서 삭제된 것 → 문서 업데이트 필요
- **부모-자식 변경**: 동일 프레임인데 부모가 바뀐 것 → **주의 필요** (C54 재현 가능)

### STEP 4 — 런타임 TF 검증 (ROS2 실행 중일 때만)

ROS2가 실행 중이면 런타임 TF 트리를 확인:

```bash
# TF 트리 PDF 생성
ros2 run tf2_tools view_frames

# 특정 프레임 간 TF 확인
ros2 run tf2_ros tf2_echo base_link laser_frame

# TF 발행자 확인
ros2 topic echo /tf_static --once
```

`view_frames`가 생성한 `frames.pdf`를 분석.

ROS2가 실행 중이 아니면 → "런타임 검증 건너뜀 (정적 분석만 수행)" 알림.

### STEP 5 — 중복 발행자 확인

동일 TF를 여러 노드가 발행하면 충돌 발생:

```bash
ros2 topic echo /tf --once | grep "child_frame_id"
ros2 topic echo /tf_static --once | grep "child_frame_id"
```

동일 `child_frame_id`가 2회 이상 나오면 → **중복 발행자** 경고.

### STEP 6 — 결과 요약

```
📐 TF 트리 검증 — {날짜}

| # | 검증 항목 | 결과 | 상세 |
|---|----------|------|------|
| 1 | xacro 파싱 | ✅ PASS | link 12개, joint 11개 |
| 2 | ARCH-001 대조 | ⚠️ WARN | laser_frame 추가됨 (문서 미반영) |
| 3 | 부모-자식 일관성 | ✅ PASS | |
| 4 | 중복 발행자 | ✅ PASS | |
| 5 | 런타임 TF | ⏭️ SKIP | ROS2 미실행 |

조치 필요:
- ARCH-001에 laser_frame 추가 (parent: base_link, type: fixed)
```

### STEP 7 — ARCH-001 업데이트 판단

프레임 추가/제거/변경이 있으면:
- `docs/arch/ARCH-001_tf_tree.md` 업데이트
- 변경 사유와 영향 범위 기록

---

## 알려진 TF 구조 (ARCH-001 기준)

주요 프레임 체인:
```
map → odom → base_footprint → base_link
                                  ├── {wheel_frames}
                                  ├── imu_link
                                  ├── gps_link
                                  └── {sensor_frames}
```

- `map → odom`: EKF (ekf_filter_node_map) 발행
- `odom → base_footprint`: EKF (ekf_filter_node_odom) 발행
- `base_footprint → base_link`: robot_state_publisher (URDF에서 fixed)
- 나머지: robot_state_publisher (URDF에서 정의)

---

## 판단 규칙

| 상황 | 행동 |
|------|------|
| 프레임 추가 | ARCH-001 업데이트 + 관련 노드 설정 확인 |
| 프레임 부모 변경 | **고위험** — EKF/Nav2 영향 분석 필수 |
| 중복 TF 발행 | 발행 노드 식별 → 하나만 남기도록 수정 |
| ARCH-001과 불일치 | 문서 vs 코드 중 어느 쪽이 맞는지 확인 후 동기화 |
| 런타임 TF 끊김 | 해당 노드 상태 확인 → launch 파일 점검 |

---

## 주의사항

- `view_frames`는 5초간 TF 메시지를 수집하므로 모든 노드가 떠 있어야 정확
- xacro 파일에서 `xacro:if` 조건부 프레임은 정적 분석에서 놓칠 수 있음
- Gazebo가 자체 TF를 발행하는 경우 `bridge_config.yaml`에서 remap 확인
- URDF 변경 후 반드시 `ad_simulation` 패키지 빌드 필요 → `build-smart` 스킬 연계
