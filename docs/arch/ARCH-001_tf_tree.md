# ARCH-001: TF 트리 구조

- 상태: `채택됨`
- 결정일: 2026-03-01
- 관련 TASK: C50, C54, C55

## 배경

ROS2 Nav2 + Dual-EKF + Gazebo Harmonic 환경에서 TF(Transform) 트리는
여러 노드(robot_state_publisher, EKF, AMCL, static_transform_publisher)가
경쟁적으로 프레임 변환을 발행할 수 있어 구조를 명확히 고정해야 한다.
C50~C55 버그 디버깅 과정에서 TF 부모 충돌, map 프레임 미발행 등 6개 버그가
TF 설계 불명확에서 비롯됐음을 확인.

## 결정

**현재 채택된 TF 트리:**

```
/map (고정)
 └── /odom  ← static_transform_publisher (map_to_odom_static) [C50]
      └── /base_footprint  ← ekf_local (동적, 50Hz) [C54]
           └── /base_link  ← robot_state_publisher (URDF 정의)
                ├── /lidar_link  ← robot_state_publisher
                │    └── /ss500/lidar_link/gpu_lidar  ← lidar_frame_bridge [C55]
                └── /imu_link  ← robot_state_publisher
                     └── /ss500/imu_link/imu_sensor  ← imu_frame_bridge [C55]
```

**각 프레임의 발행 책임:**

| 변환 | 발행 노드 | 토픽 | 비고 |
|------|-----------|------|------|
| `map → odom` | `map_to_odom_static` | `/tf_static` | C50 임시 고정. C51 완료 후 ekf_global로 전환 예정 |
| `odom → base_footprint` | `ekf_local` | `/tf` | IMU + 휠 오도메트리 융합, 50Hz |
| `base_footprint → base_link` | `robot_state_publisher` | `/tf` | URDF 정의, 고정 변환 |
| `base_link → sensor frames` | `robot_state_publisher` | `/tf_static` | URDF 정의, 고정 변환 |
| `lidar_link → ss500/.../gpu_lidar` | `lidar_frame_bridge` | `/tf_static` | Gazebo scoped name 브릿지 |
| `imu_link → ss500/.../imu_sensor` | `imu_frame_bridge` | `/tf_static` | Gazebo scoped name 브릿지 |

## 근거

- **base_footprint 선택**: URDF 루트 링크가 `base_footprint`. EKF가 `base_link`를 직접 발행하면 robot_state_publisher의 `base_footprint → base_link` 변환과 TF2 부모 충돌 발생 (C54). URDF 구조 변경 없이 EKF가 `base_footprint`를 발행하는 것이 최소 변경.
- **map→odom 임시 고정**: AMCL은 빈 평지에서 특징점 부족으로 파티클 필터 초기화 실패 → map 프레임 미발행 → Nav2 전체 블로킹 (C50). GPS datum 고정(C51) 완료 전까지는 ekf_global도 불안정하므로 identity static TF가 가장 안전.
- **Gazebo scoped name 브릿지**: Gazebo Harmonic 8.x에서 `<frame_id>` 태그 재정의가 ros_gz_bridge에 무시되는 버그 존재 (C55). identity static TF로 TF 트리에 연결하는 것이 공식 권장 방법.

## 검토한 대안

| 대안 | 기각 이유 |
|------|-----------|
| AMCL이 map→odom 발행 | 빈 평지에서 LiDAR 특징점 부족 → 초기화 실패. 농경지 시뮬에서 반복 재현 |
| ekf_global이 map→odom 발행 (즉시) | GPS datum 미수렴 시 map→odom 오염. C51 완료 후 가능 |
| URDF에서 base_footprint 제거 | URDF 대규모 수정 필요. Nav2 표준 패턴(base_footprint 사용)에서 이탈 |
| SDF `<frame_id>` 태그로 scoped name 수정 | Gazebo Harmonic 8.x 버그로 ros_gz_bridge가 무시 |

## 결과 및 트레이드오프

- 긍정: Nav2 초기화 안정, TF 충돌 없음, 센서 데이터 정상 조회
- 부정/제약: map→odom이 고정(identity)이므로 글로벌 GPS 위치 추정 미동작 (C52 검증 후 전환 예정)

## 마이그레이션 계획 (C52 검증 완료 후)

```
현재:  map_to_odom_static (identity)
전환:  ekf_global.publish_tf: true  +  map_to_odom_static 노드 제거
조건:  GPS datum 안정 확인 (C51 완료), odom→base_footprint 동적 TF 정상 확인
```

## 변경 이력

| 날짜 | 변경 내용 |
|------|-----------|
| 2026-03-01 | C50~C55 버그 수정 후 최초 확정 |
