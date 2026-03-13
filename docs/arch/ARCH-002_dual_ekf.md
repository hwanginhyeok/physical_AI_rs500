# ARCH-002: Dual-EKF 센서 퓨전 아키텍처

- 상태: `채택됨`
- 결정일: 2026-02-21 (C19), 2026-03-01 업데이트 (C51, C54)
- 관련 TASK: C19, C51, C54

## 배경

SS500 궤도차량의 위치 추정을 위해 IMU, 휠 오도메트리, GPS 세 개의 센서를 융합해야 한다.
각 센서는 특성이 다르다:
- 휠 오도메트리: 고주파(50Hz), 슬립 시 드리프트
- IMU: 고주파(100Hz+), 적분 오차 누적
- GPS: 저주파(10Hz), 절대 위치이나 점프·노이즈 있음

단일 EKF로 모두 합치면 고주파 센서와 저주파 GPS가 섞여 추정 품질 저하.

## 결정

**Dual-EKF 구성**: `robot_localization` 패키지의 EKF 인스턴스 두 개를 분리 운영.

```
센서 입력                  EKF 인스턴스              출력
──────────────────────────────────────────────────────────────
/sensor/imu   ──┐
/odom          ─┤──▶  ekf_local (50Hz)  ──▶  /odometry/local
                │        odom 프레임            odom→base_footprint TF
                │
/sensor/gps  ──▶  navsat_transform  ──▶  /odometry/gps
                │        (GPS→UTM 변환)
/odometry/local─┤
/odometry/gps  ─┴──▶  ekf_global (10Hz) ──▶  /odometry/global
                         map 프레임             (map→odom TF: 미래 전환)
```

**핵심 파라미터 결정:**

| 파라미터 | 값 | 이유 |
|----------|-----|------|
| `ekf_local.world_frame` | `odom` | 로컬 EKF는 odom 기준 단기 추정 |
| `ekf_global.world_frame` | `map` | 글로벌 EKF는 map 기준 절대 위치 |
| `ekf_local.publish_tf` | `true` | odom→base_footprint 발행 |
| `ekf_global.publish_tf` | `false` | 현재 map→odom은 static TF 담당 (ARCH-001) |
| `base_link_frame` | `base_footprint` | URDF 루트 링크와 일치 (C54) |
| `navsat.wait_for_datum` | `true` | datum 하드코딩 (C51) |
| `navsat.datum` | `[37.5665, 126.978, 0.0]` | flat_field.sdf 원점과 일치 |

## 근거

- **Dual-EKF 패턴**: `robot_localization` 공식 권장 패턴. 고주파 센서를 로컬 EKF에서 처리하고, GPS(저주파, 절대)는 글로벌 EKF에서만 처리하여 주파수 불일치 문제 회피.
- **navsat datum 하드코딩**: `wait_for_datum: false` 시 Gazebo NavSat 플러그인 초기화 중 불안정한 값이 datum을 수십km 점핑시킴 (C51). 월드 파일 좌표와 동기화하면 GPS→UTM 변환이 항상 일관됨.
- **base_footprint**: URDF 루트 링크. robot_state_publisher가 `base_footprint→base_link` TF를 발행하므로 EKF가 `base_link`를 직접 목표로 하면 TF 부모 충돌 (C54).

## 검토한 대안

| 대안 | 기각 이유 |
|------|-----------|
| 단일 EKF (IMU+Odom+GPS 통합) | GPS 10Hz와 IMU 100Hz 혼합 시 공분산 관리 복잡. 업데이트 스텝 지연으로 고주파 추정 품질 저하 |
| GPS 없이 IMU+Odom만 | 장거리 이동 시 누적 오차 보정 불가. 농경지 반복 경로에서 드리프트 |
| LIO-SAM SLAM으로 전체 대체 | WSL2 + Gazebo 환경에서 실시간 처리 부담. 실내가 아닌 농경지에서 LiDAR SLAM 성능 미검증 |

## 결과 및 트레이드오프

- 긍정: 고주파 로컬 추정 안정, GPS 점프가 글로벌 EKF에만 영향
- 부정/제약: 두 EKF 간 타임스탬프 동기화 주의 필요. ekf_global이 odom 토픽을 구독하므로 ekf_local 지연 시 연쇄 지연

## 변경 이력

| 날짜 | 변경 내용 |
|------|-----------|
| 2026-02-21 | C19에서 최초 구현 |
| 2026-03-01 | C51: datum 하드코딩 추가. C54: base_link_frame → base_footprint 수정 |
