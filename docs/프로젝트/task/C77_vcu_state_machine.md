# C77: VCU 상태 머신 — 4모드 제어 + Safety 로직

## 배경

실차 VCU는 4개 제어 모드(RC/LCD/ADT/TEST)와 다층 안전 로직을 갖는다.
시뮬레이션에서 이를 재현하면 자율주행 소프트웨어의 모드 전환 및 안전 시나리오를 테스트할 수 있다.

## HIH_2 출처

- **상태 머신**: `/home/gint_pcd/projects/HIH_2/1)프로젝트/5)농업/ssvcu_branch_.../SSVCU/Asw/Dm.h`
- **VCU SRS**: `/home/gint_pcd/projects/HIH_2/HIH_Claude/산출물/SS500_VCU_SRS_V01_260226.md`
- **제어권 변경**: `/home/gint_pcd/projects/HIH_2/1)프로젝트/5)농업/1)제품설계/제어권 변경에 따른 SS기 동작 상태 확인.xlsx`

### 제어 모드

| 모드 | 설명 | 시뮬 동작 |
|------|------|----------|
| CTRL_RC | 리모컨 수동 | cmd_vel 외부 입력 허용 |
| CTRL_LCD | LCD 패널 수동 | cmd_vel 외부 입력 허용 |
| CTRL_ADT | 자율주행 | Nav2 cmd_vel만 허용 (기본) |
| CTRL_TEST | 테스트 | 모든 입력 허용 + 로깅 강화 |

### Safety 로직

| 조건 | 동작 | 시뮬 구현 |
|------|------|----------|
| SOC < 30% | 저전압 경고 | `/diagnostics` WARN + 속도 50% 제한 |
| SOC < 10% | 강제 정지 | `/diagnostics` ERROR + cmd_vel 차단 |
| 속도 > 3.0 km/h | 과속 제한 | cmd_vel 클램핑 |
| 범프 센서 감지 | 즉시 정지 | E-Stop 트리거 |
| 비상 정지 버튼 | 즉시 정지 + 릴레이 차단 | cmd_vel 제로 + 모터 비활성 |

## 변경 내역

### 1. VCU 시뮬레이터 노드 (신규)

`src/ad_control/ad_control/vcu_simulator_node.py`:
- 상태 머신: INIT → READY → {RC, LCD, ADT, TEST}
- 모드 전환: ROS 서비스 `/vcu/set_mode`
- Safety 모니터: SOC, 속도, 범프 감지 구독
- cmd_vel 게이트: 현재 모드에 따라 필터링/클램핑 후 발행
- 상태 발행: `/vcu/status` (JSON 또는 커스텀 메시지)

### 2. VCU 파라미터 (신규)

`src/ad_bringup/config/vcu_params.yaml`:
- `max_speed_kph: 3.0`
- `low_battery_warn_soc: 30`
- `low_battery_stop_soc: 10`
- `default_mode: ADT`

### 3. 런치 파일 통합

`simulation_launch.py`에 `vcu_simulator` 노드 추가 (선택적).

## 수정/생성 파일

- `src/ad_control/ad_control/vcu_simulator_node.py` (신규)
- `src/ad_bringup/config/vcu_params.yaml` (신규)
- `src/ad_bringup/launch/simulation_launch.py` (노드 추가)
- `src/ad_control/setup.py` (entry_point 추가)

## 검증

- `ros2 service call /vcu/set_mode ...` → 모드 전환 확인
- ADT 모드에서 Nav2 cmd_vel 정상 전달 확인
- SOC 30% 이하 → 속도 50% 제한 동작 확인
- E-Stop 서비스 호출 → 즉시 정지 확인
