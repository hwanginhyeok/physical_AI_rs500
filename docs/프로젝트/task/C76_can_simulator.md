# C76: CAN 시뮬레이터 — DBC 파서 + CAN 토픽 발행 노드

## 배경

실차 SS500은 CAN 버스(500kbps)로 모든 서브시스템이 통신한다.
시뮬레이션에서 실차 CAN 메시지와 동일한 ROS 토픽을 발행하면,
자율주행 소프트웨어를 sim/real 전환 없이 동일 인터페이스로 테스트할 수 있다.

## HIH_2 출처

- **DBC 파일**: `/home/gint_pcd/projects/HIH_2/1)프로젝트/5)농업/4)CAN통신/SSVCU_CANdb_250317_01.dbc`
- **CAN ICD**: `/home/gint_pcd/projects/HIH_2/HIH_Claude/산출물/SS500_CAN_ICD_V03_260223.md`
- **BMS ICD**: `/home/gint_pcd/projects/HIH_2/HIH_Claude/산출물/SS500_BMS_ICD_V03_260223.md`

### 핵심 메시지 (시뮬레이션 대상)

| ID | 이름 | 주기 | 시뮬 소스 | 설명 |
|----|------|------|----------|------|
| 0x611 | BPA_Calc | 10ms | 배터리 모델 | 전압, 전류, SOC, SOH |
| 0x612 | BPA_State | 10ms | 배터리 모델 | 릴레이/시스템 상태 |
| 0x614 | BPA_POWER | 100ms | 배터리 모델 | 충/방전 파워 한계 |
| 0x331 | VCU2ADT1 | — | Gazebo 상태 | 차량 속도, 장애, 범프 |
| 0x341 | SNS2ADT1 | — | Gazebo IMU | 가속도 x,y,z |
| 0x342 | SNS2ADT2 | — | Gazebo IMU | 각속도 x,y,z |
| 0x343 | SNS2ADT3 | — | Gazebo IMU | 자세 Roll/Pitch/Yaw |
| 0x344 | SNS2ADT4 | — | Gazebo GPS | 위도, 경도 |
| 0x345 | SNS2ADT5 | — | Gazebo GPS | 속도, 방위각 |
| 0x346 | SNS2ADT6 | — | Gazebo GPS | Fix 상태, 위성 수 |

## 변경 내역

### 1. DBC 파서 (신규)

`src/ad_core/can/dbc_parser.py`:
- DBC 파일 파싱 → 메시지/시그널 Python 구조체
- 인코딩/디코딩 유틸리티

### 2. CAN 메시지 타입 정의 (신규)

`src/ad_core/can/ss500_messages.py`:
- BPA_Calc, BPA_State, VCU2ADT1, SNS2ADT1~6 dataclass
- DBC 기반 factor/offset/range 반영

### 3. CAN 시뮬레이터 노드 (신규)

`src/ad_bringup/ad_bringup/can_simulator_node.py`:
- Gazebo 토픽 구독 (/odometry/local, /sensor/imu, /sensor/gps)
- CAN 형식 ROS 토픽 발행 (/can/bpa_calc, /can/vcu2adt1, /can/sns2adt1~6)
- 배터리 SOC 시뮬레이션 (시간 경과에 따른 감소)
- 실차 `ad_can_bridge`와 동일 토픽명 사용

### 4. 배터리 모델 (기본)

`src/ad_core/can/battery_model.py`:
- 48V 460Ah LiFePO4 모델
- SOC: 시간 + 전류 기반 쿨롱 카운팅
- 전압: SOC-OCV 테이블 (선형 근사)

## 수정/생성 파일

- `src/ad_core/can/dbc_parser.py` (신규)
- `src/ad_core/can/ss500_messages.py` (신규)
- `src/ad_core/can/battery_model.py` (신규)
- `src/ad_bringup/ad_bringup/can_simulator_node.py` (신규)
- `src/ad_bringup/setup.py` (entry_point 추가)

## 검증

- `ros2 topic echo /can/bpa_calc` → SOC/전압/전류 값 확인
- SOC가 시간 경과에 따라 감소하는지 확인
- Foxglove에서 CAN 메시지 모니터링 (Plot 패널)
- 실차 `ad_can_bridge` 연결 시 동일 토픽명으로 심리스 전환 확인
