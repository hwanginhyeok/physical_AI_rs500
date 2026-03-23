# C78: 분무 시스템 시뮬 — 펌프/팬/솔레노이드 5ch 제어

## 배경

SS500의 핵심 기능인 분무(Speed Sprayer) 시스템을 시뮬레이션에 기본 구현한다.
물리적 분무 입자 시뮬레이션은 향후로 미루고, 제어 인터페이스와 상태 발행을 우선 구현한다.

## HIH_2 출처

- **펌프**: QINGFENG 48V, DAC(PWM) 제어
- **팬**: Hobbywing M6215HP, XRotor H80A ESC, CAN FD 제어
- **솔레노이드**: 5채널, TLE8110 드라이버, 12V
- **교반기**: 12V 모터, TLE8110 드라이버
- **ADT 살포 모드** (Dm.h):
  - 양쪽 과수: 솔레노이드 1,4 ON
  - 좌측 무과수: 솔레노이드 1,2 ON
  - 우측 무과수: 솔레노이드 3,4 ON
  - 행 전환/U턴: 펌프+팬+교반기 OFF

## 변경 내역

### 1. 분무 제어 노드 (신규)

`src/ad_control/ad_control/spray_simulator_node.py`:
- 서비스: `/spray/set_mode` (off, left_only, right_only, both, custom)
- 서비스: `/spray/set_pump` (on/off, duty 0~100%)
- 서비스: `/spray/set_fan` (rpm 0~3000)
- 토픽 발행: `/spray/status` (펌프 상태, 팬 RPM, 솔레노이드 5ch, 탱크 잔량)
- 탱크 모델: 500L, 유량 기반 감소 (펌프 ON 시 ~10L/min)

### 2. ADT 살포 로직 연동

자율주행 중 작물 행 인식 결과에 따라 자동 살포 모드 전환:
- `/perception/crop_rows` 구독 → 좌/우 과수 존재 판단 → 솔레노이드 패턴 결정

## 수정/생성 파일

- `src/ad_control/ad_control/spray_simulator_node.py` (신규)
- `src/ad_control/setup.py` (entry_point 추가)

## 검증

- `/spray/set_mode both` → `/spray/status`에서 솔레노이드 1,4 ON 확인
- 펌프 ON 후 탱크 잔량 감소 확인 (Plot)
- C77 VCU 연동: U턴 시 자동 OFF 확인
