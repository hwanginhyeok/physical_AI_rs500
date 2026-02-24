# C36: SS500 CAN 코덱 + CAN 브릿지

- 분야: 인프라
- 담당: Claude
- 기간: 2026-02-23
- 상태: 완료

## 배경 및 목적

SS500 차량의 CAN ICD에 맞는 메시지 인코딩/디코딩 코덱과 ROS2 CAN 브릿지 노드.

## 변경 사항

### 주요 파일

**`src/ad_can_bridge/ad_can_bridge/ss500_codec.py`** — 234줄, 5개 클래스:
- CAN2 버스 기반 통신
- 0x301: ADT→VCU 명령 메시지
- 0x331, 0x332: VCU→ADT 피드백 메시지
- Little Endian, S16 factor=0.01

**`src/ad_can_bridge/ad_can_bridge/can_bridge_node.py`** — 157줄:
- CAN 소켓 ↔ ROS2 토픽 브릿지
- `VehicleInterface` ABC(ad_core) 구현

### CAN ICD 사양

| CAN ID | 방향 | 내용 |
|--------|------|------|
| 0x301 | ADT→VCU | 차량 제어 명령 |
| 0x331 | VCU→ADT | 피드백 1 |
| 0x332 | VCU→ADT | 피드백 2 |

- 인코딩: Little Endian
- 데이터 타입: Signed 16-bit
- 스케일링 팩터: 0.01

### 설계 결정

- 코덱(ss500_codec)은 can_bridge 패키지 내에 위치 (ad_core가 아님 — 차량 특화)
- `VehicleInterface` ABC는 ad_core에 정의 (다른 차량에도 적용 가능)
- python-can 라이브러리 기반

## 테스트

- `src/ad_can_bridge/test/test_ss500_codec.py` — 236줄, 24개 테스트
- 인코딩/디코딩 왕복 검증, 경계값 테스트
