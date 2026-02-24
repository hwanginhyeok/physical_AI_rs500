# C31: Camera + LiDAR Late Fusion

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

카메라 객체 탐지(C23)와 LiDAR 장애물 감지(C22)를 후기 융합하여 3D 객체 위치를 추정하는 모듈.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/sensor_fusion.py` — 540줄, 4개 클래스

### 핵심 구조

- 카메라 Detection + LiDAR Obstacle → 융합된 3D 객체
- 시간 동기화 (타임스탬프 기반)
- 매칭 알고리즘 (IoU 또는 거리 기반)
- 신뢰도 융합

### 설계 결정

- Late Fusion 방식: 각 센서의 독립 결과를 후처리 단계에서 융합
- numpy만 의존 (ad_core 원칙)
- ROS2 메시지 변환은 perception_node에서 처리

## 테스트

- `src/ad_core/test/test_sensor_fusion.py` — 220줄, 20개 테스트
