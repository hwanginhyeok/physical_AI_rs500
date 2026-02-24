# C9: 선행연구 — 인지/센서융합

- 분야: 연구
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

LiDAR/카메라 인지 파이프라인 및 멀티센서 융합 기법 문헌 조사.

## 산출물

`docs/research/perception_sensor_fusion_survey.md` — 700줄

### 주요 내용
- LiDAR: 지면 분리 (Patchwork++, Ray Filter), 클러스터링 (DBSCAN), 3D 탐지 (PointPillars)
- 카메라: 객체 탐지 (YOLOv8/11), 세그멘테이션 (ENet, BiSeNetV2)
- 센서 융합: Early/Mid/Late Fusion 전략 비교
- 농경지 특화: 작물 행 인식, 경계 탐지, Traversability 추정
- ROS2 구현: Autoware 아키텍처, 커스텀 노드 설계

## 활용

C22(LiDAR 장애물 감지), C23(YOLO), C30(세그멘테이션), C31(Late Fusion) 구현에 참조됨.
