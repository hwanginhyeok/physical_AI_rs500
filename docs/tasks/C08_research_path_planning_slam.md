# C8: 선행연구 — 경로계획/SLAM

- 분야: 연구
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

자율주행 경로계획 알고리즘과 SLAM 기법의 문헌 조사.

## 산출물

`docs/research/path_planning_and_slam_literature_review.md` — 792줄

### 주요 내용
- 전역 경로 계획: A*, Hybrid A*, RRT*, D* Lite, State Lattice
- 지역 경로 계획: DWA, TEB, MPPI, MPC
- 농경지 특화: Boustrophedon, Furrow Following, Fields2Cover
- SLAM: LiDAR 기반 (FAST-LIO2, LIO-SAM), Visual (ORB-SLAM3, VINS-Fusion)
- Nav2 프레임워크 구조
- GPS-RTK/IMU 센서 퓨전

## 활용

C20(Nav2 SmacPlannerLattice), C21(MPPI), C28(CoveragePlanner), C29(LIO-SAM) 구현에 참조됨.
