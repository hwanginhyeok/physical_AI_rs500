# C29: LIO-SAM SLAM 통합

- 분야: 알고리즘/인프라
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

GPS 신호 불량/단절 시 LiDAR-IMU 기반 SLAM(LIO-SAM)으로 자동 전환하는 위치 추정 시스템.

## 변경 사항

### 주요 파일

- `src/ad_perception/ad_perception/localization_node.py` — 802줄
- `src/ad_bringup/config/lio_sam_params.yaml` — LIO-SAM 파라미터

### 구현 방식

LIO-SAM은 **외부 프로세스**로 실행. localization_node가 `/odometry/slam` 토픽을 구독하여 모드 전환을 관리.

**GPS 모드 전환 로직** (localization_node.py):
- `GPS_AVAILABLE` → Dual-EKF (C19) 사용
- `GPS_DEGRADED` → GPS + SLAM 융합
- `GPS_UNAVAILABLE` → `SLAM_ONLY` 모드: LIO-SAM 단독 위치 추정 (420행)

### 설계 결정

- LIO-SAM을 직접 구현하지 않고 외부 패키지 활용
- localization_node가 GPS 품질 모니터링 + 모드 전환 관리자 역할
- 모드 전환 시 위치 점프 방지를 위한 스무딩 로직 포함

## 비고

- LIO-SAM 파라미터(`lio_sam_params.yaml`)의 상세 내용은 미확인
- 실제 LIO-SAM 패키지 설치 및 실행 테스트는 미수행 (WSL2 환경 제약)
