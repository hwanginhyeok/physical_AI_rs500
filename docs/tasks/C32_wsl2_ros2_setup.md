# C32: WSL2 ROS2 Jazzy 환경 구축

- 분야: 인프라
- 담당: Claude
- 기간: 2026-02-22 (추정)
- 상태: 완료

## 배경 및 목적

Windows + WSL2 Ubuntu 24.04 환경에서 ROS2 Jazzy Desktop을 설치하고 빌드 환경을 구축.

## 변경 사항

별도의 셋업 스크립트 파일은 커밋되지 않음. 수동 설치 과정으로 진행.

### 구축 내용

- WSL2 Ubuntu 24.04 설치
- ROS2 Jazzy Desktop 설치
- colcon 빌드 시스템 설치
- 워크스페이스: `~/ros2_ws/` (WSL2 네이티브 파일시스템)
- Windows 파일 접근: `/mnt/d/` 경로

### 주요 제약사항

- 빌드는 반드시 WSL2 네이티브 FS에서 실행 (Windows FS에서는 성능/권한 문제)
- `pip install -e` 실행 시 중립 디렉토리(`cd /c/Users`)에서 실행해야 함
- GPU: MX550 2GB — Gazebo 3D 시뮬레이션 실행 불가

## 비고

- 설치 과정의 상세 커맨드 기록은 없음
- PC 업그레이드 예정 (RTX 3060+)으로 GPU 제약 해소 계획
