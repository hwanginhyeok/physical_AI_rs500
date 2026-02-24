# C15: Gazebo gz-sim 시뮬레이션 연동

- 분야: 시뮬레이션
- 담당: Claude
- 기간: 2026-02-21 (추정)
- 상태: 삭제됨

## 배경 및 목적

Gazebo gz-sim 환경에서 궤도차량 시뮬레이션을 실행하고 ROS2와 연동하는 기능.

## 현재 상태

C34 프로젝트 재구조화(commit 6c4ff96, 2026-02-23)에서 `ad_simulation/` 패키지 전체가 삭제됨.
현재 코드베이스에 관련 파일이 존재하지 않아 구현 상세를 알 수 없음.

## 관련 자료

- `docs/research/gazebo_tracked_vehicle_simulation_research.md` (903줄) — C10에서 작성한 선행연구 문서가 참조 자료로 남아 있음
- GPU 제약(MX550 2GB)으로 3D 시뮬레이션 실행 불가 상태였음
