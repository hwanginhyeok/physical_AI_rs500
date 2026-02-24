# C3: launch 월드 선택 기능

- 분야: 시뮬레이션
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료 (코드 삭제됨 — C34 구조 재설계 시 ad_simulation 패키지 제거)

## 배경 및 목적

Gazebo 시뮬레이션 launch 시 `world:=` 인자로 월드 파일을 선택할 수 있도록 구현.

## 변경 사항 (삭제됨)

### 당시 생성된 파일
- `src/ad_simulation/launch/simulation_launch.py` — 102줄, `world` launch argument 지원

### 삭제 시점
커밋 6c4ff96 (2026-02-23).

## 현재 상태

코드가 프로젝트에 존재하지 않음. 현재 launch 시스템은 `ad_bringup/launch/`로 이관됨.
