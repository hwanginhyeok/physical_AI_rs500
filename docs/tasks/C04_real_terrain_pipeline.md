# C4: 실제 지형 데이터 파이프라인 (SRTM DEM + OSM)

- 분야: 시뮬레이션
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료 (코드 삭제됨 — C34 구조 재설계 시 ad_simulation 패키지 제거)

## 배경 및 목적

임의의 GPS 좌표를 입력하면 SRTM DEM 고도 데이터와 OSM 도로/건물 데이터를 자동 수집하여 Gazebo SDF 월드를 생성하는 파이프라인.

## 설계 결정

- SRTM 30m 해상도 DEM 사용 (무료, 전 세계 커버리지)
- OSM Overpass API로 도로/건물/수계 쿼리
- heightmap PNG로 변환 → Gazebo `<heightmap>` 활용

## 변경 사항 (삭제됨)

### 당시 생성된 파일
- `src/ad_simulation/worlds/real_terrain/fetch_terrain_data.py` — 966줄
- `src/ad_simulation/worlds/real_terrain/generate_orchard_world.py` — 977줄

### 삭제 시점
커밋 6c4ff96 (2026-02-23).

## 현재 상태

코드가 프로젝트에 존재하지 않음. 파이프라인 로직은 git 히스토리에서 복원 가능.
