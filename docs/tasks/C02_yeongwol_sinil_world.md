# C2: 영월 신일리 실제 지형 월드

- 분야: 시뮬레이션
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료 (코드 삭제됨 — C34 구조 재설계 시 ad_simulation 패키지 제거)

## 배경 및 목적

SRTM DEM + OSM 데이터 기반 실제 지형 월드. 영월군 신일리 500m×500m 영역을 Gazebo 월드로 변환.

## 변경 사항 (삭제됨)

### 당시 생성된 파일 (git 히스토리 커밋 c004a19 기반)
- `src/ad_simulation/worlds/real_terrain/yeongwol_sinil.sdf` — 2,786줄
- `src/ad_simulation/worlds/real_terrain/fetch_terrain_data.py` — 966줄, SRTM DEM + OSM 데이터 수집
- `src/ad_simulation/worlds/real_terrain/sinil_heightmap.png` — 고도맵
- `src/ad_simulation/worlds/real_terrain/sinil_osm_data.json` — OSM 데이터

### 삭제 시점
커밋 6c4ff96 (2026-02-23).

## 현재 상태

코드가 프로젝트에 존재하지 않음. DEM/OSM 파이프라인은 C4와 동일.
