# C1: 청송 사과 과수원 월드 생성

- 분야: 시뮬레이션
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료 (코드 삭제됨 — C34 구조 재설계 시 ad_simulation 패키지 제거)

## 배경 및 목적

Gazebo gz-sim용 농업 환경 시뮬레이션 월드. 청송군 사과 과수원을 모델링하여 과수원 내 자율주행 테스트 환경 구축.

## 변경 사항 (삭제됨)

### 당시 생성된 파일 (git 히스토리 커밋 1ece5bf 기반)
- `src/ad_simulation/worlds/real_terrain/cheongsong_orchard.sdf` — 94,559줄, 2,400그루 사과나무 배치
- `src/ad_simulation/worlds/real_terrain/cheongsong_heightmap.png` — 고도맵
- `src/ad_simulation/worlds/real_terrain/cheongsong_osm_data.json` — OSM 지형 데이터
- `src/ad_simulation/worlds/real_terrain/generate_orchard_world.py` — 977줄, 월드 생성 스크립트
- `src/ad_simulation/worlds/real_terrain/visualize_terrain.py` — 250줄, 3D 시각화

### 삭제 시점
커밋 6c4ff96 (2026-02-23): 프로젝트 구조 재설계 시 `ad_simulation` 패키지 전체 삭제.

## 현재 상태

코드가 프로젝트에 존재하지 않음. git 히스토리에서만 확인 가능. GPU 업그레이드(RTX 3060+) 후 Gazebo 환경 재구축 시 참고 가능.
