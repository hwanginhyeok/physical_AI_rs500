# C79: 농업 환경 월드 — 과수원 + 논/밭 SDF

## 배경

현재 시뮬레이션은 빈 평면 월드를 사용한다.
실제 운용 시나리오(과수원 행간 주행, 논 방제, 밭 커버리지)에 맞는 Gazebo 월드를 구축한다.

## 시나리오 (확정 사항 참조)

1. **과수원**: 행간 주행 + 분무 (주 운용)
2. **논**: 방제 주행 (커버리지)
3. **밭**: 커버리지 주행

## 변경 내역

### 1. 과수원 월드 (orchard.sdf)

- **나무 배치**: 행간 3.5m, 주간 4m, 5열 x 10주 (50그루)
- **나무 모델**: 원통(trunk) + 구(canopy) 단순 형상, collision 있음
- **지면**: 흙 텍스처, 마찰 μ=1.2
- **경사**: 월드 일부 구간 10~15° 경사면
- **크기**: 50m x 40m

### 2. 논/밭 월드 (field.sdf)

- **지면**: 평탄, 이랑 패턴 (범프 높이 0.05m, 간격 0.6m)
- **크기**: 100m x 60m
- **경계**: 낮은 둑(0.3m) 또는 펜스
- **물**: 논의 경우 수면 텍스처 (시각적만)

### 3. 런치 파일 월드 선택

`simulation_launch.py`에 `world` 인자 추가:
```bash
ros2 launch ad_bringup simulation_launch.py world:=orchard
ros2 launch ad_bringup simulation_launch.py world:=field
ros2 launch ad_bringup simulation_launch.py world:=empty  # 기본값 (현재)
```

### 4. GPS datum 설정

각 월드별 GPS 원점(datum) 설정:
- 과수원: 영월 좌표 (37.182°N, 128.462°E)
- 논/밭: 청송 좌표 (36.431°N, 129.057°E)

## 수정/생성 파일

- `src/ad_bringup/worlds/orchard.sdf` (신규)
- `src/ad_bringup/worlds/field.sdf` (신규)
- `src/ad_bringup/launch/simulation_launch.py` (world 인자 추가)
- `src/ad_bringup/config/nav2_params.yaml` (월드별 datum)

## 검증

- 과수원 월드: 나무 행 사이로 Nav2 웨이포인트 주행 성공
- costmap에 나무가 장애물로 인식되는지 확인
- 경사면에서 차량 안정성 확인
- 논/밭 월드: Fields2Cover 경로 커버리지 주행 성공
