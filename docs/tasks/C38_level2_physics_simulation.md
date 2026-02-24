# C38~C44: Level 2 물리 시뮬레이션 업그레이드

- 분야: 시뮬레이션
- 담당: Claude
- 기간: 2026-02-24 ~ 2026-02-24
- 상태: 완료
- 관련 작업: C38(동역학), C39(구동계), C40(지면상호작용), C41(센서노이즈), C42(통합시뮬), C43(지형확장), C44(테스트)

---

## 배경 및 목적

기존 시스템은 Level 1(운동학적 시뮬레이션) 수준으로, `SkidSteerModel`이 순수 기구학(kinematics)만 처리했다. 질량, 관성, 지면 상호작용, 센서 노이즈 등이 없어 실차 거동 예측이 부정확했다.

Level 2(엔지니어링 시뮬레이션)로 업그레이드하여:
- 설계 검증 가능한 수준의 차량 거동 재현
- 논/밭 등 농업 환경 특화 지면 모델 구축
- 센서 현실화로 인지 알고리즘 강건성 사전 검증

디폴트 파라미터로 프레임워크를 먼저 구축하고, 실측 데이터는 추후 주입하는 전략.

---

## 설계 결정

### 1. 기존 SkidSteerModel과의 관계: 래핑(wrapping) 방식 채택

**선택**: VehicleDynamics가 SkidSteerModel을 내부에 포함하여 운동학 계산을 위임
**대안**: SkidSteerModel을 상속하여 확장
**이유**: 운동학과 동역학을 명확히 분리. Level 1/Level 2 전환이 자유롭고, 운동학 모델의 독립성 유지.

### 2. 지면 모델: 간소화 Bekker + Mohr-Coulomb

**선택**: 학술적 Bekker/Mohr-Coulomb의 간소화 버전
**대안**: Wong의 경험적 모델, Magic Formula
**이유**: 궤도차량에 적합한 Bekker 모델 기반이 자연스럽고, 파라미터가 토양역학 문헌에서 쉽게 구할 수 있음. 실시간성도 충분.

### 3. 센서 노이즈: Allan Variance 기반 IMU

**선택**: ARW(Angle Random Walk) + 1차 마르코프 바이어스 드리프트
**대안**: 단순 가우시안 노이즈만 적용
**이유**: 가우시안만으로는 바이어스 드리프트를 재현 불가. SLAM/EKF 테스트에 바이어스 모델이 필수.

### 4. 모듈 구조: 4개 독립 코어 + 1개 통합 시뮬레이터

각 모듈(동역학, 구동계, 지면, 센서)을 `ad_core/`에 독립 배치. 개별 사용 가능하고, `tools/physics_simulator.py`에서 조합. ROS2 독립 원칙 유지.

---

## 변경 사항

### 신규 파일

| 파일 | 내용 |
|------|------|
| `src/ad_core/ad_core/vehicle_dynamics.py` | VehicleDynamicsConfig, VehicleDynamics: F=ma 종방향, tau=I*alpha 요회전, 하중전이, 경사면 중력분해 |
| `src/ad_core/ad_core/drivetrain_model.py` | DrivetrainConfig, SingleMotorModel, DrivetrainModel: 1차 지연, 데드존, 좌/우 독립, 감속기 효율(정/역방향 비대칭) |
| `src/ad_core/ad_core/track_terrain_interaction.py` | TerrainConfig, TerrainType(10종), TrackTerrainInteraction: Bekker 침하, Mohr-Coulomb 견인력, 선회저항 |
| `src/ad_core/ad_core/sensor_noise_model.py` | GPS(멀티패스+드랍아웃), IMU(ARW+바이어스드리프트), LiDAR(기상조건), Camera(모션블러), SensorNoiseBundle |
| `tools/physics_simulator.py` | PhysicsSimulator: L1/L2 전환, 지형순환, 4패널 시각화(궤적/속도슬립/하중/IMU), CLI 인자 지원 |
| `src/ad_core/test/test_vehicle_dynamics.py` | 12개 테스트: 정지유지, 가속제한, 감속제한, 직진, 회전, 경사면, 하중전이, 등판한계, 에너지보존 등 |
| `src/ad_core/test/test_drivetrain_model.py` | 12개 테스트: 데드존, 스텝응답, 오버슈트, 역방향, 좌우대칭, 독립모터, 견인력 등 |
| `src/ad_core/test/test_track_terrain_interaction.py` | 19개 테스트: 침하, 견인력, 선회저항, 구름저항, 경사면, 지형비교, 통합계산 등 |
| `src/ad_core/test/test_sensor_noise_model.py` | 15개 테스트: GPS 평균/분산/드랍아웃/RTK, IMU 바이어스드리프트, LiDAR 범위/기상, Camera 노이즈 |

### 수정 파일

| 파일 | 변경 내용 |
|------|----------|
| `src/ad_core/ad_core/terrain_classifier.py` | TerrainType에 PADDY_WET(0.7), PADDY_DRY(0.45), FIELD_SOFT(0.55), FIELD_HARD(0.35) 추가 |

---

## 디폴트 파라미터 (SS500 추정)

| 파라미터 | 값 | 비고 |
|---------|-----|------|
| 차량 질량 | 200 kg | SS500 공차중량 추정 |
| Izz | 50 kg*m^2 | 균일 직육면체 가정 |
| CoG 높이 | 0.4 m | |
| 최대 가속도 | 1.0 m/s^2 | |
| 최대 감속도 | 2.0 m/s^2 | |
| 모터 시정수 | 0.15 s | |
| 감속비 | 30:1 | |
| 감속기 효율 | 85% (정방향) / 75% (역방향) | |
| 데드존 | 0.05 m/s | |
| GPS sigma (단독) | 2.5 m | RTK: 0.02 m |
| 자이로 ARW | 0.01 rad/s/sqrt(Hz) | |
| LiDAR sigma | 0.02 m | |

---

## 테스트 결과

```
신규 테스트: 58개 전부 통과
기존 테스트: 120개 회귀 없음
총 테스트:   178개 통과 (0.40s)
```

---

## 향후 과제

1. **파라미터 튜닝**: SS500 실차 데이터 확보 시 System Identification으로 디폴트 파라미터 교체
2. **pure_pursuit.py 동적 슬립 피드백**: TrackTerrainInteraction의 슬립 출력을 SlipCompensatedPurePursuit에 연결
3. **지형 맵 로딩**: 격자 기반 terrain type 맵 파일 지원 (YAML/이미지)
4. **YAML 시나리오**: 초기위치, 경로, 지형, 기상조건을 시나리오 파일로 정의
5. **Level 1 vs Level 2 정량 비교**: 동일 경로 CTE/완주시간 비교 리포트 자동 생성
6. **ROS2 연동**: PhysicsSimulator를 ROS2 노드로 래핑하여 SIL 테스트 지원
