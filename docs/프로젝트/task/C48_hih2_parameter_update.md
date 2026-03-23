# C48: HIH-2 실제 데이터 기반 시뮬레이션 파라미터 업데이트

- 분야: 시뮬레이션
- 담당: 그린
- 기간: 2026-03-23
- 상태: 완료

## 배경

RS500 시뮬레이션이 추정값(200kg, 감속비 30, 시정수 0.15s 등)으로 동작하고 있었다.
HIH-2 프로젝트(`~/HIH_2/`)의 PLUVA FRIEND SS500 실차 데이터를 분석하여
시뮬레이션 파라미터를 실차 기준으로 교정한다.

### 데이터 소스

| 소스 | 경로 | 추출 정보 |
|------|------|----------|
| CAN DBC (VCU) | `~/HIH_2/1)프로젝트/5)농업/4)CAN통신/SSVCU_CANdb_250317_01.dbc` | 차속 범위, 배터리 스펙, 센서 해상도 |
| CAN DBC (트랙션) | `~/HIH_2/.../TRACTION_MOTOR/3)DMKE_CANdb.dbc` | 모터 RPM/전류/PWM 범위 |
| 모터 데이터시트 | `~/HIH_2/.../8)구동모터/모터3/0)Datasheet/확정_DB130-48(3000-20SM&SCKB75-20).pdf` | 모터 전체 스펙 |
| BLACKTAN 시험 | `~/HIH_2/.../240807_BLACKTAN/` | 돌입 전류, 정격 전류, 선회 전류, 무부하 PWM |

## 결정 사항

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| 감속비 30→20 | DB130-48 SCK75-20 데이터시트 확정 | 없음 (데이터시트 확정값) |
| 모터 RPM 3000→2000 | DB130-48 정격 2000rpm ±10% | 없음 |
| 차량 질량 200→800kg | 배터리 48V 460Ah (~200kg) + 차체 + 모터 등 공차 추정 | 1000kg (SDF 기존값) → 공차/만차 구분 필요 |
| 시정수 0.15→0.20s | BLACKTAN 돌입 전류 파형에서 ~2-4s 내 정상상태 도달 | 0.15s 유지 (변경 폭 작음) |
| 스프로킷 반경 0.15→0.106m | v_max/(ω_output) = 1.111/(100rpm*2π/60) | 없음 (기하학적 계산) |
| GPS stddev 2.5→0.5m | CAN에 RTK 지원(CarrSoln), 과수원 환경 감안 중간값 | 0.02m (RTK only) → 환경 혼합 감안 |
| 윤거 1.4→1.2m | SS500 소형 궤도차 추정 | 1.0m → 너무 좁음 |

## 파라미터 비교 테이블

### VehicleDynamicsConfig

| 파라미터 | 기존값 | 실차 교정값 | 소스 |
|---------|--------|-----------|------|
| mass | 200 kg | **800 kg** | 추정 (TODO: xlsx) |
| Izz | 50 kg·m² | **250 kg·m²** | 추정 (TODO: SysID) |
| cog_height | 0.4 m | **0.5 m** | 추정 |
| wheelbase | 1.5 m | **1.8 m** | 추정 (TODO: xlsx) |
| track_width | 1.4 m | **1.2 m** | 추정 (TODO: xlsx) |
| max_accel | 1.0 m/s² | **0.5 m/s²** | 800kg, 3kW×2 모터 |
| max_decel | 2.0 m/s² | **1.5 m/s²** | 브레이크 16Nm 감안 |
| max_speed | 1.0 m/s | **1.111 m/s** | CAN ±4km/h |

### DrivetrainConfig

| 파라미터 | 기존값 | 실차 교정값 | 소스 |
|---------|--------|-----------|------|
| time_constant | 0.15 s | **0.20 s** | BLACKTAN 돌입 |
| gear_ratio | 30 | **20** | SCK75-20 |
| max_motor_torque | 20 Nm | **14.32 Nm** | 3000W/209.4rad/s |
| max_motor_rpm | 3000 | **2000** | DB130-48 |
| sprocket_radius | 0.15 m | **0.106 m** | v/ω 역산 |

### 센서 노이즈 (CAN DBC 해상도 기반)

| 파라미터 | 기존값 | 실차 교정값 | CAN 해상도 |
|---------|--------|-----------|-----------|
| GPS position_stddev | 2.5 m | **0.5 m** | RTK 지원 |
| GPS multipath | 0.5 m | **0.3 m** | 과수원 환경 |
| IMU gyro_noise | 0.01 | **0.005** | 0.01°/s |
| IMU accel_noise | 0.02 | **0.01** | 0.001g |

### Gazebo SDF (model.sdf)

| 파라미터 | 기존값 | 실차 교정값 |
|---------|--------|-----------|
| body mass | 1000 kg | **600 kg** (+ 궤도 100kg×2 = 800kg total) |
| track offset | ±0.7 m | **±0.6 m** (윤거 1.2m) |
| track mass | 80 kg | **100 kg** |
| max/min velocity | ±2.0/3.0 | **±1.111** |
| track_width plugin | 1.4 m | **1.2 m** |

## 변경 내역

### 수정 파일
- `src/ad_core/ad_core/vehicle_dynamics.py` — VehicleDynamicsConfig 디폴트값 전면 교정
- `src/ad_core/ad_core/drivetrain_model.py` — DrivetrainConfig 디폴트값 전면 교정
- `src/ad_core/ad_core/track_terrain_interaction.py` — track_length 1.2→1.5m
- `src/ad_core/ad_core/sensor_noise_model.py` — GPS/IMU 노이즈 파라미터 교정
- `src/ad_bringup/models/ss500/model.sdf` — 질량, 관성, 윤거, 속도 제한 교정
- `src/ad_core/test/test_vehicle_dynamics.py` — 디폴트값 검증 테스트 업데이트

## 검증

- **단위 테스트**: 218건 전체 통과 (5.0s)
- **시나리오 시뮬레이션**: matplotlib/numpy 호환 이슈로 미실행 (TODO)
- **리포트 생성**: 동일 이슈로 미실행 (TODO)

## BLACKTAN 시험 데이터 분석 요약

### 돌입 전류 (Inrush-Acc1, Load)
- C1 Max: 50.8A, RMS: 14.43A
- C2 Max: 45.02A, RMS: 17.32A
- 정상상태 도달: ~2-4s → 시정수 ~0.20s 추정

### 돌입 전류 (Inrush-AccFast, Load)
- C1 Max: 44.02~50.8A, Mean: 47.75A
- C2 Max: 52.67A, Mean: 46.29A
- 빠른 가속 시 더 높은 피크 전류

### 정격 전류 (Spd2, Load)
- C1 Max: 22.87A, RMS: 14.35A
- C2 Max: 26.35A, RMS: 16.75A
- 안정적 운행, 좌/우 약간의 비대칭 (~15%)

### 선회 (Spd1, Turn)
- C1 Max: 49.21A, Min: -29.71A
- C2 Max: 48.7A, Min: -19.32A
- 한쪽 모터 역회전 확인, 피크 대칭적

### 무부하 (Spd1, Left Motor)
- PWM DutyCycle: ~10.4%
- 전류 RMS: 10.71A, Min: 5.05A

## 미결 이슈

- [ ] 차량 총 중량 (공차/만차) — xlsx 파일 파싱 필요
- [ ] 차체 정확한 크기 (L×W×H) — xlsx 파일 파싱 필요
- [ ] 궤도 폭, 접지 길이, 윤거 정확한 값 — xlsx 파일 파싱 필요
- [ ] matplotlib/numpy 호환 문제 해결 후 시나리오 시뮬레이션 재실행
- [ ] module_test_report.py 리포트 생성
- [ ] 실차 System Identification으로 Izz, 시정수, 효율 정밀 튜닝
- [ ] 실차 주행 데이터로 시뮬레이션 검증 (Phase 3)
