# C73: SDF 모터 특성 — DB130-48 PI 제어 + 속도 램핑

## 배경

현재 TrackedVehicle 플러그인은 기본 gain 값을 사용한다.
실차 SS500은 좌/우 독립 DB130-48 BLDC 모터(3kW)를 DMD2K 드라이버로 PI 제어한다.
펌웨어의 PI 게인과 속도 램핑 로직을 시뮬레이션에 반영한다.

## HIH_2 출처

- **모터 스펙**: DB130-48, 3kW, BLDC + 전자식 브레이크
- **컨트롤러**: DMD2K (MDROBOT), CAN 통신
- **PI 게인**: `/home/gint_pcd/projects/HIH_2/1)프로젝트/5)농업/ssvcu_branch_20260105_merge_20260128/SSVCU/MotorControl/PowerTrain_Control.c`
  - Kp = 0.05
  - KiT = 0.006
  - KcT = 0.05
- **속도 램프**: 가속/감속 레이트 정의 (PowerTrain_Control.c의 ramp 로직)

## 변경 내역

### 1. model.sdf — TrackedVehicle 플러그인 gain 튜닝

```xml
<!-- DB130-48 모터 특성 반영 -->
<linear_velocity>
  <x_p_gain>0.05</x_p_gain>
  <x_i_gain>0.006</x_i_gain>
</linear_velocity>
```

### 2. 가감속 제한 (램핑)

TrackedVehicle 플러그인 또는 velocity_smoother에서 처리:
- 가속: 최대 0.5 m/s² (농업 장비 안전 기준)
- 감속: 최대 1.0 m/s² (제동 능력)

### 3. 좌/우 독립 모터 동작 확인

스키드 스티어 회전 시 좌/우 모터 속도 차이가 실차와 유사한지 검증

## 수정 대상 파일

- `src/ad_bringup/models/ss500/model.sdf`

## 검증

- cmd_vel 스텝 입력 → 실제 속도 응답 곡선 비교 (오버슈트, 정착 시간)
- 좌/우 독립 속도 명령 시 회전 반응 확인
- Foxglove Plot: cmd_vel vs odom 응답 비교
