# C75: SDF 센서 노이즈 — EBIMU-9DOFV5 + ZED-F9P 실차 스펙

## 배경

현재 시뮬레이션의 IMU/GPS 노이즈는 일반적인 가우시안 값이다.
HIH_2에 실차 센서 스펙이 있으므로 데이터시트 기반 노이즈 모델로 교체한다.

## HIH_2 출처

### IMU: EBIMU-9DOFV5 (9축)
- **CAN 메시지**: SNS2ADT1~3 (0x341~343)
- **가속도**: 분해능 0.001g → 노이즈 ~0.01 m/s² (1σ)
- **각속도**: 분해능 0.01 deg/s → 노이즈 ~0.005 rad/s (1σ)
- **자세**: Roll/Pitch/Yaw 분해능 0.01°

### GPS: ZED-F9P (듀얼밴드 GNSS)
- **CAN 메시지**: SNS2ADT4~6 (0x344~346)
- **수평 정밀도**: RTK Fix 시 0.01m CEP
- **속도 정밀도**: 0.05 km/h
- **방위각 정밀도**: 0.3° (듀얼 안테나)
- **업데이트 주기**: 10Hz
- **Status 필드**: FixType, numSV, FixedOK, DiffSoln, CarrSoln

## 변경 내역

### 1. model.sdf — IMU 센서 노이즈

```xml
<sensor name="imu_sensor" type="imu">
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.005</stddev></noise></x>
      <!-- y, z 동일 -->
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
      <!-- y, z 동일 -->
    </linear_acceleration>
  </imu>
  <update_rate>100</update_rate>
</sensor>
```

### 2. model.sdf — GPS 센서 노이즈

```xml
<sensor name="navsat_sensor" type="navsat">
  <position_sensing>
    <horizontal>
      <noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise>
    </horizontal>
    <vertical>
      <noise type="gaussian"><mean>0</mean><stddev>0.02</stddev></noise>
    </vertical>
  </position_sensing>
  <velocity_sensing>
    <horizontal>
      <noise type="gaussian"><mean>0</mean><stddev>0.014</stddev></noise>
    </horizontal>
  </velocity_sensing>
  <update_rate>10</update_rate>
</sensor>
```

### 3. EKF 파라미터 조정

센서 노이즈 변경에 따른 EKF 공분산 행렬 조정 필요 여부 확인.

## 수정 대상 파일

- `src/ad_bringup/models/ss500/model.sdf`
- `src/ad_bringup/config/ekf_params.yaml` (필요 시)

## 검증

- 정지 상태에서 `/sensor/imu` 표준편차 측정 → 0.01 m/s² 근처
- 정지 상태에서 `/sensor/gps` 위치 산포 측정 → 0.01m 반경 이내
- EKF 출력 `/odometry/local` 드리프트 확인
