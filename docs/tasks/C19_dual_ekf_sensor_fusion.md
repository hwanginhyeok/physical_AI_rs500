# C19: Dual-EKF 센서 퓨전

- 분야: 알고리즘/설정
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

GPS + IMU + 휠 오도메트리를 융합하는 이중 EKF 구조 설정. robot_localization 패키지의 dual EKF 구성.

## 변경 사항

### 주요 파일

`src/ad_bringup/config/dual_ekf.yaml` — 190줄

별도의 Python EKF 구현 없이 robot_localization 패키지의 설정 파일로 구성.

### 구조

**ekf_local** (로컬 오도메트리, 50Hz):
- 입력: `/odom` (휠 오도메트리) + `/sensor/imu`
- 출력: odom 프레임 기준 위치 추정
- process_noise: 1.0e-3 대각 (15×15)

**ekf_global** (전역 위치, 10Hz):
- 입력: `/odometry/local` (ekf_local 출력) + `/odometry/gps` (navsat_transform 출력)
- 출력: map 프레임 기준 위치 추정
- process_noise: 5.0e-3 (위치) / 3.0e-3 (회전)

**navsat_transform** (GPS 변환, 10Hz):
- GPS NavSatFix → Odometry 변환
- `zero_altitude: true` (2D 운용)
- `publish_filtered_gps: true`

### 설계 결정

- 이중 EKF: 로컬(고빈도, 정밀 단기) + 글로벌(저빈도, GPS 보정) 분리
- robot_localization 패키지 활용 (직접 EKF 구현 없음)
- GPS 단절 시 ekf_local만으로 운용 가능 (C29 LIO-SAM과 연계)
