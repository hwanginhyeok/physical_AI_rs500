# 궤도차량 자율주행 - 동역학 및 제어 선행연구 보고서

> 작성일: 2026-02-21
> 대상 플랫폼: RS500 (1톤급, 농업용 -> 군사용 확장)
> 시뮬레이션 환경: ROS2 Humble + Gazebo (gz-sim)

---

## 목차

1. [궤도차량 동역학 모델링](#1-궤도차량-동역학-모델링)
2. [슬립 모델링 (Track-Terrain Interaction)](#2-슬립-모델링-track-terrain-interaction)
3. [조향 알고리즘 (Differential Steering)](#3-조향-알고리즘-differential-steering)
4. [경로 추종 제어](#4-경로-추종-제어)
5. [ROS2 + Gazebo 시뮬레이션 사례](#5-ros2--gazebo-시뮬레이션-사례)
6. [RS500 프로젝트 종합 시사점](#6-rs500-프로젝트-종합-시사점)

---

## 1. 궤도차량 동역학 모델링

### 1.1 핵심 개념

궤도차량(Tracked Vehicle)은 일반 차륜형 차량과 달리 **스키드 스티어링(Skid Steering)** 방식으로 조향한다. 좌우 궤도(track)의 속도 차이를 통해 선회하며, 이 과정에서 궤도의 측면 미끄러짐(lateral skidding)이 필연적으로 발생한다.

#### 기본 키네마틱 모델 (Differential Drive Approximation)

가장 단순한 형태로, 궤도차량을 차동구동(differential drive) 로봇으로 근사할 수 있다:

```
v = (v_R + v_L) / 2          (전진 속도)
omega = (v_R - v_L) / B      (요 레이트)

x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = omega
```

여기서 `v_R`, `v_L`은 우측/좌측 궤도 속도, `B`는 궤도 간 거리(tread width)이다. 그러나 이 모델은 슬립을 무시하므로 실제 동작과 괴리가 크다.

#### ICR 기반 키네마틱 모델

보다 정확한 모델은 **순간 회전 중심(Instantaneous Center of Rotation, ICR)** 개념을 도입한다. 좌우 각 궤도의 ICR 위치를 `y_ICR_L`, `y_ICR_R`로 정의하면:

```
omega = v_L / y_ICR_L = v_R / y_ICR_R

v_x = omega * x_ICR          (종방향 속도)
v_y = omega * y_ICR           (횡방향 속도, 횡방향 슬립 반영)

x_dot = v_x * cos(theta) - v_y * sin(theta)
y_dot = v_x * sin(theta) + v_y * cos(theta)
theta_dot = omega
```

ICR 위치는 지형, 속도, 하중 등에 따라 동적으로 변하므로 실시간 추정이 필요하다.

#### 동역학 모델 (Dynamics)

동역학 수준에서는 다음을 추가 고려한다:
- **관성력(Inertia)**: 차량 질량 `m`과 요 관성모멘트 `I_z`
- **궤도-지면 마찰력**: 종방향 추진력(traction force)과 횡방향 저항력(lateral resistance)
- **원심력**: 선회 시 차체에 작용하는 원심력
- **경사 효과**: 경사면에서의 중력 성분

뉴턴-오일러 방정식 기반:
```
m * (v_x_dot - v_y * omega) = F_xR + F_xL - R_x
m * (v_y_dot + v_x * omega) = F_yR + F_yL - R_y
I_z * omega_dot = (F_xR - F_xL) * B/2 + M_r
```

여기서 `F_x`, `F_y`는 각 궤도의 종/횡 방향 힘, `R`은 저항력, `M_r`은 선회 저항 모멘트이다.

### 1.2 주요 논문 및 참고자료

| 저자 | 제목 | 연도 | 핵심 기여 |
|------|------|------|-----------|
| Martinez, Mandow, Morales, Pedraza, Garcia-Cerezo | [Approximating Kinematics for Tracked Mobile Robots](https://journals.sagepub.com/doi/10.1177/0278364905058239) | 2005 | ICR 경계 영역 기반 근사 키네마틱 모델 제안, 특정 지형에 대한 최적 상수 ICR 위치 도출 |
| Pentzer, Brennan, Reichard | [Model-based Prediction of Skid-steer Robot Kinematics Using Online Estimation of Track ICRs](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21509) | 2014 | 스키드 스티어를 등가 유니사이클 모델로 매핑하는 ICR 기반 방법, 온라인 ICR 추정 |
| Lu, Xiong | [Motion Predicting of Autonomous Tracked Vehicles with Online Slip Model Identification](https://onlinelibrary.wiley.com/doi/10.1155/2016/6375652) | 2016 | EKF 기반 실시간 ICR 추정, 전방 궤적 예측에 활용, RTK GPS 없이 위치 오차 24% 이상 감소 |
| Discover Applied Sciences | [Dynamics of Tracked Vehicles During Nonuniform Turning on Level Terrain and Slopes](https://link.springer.com/article/10.1007/s42452-024-06039-1) | 2024 | 비균일 선회 및 경사면 동역학 통합 해석, 원심력/관성력 고려 |
| Mandow, Martinez | [Experimental Kinematics for Wheeled Skid-steer Mobile Robots](https://www.semanticscholar.org/paper/Experimental-kinematics-for-wheeled-skid-steer-Mandow-Mart%C3%ADnez/760c81c0a19f8358dde38691b81fe2f20c829b44) | 2009 | 실험 기반 스키드 스티어 키네마틱 파라미터 식별, 지형별 실측 데이터 |

### 1.3 실제 적용 시 고려사항

- **모델 복잡도 vs 실시간성 트레이드오프**: 전체 동역학 모델은 정확하지만 계산 비용이 높다. 제어 루프(>50Hz)에는 ICR 기반 키네마틱 모델이 적합하고, 경로 계획(1~10Hz)에는 동역학 모델 활용을 고려한다.
- **파라미터 식별**: 차량 질량, 관성모멘트, 궤도-지면 마찰계수 등은 사전 측정 또는 온라인 추정이 필요하다.
- **모델 전환**: 저속 정밀 작업(농업)과 고속 기동(군사)에서 지배적인 동역학 항이 다르므로, 속도 범위에 따른 모델 전환 전략이 유효하다.

### 1.4 RS500 프로젝트 시사점

- RS500(1톤급)은 질량이 상당하므로 **관성 효과가 무시할 수 없는 수준**이다. 순수 키네마틱 모델만으로는 부족하며, 최소한 ICR 기반 슬립 보상 키네마틱 모델이 필요하다.
- 농업용 비포장 환경에서의 ICR 파라미터는 아스팔트 대비 변동 폭이 크므로, **온라인 파라미터 추정**(EKF 또는 UKF 기반)을 반드시 구현해야 한다.
- 군사용 확장 시 고속 기동이 요구되므로, 원심력/관성력을 포함한 **전체 동역학 모델 기반 제어**로의 전환 경로를 설계 단계에서 확보해야 한다.

---

## 2. 슬립 모델링 (Track-Terrain Interaction)

### 2.1 핵심 개념

궤도차량의 가장 큰 제어 난제는 **슬립(Slip)**이다. 궤도가 지면과 상호작용할 때 이상적인 무슬립 조건은 거의 성립하지 않으며, 특히 비포장/연약지반에서는 종방향 슬립(longitudinal slip)과 횡방향 슬립(lateral slip)이 동시에 크게 발생한다.

#### 종방향 슬립 (Longitudinal Slip)

궤도의 회전 속도와 실제 차량 전진 속도의 차이:

```
s_long = (v_track - v_actual) / max(v_track, v_actual)
```

- 양수: 구동 슬립 (엔진 출력 > 지면 견인력)
- 음수: 제동 슬립

#### 횡방향 슬립 (Lateral Slip)

스키드 스티어링 선회 시 필연적으로 발생하는 측면 미끄러짐. 차량의 실제 이동 방향과 궤도 방향 사이의 각도(slip angle)로 정의된다.

#### Bekker-Wong 지반역학 모델 (Terramechanics)

궤도-지형 상호작용의 고전적 모델:

**압력-침하(Pressure-Sinkage) 관계 (Bekker 모델)**:
```
p = (k_c / b + k_phi) * z^n
```
- `p`: 접지 압력
- `z`: 침하량(sinkage)
- `b`: 궤도 폭
- `k_c`, `k_phi`: 토양의 응집 및 마찰 변형 계수
- `n`: 침하 지수

**전단응력-전단변위 관계 (Janosi-Hanamoto 모델)**:
```
tau = (c + sigma * tan(phi)) * (1 - exp(-j / K))
```
- `tau`: 전단응력 (견인력의 원천)
- `c`: 토양 응집력(cohesion)
- `sigma`: 수직 응력
- `phi`: 내부 마찰각
- `j`: 전단 변위 (슬립에 비례)
- `K`: 전단 변형 계수

**견인력(Traction Force) 계산**:
```
F_traction = integral_0^L [ tau(x) * b ] dx
```
궤도 접지 길이 `L`에 걸쳐 전단응력을 적분하여 총 견인력을 산출한다.

### 2.2 주요 논문 및 참고자료

| 저자 | 제목 | 연도 | 핵심 기여 |
|------|------|------|-----------|
| Wong, J.Y. | Theory of Ground Vehicles (교과서) | 2008 (4th ed.) | 궤도차량 지반역학의 바이블, Bekker-Wong 모델 체계화 |
| Bekker, M.G. | Introduction to Terrain-Vehicle Systems | 1969 | 지반역학 기초 이론, 압력-침하 모델 원전 |
| Laughery, S. | [Bekker's Terramechanics Model for Off-road Vehicle Research](https://apps.dtic.mil/sti/tr/pdf/ADA457955.pdf) | 2005 | Bekker 모델의 실용적 적용 가이드 (DTIC 보고서) |
| Endo, Okada, Nagatani, Yoshida | [Path Following Control for Tracked Vehicles Based on Slip-Compensating Odometry](https://www.researchgate.net/publication/224296486_Path_following_control_for_tracked_vehicles_based_on_slip-compensating_odometry) | 2007 | 궤도차량 슬립 보상 오도메트리 기반 경로 추종 제어 |
| Nagatani et al. | [Online Slip Parameter Estimation for Tracked Vehicle Odometry on Loose Slope](https://k-nagatani.org/pdf/2016-SSRR-GEN-online.pdf) | 2016 | 느슨한 경사면에서의 실시간 슬립 파라미터 추정 |
| Lu, Xiong et al. | [An Improved Real-Time Slip Model Identification Method for Autonomous Tracked Vehicles](https://www.researchgate.net/publication/348302277_An_Improved_Real-Time_Slip_Model_Identification_Method_for_Autonomous_Tracked_Vehicles_Using_Forward_Trajectory_Prediction_Compensation) | 2021 | 전방 궤적 예측 보상을 활용한 개선된 실시간 슬립 모델 식별 |
| Kim et al. | [A Survey of Off-Road Mobile Robots: Slippage Estimation, Robot Control, and Sensing Technology](https://link.springer.com/article/10.1007/s10846-023-01968-2) | 2023 | 비포장 이동로봇 슬립 추정/제어 기술 종합 서베이 |
| ScienceDirect | [Soft Soil Track Interaction Modeling in Single Rigid Body Tracked Vehicle Models](https://www.sciencedirect.com/science/article/abs/pii/S0022489817301295) | 2017 | 연약지반에서의 궤도 상호작용 단일 강체 모델링 |
| Terramechanics + ML | [Terramechanics Models Augmented by Machine Learning Representations](https://www.sciencedirect.com/science/article/abs/pii/S0022489823000198) | 2023 | 머신러닝으로 보강된 지반역학 모델, 데이터 기반 토양 파라미터 학습 |

### 2.3 실제 적용 시 고려사항

- **토양 파라미터 사전 식별의 한계**: Bekker-Wong 모델은 토양 파라미터(k_c, k_phi, n, c, phi, K)를 Bevameter 시험으로 측정해야 하지만, 실제 운용 환경에서는 지형이 수시로 변한다. 따라서 **적응형(Adaptive) 또는 학습 기반 접근**이 필수적이다.
- **실시간 슬립 추정 센서 구성**: GPS/INS(관성항법) 융합으로 실제 차량 속도를 측정하고, 궤도 엔코더 속도와의 차이에서 슬립을 추정한다. RTK-GPS가 이상적이나, GPS 음영 지역에서는 IMU 기반 추정으로 전환해야 한다.
- **침하(Sinkage) 효과**: 1톤급 차량의 접지압은 소형 로봇보다 높아 연약지반 침하가 더 크게 발생한다. 이는 주행 저항 증가와 기동 불능으로 이어질 수 있다.

### 2.4 RS500 프로젝트 시사점

- 농업 현장(논밭)의 토양은 수분 함량에 따라 물성이 극적으로 변한다. **계절/기상 조건별 토양 파라미터 데이터베이스** 구축이 장기적으로 필요하다.
- 군사용 확장 시 다양한 지형(모래, 진흙, 자갈, 눈)을 통과해야 하므로, **지형 분류(Terrain Classification) 시스템**과 연동된 적응형 슬립 보상이 필수이다.
- 1톤급 중량은 소형 로봇 대비 견인력은 크지만 침하 위험도 높다. **궤도 접지 면적 및 접지압 최적화**가 하드웨어 설계 시 고려되어야 한다.
- 실시간 슬립 추정을 위해 **IMU + 휠 엔코더 + GPS(RTK 가능 시)**의 센서 퓨전 파이프라인을 우선 구축해야 한다.

---

## 3. 조향 알고리즘 (Differential Steering)

### 3.1 핵심 개념

궤도차량의 조향은 좌우 궤도의 **속도 차이(differential speed)**로 이루어진다. Ackermann 조향 기구가 없으므로 조향각 명령 대신 좌우 궤도 속도 명령 `(v_L, v_R)`을 직접 제어한다.

#### 조향 모드 분류

| 모드 | 좌측 궤도 (v_L) | 우측 궤도 (v_R) | 동작 |
|------|-----------------|-----------------|------|
| 직진 | V | V | 전진 |
| 완만 선회 | V - dV | V + dV | 곡선 주행 |
| 피벗 선회 | -V | V | 제자리 회전 |
| 급선회(래터럴) | 0 | V | 한쪽 궤도 중심 회전 |

#### 조향 명령 변환 (cmd_vel -> track speeds)

ROS2의 `geometry_msgs/Twist` 메시지 (`v_linear`, `omega`)를 좌우 궤도 속도로 변환:

```
v_R = v_linear + omega * B / 2
v_L = v_linear - omega * B / 2
```

역변환:
```
v_linear = (v_R + v_L) / 2
omega = (v_R - v_L) / B
```

여기서 `B`는 궤도 간 유효 너비(effective track width)이다. 실제로는 슬립 때문에 유효 너비가 기하학적 너비와 다르며, 이를 보정하는 것이 핵심이다.

#### 속도 제한 및 포화 처리

모터의 최대 속도 `v_max`에 의한 포화 처리가 필요하다:
```python
if abs(v_R) > v_max or abs(v_L) > v_max:
    scale = v_max / max(abs(v_R), abs(v_L))
    v_R *= scale
    v_L *= scale
```
이 방식은 선회 반경(curvature)을 유지하면서 전체 속도만 줄인다.

### 3.2 주요 논문 및 참고자료

| 저자 | 제목 | 연도 | 핵심 기여 |
|------|------|------|-----------|
| Robots For Roboticists | [Drive Kinematics: Skid Steer & Mecanum (ROS Twist included)](https://www.robotsforroboticists.com/drive-kinematics/) | - | 실용적인 스키드 스티어 키네마틱 구현 가이드 (ROS Twist 포함) |
| Zhang et al. | [A Control Method for the Differential Steering of Tracked Vehicles Driven by Dual Hydraulic Motor](https://www.mdpi.com/2076-3417/12/13/6355) | 2022 | 이중 유압 모터 기반 차동 조향 제어, RS500 유압 구동 시 참고 |
| Chen et al. | [Research on Trajectory Tracking and Body Attitude Control of AGV Based on Differential Steering](https://pmc.ncbi.nlm.nih.gov/articles/PMC9907824/) | 2023 | 차동 조향 AGV의 궤적 추종 및 자세 제어 통합 |
| Wei et al. | [Control of Pivot Steering for Bilateral Independent Electrically Driven Tracked Vehicles Based on GWO-PID](https://www.mdpi.com/2032-6653/15/6/231) | 2024 | GWO(Grey Wolf Optimizer) 기반 PID 튜닝, 피벗 선회 제어 |
| Hou, Ma, Xiang | [Steering Control of Electric Tracked Vehicle Based on Second-Order Disturbance Observer and Multi-objective Optimization](https://journals.sagepub.com/doi/10.1177/09544070241246046) | 2025 | 2차 외란 관측기 기반 조향 제어, 요 레이트 추종 오차 42.6% 감소 |
| Pentzer et al. | [The Use of Unicycle Robot Control Strategies for Skid-steer Robots Through the ICR Kinematic Mapping](https://www.semanticscholar.org/paper/The-use-of-unicycle-robot-control-strategies-for-Pentzer-Brennan/caeca6a8c986f0780b66f109bcb439a56e5fd062) | 2014 | ICR 매핑을 통한 유니사이클 제어 전략의 스키드 스티어 적용 |

### 3.3 실제 적용 시 고려사항

- **유효 궤도 폭(Effective Track Width) 보정**: 기하학적 궤도 간 거리와 실제 유효 조향 폭은 다르다. 지형과 속도에 따라 10~30%까지 차이가 발생할 수 있으며, 이를 온라인으로 보정해야 정확한 조향이 가능하다.
- **모터 응답 비대칭**: 좌우 모터/유압 시스템의 응답 특성이 미세하게 다를 수 있으며, 이는 직진 시 편향(drift)을 유발한다. 개별 모터 특성 식별 및 보상이 필요하다.
- **피벗 선회 시 과도 토크**: 제자리 회전(pivot turn)은 궤도-지면 간 전단력이 최대가 되어 지면 손상과 과도한 에너지 소모를 유발한다. 농업용에서는 논밭 보호를 위해 피벗 선회를 최소화하는 경로 계획이 바람직하다.
- **데드존(Dead Zone) 처리**: 유압 구동 시 밸브 데드존으로 인해 미세 속도 제어가 어렵다. 이를 피드포워드 보상하는 로직이 필요하다.

### 3.4 RS500 프로젝트 시사점

- RS500이 유압 구동 방식이라면 Zhang et al.(2022)의 이중 유압 모터 차동 조향 연구가 직접적으로 참고할 수 있다.
- **조향 명령 인터페이스 설계**: ROS2 `cmd_vel`(Twist) -> 좌우 궤도 속도 -> 모터/유압 명령의 3단계 변환 파이프라인을 명확히 정의해야 한다.
- 농업 현장에서 논밭 손상을 최소화하기 위해 **최소 선회 반경 제약**을 경로 계획에 반영하고, 피벗 선회 대신 완만한 곡선 선회를 선호하도록 비용함수를 설계한다.
- 군사용에서는 반대로 빠른 피벗 선회가 요구되므로, **운용 모드(농업/군사)에 따른 조향 파라미터 전환**이 필요하다.

---

## 4. 경로 추종 제어

### 4.1 핵심 개념

경로 추종(Path Tracking)은 사전에 계획된 경로(waypoints 또는 연속 곡선)를 차량이 정확히 따라가도록 하는 제어 문제이다. 궤도차량에서는 슬립과 비선형 조향 특성 때문에 일반 차량 대비 더 높은 난이도를 갖는다.

### 4.2 Pure Pursuit

#### 알고리즘 개요

기하학적(geometric) 경로 추종 방법으로, 차량 전방의 목표점(lookahead point)을 향해 원호(arc)를 그리며 주행하는 단순하고 직관적인 알고리즘이다.

```
곡률(curvature) = 2 * sin(alpha) / L_d

여기서:
  alpha = 목표점과 차량 방향 사이의 각도
  L_d = 전방 주시 거리 (lookahead distance)
```

궤도차량에서의 적용:
```
omega = 2 * v * sin(alpha) / L_d
v_R = v + omega * B / 2
v_L = v - omega * B / 2
```

#### 장점과 한계

- **장점**: 구현 간단, 계산량 적음, 튜닝 파라미터 1개(L_d)
- **한계**: 슬립을 고려하지 않으면 비포장에서 큰 추종 오차 발생, 곡률 변화가 큰 경로에서 성능 저하, 최적 L_d는 속도와 지형에 따라 변함

#### 슬립 보상 Pure Pursuit

최근 연구(2025)에서는 온라인 슬립 추정을 통해 Pure Pursuit의 성능을 개선한 사례가 보고되었다. 슬립 파라미터를 실시간으로 추정하여 lookahead 거리와 조향 명령을 보정하는 방식이다.

### 4.3 Stanley Method

#### 알고리즘 개요

Stanford 대학의 DARPA Grand Challenge 팀이 사용한 방법으로, **횡방향 오차(cross-track error)**와 **방향 오차(heading error)**를 동시에 고려한다:

```
delta = theta_e + arctan(k * e_ct / v)

여기서:
  theta_e = 방향 오차 (heading error)
  e_ct = 횡방향 오차 (cross-track error)
  k = 게인 파라미터
  v = 차량 속도
```

궤도차량 적용 시 `delta`(조향각)를 직접 사용할 수 없으므로, 요 레이트 명령으로 변환:
```
omega = v * delta / L_wb    (L_wb: 등가 휠베이스)
```

#### 장점과 한계

- **장점**: 횡방향 오차를 직접 최소화하므로 경로 수렴 특성이 좋음
- **한계**: Ackermann 기반 설계이므로 궤도차량 직접 적용에 변환 필요, 슬립 미고려 시 성능 저하

### 4.4 MPC (Model Predictive Control)

#### 알고리즘 개요

모델 예측 제어는 차량 모델을 사용하여 미래 상태를 예측하고, 비용함수를 최소화하는 최적 제어 입력 시퀀스를 반복적으로 계산한다.

```
최소화: J = sum_{k=0}^{N} [ Q * (x_k - x_ref_k)^2 + R * u_k^2 ]

제약 조건:
  x_{k+1} = f(x_k, u_k)          (차량 모델)
  u_min <= u_k <= u_max           (입력 제한)
  v_L_min <= v_L <= v_L_max       (좌측 궤도 속도 제한)
  v_R_min <= v_R <= v_R_max       (우측 궤도 속도 제한)
```

#### KAMPC (Kinematics-Aware MPC)

궤도차량 전용 MPC로, ICR 기반 슬립 키네마틱 모델을 예측 모델에 통합하여 슬립을 적극적으로 보상하는 방법이다:

```
예측 모델: x_{k+1} = f_ICR(x_k, v_L_k, v_R_k, ICR_params)
ICR 파라미터: EKF로 온라인 추정하여 MPC에 갱신
```

6개의 슬립 파라미터(좌우 ICR의 x, y 좌표 + 추가 보정 파라미터)를 실시간 추정하여 13.6톤 원격 제어 궤도차량에서 검증된 사례가 있다.

#### 비선형 MPC (NMPC)

슬립에 의한 강한 비선형성을 고려할 때, 선형 MPC보다 NMPC가 더 정확한 추종을 제공한다. 그러나 계산 비용이 크므로 실시간 구현을 위해 ACADOS, CasADi 등의 고속 최적화 솔버를 활용한다.

### 4.5 슬라이딩 모드 제어 (SMC) + 외란 관측기

#### 알고리즘 개요

슬립을 외란(disturbance)으로 간주하고, 외란 관측기(Disturbance Observer, DOB)로 추정한 후, 슬라이딩 모드 제어(SMC)로 강건하게 추종하는 방법이다.

```
슬라이딩 면: s = e_dot + lambda * e
제어 법칙: u = u_eq + u_sw - d_hat

여기서:
  e = 추종 오차
  u_eq = 등가 제어 입력
  u_sw = 스위칭 제어 입력 (채터링 방지를 위해 sigmoid 또는 boundary layer 적용)
  d_hat = 외란 관측기 추정값 (슬립에 의한 외란)
```

### 4.6 DRL 기반 적응형 제어

#### Deep RL + Pure Pursuit (DRAPP)

Deep Reinforcement Learning을 활용하여 Pure Pursuit의 lookahead 거리를 지형과 속도에 따라 적응적으로 조절하는 방법이다.

```
상태(State): [e_ct, theta_e, v, curvature, terrain_type, ...]
행동(Action): L_d (lookahead distance)
보상(Reward): -|e_ct| - alpha * |theta_e|  (추종 오차 최소화)
```

### 4.7 주요 논문 및 참고자료

| 저자 | 제목 | 연도 | 핵심 기여 |
|------|------|------|-----------|
| Coulter, R.C. | Implementation of the Pure Pursuit Path Tracking Algorithm (CMU-RI-TR-92-01) | 1992 | Pure Pursuit 알고리즘 원전 |
| Thrun et al. (Stanford) | Stanley: The Robot That Won the DARPA Grand Challenge | 2006 | Stanley 알고리즘 원전, 실제 자율주행 적용 |
| KAMPC 연구진 | [Kinematics-Aware MPC for Autonomous High-Speed Tracked Vehicles Under Off-Road Conditions](https://www.sciencedirect.com/science/article/abs/pii/S0888327019300068) | 2019 | ICR 기반 슬립 모델 + MPC 통합, 비포장 고속 궤도차량 적용 |
| PMC (2025) | [Performance Improvement of Pure Pursuit Algorithm via Online Slip Estimation for Off-Road Tracked Vehicle](https://pmc.ncbi.nlm.nih.gov/articles/PMC12300937/) | 2025 | 온라인 슬립 추정으로 Pure Pursuit 성능 개선, 비포장 궤도차량 실험 검증 |
| Dey, Sreenath | [Deep Reinforcement Learning Based Adaptation of Pure-Pursuit Path-Tracking for Skid-Steered Vehicles](https://www.sciencedirect.com/science/article/pii/S2405896322028592) | 2022 | DRL 기반 적응형 lookahead 거리, 스키드 스티어 차량 적용 |
| Liang et al. | [Deep RL for Autonomous Dynamic Skid Steer Vehicle Trajectory Tracking](https://www.mdpi.com/2218-6581/11/5/95) | 2022 | DDPG 기반 스키드 스티어 궤적 추종 제어 |
| MDPI Actuators | [Autonomous Tracked Vehicle Trajectory Tracking Control Based on Disturbance Observation and SMC](https://www.mdpi.com/2076-0825/14/2/51) | 2025 | 외란 관측기 + SMC 기반 궤도차량 경로 추종, 슬립 외란 추정 |
| MDPI Machines | [A Slip-Based MPC Approach for Trajectory Following of Unmanned Tracked Vehicles](https://www.mdpi.com/2075-1702/13/9/817) | 2025 | 슬립 기반 비선형 2-트랙 예측 모델 MPC |
| Snider, J.M. (CMU) | [Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf) | 2009 | Pure Pursuit, Stanley, MPC 등 자율 조향 방법 종합 비교 |

### 4.8 실제 적용 시 고려사항

- **알고리즘 선택 가이드**:
  - 빠른 프로토타이핑/저속 운용: **Pure Pursuit** (구현 용이, 튜닝 간단)
  - 중속/정밀 경로 추종: **MPC** (슬립 보상 통합 가능, 제약 조건 처리)
  - 고속/불확실 환경: **NMPC + 슬립 모델** 또는 **SMC + DOB** (강건성 우수)
  - 장기 적응: **DRL 기반** (환경 변화에 자동 적응)

- **계산 플랫폼**: MPC/NMPC는 ARM 기반 임베디드에서 실시간 구현이 도전적이다. ACADOS, CasADi 같은 코드 생성 도구 또는 GPU 가속을 고려해야 한다.

- **하이브리드 접근**: 저속에서는 Pure Pursuit, 고속에서는 MPC로 전환하는 하이브리드 전략이 실용적이다.

### 4.9 RS500 프로젝트 시사점

- **1단계 (개발 초기)**: Pure Pursuit로 시작하여 기본적인 경로 추종 프레임워크를 검증한다. 구현이 간단하고 디버깅이 용이하다.
- **2단계 (슬립 보상 통합)**: 온라인 슬립 추정 모듈을 추가하고, 이를 기반으로 Pure Pursuit의 파라미터를 적응적으로 조절한다. 비포장 농업 환경에서 성능 향상을 검증한다.
- **3단계 (고급 제어)**: MPC 기반 제어로 전환하여 슬립 모델을 예측 모델에 통합한다. 군사용 고속 기동 요구사항 충족을 위해 NMPC를 고려한다.
- **장기**: DRL 기반 적응형 파라미터 튜닝을 통해 다양한 지형에서의 자동 최적화를 목표로 한다.
- 13.6톤 궤도차량에서 검증된 KAMPC 사례는 1톤급 RS500에도 원리적으로 적용 가능하지만, 파라미터 튜닝 범위와 동역학 특성 차이에 대한 재식별이 필요하다.

---

## 5. ROS2 + Gazebo 시뮬레이션 사례

### 5.1 핵심 개념

ROS2 + Gazebo 환경에서 궤도차량을 시뮬레이션하는 것은 아직 차륜형 로봇 대비 지원이 제한적이다. 주요 접근 방식은 다음과 같다.

#### 접근 방식 1: Differential Drive 근사

궤도를 원통형 휠로 근사하고, `diff_drive_controller`(ros2_control)를 사용하는 방법이다.

```xml
<!-- URDF 예시 -->
<ros2_control name="tracked_vehicle" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="left_track_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_track_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

- **장점**: 구현 간단, ros2_control 생태계 활용 가능
- **한계**: 연속 궤도(continuous track)의 물리적 특성(접지 분포, 궤도 장력 등) 미반영

#### 접근 방식 2: Gazebo TrackedVehicle 시스템 플러그인

Gazebo(gz-sim)는 `TrackedVehicle` 시스템 플러그인을 내장하고 있다. 이 플러그인은 좌우 트랙 그룹에 대해 속도 명령을 받아 시뮬레이션한다.

```xml
<!-- SDF 예시 -->
<plugin filename="gz-sim-tracked-vehicle-system"
        name="gz::sim::systems::TrackedVehicle">
  <body_link>base_link</body_link>
  <left_track>
    <link>left_track</link>
    <velocity_topic>/left_track/cmd_vel</velocity_topic>
  </left_track>
  <right_track>
    <link>right_track</link>
    <velocity_topic>/right_track/cmd_vel</velocity_topic>
  </right_track>
</plugin>
```

#### 접근 방식 3: gazebo_ros_tracked_vehicle_interface

ROS와 Gazebo의 SimpleTrackedVehicle 플러그인을 연결하는 인터페이스로, `/cmd_vel` 토픽을 Gazebo 내부 토픽으로 전달하고 `/odom`을 발행한다.

#### 접근 방식 4: 멀티 휠 근사

궤도를 다수의 작은 휠로 근사하여 물리적으로 더 사실적인 접지 특성을 구현하는 방법이다. 각 휠에 개별 마찰 특성을 적용할 수 있다.

### 5.2 주요 참고자료 및 프로젝트

| 프로젝트/문서 | 링크 | 설명 |
|--------------|------|------|
| gazebo_ros_tracked_vehicle_interface | [GitHub](https://github.com/lprobsth/gazebo_ros_tracked_vehicle_interface) | ROS - Gazebo SimpleTrackedVehicle 연동 플러그인 |
| gz_ros2_control | [GitHub](https://github.com/ros-controls/gz_ros2_control) | 새 Gazebo(gz-sim) + ros2_control 통합 |
| Gazebo TrackedVehicle API | [Gazebo Docs](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1TrackedVehicle.html) | TrackedVehicle 시스템 플러그인 API 문서 |
| gazebo_continuous_track_example | [GitHub](https://github.com/yoshito-n-students/gazebo_continuous_track_example) | 연속 궤도 시각적 시뮬레이션 예제 (Gazebo Classic 7/9) |
| Gazebo Classic tracked_vehicle_simple.world | [GitHub](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/worlds/tracked_vehicle_simple.world) | Gazebo Classic 궤도차량 월드 파일 예제 |
| ROS2 + Gazebo Integration 가이드 | [Gazebo Docs](https://gazebosim.org/docs/latest/ros2_integration/) | 최신 Gazebo-ROS2 연동 가이드 |
| ROS2 Humble Gazebo 설정 | [ROS2 Docs](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html) | ROS2 Humble에서 Gazebo 시뮬레이션 설정 튜토리얼 |

### 5.3 실제 적용 시 고려사항

- **Gazebo Classic vs 새 Gazebo (gz-sim)**: Gazebo Classic 11은 2025년 1월부터 EOL이므로, 새 프로젝트는 **gz-sim (Harmonic LTS 이상)**을 사용해야 한다. 다만 궤도차량 관련 플러그인 생태계는 gz-sim에서 아직 성숙하지 않은 상태이다.
- **물리 엔진 선택**: gz-sim은 Bullet, DART, ODE 등 다중 물리 엔진을 지원한다. 궤도-지면 상호작용의 정밀도와 시뮬레이션 속도의 트레이드오프를 고려하여 선택한다.
- **지형 모델링**: 비포장 환경 시뮬레이션을 위해 DEM(Digital Elevation Model) 기반 지형, 마찰 계수 맵, 변형 가능 지형(deformable terrain) 플러그인 등을 고려한다. 현재 gz-sim의 변형 지형 지원은 제한적이다.
- **Sim-to-Real Gap**: 시뮬레이션의 슬립 특성은 실제와 크게 다를 수 있다. Domain randomization(마찰계수, 지형 특성 등을 무작위 변동)을 적용하여 실제 환경 전이 시 강건성을 확보한다.

### 5.4 RS500 프로젝트 시사점

- 현재 프로젝트의 `ad_simulation` 패키지가 이미 gz-sim 연동을 구현하고 있으므로, 이를 기반으로 궤도차량 모델을 확장한다.
- **권장 구현 순서**:
  1. Differential Drive 근사로 기본 시뮬레이션 구축 (빠른 프로토타이핑)
  2. `TrackedVehicle` 시스템 플러그인으로 전환 (궤도 특성 반영)
  3. 커스텀 슬립 모델 플러그인 개발 (정밀 시뮬레이션)
  4. 비포장 지형 월드 제작 (DEM 기반 또는 절차적 생성)
- **URDF/SDF 모델**: RS500의 물리적 치수, 질량, 관성 텐서를 정확히 반영한 모델 파일을 작성해야 한다. CAD 데이터가 있다면 이를 활용한다.
- `gz_ros2_control`을 사용하여 ros2_control 프레임워크와 통합하면, 시뮬레이션에서 검증한 제어기를 실차에 동일 인터페이스로 적용할 수 있다(Sim-to-Real 전이 용이).

---

## 6. RS500 프로젝트 종합 시사점

### 6.1 기술 스택 권장안

```
[센서 계층]
  GPS(RTK) + IMU + 엔코더 + LiDAR + 카메라
           |
[상태 추정 계층]
  EKF/UKF 기반 위치/속도/슬립 추정
           |
[경로 계획 계층]
  글로벌: A*/RRT* + 지형 비용맵
  로컬: DWA 또는 TEB (궤도차량 모델 통합)
           |
[경로 추종 계층]
  Phase 1: Pure Pursuit (슬립 보상)
  Phase 2: MPC (ICR 기반 슬립 모델 통합)
  Phase 3: NMPC + DRL 적응 (군사용 확장)
           |
[조향 제어 계층]
  cmd_vel -> (v_L, v_R) 변환 + 모터/유압 PID
           |
[액추에이터]
  좌측 궤도 모터/유압    우측 궤도 모터/유압
```

### 6.2 단계별 개발 로드맵

| 단계 | 목표 | 핵심 기술 | 예상 난이도 |
|------|------|-----------|------------|
| Phase 1 | 시뮬레이션 기본 주행 | Diff Drive 근사 + Pure Pursuit + Gazebo | 중 |
| Phase 2 | 비포장 슬립 보상 | ICR 기반 슬립 모델 + EKF 추정 + 적응형 Pure Pursuit | 상 |
| Phase 3 | MPC 기반 정밀 제어 | KAMPC + ACADOS 솔버 + 실시간 최적화 | 상 |
| Phase 4 | 실차 적용 | Sim-to-Real 전이 + 센서 퓨전 + 안전 시스템 | 최상 |
| Phase 5 | 군사용 확장 | 고속 NMPC + 지형 분류 + 강건 제어 | 최상 |

### 6.3 핵심 연구 과제

1. **RS500 동역학 파라미터 식별**: 차량 질량, 관성모멘트, 궤도-지면 마찰특성 실측
2. **지형 적응형 슬립 보상**: 다양한 토양/지형 조건에서의 실시간 슬립 모델 전환
3. **Sim-to-Real Gap 최소화**: 도메인 랜덤화 기반 시뮬레이션 학습, 실차 미세 조정
4. **안전 제약 통합**: 전복 방지(rollover prevention), 장애물 회피, 속도 제한의 MPC 제약 통합
5. **이중 용도(Dual-Use) 인터페이스**: 농업/군사 모드 전환 가능한 모듈화된 제어 아키텍처

### 6.4 참고할 유사 프로젝트

- **대동 RT100**: 한국 대동사가 개발한 농업용 운반 로봇, 자율주행 모델 2025년 출시 예정
- **NDIA GVSETS**: 미국 지상차량 시스템 공학 학회, 매년 군용 자율 지상차량 최신 연구 발표
- **ISTVS**: 국제 지형-차량 시스템 학회, 궤도차량 지반역학 관련 최신 연구 ([2025 학회](https://2025.istvs.org/submissions/abstracts))

---

## 참고문헌 전체 목록

### 동역학 모델링
1. Martinez, J.L. et al., "Approximating Kinematics for Tracked Mobile Robots," *IJRR*, 2005. [Link](https://journals.sagepub.com/doi/10.1177/0278364905058239)
2. Pentzer, J. et al., "Model-based Prediction of Skid-steer Robot Kinematics Using Online Estimation of Track ICRs," *J. Field Robotics*, 2014. [Link](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21509)
3. Lu, H. and Xiong, G., "Motion Predicting of Autonomous Tracked Vehicles with Online Slip Model Identification," *Mathematical Problems in Engineering*, 2016. [Link](https://onlinelibrary.wiley.com/doi/10.1155/2016/6375652)
4. "Dynamics of Tracked Vehicles During Nonuniform Turning," *Discover Applied Sciences*, 2024. [Link](https://link.springer.com/article/10.1007/s42452-024-06039-1)

### 슬립 모델링
5. Wong, J.Y., *Theory of Ground Vehicles*, 4th ed., Wiley, 2008.
6. Bekker, M.G., *Introduction to Terrain-Vehicle Systems*, Univ. of Michigan Press, 1969.
7. Laughery, S., "Bekker's Terramechanics Model for Off-road Vehicle Research," DTIC, 2005. [Link](https://apps.dtic.mil/sti/tr/pdf/ADA457955.pdf)
8. Endo, D. et al., "Path Following Control for Tracked Vehicles Based on Slip-Compensating Odometry," 2007. [Link](https://www.researchgate.net/publication/224296486)
9. Nagatani, K. et al., "Online Slip Parameter Estimation for Tracked Vehicle Odometry on Loose Slope," 2016. [Link](https://k-nagatani.org/pdf/2016-SSRR-GEN-online.pdf)
10. Kim et al., "A Survey of Off-Road Mobile Robots: Slippage Estimation, Robot Control, and Sensing Technology," *JIRS*, 2023. [Link](https://link.springer.com/article/10.1007/s10846-023-01968-2)
11. "Terramechanics Models Augmented by Machine Learning," *J. Terramechanics*, 2023. [Link](https://www.sciencedirect.com/science/article/abs/pii/S0022489823000198)

### 조향 알고리즘
12. Zhang et al., "A Control Method for the Differential Steering of Tracked Vehicles Driven by Dual Hydraulic Motor," *Applied Sciences*, 2022. [Link](https://www.mdpi.com/2076-3417/12/13/6355)
13. Chen et al., "Trajectory Tracking and Body Attitude Control of AGV Based on Differential Steering," *PLOS ONE*, 2023. [Link](https://pmc.ncbi.nlm.nih.gov/articles/PMC9907824/)
14. Hou et al., "Steering Control of Electric Tracked Vehicle Based on Second-Order Disturbance Observer," *J. Automobile Engineering*, 2025. [Link](https://journals.sagepub.com/doi/10.1177/09544070241246046)
15. "Drive Kinematics: Skid Steer & Mecanum," Robots For Roboticists. [Link](https://www.robotsforroboticists.com/drive-kinematics/)

### 경로 추종 제어
16. Coulter, R.C., "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01, 1992.
17. Thrun, S. et al., "Stanley: The Robot That Won the DARPA Grand Challenge," *J. Field Robotics*, 2006.
18. "Kinematics-Aware MPC for Autonomous High-Speed Tracked Vehicles Under Off-Road Conditions," *MSSP*, 2019. [Link](https://www.sciencedirect.com/science/article/abs/pii/S0888327019300068)
19. Dey, B. and Sreenath, K., "Deep RL Based Adaptation of Pure-Pursuit for Skid-Steered Vehicles," *IFAC-PapersOnLine*, 2022. [Link](https://www.sciencedirect.com/science/article/pii/S2405896322028592)
20. "Performance Improvement of Pure Pursuit via Online Slip Estimation for Off-Road Tracked Vehicle," PMC, 2025. [Link](https://pmc.ncbi.nlm.nih.gov/articles/PMC12300937/)
21. "Autonomous Tracked Vehicle Trajectory Tracking Based on DOB and SMC," *Actuators*, 2025. [Link](https://www.mdpi.com/2076-0825/14/2/51)
22. "A Slip-Based MPC Approach for Trajectory Following of Unmanned Tracked Vehicles," *Machines*, 2025. [Link](https://www.mdpi.com/2075-1702/13/9/817)
23. Snider, J.M., "Automatic Steering Methods for Autonomous Automobile Path Tracking," CMU, 2009. [Link](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

### ROS2 + Gazebo 시뮬레이션
24. gazebo_ros_tracked_vehicle_interface. [GitHub](https://github.com/lprobsth/gazebo_ros_tracked_vehicle_interface)
25. gz_ros2_control. [GitHub](https://github.com/ros-controls/gz_ros2_control)
26. Gazebo TrackedVehicle System Plugin. [API Docs](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1TrackedVehicle.html)
27. ROS2 Humble + Gazebo Integration. [ROS2 Docs](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

---

> 본 문서는 RS500 궤도차량 자율주행 프로젝트의 동역학 및 제어 분야 선행연구를 정리한 것이다.
> 각 분야의 최신 연구 동향(2024~2025)을 반영하였으며, 프로젝트 단계별 적용 전략을 포함한다.
