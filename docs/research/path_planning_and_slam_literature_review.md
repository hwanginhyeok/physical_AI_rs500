# 궤도차량 자율주행 경로 계획 및 SLAM 선행연구 보고서

**작성일**: 2026-02-21
**대상 플랫폼**: RS500 1톤급 궤도차량 (농업 -> 군사 확장)
**시뮬레이션 환경**: Gazebo gz-sim, ROS2

---

## 목차

1. [비포장/험지 환경 전역 경로 계획](#1-비포장험지-환경-전역-경로-계획)
2. [지역 경로 계획 및 장애물 회피](#2-지역-경로-계획-및-장애물-회피)
3. [농경지 특화 경로 패턴](#3-농경지-특화-경로-패턴)
4. [SLAM](#4-slam)
5. [ROS2 Nav2 프레임워크 활용](#5-ros2-nav2-프레임워크-활용)
6. [GPS-RTK + IMU 센서 퓨전 위치 추정](#6-gps-rtk--imu-센서-퓨전-위치-추정)
7. [RS500 프로젝트 종합 적용 권장사항](#7-rs500-프로젝트-종합-적용-권장사항)

---

## 1. 비포장/험지 환경 전역 경로 계획

### 1.1 A* 및 Hybrid A*

**핵심 개념**
A*는 그래프 기반 탐색 알고리즘으로, 휴리스틱 함수 h(n)을 사용하여 시작점에서 목표점까지의 최적 경로를 탐색한다. Hybrid A*는 이를 확장하여 차량의 운동학적 제약(최소 회전 반경, 전진/후진)을 반영한 연속 상태 공간(x, y, theta)에서 탐색을 수행한다.

**주요 참고자료**
- Hart, P.E., Nilsson, N.J., Raphael, B. "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," IEEE Trans. Systems Science and Cybernetics, 1968
- Dolgov, D., Thrun, S., Montemerlo, M., Diebel, J. "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments," International Journal of Robotics Research, 2010
- Macenski, S., et al. "Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics," arXiv:2401.13078, 2024

**장단점**

| 항목 | A* | Hybrid A* |
|------|-----|-----------|
| **장점** | 최적 경로 보장, 구현 단순, 계산 효율적 | 운동학적 실현 가능 경로 생성, Dubin/Reeds-Shepp 모델 지원 |
| **단점** | 이산 격자 기반으로 비홀로노믹 제약 미반영, 고차원에서 메모리 폭발 | A*보다 계산 비용 높음, 격자 해상도에 따른 경로 품질 의존 |

**RS500 적용 권장사항**
궤도차량은 제자리 회전(skid-steer)이 가능하므로 순수 A*도 유효하나, 실제 주행 시 궤도 슬립과 관성을 고려하면 Hybrid A*가 더 현실적인 경로를 생성한다. Nav2의 SmacPlannerHybrid 플러그인으로 즉시 적용 가능하다.

---

### 1.2 RRT* (Rapidly-exploring Random Tree Star)

**핵심 개념**
RRT*는 샘플링 기반 경로 계획 알고리즘으로, 무작위 샘플링을 통해 탐색 트리를 확장하면서 점근적 최적성(asymptotic optimality)을 보장한다. 비정형 환경에서 고차원 구성 공간 탐색에 유리하다.

**주요 참고자료**
- Karaman, S., Frazzoli, E. "Sampling-based Algorithms for Optimal Motion Planning," International Journal of Robotics Research, 2011
- Kuffner, J.J., LaValle, S.M. "RRT-connect: An Efficient Approach to Single-Query Path Planning," ICRA, 2000
- 하이브리드 접근: "Hybrid Terrain-Aware Path Planning: Integrating VD-RRT* Exploration and VD-D* Lite Repair," arXiv:2510.12169, 2025

**장단점**

| 장점 | 단점 |
|------|------|
| 고차원 공간에서 효과적 | 수렴 속도가 느림 (많은 샘플 필요) |
| 장애물이 복잡한 환경에서 강건 | 실시간 재계획 어려움 |
| 점근적 최적성 보장 | 경로 평활화 후처리 필요 |
| 운동학적 제약 통합 가능 (Kinodynamic RRT*) | 동적 환경 대응 한계 |

**RS500 적용 권장사항**
험지 환경에서 초기 전역 경로 탐색에 적합하다. 최근 VD-RRT* + VD-D* Lite 하이브리드 접근법이 비정형 지형에서 밀리초 단위 재계획을 달성하여, RS500의 군사 확장 시나리오(미지 지형 탐색)에 유망하다.

---

### 1.3 D* Lite

**핵심 개념**
D* Lite는 증분(incremental) 탐색 알고리즘으로, 환경이 동적으로 변할 때 처음부터 재계획하지 않고 이전 탐색 결과를 재활용하여 빠르게 경로를 수정한다. 목표점에서 시작점 방향으로 역방향 탐색을 수행한다.

**주요 참고자료**
- Koenig, S., Likhachev, M. "D* Lite," AAAI, 2002
- 개선 연구: "Improvement and Fusion of D*Lite Algorithm and Dynamic Window Approach for Path Planning in Complex Environments," Machines, 2024
- 하이브리드: "Path Planning for Intelligent Vehicles Based on Improved D* Lite," J. Supercomputing, 2023

**장단점**

| 장점 | 단점 |
|------|------|
| 동적 환경에서 빠른 재계획 | 격자 기반이므로 해상도에 의존 |
| 센서 데이터 실시간 반영 | 비홀로노믹 제약 직접 반영 불가 |
| LPA* 기반으로 계산 효율적 | 3D 지형 비용 통합 시 복잡도 증가 |

**RS500 적용 권장사항**
농경지/군사 환경 모두에서 동적 장애물(사람, 동물, 차량 등)에 대한 실시간 경로 재계획에 적합하다. D* Lite + DWA 융합 연구(2024)가 복합 환경에서 유효성을 입증하였다.

---

### 1.4 State Lattice Planner

**핵심 개념**
State Lattice Planner는 사전에 계산된 운동학적으로 실현 가능한 모션 프리미티브(motion primitives) 집합을 격자(lattice) 위에 배치하고, 이를 그래프 탐색하여 최적 경로를 생성한다. 차량의 물리적 제약을 정확히 반영한다.

**주요 참고자료**
- Pivtoraiko, M., Knepper, R.A., Kelly, A. "Differentially Constrained Mobile Robot Motion Planning in State Lattices," Journal of Field Robotics, 2009
- McNaughton, M., et al. "Motion Planning for Autonomous Driving with a Conformal Spatiotemporal Lattice," ICRA, 2011
- Nav2 구현: Macenski, S., et al. "Smac State Lattice Planner," Nav2 Documentation, 2024

**장단점**

| 장점 | 단점 |
|------|------|
| 운동학적 실현 가능성 보장 | 모션 프리미티브 사전 생성 필요 |
| 지형 비용 통합 용이 | 프리미티브 해상도에 따른 경로 품질 변화 |
| Nav2에서 직접 지원 | 고차원 상태 공간에서 계산 비용 증가 |
| 다양한 차량 유형 적용 가능 | 궤도차량 전용 프리미티브 커스텀 필요 |

**RS500 적용 권장사항**
Nav2의 SmacPlannerLattice를 통해 RS500의 skid-steer 운동학 모델에 맞춤 모션 프리미티브를 생성하면 가장 정확한 전역 경로를 얻을 수 있다. 지형 통행 가능성(traversability) 비용맵과 결합이 자연스럽다.

---

### 1.5 전역 경로 계획 알고리즘 비교 종합

| 알고리즘 | 최적성 | 운동학 반영 | 동적 환경 | 계산 비용 | Nav2 지원 | RS500 추천도 |
|----------|--------|------------|-----------|-----------|-----------|-------------|
| A* | 최적 | X | X | 낮음 | O (Smac2D) | ** |
| Hybrid A* | 준최적 | O | X | 중간 | O (SmacHybrid) | **** |
| RRT* | 점근 최적 | 확장 가능 | X | 높음 | X | ** |
| D* Lite | 최적 | X | O | 중간 | X (커스텀) | *** |
| State Lattice | 준최적 | O | X | 중간~높음 | O (SmacLattice) | ***** |

---

## 2. 지역 경로 계획 및 장애물 회피

### 2.1 DWA (Dynamic Window Approach)

**핵심 개념**
DWA는 로봇의 속도 공간(v, omega)에서 동적 윈도우(현재 속도로부터 가속/감속 제약 내에서 도달 가능한 속도 범위)를 정의하고, 각 후보 속도에 대해 시뮬레이션된 궤적을 평가하여 최적의 제어 명령을 선택한다.

**주요 참고자료**
- Fox, D., Burgard, W., Thrun, S. "The Dynamic Window Approach to Collision Avoidance," IEEE Robotics & Automation Magazine, 1997
- Nav2 DWB 구현: Nav2 DWB Controller Plugin Documentation
- 개선: "Research on Autonomous Navigation of Mobile Robots Based on IA-DWA Algorithm," Scientific Reports, 2024

**장단점**

| 장점 | 단점 |
|------|------|
| 계산 비용 낮음 (실시간 100Hz+) | 좁은 통로에서 제한적 |
| 차량 동적 제약 직접 반영 | 전방 예측 호라이즌 짧음 |
| 구현 및 튜닝 간단 | 동적 장애물 대응 한계 |
| skid-steer에 자연스럽게 적용 | 지역 최소값(local minima) 빠질 수 있음 |

**RS500 적용 권장사항**
RS500의 differential-drive 근사 모델에 잘 맞으며, 저속(3 m/s 이하) 농경지 주행에서 충분한 성능을 보인다. 다만 장애물이 복잡한 환경에서는 TEB 또는 MPPI로 전환을 고려해야 한다.

---

### 2.2 TEB (Timed Elastic Band)

**핵심 개념**
TEB는 경로를 시간 정보가 포함된 탄성 띠(elastic band)로 모델링하고, 다목적 최적화를 통해 경로 길이, 실행 시간, 장애물 거리, 운동학적 제약을 동시에 최적화한다. 경로의 위상적(topological) 탐색을 통해 여러 후보 경로를 동시에 평가한다.

**주요 참고자료**
- Rosmann, C., Feiten, W., Wosch, T., Hoffmann, F., Bertram, T. "Trajectory Modification Considering Dynamic Constraints of Autonomous Robots," German Conference on Robotics, 2012
- Rosmann, C., Hoffmann, F., Bertram, T. "Integrated Online Trajectory Planning and Optimization in Distinctive Topologies," Robotics and Autonomous Systems, 2017
- 비교 연구: "Performance Comparison Analysis of TEB and DWA Algorithms," ResearchGate, 2024

**장단점**

| 장점 | 단점 |
|------|------|
| 시간 최적 경로 생성 | 계산 비용 DWA보다 높음 |
| 동적 장애물 고려 가능 | 파라미터 튜닝 복잡 |
| 다중 후보 경로 탐색 | 고속에서 불안정해질 수 있음 |
| 비홀로노믹 제약 반영 | 궤도차량 모델 커스터마이징 필요 |

**RS500 적용 권장사항**
농경지 고랑 사이 좁은 통로 주행이나 장애물 밀집 환경에서 DWA보다 우수하다. 다만 RS500의 skid-steer 모델을 TEB에 정확히 반영하려면 추가 커스터마이징이 필요하다.

---

### 2.3 MPPI (Model Predictive Path Integral)

**핵심 개념**
MPPI는 MPC의 변형으로, 가우시안 분포에서 수천 개의 제어 입력 시퀀스를 무작위 샘플링하고, 각 샘플의 비용을 평가한 후 가중 평균으로 최적 제어를 계산한다. 비선형 시스템에 적합하며 비용 함수의 미분 가능성을 요구하지 않는다.

**주요 참고자료**
- Williams, G., Aldrich, A., Theodorou, E.A. "Model Predictive Path Integral Control: From Theory to Parallel Computation," Journal of Guidance, Control, and Dynamics, 2017
- Nav2 MPPI Controller: Macenski, S., et al. Nav2 MPPI Controller Documentation, 2023
- 험지 적용: Xiao, X., et al. "MPPI-Based Aggressive Navigation Over Rough Terrain," IEEE RA-L, 2022

**장단점**

| 장점 | 단점 |
|------|------|
| 비선형 동역학 모델 직접 사용 가능 | 비용 함수 설계에 전문성 필요 |
| 병렬 연산으로 실시간 가능 (100Hz+) | GPU 가속 시 효과 극대화 |
| Nav2에 플러그인으로 직접 통합 | 파라미터(샘플 수, 노이즈, 온도) 튜닝 민감 |
| 플러그인 기반 비용 함수 확장 | 궤도 슬립 모델 통합 시 추가 개발 필요 |
| Differential, Ackermann, Omni 지원 | |

**RS500 적용 권장사항**
MPPI는 RS500 프로젝트에 가장 강력히 추천하는 지역 경로 계획기이다. 이유는 다음과 같다:
1. 궤도차량의 비선형 슬립 동역학을 비용 함수에 직접 반영 가능
2. Nav2에서 공식 지원하며, differential-drive 모델로 즉시 적용 가능
3. 농경지 지형 비용(경사, 토질)을 커스텀 critic 플러그인으로 추가 가능
4. 군사 확장 시 공격적 기동(aggressive maneuver) 계획에 유리

---

### 2.4 MPC (Model Predictive Control) 기반 접근

**핵심 개념**
전통적 MPC는 시스템 모델을 사용하여 유한 예측 호라이즌(prediction horizon)에 걸쳐 비용 함수를 최적화하고, 첫 번째 제어 입력만 적용한 뒤 다음 스텝에서 반복한다. QP(Quadratic Programming) 또는 NLP(Nonlinear Programming) 솔버를 사용한다.

**주요 참고자료**
- Borrelli, F., Bemporad, A., Morari, M. "Predictive Control for Linear and Hybrid Systems," Cambridge University Press, 2017
- 계층적 접근: "Autonomous Navigation of Mobile Robots: A Hierarchical Planning-Control Framework with Integrated DWA and MPC," PMC, 2025
- 비포장: Krusi, P., et al. "Driving on Point Clouds: MPC for Autonomous Driving on Rough Terrain," IROS, 2015

**장단점**

| 장점 | 단점 |
|------|------|
| 정밀한 제약 조건 처리 | NLP 솔버 계산 비용 높음 |
| 다변수 시스템 동시 제어 | 모델 정확도에 크게 의존 |
| 예측 기반으로 선제적 대응 | 실시간성 확보 어려울 수 있음 |
| 궤도차량 동역학 모델 정밀 반영 가능 | 구현 복잡도 높음 |

**RS500 적용 권장사항**
정밀 경로 추종(trajectory tracking)에는 MPC가 이상적이나, 장애물 회피와 결합 시 계산 부담이 크다. DWA/MPPI로 장애물 회피를 수행하고, 생성된 경로에 대한 저수준 추종 제어로 MPC를 사용하는 계층적 구조를 권장한다.

---

### 2.5 지역 경로 계획기 비교 종합

| 알고리즘 | 장애물 회피 | 동적 환경 | 계산 비용 | 비선형 모델 | Nav2 지원 | RS500 추천도 |
|----------|-----------|-----------|-----------|------------|-----------|-------------|
| DWA | 보통 | 제한적 | 매우 낮음 | 제한적 | O (DWB) | *** |
| TEB | 우수 | 양호 | 중간 | 제한적 | O | *** |
| MPPI | 우수 | 양호 | 중간 | O | O | ***** |
| MPC (NLP) | 우수 | 우수 | 높음 | O | X (커스텀) | *** |

---

## 3. 농경지 특화 경로 패턴

### 3.1 커버리지 경로 계획 (Coverage Path Planning, CPP)

**핵심 개념**
커버리지 경로 계획은 주어진 영역의 모든 지점을 최소한 한 번 방문하는 경로를 생성하는 문제이다. 농업에서는 밭 전체에 대한 파종, 시비, 방제, 수확 작업을 수행할 때 핵심적으로 필요하다. 주요 분해(decomposition) 기법으로 영역을 단순 셀로 분할한 뒤, 각 셀 내에서 왕복(back-and-forth) 패턴으로 커버한다.

**주요 참고자료**
- Galceran, E., Carreras, M. "A Survey on Coverage Path Planning for Robotics," Robotics and Autonomous Systems, 2013
- Hoffmann, L., et al. "Optimal Guidance Track Generation for Precision Agriculture: A Review of Coverage Path Planning Techniques," Journal of Field Robotics, 2024
- Pour Arab, A., et al. "3D Hybrid Path Planning for Optimized Coverage of Agricultural Fields," Journal of Field Robotics, 2025

**RS500 적용 권장사항**
농경지 작업에서 필수적인 알고리즘이다. 아래 구체적 패턴들과 함께 Fields2Cover 라이브러리 활용을 강력 권장한다.

---

### 3.2 Boustrophedon 분해 및 경로

**핵심 개념**
"소가 밭을 가는 방식"에서 유래한 Boustrophedon 패턴은 영역을 수직선(sweepline)으로 분해하여 단조(monotone) 셀을 생성하고, 각 셀 내에서 왕복 직선 주행으로 전체 영역을 커버한다. 비볼록(non-convex) 영역에서도 분해를 통해 적용 가능하다.

**주요 참고자료**
- Choset, H. "Coverage of Known Spaces: The Boustrophedon Cellular Decomposition," Autonomous Robots, 2000
- 개선: "A Boustrophedon-Optimized Neural Network for Autonomous Path Planning in Large-Scale Photovoltaic Farms," Journal of Field Robotics, 2025
- 다중 로봇: "Resilient Multi-Robot Coverage Path Redistribution Using Boustrophedon Decomposition for Environmental Monitoring," PMC, 2024

**장단점**

| 장점 | 단점 |
|------|------|
| 직관적이고 구현 간단 | 비볼록 영역에서 분해 품질에 의존 |
| 직선 주행 위주로 궤도차량에 적합 | 회전(headland turn) 효율 최적화 필요 |
| 커버리지 완전성 보장 | 장애물 밀집 환경에서 비효율적 |
| 트랙 간격 균일 제어 용이 | 경사지 고려 시 추가 최적화 필요 |

**RS500 적용 권장사항**
RS500의 주요 농업 작업 패턴으로 Boustrophedon을 기본 채택한다. 궤도차량은 제자리 회전이 가능하므로 headland turn 효율이 바퀴 차량보다 유리하다. 밭의 형상에 따라 최적 주행 방향(sweeping direction)을 자동 결정하는 알고리즘을 포함해야 한다.

---

### 3.3 고랑 따라가기 (Furrow Following)

**핵심 개념**
농경지의 기존 고랑(furrow) 또는 작물 열(crop row)을 따라가는 경로 추종 기법이다. LiDAR 또는 카메라로 고랑/작물 열을 실시간 감지하고, 이를 추종하는 경로를 생성한다. 전역 경로 계획 없이 반응적(reactive) 주행이 가능하다.

**주요 참고자료**
- Astolfi, A., Bolognani, S. "Robust Furrow Following for Agricultural Robots," Autonomous Robots, 2003
- 최신: "Application of Navigation Path Planning and Trajectory Tracking Control Methods for Agricultural Robots," Agriculture, 2024
- 비전 기반: Winterhalter, W., et al. "Crop Row Detection on Tiny Plants with the Pattern Hough Transform," IEEE RA-L, 2018

**장단점**

| 장점 | 단점 |
|------|------|
| 사전 맵 불필요 | 고랑이 없는 환경에서 사용 불가 |
| 센서 기반 실시간 적응 | 작물 성장 단계에 따른 감지 난이도 변화 |
| 궤도차량의 직진 안정성과 시너지 | 끝단(headland) 전환 로직 별도 필요 |
| GPS 불필요 (로컬 센서 의존) | 긴 직선에서 누적 오차 가능 |

**RS500 적용 권장사항**
고랑 따라가기는 RS500의 전방 카메라(/sensor/camera/front)와 LiDAR(/sensor/lidar)를 조합하여 구현 가능하다. 작물 열 감지에는 딥러닝 기반 시맨틱 세그먼테이션이 효과적이며, 감지된 열에 대한 추종은 PID 또는 Pure Pursuit 제어기로 구현한다.

---

### 3.4 Fields2Cover 라이브러리

**핵심 개념**
Fields2Cover는 농업 자율주행 차량을 위한 오픈소스 커버리지 경로 계획 라이브러리이다. 모듈형 아키텍처로 headland 생성, swath 생성, route 계획, path 계획의 4단계 파이프라인을 제공한다. C++17로 구현되었으며 Python 바인딩과 ROS2 인터페이스를 지원한다.

**주요 참고자료**
- Mier, G., Valente, J., de Bruin, S. "Fields2Cover: An Open-Source Coverage Path Planning Library for Unmanned Agricultural Vehicles," IEEE RA-L, 2023
- GitHub: https://github.com/Fields2Cover/Fields2Cover
- ROS2 패키지: fields2cover_ros2 (ROS2 Rolling/Iron 지원)

**핵심 기능**
- Headland 생성: 필드 경계로부터 headland 영역 자동 생성
- Swath 생성: 최적 주행 방향 결정 및 swath 패턴 생성 (8가지 알고리즘)
- Route 계획: swath 간 연결 순서 최적화 (7가지 목적 함수)
- Path 계획: Dubins, Reeds-Shepp 곡선 지원
- v2.0: 비볼록 필드 및 장애물 처리 지원

**RS500 적용 권장사항**
Fields2Cover를 RS500 프로젝트의 농업 커버리지 경로 계획의 핵심 라이브러리로 채택할 것을 강력 권장한다. 이유:
1. ROS2 네이티브 통합 (fields2cover_ros2 패키지)
2. 궤도차량의 Reeds-Shepp 곡선으로 제자리 회전 반영
3. 비볼록 밭 형상 처리 지원 (v2.0)
4. RVIZ 시각화 지원으로 Gazebo 시뮬레이션과 연계 용이
5. BSD-3 라이센스로 군사 확장 시 라이센스 제약 없음

---

## 4. SLAM (Simultaneous Localization and Mapping)

### 4.1 LiDAR SLAM

#### 4.1.1 FAST-LIO2

**핵심 개념**
FAST-LIO2는 직접(direct) LiDAR-관성 오도메트리 프레임워크로, 특징점 추출 없이 원시 포인트 클라우드를 직접 사용하여 iKD-Tree 기반 맵 관리와 반복적 오차 상태 칼만 필터(iterated Error-State Kalman Filter)를 결합한다.

**주요 참고자료**
- Xu, W., Zhang, F. "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry," IEEE TRO, 2022
- 확장: "FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry," arXiv:2408.14035, 2024
- 비교: engcang/SLAM-application (GitHub, 다중 SLAM 비교 리포지토리)

**장단점**

| 장점 | 단점 |
|------|------|
| 특징점 추출 불필요 (직접 방법) | 루프 클로저 미지원 (오도메트리 전용) |
| 계산 효율 매우 높음 (임베디드 가능) | 장거리 주행 시 드리프트 누적 |
| 퇴화 환경(터널 등)에서 강건 | IMU 품질에 민감 |
| iKD-Tree로 효율적 맵 관리 | ROS2 포팅 필요 (공식은 ROS1) |

#### 4.1.2 LIO-SAM

**핵심 개념**
LIO-SAM은 LiDAR-관성 오도메트리를 인수 그래프(factor graph) 최적화 프레임워크 위에 구축하여, IMU 사전분포, LiDAR 오도메트리, GPS, 루프 클로저를 하나의 그래프에서 동시 최적화한다.

**주요 참고자료**
- Shan, T., Englot, B., Meyers, D., Wang, W., Ratti, C., Rus, D. "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping," IROS, 2020
- 3D 야외: "Research on 3D LiDAR Outdoor SLAM Algorithm Based on LiDAR/IMU Tight Coupling," Scientific Reports, 2025
- 농업: "Map Construction and Positioning Method for LiDAR SLAM-Based Navigation of Agricultural Field Inspection Robot," Agronomy, 2024

**장단점**

| 장점 | 단점 |
|------|------|
| 루프 클로저 지원 (드리프트 교정) | FAST-LIO2보다 계산 비용 높음 |
| GPS 팩터 통합 가능 | 특징이 적은 환경에서 성능 저하 |
| 인수 그래프로 확장성 우수 | 9축 IMU 필수 |
| 대규모 맵핑에 적합 | 장거리에서 드리프트 관찰됨 |

#### 4.1.3 Voxel-SLAM (2024, 최신)

**주요 참고자료**
- "Voxel-SLAM: A Complete, Accurate, and Versatile LiDAR-Inertial SLAM System," arXiv:2410.08935, 2024

완전한 SLAM 파이프라인(오도메트리 + 맵핑 + 루프 클로저 + 재위치 추정)을 제공하는 최신 프레임워크.

---

### 4.2 Visual SLAM

#### 4.2.1 ORB-SLAM3

**핵심 개념**
ORB-SLAM3는 특징점 기반 Visual-Inertial SLAM으로, 단안/스테레오/RGB-D 카메라 + IMU를 지원하며, 다중 맵(multi-map) 시스템과 Atlas 구조를 통해 트래킹 실패 후에도 이전 맵과 병합 가능하다.

**주요 참고자료**
- Campos, C., Elvira, R., Rodriguez, J.J.G., Montiel, J.M.M., Tardos, J.D. "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multimap SLAM," IEEE TRO, 2021
- 야외 적응: "An Adaptive ORB-SLAM3 System for Outdoor Dynamic Environments," PMC, 2023
- 벤치마크: "Visual-Inertial SLAM for Unstructured Outdoor Environments: Benchmarking the Benefits and Computational Costs of Loop Closing," Journal of Field Robotics, 2025

#### 4.2.2 VINS-Fusion

**핵심 개념**
VINS-Fusion은 스테레오 카메라 + IMU를 타이트 커플링(tight coupling)하여 VIO(Visual-Inertial Odometry)를 수행하며, GPS 통합과 루프 클로저를 지원한다.

**주요 참고자료**
- Qin, T., Li, P., Shen, S. "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator," IEEE TRO, 2018
- 딥러닝 확장: "SuperVINS: A Visual-Inertial SLAM Framework Integrated Deep Learning Features," arXiv:2407.21348, 2024

**Visual SLAM 장단점 비교**

| 항목 | ORB-SLAM3 | VINS-Fusion |
|------|-----------|-------------|
| 센서 | 단안/스테레오/RGB-D + IMU | 스테레오 + IMU + GPS |
| 루프 클로저 | O (DBoW2) | O |
| 다중 맵 | O (Atlas) | X |
| 야외 강건성 | 조명 변화에 민감 | GPS 통합으로 보완 |
| 계산 비용 | 중간 | 중간 |
| 실시간성 | O | O |

---

### 4.3 GPS-Denied 환경 대응 전략

**핵심 개념**
GPS 신호가 차단되는 환경(수목 아래, 건물 사이, 실내, 전자전 환경)에서의 위치 추정은 SLAM과 관성 항법, 비전 기반 오도메트리의 융합에 의존한다.

**주요 참고자료**
- "GPS-Denied LiDAR-Based SLAM -- A Survey," IET Cyber-Systems and Robotics, 2025 (Jiang et al.)
- "Robust 2D LiDAR-Based SLAM in Arboreal Environments Without IMU/GNSS," arXiv:2505.10847, 2025
- "Real-Time Localization and Mapping Utilizing Multi-Sensor Fusion and Visual-IMU-Wheel Odometry for Agricultural Robots in GPS-Denied Greenhouse Environments," Agronomy, 2022

**GPS-Denied 대응 기법 요약**

| 기법 | 정밀도 | 계산 비용 | 드리프트 | 환경 제약 |
|------|--------|-----------|---------|-----------|
| LiDAR SLAM (FAST-LIO2) | 높음 | 중간 | 낮음 | 구조물/지형 특징 필요 |
| Visual SLAM (ORB-SLAM3) | 중간 | 중간 | 중간 | 조명/텍스처 의존 |
| VIO (VINS-Fusion) | 중간 | 낮음 | 중간 | IMU 품질 의존 |
| LiDAR-Visual 융합 (FAST-LIVO2) | 매우 높음 | 높음 | 매우 낮음 | 복합 센서 필요 |
| 휠 오도메트리 + IMU | 낮음 | 매우 낮음 | 높음 | 슬립에 취약 |

**RS500 적용 권장사항 (SLAM 종합)**
1. **1차 권장**: LIO-SAM -- RS500에 이미 LiDAR, IMU, GPS가 장착되어 있으며, LIO-SAM은 이 세 센서를 인수 그래프에서 동시 최적화한다. GPS 가용 시 드리프트 교정, GPS 불가 시 LiDAR-IMU로 자율 위치 추정이 가능하다.
2. **보조 시스템**: ORB-SLAM3 또는 VINS-Fusion을 전방 카메라(/sensor/camera/front)로 구동하여 LiDAR SLAM과 교차 검증한다.
3. **군사 확장**: GPS-denied 환경(전자전)을 대비하여 FAST-LIO2를 fallback 오도메트리로 구성하고, 루프 클로저는 위치 태그(place recognition)로 보완한다.
4. **Gazebo 시뮬레이션**: engcang/SLAM-application 리포지토리에서 Gazebo 상의 다중 SLAM 알고리즘 비교 환경을 참조할 수 있다.

---

## 5. ROS2 Nav2 프레임워크 활용

### 5.1 Nav2 아키텍처 개요

**핵심 개념**
Nav2는 ROS2 기반의 프로덕션급 내비게이션 프레임워크로, Behavior Tree(BT) 기반 태스크 스케줄링, 플러그인 아키텍처, 라이프사이클 노드 관리를 제공한다.

**주요 참고자료**
- Macenski, S., Foote, T., Gerkey, B., Lalancette, C., Woodall, W. "Robot Operating System 2: Design, Architecture, and Uses in the Wild," Science Robotics, 2022
- Macenski, S., et al. "Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics," arXiv:2401.13078, 2024
- 공식 문서: https://docs.nav2.org/
- GitHub: https://github.com/ros-navigation/navigation2

**Nav2 핵심 컴포넌트**

```
┌────────────────────────────────────────────────────┐
│                  Behavior Tree (BT)                │
│          (NavigateToPose / NavigateThroughPoses)    │
├──────────────┬──────────────┬──────────────────────┤
│   Planner    │  Controller  │    Recovery          │
│   Server     │   Server     │    Server            │
│ ┌──────────┐ │ ┌──────────┐ │ ┌──────────────────┐ │
│ │SmacHybrid│ │ │  MPPI    │ │ │  Spin / BackUp   │ │
│ │SmacLattice│ │ │  DWB     │ │ │  Wait / Assisted │ │
│ │NavFn     │ │ │  TEB     │ │ │  Teleop          │ │
│ └──────────┘ │ └──────────┘ │ └──────────────────┘ │
├──────────────┴──────────────┴──────────────────────┤
│              Costmap 2D (Global + Local)           │
│     ┌──────────┬──────────┬──────────────────┐     │
│     │ Static   │ Obstacle │ Inflation / Voxel│     │
│     │ Layer    │ Layer    │ Layer            │     │
│     └──────────┴──────────┴──────────────────┘     │
├────────────────────────────────────────────────────┤
│        AMCL / SLAM Toolbox / robot_localization    │
└────────────────────────────────────────────────────┘
```

### 5.2 궤도차량에 Nav2 적용 시 고려사항

**운동학 모델 설정**
궤도차량(skid-steer)은 Nav2에서 `differential` 로봇 유형으로 설정한다. cmd_vel 토픽을 통해 linear.x (전후진)와 angular.z (회전)를 제어한다 -- 이는 현재 RS500 프로젝트의 bridge_config.yaml에서 이미 구성된 `/cmd_vel` Twist 메시지와 정확히 일치한다.

**권장 플러그인 구성**

| 컴포넌트 | 1차 권장 플러그인 | 대안 | 근거 |
|----------|-------------------|------|------|
| Global Planner | SmacPlannerLattice | SmacPlannerHybrid | 궤도차량 운동학 프리미티브 반영 |
| Local Controller | MPPI | DWB | 비선형 동역학, 지형 비용 반영 |
| Costmap | Voxel Layer + Obstacle Layer | - | 3D LiDAR 데이터 활용 |
| Localization | SLAM Toolbox + robot_localization | AMCL | GPS+IMU+LiDAR 융합 |
| Recovery | Spin, BackUp, Wait | - | 궤도차량 제자리 회전 활용 |
| Behavior Tree | NavigateThroughPoses | NavigateToPose | 커버리지 경로의 다중 waypoint 지원 |

### 5.3 Nav2 기반 궤도차량 적용 사례

**주요 참고자료**
- "ROS-Based Navigation and Obstacle Avoidance: A Study of Architectures, Methods, and Trends," PMC, 2025
- "Mastering ROS 2's New Navigation 2 (Nav2) Library for Accurate Robot Localization," Poespas Blog, 2025
- Foxglove: "Autonomous Robot Navigation and Nav2: The First Steps," 2024

**RS500 프로젝트에 대한 Nav2 통합 로드맵**

```
Phase 1: 기본 내비게이션 (현재 -> 3개월)
├── Nav2 Bringup with differential-drive 설정
├── Static map + AMCL 위치 추정
├── SmacPlannerHybrid + DWB 기본 구성
└── Gazebo 시뮬레이션에서 point-to-point 내비게이션 검증

Phase 2: 센서 융합 및 SLAM (3~6개월)
├── robot_localization (dual-EKF) 통합
├── GPS-RTK + IMU + Wheel Odometry 융합
├── SLAM Toolbox 또는 LIO-SAM 연동
└── 동적 장애물 회피 (MPPI 전환)

Phase 3: 농업 커버리지 (6~9개월)
├── Fields2Cover 연동
├── Boustrophedon 커버리지 경로 생성
├── NavigateThroughPoses BT 활용
└── 고랑 따라가기 모드 통합

Phase 4: 군사 확장 (9~12개월)
├── GPS-denied SLAM fallback (FAST-LIO2)
├── 지형 통행 가능성 분석 costmap 레이어
├── 공격적 기동 MPPI 파라미터 프로파일
└── 다중 차량 협조 내비게이션
```

---

## 6. GPS-RTK + IMU 센서 퓨전 위치 추정

### 6.1 핵심 개념

GPS-RTK(Real-Time Kinematic)는 기준국(base station)으로부터의 보정 데이터를 활용하여 센티미터 수준의 위치 정밀도를 달성한다. 그러나 GPS 단독으로는:
- 업데이트 레이트가 낮음 (1~20 Hz)
- 일시적 신호 차단에 취약
- 방향(heading) 정보가 부족 (단일 안테나 시)

이를 IMU(Inertial Measurement Unit)와 융합하면:
- 고주파 자세/가속도 데이터 (100~400 Hz)
- GPS 중단 시 관성 항법으로 브릿징
- 정확한 heading 추정

### 6.2 센서 퓨전 아키텍처

#### Loosely-Coupled (느슨한 결합)

```
GPS-RTK ──→ [위치] ──→ ┌──────────┐
                       │   EKF    │ ──→ 융합된 상태 추정
IMU    ──→ [자세/가속] ──→ │          │     (x, y, z, roll, pitch, yaw,
                       │          │      vx, vy, vz, wx, wy, wz)
Wheel Odom ──→ [속도] ──→ └──────────┘
```

각 센서의 출력을 독립적으로 처리한 후 칼만 필터로 융합한다.

#### Tightly-Coupled (밀접한 결합)

```
GPS Raw (의사거리, 반송파) ──→ ┌──────────────┐
                              │ ESKF / FGO   │ ──→ 융합된 상태 추정
IMU Raw (가속도, 각속도)   ──→ │              │
                              │              │
LiDAR/Vision Odometry     ──→ └──────────────┘
```

센서의 원시 측정값을 직접 사용하여 더 정확한 추정을 달성한다.

### 6.3 ROS2 robot_localization 패키지

**핵심 개념**
robot_localization은 ROS2에서 가장 널리 사용되는 센서 퓨전 패키지로, EKF(Extended Kalman Filter)와 UKF(Unscented Kalman Filter)를 제공한다.

**주요 참고자료**
- Moore, T., Stouch, D. "A Generalized Extended Kalman Filter Implementation for the Robot Operating System," Intelligent Autonomous Systems, 2016
- ROS2 패키지: https://index.ros.org/p/robot_localization/
- 튜토리얼: "Sensor Fusion and Robot Localization Using ROS 2 Jazzy," AutomaticAddison, 2024

**권장 아키텍처: Dual-EKF 구성**

```yaml
# EKF 1: Local Odometry (odom frame)
# - IMU (orientation, angular velocity, linear acceleration)
# - Wheel Odometry (position, velocity)
# - 출력: odom -> base_link TF
ekf_local:
  frequency: 50.0
  sensor_timeout: 0.1
  two_d_mode: true  # 평지 농경지에서는 2D 모드
  odom0: /odom
  odom0_config: [true, true, false,   # x, y, z
                 false, false, true,   # roll, pitch, yaw
                 true, true, false,    # vx, vy, vz
                 false, false, true,   # vroll, vpitch, vyaw
                 false, false, false]  # ax, ay, az
  imu0: /sensor/imu
  imu0_config: [false, false, false,
                true, true, true,
                false, false, false,
                true, true, true,
                true, true, true]

# EKF 2: Global Position (map frame)
# - EKF 1의 출력 + GPS 변환 데이터
# - 출력: map -> odom TF
ekf_global:
  frequency: 10.0
  odom0: /odometry/local  # EKF 1 출력
  odom0_config: [true, true, false,
                 false, false, true,
                 true, true, false,
                 false, false, true,
                 false, false, false]
  odom1: /odometry/gps  # navsat_transform 출력
  odom1_config: [true, true, false,
                 false, false, false,
                 false, false, false,
                 false, false, false,
                 false, false, false]
```

### 6.4 주요 논문 및 참고자료

| 논문/자료 | 저자 | 연도 | 핵심 내용 |
|-----------|------|------|-----------|
| Positioning and Attitude determination for Precision Agriculture Robots based on IMU and Two RTK GPSs Sensor Fusion | Technology & Strategy | 2022 | 듀얼 RTK-GPS + IMU 융합으로 정밀 자세 추정, 경로 추종 RMSE 7-9cm |
| Tightly coupled GNSS/IMU/vision integrated system for positioning in agricultural scenarios | ScienceDirect | 2025 | GNSS/IMU/Vision 밀접 결합, 농업 환경 특화 |
| Application of multi-sensor fusion localization algorithm based on recurrent neural networks | Nature Scientific Reports | 2025 | EKF + RNN 하이브리드 융합, 센서 비동기 및 노이즈 대응 |
| Low-Cost Real-Time Localisation for Agricultural Robots in Unstructured Farm Environments | Machines, MDPI | 2024 | ESKF 기반 저가 RTK-GNSS + IMU + 휠 오도 + LIO-SAM 융합, 실제 농장 검증 |
| Agricultural machinery GNSS/IMU-integrated navigation based on fuzzy adaptive FIR Kalman filtering | ScienceDirect | 2021 | 퍼지 적응 필터로 GNSS/IMU 통합, 농기계 적용 |

### 6.5 RS500 적용 권장사항

1. **Phase 1 (즉시 적용)**: robot_localization Dual-EKF 구성
   - 현재 RS500 시뮬레이션에 이미 GPS(/sensor/gps), IMU(/sensor/imu), Odom(/odom) 토픽이 구성되어 있으므로 즉시 통합 가능
   - `navsat_transform_node`로 GPS NavSatFix를 odometry 메시지로 변환

2. **Phase 2 (정밀도 향상)**:
   - 실차량에 듀얼 RTK-GPS 안테나 장착으로 heading 정밀도 향상 (단일 안테나 대비 10배)
   - 궤도차량 슬립 모델을 반영한 커스텀 motion model 적용

3. **GPS-Denied Fallback**:
   - LIO-SAM의 GPS 팩터를 활용하여 GPS 가용 시 맵에 앵커링
   - GPS 불가 시 LiDAR-IMU 오도메트리로 자동 전환
   - 군사 시나리오에서 EW(Electronic Warfare) 대응 필수

4. **성능 목표**:
   - GPS 가용 시: 절대 위치 오차 < 5 cm (RTK-Fix)
   - GPS 불가 시: 100m 주행 당 상대 오차 < 1% (LiDAR-IMU)
   - 위치 추정 주파수: 50 Hz 이상

---

## 7. RS500 프로젝트 종합 적용 권장사항

### 7.1 권장 소프트웨어 스택

```
┌─────────────────────────────────────────────────────────┐
│                    Mission Planning                      │
│           Fields2Cover (Coverage) + Custom BT            │
├─────────────────────────────────────────────────────────┤
│                    Nav2 Framework                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ SmacLattice  │  │    MPPI      │  │  Recovery    │  │
│  │ (Global)     │  │  (Local)     │  │  Behaviors   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
├─────────────────────────────────────────────────────────┤
│                    Costmap 2D                            │
│         Static + Obstacle + Voxel + Inflation            │
│         + Custom Traversability Layer                    │
├─────────────────────────────────────────────────────────┤
│                    Localization                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │robot_local-  │  │  LIO-SAM    │  │  ORB-SLAM3   │  │
│  │ization      │  │  (SLAM)     │  │  (Backup)    │  │
│  │(Dual-EKF)   │  │             │  │              │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
├─────────────────────────────────────────────────────────┤
│                    Sensors                               │
│   GPS-RTK    IMU    LiDAR    Camera    Wheel Encoder    │
│  (/sensor/  (/sensor/ (/sensor/ (/sensor/   (/odom)     │
│    gps)      imu)    lidar)   camera/                   │
│                               front)                     │
├─────────────────────────────────────────────────────────┤
│              Gazebo gz-sim Simulation                    │
│           agricultural_field / tracked_vehicle           │
└─────────────────────────────────────────────────────────┘
```

### 7.2 알고리즘 선정 근거 요약

| 기능 | 선정 알고리즘 | 핵심 근거 |
|------|-------------|-----------|
| 전역 경로 계획 | SmacPlannerLattice | 궤도차량 운동학 프리미티브 지원, Nav2 네이티브 |
| 지역 경로/장애물 회피 | MPPI | 비선형 동역학 반영, 지형 비용 커스텀 critic, Nav2 네이티브 |
| 농업 커버리지 | Fields2Cover + Boustrophedon | 농업 전용 오픈소스, ROS2 통합, 비볼록 필드 지원 |
| SLAM | LIO-SAM (주) + FAST-LIO2 (보조) | GPS 통합 가능, 야외 강건성, 인수 그래프 확장성 |
| 위치 추정 | robot_localization Dual-EKF | GPS/IMU/Odom 융합, ROS2 표준, 즉시 적용 가능 |
| GPS-Denied 대응 | FAST-LIO2 fallback + ORB-SLAM3 보조 | 계산 효율, 임베디드 실행 가능, 다중 센서 교차 검증 |

### 7.3 현재 프로젝트 코드 대비 개선 필요 사항

현재 `planning.py`의 PlanningModule은 단순 반응형(reactive) 장애물 회피만 구현하고 있다. Nav2 통합을 위해 다음 개선이 필요하다:

1. **PlanningModule -> Nav2 Planner/Controller 전환**: 현재 수동 구현된 장애물 거리 기반 회피를 Nav2의 MPPI 컨트롤러로 대체
2. **DrivingMode 확장**: `COVERAGE`, `FURROW_FOLLOWING`, `WAYPOINT_NAVIGATION` 모드 추가
3. **Costmap 통합**: LiDAR(/sensor/lidar) 데이터를 Nav2 costmap의 obstacle layer로 연결
4. **TF Tree 구성**: map -> odom -> base_link -> sensor frames TF 트리 구성 필요

### 7.4 Gazebo 시뮬레이션 검증 계획

| 단계 | 시나리오 | 검증 지표 |
|------|----------|-----------|
| 1 | 빈 평지에서 waypoint 내비게이션 | 도착 정밀도, 경로 추종 오차 |
| 2 | 장애물 배치 환경 내비게이션 | 충돌 회피 성공률, 우회 경로 효율 |
| 3 | 농경지 boustrophedon 커버리지 | 커버리지율, 중복률, 작업 시간 |
| 4 | 고랑 따라가기 | 좌우 편차, 연속 추종 거리 |
| 5 | GPS 차단 시 SLAM 전환 | 위치 추정 오차, 전환 지연 시간 |
| 6 | 험지(경사, 돌밭) 주행 | 주행 안정성, 경로 실현 가능성 |

---

## 참고 문헌 및 웹 자료

### 전역 경로 계획
- [Real-time Path Planning for Autonomous Vehicle Off-road Driving (PeerJ, 2024)](https://peerj.com/articles/cs-2209/)
- [Physics Based Path Planning for Autonomous Tracked Vehicle in Challenging Terrain (Springer, 2018)](https://link.springer.com/article/10.1007/s10846-018-0851-3)
- [Hybrid Terrain-Aware Path Planning: VD-RRT* + VD-D* Lite (arXiv, 2025)](https://arxiv.org/abs/2510.12169)
- [Improvement and Fusion of D*Lite and DWA (MDPI Machines, 2024)](https://www.mdpi.com/2075-1702/12/8/525)
- [Unmanned Vehicle Off-road Path-planning with Environmental Factors (Taylor & Francis, 2024)](https://www.tandfonline.com/doi/full/10.1080/17538947.2024.2408453)
- [Path Planning Trends for AMR Navigation: A Review (PMC, 2025)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11861809/)

### 지역 경로 계획 및 장애물 회피
- [Autonomous Navigation: Hierarchical DWA+MPC Framework (PMC, 2025)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11991454/)
- [IA-DWA Algorithm for Mobile Robots (Scientific Reports, 2024)](https://www.nature.com/articles/s41598-024-84858-3)
- [Nav2 MPPI Controller Documentation](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [TEB vs DWA Performance Comparison (ResearchGate, 2024)](https://www.researchgate.net/publication/399168225)
- [ROS-Based Navigation: Architectures, Methods, Trends (PMC, 2025)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12300016/)

### 농경지 특화 경로 패턴
- [Optimal Guidance Track Generation for Precision Agriculture: CPP Review (J. Field Robotics, 2024)](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22286)
- [3D Hybrid Path Planning for Agricultural Fields (J. Field Robotics, 2025)](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22422)
- [Fields2Cover Library (GitHub)](https://github.com/Fields2Cover/Fields2Cover)
- [Fields2Cover Paper (IEEE RA-L, 2023)](https://ieeexplore.ieee.org/document/10050562/)
- [Boustrophedon-Optimized Neural Network for Path Planning (J. Field Robotics, 2025)](https://onlinelibrary.wiley.com/doi/10.1002/rob.70116)
- [Agricultural Robot Navigation: Path Planning and Tracking Control (Agriculture, 2024)](https://www.mdpi.com/2077-0472/16/1/64)

### SLAM
- [GPS-Denied LiDAR-Based SLAM Survey (IET, 2025)](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/csy2.70031)
- [3D LiDAR Outdoor SLAM: LiDAR/IMU Tight Coupling (Scientific Reports, 2025)](https://www.nature.com/articles/s41598-025-95730-3)
- [LiDAR SLAM-Based Navigation for Agricultural Robot (Agronomy, 2024)](https://www.mdpi.com/2073-4395/14/10/2365)
- [Voxel-SLAM: Complete LiDAR-Inertial SLAM (arXiv, 2024)](https://arxiv.org/html/2410.08935v1)
- [FAST-LIVO2: LiDAR-Inertial-Visual Odometry (arXiv, 2024)](https://arxiv.org/html/2408.14035v2)
- [Visual-Inertial SLAM for Unstructured Outdoor Environments (J. Field Robotics, 2025)](https://onlinelibrary.wiley.com/doi/10.1002/rob.22581)
- [SuperVINS: Deep Learning Visual-Inertial SLAM (arXiv, 2024)](https://arxiv.org/html/2407.21348v1)
- [Robust 2D LiDAR SLAM in Arboreal Environments (arXiv, 2025)](https://arxiv.org/html/2505.10847v1)
- [SLAM-application: Multi-SLAM Comparison on Gazebo (GitHub)](https://github.com/engcang/SLAM-application)

### ROS2 Nav2
- [Nav2 Official Documentation](https://docs.nav2.org/)
- [Nav2 GitHub Repository](https://github.com/ros-navigation/navigation2)
- [Smac Hybrid-A* Planner Configuration](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
- [Smac State Lattice Planner Configuration](https://docs.nav2.org/configuration/packages/smac/configuring-smac-lattice.html)
- [Nav2 Localization Improvements (Poespas Blog, 2025)](https://blog.poespas.me/posts/2025/03/03/ros2-nav2-localization-improvements/)

### GPS-RTK + IMU 센서 퓨전
- [Positioning for Precision Agriculture: IMU + Two RTK GPSs Fusion (ScienceDirect, 2022)](https://www.sciencedirect.com/science/article/pii/S2405896322027483)
- [Tightly Coupled GNSS/IMU/Vision for Agricultural Scenarios (ScienceDirect, 2025)](https://www.sciencedirect.com/science/article/abs/pii/S0168169925005848)
- [Multi-Sensor Fusion Based on RNN (Scientific Reports, 2025)](https://www.nature.com/articles/s41598-025-90492-4)
- [Low-Cost Localisation for Agricultural Robots (MDPI Machines, 2024)](https://www.mdpi.com/2075-1702/12/9/612)
- [robot_localization ROS2 Package](https://index.ros.org/p/robot_localization/)
- [Sensor Fusion Using robot_localization (AutomaticAddison, 2024)](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)

### 군사 UGV
- [Ukraine's Robot Army: UGVs in Modern Warfare (2025)](https://sldinfo.com/2025/10/ukraines-robot-army-the-rise-of-unmanned-ground-vehicles-in-modern-warfare/)
- [US Military UGV Autonomous Driving Initiatives (2025)](https://www.militaryaerospace.com/uncrewed/article/55241568/machine-autonomy-unmanned-ground-vehicles-ugvs-off-road)
- [Evolving Landscape of Military UGVs (AI Insider, 2025)](https://theaiinsider.tech/2025/09/13/the-evolving-landscape-of-military-unmanned-ground-vehicles-in-the-us-beyond-ordnance-disposal-in-modern-warfare/)

---

*본 문서는 RS500 궤도차량 자율주행 프로젝트의 경로 계획 및 SLAM 분야 선행연구를 정리한 것으로, 최신 연구 동향(2024-2025)을 반영하였습니다.*
