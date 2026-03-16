# Camera-Only Visual SLAM 솔루션 조사 보고서
## SS500 농업용 자율주행 궤도 차량 적용 가능성 연구

**작성일**: 2026년 3월 16일
**버전**: v1.0
**저장 경로 권장**: `docs/research/visual_slam_survey_2026.md`

---

## 목차

1. [개요 및 연구 배경](#1-개요-및-연구-배경)
2. [시스템 요구사항 분석](#2-시스템-요구사항-분석)
3. [Visual SLAM 솔루션 상세 분석](#3-visual-slam-솔루션-상세-분석)
   - 3.1 ORB-SLAM3
   - 3.2 Stella-VSLAM
   - 3.3 RTAB-Map
   - 3.4 VINS-Fusion
   - 3.5 OpenVINS
   - 3.6 Kimera
4. [종합 비교 테이블](#4-종합-비교-테이블)
5. [GPS/VIO 전환 아키텍처](#5-gpsvio-전환-아키텍처)
6. [농업 환경 SLAM 특수 과제](#6-농업-환경-slam-특수-과제)
7. [관련 논문 리뷰 (2023-2025)](#7-관련-논문-리뷰-2023-2025)
8. [권장 사항 및 결론](#8-권장-사항-및-결론)
9. [참고문헌](#9-참고문헌)

---

## 1. 개요 및 연구 배경

### 1.1 연구 목적

본 보고서는 SS500 농업용 자율주행 궤도 차량에 Camera-Only Visual SLAM 솔루션을 적용하기 위한 기술 조사 연구이다. 현재 시스템은 Dual-EKF(ARCH-002) 기반의 GPS 측위를 사용하나, 농업 환경 특성상 GPS 음영 구역(과수원 수관 아래, 건물 밀집 지역, 고밀도 식생 구역)에서의 정밀 측위가 불가능하다. 이를 보완하기 위해 카메라와 IMU를 활용한 Visual SLAM 솔루션의 도입 가능성을 평가한다.

### 1.2 현재 시스템 구성

| 구성 요소 | 사양 |
|-----------|------|
| 카메라 | RGBD ×3 (전방/좌/우, 각 80° FOV) |
| IMU | 100Hz |
| GPS | 10Hz |
| 플랫폼 | ROS2 Jazzy, Ubuntu 24.04 |
| GPU (노트북) | MX550 (저전력) |
| GPU (데스크탑) | RTX 2060 |
| 내비게이션 | Nav2 (SmacPlannerLattice + MPPI) |
| 시뮬레이션 | Gazebo Harmonic |

### 1.3 핵심 요구사항

1. **GPS 부재 시 연속 측위**: 과수원/논/밭의 GPS 음영 구간에서 드리프트 없는 측위 유지
2. **GPS ↔ VIO 원활한 전환**: 신호 복구 시 위치 점프 없는 자연스러운 전환
3. **ROS2 Jazzy 호환성**: 기존 Nav2, EKF 스택과 통합
4. **MX550 수준에서의 실시간 처리**: 저전력 하드웨어에서 최소 10-15Hz 이상
5. **RGBD 카메라 3대 활용**: 기존 하드웨어 자산 최대 활용
6. **농업 환경 내구성**: 반복 패턴, 계절 변화, 동적 식물 대응

---

## 2. 시스템 요구사항 분석

### 2.1 농업 환경 특성

농업 환경에서의 Visual SLAM은 일반 실내/도심 환경과 근본적으로 다른 특성을 가진다.

**시각적 특성:**
- **반복 텍스처**: 논/밭의 균일한 토양, 동일 간격으로 심어진 작물로 인해 특징점 매칭 모호성 발생
- **계절적 외관 변화**: 파종기/성장기/수확기에 따른 극단적 외관 변화로 장기 지도 재사용 어려움
- **동적 요소**: 성장하는 작물, 바람에 흔들리는 잎, 수확 장비 등 비정적 요소 다수
- **조명 변화**: 아침/오후/흐린 날씨에 따른 극단적 조도 변화

**구조적 특성:**
- **루프 클로저 어려움**: 과수원 행(row) 구조처럼 유사한 장면이 반복되어 잘못된 루프 클로저 발생 위험
- **수직 구조 부족**: 논/밭은 특징점 추출에 유리한 수직 구조물이 적음
- **GPS 음영**: 과수원 수관, 비닐하우스, 창고 주변의 국소적 GPS 열화

### 2.2 하드웨어 제약 분석

MX550은 엔트리급 노트북 GPU로 연산 자원이 제한적이다.

| 하드웨어 | 성능 수준 | 딥러닝 SLAM 실행 가능성 |
|----------|-----------|--------------------------|
| MX550 | ~1.7 TFLOPS (FP32) | 경량 특징점 기반 SLAM 가능, 딥러닝 기반 어려움 |
| RTX 2060 | ~6.5 TFLOPS (FP32) | 대부분의 Visual SLAM 실시간 가능 |

---

## 3. Visual SLAM 솔루션 상세 분석

### 3.1 ORB-SLAM3

#### 개요
ORB-SLAM3는 스페인 사라고사 대학교 UZ-SLAMLab에서 개발한 시각적 SLAM 시스템으로, 모노큘러/스테레오/RGBD/관성 데이터를 통합 지원하는 가장 널리 알려진 오픈소스 SLAM 프레임워크이다. Carlos Campos et al. (IEEE T-RO, 2021)의 논문을 기반으로 한다.

#### 주요 특징

**센서 지원:**
- 모노큘러, 스테레오, RGBD (Intel RealSense, Kinect 등)
- 모노큘러+IMU, 스테레오+IMU 비주얼-관성 모드
- 핀홀 및 어안 렌즈 카메라 모델 지원

**알고리즘 특징:**
- ORB(Oriented FAST and Rotated BRIEF) 특징점 기반
- DBoW2(Bag of Words) 기반 루프 클로저 및 재국부화
- 멀티맵(Multi-Map): 추적 실패 시 새 서브맵 생성 후 병합
- 관성 초기화: 정적 초기화 및 동적 초기화 지원
- Atlas 프레임워크: 다중 세션 지도 재사용

**성능 벤치마크:**
- EuRoC MAV 데이터셋: 스테레오-관성 모드에서 0.06m ATE (절대 궤적 오차)
- TUM-VI 데이터셋: 핸드헬드 모드에서 우수한 정확도
- CPU 기반 실시간 동작 가능 (i7 권장)

#### ROS2 호환성 평가

ORB-SLAM3 공식 저장소는 **ROS1 Melodic/Ubuntu 18.04만 지원**하며, 공식적인 ROS2 래퍼가 없다. 그러나 커뮤니티 포크들이 ROS2 Humble/Foxy 래퍼를 제공하고 있으나, Jazzy(Ubuntu 24.04)에 대한 공식 지원은 아직 미비하다.

**관련 커뮤니티 패키지:**
- `ros2_orb_slam3` (비공식 래퍼): Humble까지 테스트됨, Jazzy에서 수동 빌드 필요
- 빌드 의존성: Pangolin, OpenCV 4.x, Eigen3

#### 농업 환경 적합성

**강점:**
- RGBD 모드에서 깊이 정보 활용으로 텍스처 부족 환경에서 일부 보완
- 멀티맵으로 추적 실패 복구 가능
- Atlas 프레임워크로 이전 방문 구역 재인식

**약점:**
- ORB 특징점은 균일한 토양/잔디에서 특징점 수 급감
- 계절 변화 후 지도 재사용 불가 (외관 기반 루프 클로저)
- GPS 융합 기능 없음 (별도 EKF 브릿지 필요)
- 2021년 이후 공식 업데이트 없음 (마지막 커밋 2021년 12월)

#### 유지보수 상태

저장소는 사실상 **아카이브 상태**다. 2021년 12월 이후 공식 커밋이 없으며, 수백 개의 이슈가 방치되어 있다. 학술 참조로는 최고이지만 프로덕션 사용에는 위험 부담이 있다.

**종합 평점 (SS500 기준):**
- ROS2 Jazzy 호환성: ★★☆☆☆ (커뮤니티 포크로 가능하지만 불안정)
- RGBD 지원: ★★★★★
- IMU 융합: ★★★★★
- GPS 전환 지원: ★☆☆☆☆ (없음)
- MX550 실행 가능성: ★★★☆☆ (CPU 모드 가능)
- 농업 환경 적합성: ★★☆☆☆
- 유지보수 상태: ★☆☆☆☆

---

### 3.2 Stella-VSLAM

#### 개요
Stella-VSLAM은 ORB-SLAM2를 기반으로 완전히 재설계한 C++ 비주얼 SLAM 라이브러리로, stella-cv 조직이 주도하는 활발한 오픈소스 프로젝트이다. 원래 OpenVSLAM에서 파생되었으며, 법적 문제(ORB-SLAM2 코드 유사성 논란) 해결 후 코드베이스를 처음부터 재작성하여 독립성을 확보했다.

#### 주요 특징

**센서 지원:**
- 모노큘러, 스테레오, RGBD
- 퍼스펙티브, 어안, 등장방형 카메라 모델
- ROS2 전용 래퍼(`stella_vslam_ros`) 제공

**알고리즘 특징:**
- ORB 특징점 기반 (ORB-SLAM2 계보)
- 지도 저장/불러오기 지원 (사전 구축 지도 기반 재국부화)
- 모듈화된 아키텍처: 추적/매핑/루프클로저 분리
- 다양한 특징점 기술자 플러그인 지원 가능성 (API 문서화됨)

**핵심 차별점:**
- 코드베이스 처음부터 재작성 → 가독성, 확장성, 성능 향상
- 1,062개 이상의 커밋으로 활발한 개발 진행
- Docker 지원 (여러 변형: 표준, CUI, rosbag, socket)
- ReadTheDocs 기반 공식 문서 제공

#### ROS2 호환성 평가

`stella_vslam_ros` 패키지가 ROS2 브랜치를 별도로 유지하며, `camera/color/image_raw` + `camera/depth/image_raw` 토픽을 직접 구독한다. Jazzy 지원 여부는 명시되어 있지 않으나 ROS2 기반 아키텍처로 Jazzy 빌드가 가능할 가능성이 높다.

**IMU 융합:** 공식 저장소에서 IMU 통합은 "현재 작업 중(currently working on)" 상태로, **아직 구현되지 않은 기능**이다. 이는 SS500의 IMU 데이터 활용에 심각한 제약이다.

#### 농업 환경 적합성

**강점:**
- RGBD 지원으로 깊이 기반 특징점 삼각측량 가능
- 지도 저장/재사용 기능으로 동일 경로 재방문 시 재국부화 가능
- 활발한 유지보수로 버그 수정 신속

**약점:**
- IMU 미지원: GPS 음영 구간에서 순수 시각 오도메트리에만 의존 → 드리프트 취약
- GPS 융합 기능 없음
- 반복 텍스처 환경에서의 루프 클로저 성능 불명확
- 농업 특화 테스트 데이터 없음

**종합 평점 (SS500 기준):**
- ROS2 Jazzy 호환성: ★★★☆☆ (빌드 테스트 필요)
- RGBD 지원: ★★★★★
- IMU 융합: ★☆☆☆☆ (미구현)
- GPS 전환 지원: ★☆☆☆☆ (없음)
- MX550 실행 가능성: ★★★☆☆
- 농업 환경 적합성: ★★☆☆☆
- 유지보수 상태: ★★★★★

---

### 3.3 RTAB-Map

#### 개요
RTAB-Map(Real-Time Appearance-Based Mapping)은 캐나다 IntroLab(세르히오 라가드 개발)이 개발한 그래프 기반 비주얼-관성 SLAM 시스템이다. ROS 생태계에서 가장 통합성이 높은 SLAM 솔루션으로, ROS2 Jazzy를 공식 지원하는 몇 안 되는 시스템 중 하나이다.

#### 주요 특징

**센서 지원:**
- RGBD 카메라 (단일 및 다중: `-DRTABMAP_SYNC_MULTI_RGBD=ON` 컴파일 플래그)
- 스테레오 카메라
- LiDAR (PointCloud 기반)
- GPS 입력 지원
- IMU 융합

**알고리즘 특징:**
- 외관 기반(appearance-based) 루프 클로저: SURF, SIFT, ORB, BRIEF 등 다수 기술자 지원
- WM(Working Memory)/LTM(Long-Term Memory) 관리: 메모리 제한 환경에서 대규모 맵 처리
- 그래프 최적화: g2o, GTSAM, CVSBA 지원
- 멀티세션 매핑 가능

**ROS2 통합:**
- 공식 ROS2 패키지 (`rtabmap_ros`) 제공
- Nav2 연동 예제 제공 (rtabmap_demos 폴더)
- ROS2 Humble, Jazzy, Rolling 모두 CI 빌드 통과 (Jazzy/Ubuntu Noble AMD64 빌드 성공 뱃지 확인)

**GPS 통합:**
- RTAB-Map은 GPS 좌표를 지도의 글로벌 레퍼런스로 사용 가능
- NavSat 메시지 기반으로 GPS 입력 수신
- GPS 기반 지도 정렬 및 로컬라이제이션 지원

#### 농업 환경 적합성

**강점:**
- **다중 RGBD 카메라 동시 지원**: SS500의 3대 카메라(전방/좌/우) 동시 활용 가능 (컴파일 옵션 필요)
- **메모리 관리 모듈**: 장거리 농장 매핑 시 메모리 폭증 방지
- **GPS 네이티브 지원**: GPS 데이터를 지도 앵커로 직접 활용 가능
- **외관 기반 루프 클로저**: 다양한 특징점 기술자 선택 가능 → 농업 환경 튜닝 여지
- **Nav2 공식 통합**: 기존 Nav2 스택과 매끄럽게 연동
- **가장 활발한 유지보수**: 1,860+ 커밋, Jazzy까지 공식 빌드 지원

**약점:**
- 반복 텍스처(논/밭) 환경에서 잘못된 루프 클로저 발생 가능 → ICP/포인트클라우드 정합 활용 시 완화
- 단독 외관 기반 루프 클로저는 계절 변화에 취약
- 설정 파라미터 수가 매우 많아 농업 환경 최적화에 상당한 튜닝 시간 필요
- 멀티RGBD 빌드 옵션은 별도 컴파일 필요 (기본값 아님)

**계산 부하:**
RTAB-Map은 CPU/GPU 둘 다에서 동작하며, RGBD 모드에서는 대부분 CPU 연산이다. MX550에서 단일 RGBD 카메라 기준 실시간(10-15Hz)이 가능하나, 3대 동시 처리 시 성능 저하 가능. 독립적으로 각 카메라를 처리 후 결과 병합 방식 고려 필요.

**종합 평점 (SS500 기준):**
- ROS2 Jazzy 호환성: ★★★★★ (공식 지원, CI 통과)
- RGBD 지원: ★★★★★ (다중 카메라 포함)
- IMU 융합: ★★★★☆
- GPS 전환 지원: ★★★★☆ (GPS 입력 지원)
- MX550 실행 가능성: ★★★☆☆ (단일 카메라 가능, 3대 어려움)
- 농업 환경 적합성: ★★★☆☆
- 유지보수 상태: ★★★★★

---

### 3.4 VINS-Fusion

#### 개요
VINS-Fusion은 홍콩과기대(HKUST) Aerial Robotics 그룹이 개발한 비주얼-관성 항법 시스템이다. VINS-Mono(2018)의 후속으로, 스테레오 카메라와 IMU를 타이트하게 융합한 고정밀 VIO 시스템이다. GPS 융합 예제를 포함하는 것이 특징이나 프로덕션 수준은 아니다.

#### 주요 특징

**센서 지원:**
- 모노큘러+IMU, 스테레오+IMU, 스테레오 전용
- GPS 융합 (토이 레벨 예제: KITTI GPS 데이터)

**알고리즘 특징:**
- 타이트-커플드 비주얼-관성 융합 (비선형 최적화 기반)
- 온라인 카메라-IMU 시공간 캘리브레이션
- 루프 클로저 모듈 포함

#### 심각한 제약사항

**ROS2 미지원:** 공식 저장소는 ROS Kinetic/Melodic(ROS1)만 지원하며, ROS2 포팅이 없다. ROS2 Jazzy 환경에 직접 사용 불가.

**RGBD 미지원:** 시스템이 스테레오/모노큘러 카메라 + IMU 조합만 지원하며, 깊이 센서(RGBD) 통합 기능이 없다. SS500의 RGBD 카메라 자산 활용 불가.

**유지보수 중단:** 저장소의 마지막 커밋이 2019년 1월로 추정되며(41커밋 전체), 200개 가까운 이슈가 방치 상태다.

**종합 평점 (SS500 기준):**
- ROS2 Jazzy 호환성: ★☆☆☆☆ (미지원)
- RGBD 지원: ★☆☆☆☆ (미지원)
- IMU 융합: ★★★★★
- GPS 전환 지원: ★★☆☆☆ (토이 레벨)
- MX550 실행 가능성: ★★★☆☆ (ROS1 환경 전제)
- 농업 환경 적합성: ★★☆☆☆
- 유지보수 상태: ★☆☆☆☆

> **결론**: VINS-Fusion은 ROS2 비호환성과 RGBD 미지원으로 **SS500 프로젝트에 적합하지 않다.**

---

### 3.5 OpenVINS

#### 개요
OpenVINS는 델라웨어 대학교 RPNG(Robot Perception and Navigation Group)에서 개발한 오픈소스 비주얼-관성 항법 플랫폼이다. MSCKF(Multi-State Constraint Kalman Filter) 슬라이딩 윈도우 방식을 기반으로 하며, 연구 플랫폼으로서의 모듈성과 투명성이 강점이다.

#### 주요 특징

**핵심 알고리즘:**
- EKF 기반 관성 정보 + 희소 시각 특징 트랙 융합
- MSCKF 슬라이딩 윈도우 방식
- KLT 광학흐름 또는 기술자 기반 특징점 추적 선택 가능
- 다섯 가지 특징 표현 방식(XYZ, 역깊이, 앵커드 변형 등)

**센서 지원:**
- 모노큘러, 스테레오, 바이노큘러 카메라
- IMU (핵심 센서)
- ARUCO 마커 기반 랜드마크 지원

**ROS2 지원:**
- v2.5 (2020년 12월)부터 ROS2 공식 지원
- 최신 v2.7 (2024년 6월)까지 꾸준한 업데이트
- ROS2 Jazzy 명시적 지원 여부는 불확실하나, 구조상 빌드 가능 가능성

**IMU 기능:**
- v2.7에서 IMU 고유 파라미터(intrinsic) 온라인 캘리브레이션 추가
- 이산 및 분석적 전파(propagation) 방식 모두 지원
- 제로 속도 검출 (차량 정지 시 드리프트 억제)

#### RGBD 지원 여부

OpenVINS는 **RGBD 카메라를 직접 지원하지 않는다.** 깊이 채널 데이터를 활용하는 기능이 없으며, 스테레오 카메라처럼 기하학적으로 깊이를 추정하는 구조이다. SS500의 RGBD 카메라에서 RGB 채널만 추출하여 스테레오/모노 모드로 사용하는 방식은 가능하지만, 깊이 데이터 활용 불가는 정보 손실이다.

#### GPS 융합

공식 기능으로 GPS 융합이 포함되어 있지 않다. 별도 ROS2 EKF 노드(예: `robot_localization`)와 연동하는 방식으로 구현 가능하지만 공식 지원이 아니다.

#### 농업 환경 적합성

**강점:**
- 경량 EKF 기반으로 MX550에서도 실시간 동작 가능
- 제로 속도 검출로 정지 중 드리프트 억제 → 농업 기계의 빈번한 정지/재출발 유용
- MSCKF는 루프 클로저 없이도 단기 드리프트가 적음

**약점:**
- 루프 클로저 없음: 장거리 경로에서 누적 드리프트 불가피
- RGBD 미지원
- GPS 융합 없음
- 반복 텍스처 환경에서의 특징점 추적 품질 미검증

**종합 평점 (SS500 기준):**
- ROS2 Jazzy 호환성: ★★★☆☆ (v2.7 ROS2 지원, Jazzy 명시 불확실)
- RGBD 지원: ★★☆☆☆ (RGB 채널만 사용 가능)
- IMU 융합: ★★★★★
- GPS 전환 지원: ★★☆☆☆ (외부 EKF 연동 필요)
- MX550 실행 가능성: ★★★★★ (EKF 기반 경량)
- 농업 환경 적합성: ★★☆☆☆
- 유지보수 상태: ★★★★☆

---

### 3.6 Kimera

#### 개요
Kimera는 MIT SPARK Lab에서 개발한 메트릭-시맨틱(Metric-Semantic) SLAM 시스템이다. 단순 측위/지도 작성을 넘어 환경의 의미론적 이해(semantic understanding)를 실시간으로 수행하는 것이 핵심 특징이다. 4개의 독립 모듈(Kimera-VIO, Kimera-RPGO, Kimera-Mesher, Kimera-Semantics)로 구성된다.

#### 주요 특징

**4개 핵심 모듈:**

1. **Kimera-VIO**: 고속 비주얼-관성 오도메트리 파이프라인 (GTSAM 기반)
2. **Kimera-RPGO**: 강인한 포즈 그래프 최적화 (Robust Pose Graph Optimization)
3. **Kimera-Mesher**: 프레임별/멀티프레임 3D 메시 생성
4. **Kimera-Semantics**: 3D 메시에 의미론적 레이블 투영 (SemanticFusion 기반)

**센서 지원:**
- 스테레오 카메라 + IMU (기본 구성)
- 모노큘러 모드 (부분적)
- RGBD 지원은 공식 문서에 명시되지 않음 (스테레오 기반 설계)

**의미론적 기능 (농업 적용 가능성):**
- RGB 시맨틱 분할 결과를 3D 메시에 투영
- 농작물 클래스(식물/토양/통로)를 지도에 임베딩 가능
- 3D 식물 모델 생성 가능성 (작물 모니터링 응용)

**의존성:**
- GTSAM (≥4.1): 고성능 그래프 최적화
- OpenCV (≥3.4)
- OpenGV, Glog, Gflags, DBoW2, Kimera-RPGO

#### ROS2 호환성 평가

공식 저장소는 **Ubuntu 20.04 기준 테스트**되어 있으며, ROS2 래퍼는 별도 저장소(`Kimera-VIO-ROS`)에서 제공되나 ROS1 기반이다. ROS2 Jazzy 공식 지원은 **없다**.

#### 농업 환경 적합성

**이론적 강점:**
- 의미론적 지도는 반복 텍스처 문제를 부분 극복 가능: 토양/식물/통로 클래스로 구분된 지도는 외관 변화에 더 강건
- 작물 행(row) 검출 및 내비게이션 통로 추출에 활용 가능
- Qadri et al. (2023)의 연구처럼 식물 구조를 시맨틱 랜드마크로 활용 가능

**실제 제약:**
- **ROS2 Jazzy 미지원**: 통합에 상당한 포팅 작업 필요
- **RGBD 미지원**: SS500 하드웨어 자산 활용 불가
- **GPU 요구**: 실시간 시맨틱 분할은 RTX 2060 수준의 GPU 필요, MX550에서 불가
- 전체 Kimera 파이프라인은 연산 집약적: 스테레오 VIO + 루프 클로저 + 시맨틱 메시 동시 처리
- 45커밋의 소규모 개발팀으로 유지보수 지속성 불확실

**종합 평점 (SS500 기준):**
- ROS2 Jazzy 호환성: ★☆☆☆☆ (미지원, 포팅 필요)
- RGBD 지원: ★★☆☆☆ (미지원, 스테레오 전용)
- IMU 융합: ★★★★☆ (GTSAM 기반 고품질)
- GPS 전환 지원: ★☆☆☆☆ (없음)
- MX550 실행 가능성: ★★☆☆☆ (시맨틱 모듈 MX550 불가)
- 농업 환경 적합성: ★★★★☆ (이론적 잠재성 높음)
- 유지보수 상태: ★★☆☆☆

---

## 4. 종합 비교 테이블

### 4.1 기술 역량 비교

| 항목 | ORB-SLAM3 | Stella-VSLAM | RTAB-Map | VINS-Fusion | OpenVINS | Kimera |
|------|-----------|--------------|----------|-------------|----------|--------|
| **ROS2 Jazzy 공식 지원** | ✗ (커뮤니티) | △ (빌드 가능) | ✓ (CI 통과) | ✗ | △ | ✗ |
| **RGBD 지원** | ✓ | ✓ | ✓ (다중포함) | ✗ | ✗ (RGB만) | ✗ (스테레오) |
| **IMU 융합** | ✓ | ✗ (작업중) | ✓ | ✓ | ✓ | ✓ |
| **GPS 융합** | ✗ | ✗ | ✓ (네이티브) | △ (예제) | ✗ | ✗ |
| **루프 클로저** | ✓ | ✓ | ✓ | ✓ | ✗ | ✓ |
| **지도 저장/재사용** | ✓ (Atlas) | ✓ | ✓ | ✗ | ✗ | ✓ |
| **Nav2 통합** | △ (수동) | △ (수동) | ✓ (공식 예제) | ✗ | △ | ✗ |
| **다중 카메라** | ✗ | ✗ | ✓ (컴파일 옵션) | ✗ | ✗ | ✗ |
| **시맨틱 매핑** | ✗ | ✗ | △ (라벨 가능) | ✗ | ✗ | ✓ |

### 4.2 운영 특성 비교

| 항목 | ORB-SLAM3 | Stella-VSLAM | RTAB-Map | VINS-Fusion | OpenVINS | Kimera |
|------|-----------|--------------|----------|-------------|----------|--------|
| **MX550 실행 가능** | ✓ (단일 카메라) | ✓ | △ (단일 카메라) | N/A (ROS1) | ✓ | ✗ (시맨틱) |
| **RTX 2060 실행 가능** | ✓ | ✓ | ✓ | N/A | ✓ | ✓ |
| **마지막 업데이트** | 2021-12 | 2024+ | 2024+ | ~2019 | 2024-06 | 2022+ |
| **유지보수 상태** | 중단 | 활성 | 활성 | 중단 | 활성 | 저속 |
| **라이선스** | GPLv3 | BSD-2 | BSD | GPLv3 | GPLv3 | BSD-2 |
| **ROS2 Jazzy 빌드** | 수동 | 테스트 필요 | ✓ | 불가 | 테스트 필요 | 불가 |

### 4.3 농업 환경 적합성 종합 평가

| 평가 항목 | ORB-SLAM3 | Stella-VSLAM | RTAB-Map | VINS-Fusion | OpenVINS | Kimera |
|-----------|-----------|--------------|----------|-------------|----------|--------|
| 반복 텍스처 대응 | ★★☆ | ★★☆ | ★★★☆ | ★★☆ | ★★☆ | ★★★★ |
| 계절 변화 대응 | ★☆☆ | ★☆☆ | ★★☆ | ★☆☆ | ★★☆ | ★★★ |
| 동적 물체 처리 | ★★☆ | ★★☆ | ★★★ | ★★☆ | ★★☆ | ★★★ |
| GPS 음영 극복 | ★★☆ | ★★☆ | ★★★★ | ★★★ | ★★★ | ★★☆ |
| 장거리 일관성 | ★★★ | ★★☆ | ★★★★ | ★★☆ | ★★☆ | ★★★ |
| 실내외 전환 | ★★★ | ★★☆ | ★★★★ | ★★☆ | ★★★ | ★★☆ |
| **종합** | **★★☆** | **★★☆** | **★★★☆** | **제외** | **★★☆** | **★★★** |

---

## 5. GPS/VIO 전환 아키텍처

### 5.1 핵심 설계 원칙

GPS와 Visual SLAM/VIO의 원활한 전환은 SS500 프로젝트에서 가장 중요한 아키텍처 결정 중 하나이다. 전환 방식에 따라 위치 점프, 불연속 속도 추정, Nav2 플래닝 실패 등의 문제가 발생할 수 있다.

### 5.2 전환 아키텍처 패턴

#### Pattern A: 루즈 커플링 (Loose Coupling) — 현재 ARCH-002와 유사

```
GPS (10Hz) ──────────────┐
                          ├──▶ robot_localization EKF ──▶ /odom + /map
Visual Odometry (VIO) ───┘
IMU (100Hz) ─────────────┘
```

- **원리**: GPS와 VIO 각각의 포즈 추정 결과를 EKF에 입력, 공분산 가중치로 자동 블렌딩
- **장점**: 구현 단순, ROS2 `robot_localization` 패키지로 즉시 구현 가능
- **단점**: GPS/VIO 절대 기준계 불일치 시 위치 점프 발생, 계통 오차 누적

#### Pattern B: 상태 기계 전환 (State Machine Switching)

```
GPS Quality Monitor
  │
  ├─ GOOD ──▶ GPS-EKF 활성화, VIO 초기화 유지
  │
  └─ DEGRADED ──▶ VIO 활성화, GPS 보조 입력으로 전환
                   (GPS 공분산 크게 부여 → EKF가 자동 하향 가중)
```

- **원리**: GPS HDOP/SNR 모니터링 → 품질 저하 임계값 도달 시 VIO로 전환
- **전환 순간 처리**: 전환 전 GPS-EKF 포즈를 VIO 시작점으로 초기화 (위치 점프 최소화)
- **구현 방법**: `robot_localization` 듀얼 EKF 인스턴스 (로컬 EKF + 글로벌 EKF) + 커스텀 전환 노드

#### Pattern C: 타이트 커플링 (Tight Coupling) — 최고 성능

```
GPS 의사거리/반송파 측정값 ─┐
Visual 특징점 좌표 ──────────┼──▶ 통합 최적화 (Factor Graph) ──▶ 포즈
IMU 측정값 ──────────────────┘
```

- **원리**: 모든 센서 측정값을 하나의 최적화 문제에 통합 (인자 그래프 기반)
- **대표 구현**: Cremona et al. (2023)의 GNSS-stereo-inertial ORB-SLAM3 확장
- **장점**: 이론적으로 최고 정확도, GPS 부분 가용 시에도 최대 활용
- **단점**: 구현 복잡도 높음, 실시간 처리 부담

### 5.3 SS500 권장 아키텍처: 듀얼 EKF + 적응적 전환

현재 ARCH-002 Dual-EKF 구조를 확장하는 방향이 가장 현실적이다.

```
┌─────────────────────────────────────────────────────────────────┐
│                        SS500 측위 스택                           │
│                                                                 │
│  ┌──────────────┐    ┌──────────────────┐    ┌───────────────┐ │
│  │ GPS (10Hz)   │    │ RTAB-Map         │    │ IMU (100Hz)   │ │
│  │ /fix         │    │ RGBD SLAM        │    │ /imu/data     │ │
│  └──────┬───────┘    │ /odom (VIO)      │    └──────┬────────┘ │
│         │            └────────┬─────────┘           │          │
│         │            ┌────────┘                      │          │
│         ▼            ▼                               ▼          │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │           robot_localization (EKF Dual Instance)         │  │
│  │                                                          │  │
│  │  ekf_odom: IMU + VIO → /odometry/filtered (local)       │  │
│  │  ekf_map:  GPS + ekf_odom → /odometry/global            │  │
│  └──────────────────────────┬───────────────────────────────┘  │
│                             │                                   │
│  ┌──────────────────────────▼───────────────────────────────┐  │
│  │              GPS Quality Monitor Node                    │  │
│  │  - GPS HDOP > 2.5 → GPS 공분산 증폭 (자동 하향 가중)     │  │
│  │  - GPS fix_type < 3 → GPS 입력 차단                     │  │
│  │  - VIO 초기화 완료 전 GPS 의존 유지                      │  │
│  └──────────────────────────┬───────────────────────────────┘  │
│                             ▼                                   │
│                    Nav2 (SmacPlannerLattice + MPPI)             │
└─────────────────────────────────────────────────────────────────┘
```

**핵심 설계 결정사항:**

1. **공분산 기반 자동 전환**: GPS HDOP 값을 공분산으로 변환하여 EKF 입력. GPS가 나빠질수록 자동으로 VIO 가중치 증가. 하드 스위칭 없음 → 위치 점프 없음.

2. **VIO 오프셋 관리**: GPS와 VIO의 절대 기준 불일치 문제를 위해, GPS 신호가 양호할 때 VIO 드리프트를 지속적으로 GPS로 보정하는 앵커링 로직 필요.

3. **재진입 시 위치 점프 방지**: GPS 재획득 시 즉각 절환 대신, GPS를 EKF의 추가 입력으로 서서히 반영하는 방식으로 자연스러운 수렴 유도.

4. **RTAB-Map 내장 GPS 지원 활용**: RTAB-Map의 GPS NavSat 입력을 통해 SLAM 지도의 글로벌 기준 고정 → EKF 누적 드리프트를 주기적으로 리셋.

### 5.4 전환 트리거 임계값 (권장값)

| 측정값 | 정상 GPS 모드 | 전환 임계값 | VIO 전용 모드 |
|--------|--------------|------------|--------------:|
| HDOP | < 1.5 | 2.0 ~ 4.0 | > 4.0 |
| Fix Type | 3 (3D Fix) | 2 (2D Fix) | < 2 또는 No Fix |
| 위성 수 | > 8개 | 4 ~ 8개 | < 4개 |
| GPS 속도 불일치 | < 0.1m/s | 0.1 ~ 0.5m/s | > 0.5m/s |

---

## 6. 농업 환경 SLAM 특수 과제

### 6.1 반복 텍스처 문제 (Repetitive Texture Problem)

농업 환경에서 가장 큰 Visual SLAM 도전 과제이다.

**문제 발생 메커니즘:**
- 균일하게 배열된 작물 행(row)은 외관 기반 루프 클로저에서 "모든 행이 동일하게 보임"
- ORB/SURF/SIFT 등 특징점 기술자가 유사한 로컬 패치를 다른 위치로 매칭 → 잘못된 루프 클로저
- 잘못된 루프 클로저는 포즈 그래프 최적화 시 전체 지도 왜곡 초래

**완화 전략:**

1. **구조 기반 특징점 활용**: 작물 행의 열 끝(headland), 관개 수로, 작업로 경계 등 구별 가능한 구조물에 높은 가중치 부여

2. **GPS 기반 루프 클로저 제약**: GPS 좌표 기반 후보 위치 제한 → 동일한 행의 다른 지점 오매칭 방지. RTAB-Map의 GPS 입력이 이 목적에 활용 가능.

3. **행 카운터 결합**: 작물 행을 가로지를 때의 IMU/오도메트리 기반 행 번호 추적

4. **시맨틱 정보 활용**: 통로(path)/식물(crop)/하늘(sky)/토양(soil) 시맨틱 분할을 특징으로 추가 → Kimera 또는 커스텀 시맨틱 레이어

5. **장기 특징점 (Long-term Features)**: LTSD(Long-Term Stable Descriptor) 또는 NetVLAD 기반 장소 인식으로 외관 변화에 강건한 루프 클로저

### 6.2 계절 변화 문제 (Seasonal Appearance Change)

**문제:**
- 봄(파종/묘목) → 여름(무성한 잎) → 가을(수확) → 겨울(빈 밭): 외관이 완전히 달라짐
- 외관 기반으로 구축한 지도는 계절이 바뀌면 재국부화 실패

**완화 전략:**

1. **구조 기반 랜드마크 고정**: 계절에 상관없이 변하지 않는 구조물(건물, 수로, 전봇대, 표지판)을 불변 랜드마크로 관리

2. **다계절 지도 병렬 유지**: 계절별 서브맵 구축 후 현재 계절에 맞는 지도 선택 (RTAB-Map 멀티세션 기능 활용)

3. **GPS 앵커 포인트 전략**: 계절에 무관한 GPS 좌표 앵커를 지도의 절대 기준으로 유지, 외관 기반 의존도 최소화

4. **딥러닝 기반 장소 인식**: NetVLAD, AnyLoc 등 딥러닝 장소 인식 모델은 외관 변화에 강건하나, MX550에서 실시간 처리 어려움

### 6.3 동적 환경 문제 (Dynamic Environment)

**문제:**
- 바람에 흔들리는 잎과 가지: 광학흐름 기반 특징 추적 오염
- 이동하는 농업 기계, 동물, 작업자
- 성장하는 작물: 지도가 시간에 따라 변화

**완화 전략:**

1. **동적 객체 마스킹**: YOLO 등 실시간 객체 검출로 이동 객체 영역을 특징점 추출에서 제외

2. **깊이 정보 활용**: RGBD 카메라의 깊이 맵에서 급격히 변하는 깊이 불연속성을 동적 객체로 판단

3. **앙상블 추적**: 다수의 단기 특징점 트랙 중 이상 운동 패턴을 보이는 트랙 제거 (RANSAC 기반 아웃라이어 제거)

### 6.4 텍스처 부족 환경 (Texture-Poor Environment)

**문제:**
- 논/밭의 균일한 토양 표면: 특징점 거의 없음
- 맑은 하늘: 상단 시야에 특징점 없음
- 비닐하우스 내부: 반복적인 비닐 구조

**완화 전략:**

1. **RGBD 깊이 활용**: 특징점 부족 환경에서 ICP(Iterative Closest Point) 기반 포인트클라우드 정합으로 보완 (RTAB-Map 강점)

2. **다중 카메라 융합**: 전방 카메라가 텍스처 부족해도 좌/우 카메라가 보완 가능

3. **인공 랜드마크 설치**: QR 코드, AprilTag 등 인공 마커를 필드 경계에 설치 (OpenVINS ARUCO 지원)

4. **IMU 연속성 확보**: 텍스처 부족으로 시각 추적 실패 시 IMU로 단기 브릿징 (IMU 융합의 중요성)

### 6.5 계산 자원 제약 (Computational Constraints)

MX550은 농업 현장 노트북에 탑재되는 저전력 GPU이다. Visual SLAM 시스템별 MX550 실행 가능성은 다음과 같이 분석된다.

**MX550에서 실행 가능한 구성:**
- RTAB-Map (단일 RGBD, ORB 특징점): 예상 5-15Hz
- OpenVINS (스테레오+IMU): 예상 15-25Hz
- ORB-SLAM3 (RGBD, CPU 모드): 예상 10-20Hz

**MX550에서 어려운 구성:**
- RTAB-Map 다중 RGBD 동시 처리: 단일 프로세스로는 어려움, 병렬 구조 설계 필요
- Kimera 시맨틱 모듈: 딥러닝 시맨틱 분할은 MX550에서 실시간 불가
- 딥러닝 기반 장소 인식 (NetVLAD 등): 실시간 불가

---

## 7. 관련 논문 리뷰 (2023-2025)

### 7.1 농업 환경 SLAM 최신 연구

#### [1] Cremona et al. (2023) — GNSS-Stereo-Inertial SLAM for Arable Farming

**출처**: *Journal of Field Robotics*, 2023. arXiv:2307.12836

**연구 내용**: ORB-SLAM3의 스테레오-관성 모드에 GNSS(위성항법)를 타이트-커플링 방식으로 통합하여 대두 밭(soybean field)에서 평가. Rosario Dataset + 자체 수집 데이터 사용.

**핵심 결과**:
- 타이트-커플링 GNSS-스테레오-관성 SLAM이 비주얼-관성 기준 대비 포즈 오차 **10~30% 감소**
- 루즈-커플링 GNSS 기준 대비에서도 개선 확인
- 농업 필드의 루프 클로저는 "유사한 로컬 외관 + 기상 변화"로 인해 매우 어렵다고 명시

**SS500 적용 시사점**: GNSS를 ORB-SLAM3 파이프라인에 타이트-커플링하는 방식이 루즈-커플링 EKF 방식보다 우수. 그러나 ORB-SLAM3의 공식 RGBD 지원과 타이트-커플링 GNSS를 동시에 갖추는 것은 추가 개발이 필요.

#### [2] Soncini et al. (2024) — Addressing Loop Detection Challenges in Agricultural Environments

**출처**: arXiv, 2024

**연구 내용**: 오픈 필드의 반복 환경에서 루프 검출 특화 방법 제안. 기존 외관 기반 루프 클로저의 오매칭 문제를 분석하고 새로운 접근 방식 제시.

**핵심 결과**:
- 제안 방법으로 **중앙값 오차 15cm** 달성
- "전역적으로 일관된 지도 작성과 장기 국부화는 루프 검출의 강인성에 의존"
- GPS 보조 없이 순수 시각 루프 클로저만으로는 농업 환경에서 한계 명확

**SS500 적용 시사점**: 농업 환경에서는 반드시 GPS 또는 다른 글로벌 레퍼런스(UTM 격자점 등)를 루프 클로저와 결합해야 함. 순수 Visual SLAM만으로는 장거리 일관성 달성 어려움.

#### [3] Qadri et al. (2023) — Semantic Scene Understanding for Fine-Grained 3D Plant Modeling

**출처**: arXiv, 2023. CMU 로보틱스 연구소

**연구 내용**: 수수(sorghum) 씨앗을 시맨틱 랜드마크로 활용한 시각 SLAM 시스템. 로봇 장착 카메라로 수수 이삭의 고해상도 3D 의미론적 지도 작성.

**핵심 결과**:
- 시맨틱 랜드마크 기반 SLAM이 수수 범위의 **78% 매핑** (ORB-SLAM2: 38% 대비 2배 이상)
- 식물 구조를 랜드마크로 활용하면 반복 텍스처 문제 완화 가능성 제시

**SS500 적용 시사점**: 과수원 맥락에서 나무 줄기, 가지 구조 등을 시맨틱 랜드마크로 활용하면 행 간 혼동 문제 완화 가능. 과수원 특화 랜드마크 추출 모듈 개발 가능성.

#### [4] González-Santamarta et al. (2025) — VAULT: Mobile Mapping System for ROS2-based Autonomous Robots

**출처**: arXiv, 2025

**연구 내용**: GNSS + 비주얼-관성 오도메트리 + 비주얼 SLAM을 결합한 ROS2 기반 모바일 매핑 시스템. 농업 야외 환경의 실시간 국부화와 일관된 지도 작성 과제 대응.

**핵심 결과**:
- 농업 야외 환경에서 "실시간 국부화 및 일관된 매핑" 달성
- ROS2 네이티브 아키텍처로 센서 융합 구현

**SS500 적용 시사점**: GNSS + VIO + SLAM의 3중 융합이 ROS2 환경에서 가능함을 실증. SS500의 아키텍처 방향과 일치.

### 7.2 GPS/VIO 융합 최신 연구

#### [5] Dong et al. (2023) — Robust GNSS Carrier-Phase Positioning with Visual-Inertial Fusion

**출처**: arXiv, 2023

**연구 내용**: 고정밀 GNSS 반송파 위상 측위를 VIO(EKF)와 융합하여 cm급 글로벌 정확도 달성.

**핵심 결과**:
- GNSS 반송파 위상 + VIO EKF 융합으로 **센티미터급 글로벌 정확도** 달성
- 표준 GNSS 코드 의사거리 대비 정확도 대폭 향상

**SS500 적용 시사점**: SS500의 GPS가 RTK 수준이라면 이 접근법이 이상적이나, 일반 10Hz GPS인 경우 dm~m급 정확도에 머묾. 향후 RTK GPS 업그레이드 시 참조 가능.

#### [6] Pritzl et al. (2025) — Degradation-Aware Multi-Modal GNSS-Denied Localization

**출처**: arXiv, 2025

**연구 내용**: GPS 불능 환경에서 VIO + LiDAR 관성 오도메트리 + 3D 로봇 간 검출을 융합한 다중 로봇 국부화. 센서 열화 인식(degradation-aware) 방식.

**핵심 결과**:
- 센서 열화를 능동적으로 인식하고 보상하는 아키텍처가 단순 전환 방식보다 우수
- 다중 로봇 협력으로 개별 센서 한계 극복

**SS500 적용 시사점**: "열화 인식(degradation-aware)" 접근법은 SS500의 GPS 품질 모니터링 → 공분산 조정 아키텍처와 개념적으로 일치. LiDAR 없는 환경에서는 카메라+IMU로 대체.

#### [7] Abdollahi et al. (2022) — Improved MSCKF for Visual-Inertial Odometry

**출처**: arXiv, 2022

**연구 내용**: 향상된 Fast-MSCKF 알고리즘으로 표준 MSCKF 대비 처리 속도와 정확도 개선.

**핵심 결과**:
- **약 6배 빠른 처리 속도**, 정확도 **20% 이상 향상**
- EKF 기반 VIO의 계산 효율성 개선 가능성 제시

**SS500 적용 시사점**: MX550 같은 저전력 하드웨어에서 OpenVINS 기반 경량 VIO를 사용할 때, MSCKF 최적화 기법 적용으로 처리 속도 향상 가능.

### 7.3 무인기 야외 비주얼 SLAM

#### [8] Barbas Laina et al. (2024) — Scalable Outdoors Autonomous Drone Flight with Visual-Inertial SLAM

**출처**: arXiv, 2024

**연구 내용**: LiDAR 없이 비주얼-관성 SLAM과 밀집 서브맵만으로 야외 비정형 환경에서 자율 드론 비행. 3m/s까지 달성.

**핵심 결과**:
- LiDAR 없는 순수 Visual-Inertial SLAM으로 복잡한 야외 환경 자율 항행 가능
- 밀집 서브맵(dense submaps) 방식으로 대규모 환경 확장성 확보

**SS500 적용 시사점**: 지상 차량의 속도와 환경이 다르지만, LiDAR 없는 Visual-Inertial SLAM이 야외 자율주행에 충분히 적용 가능함을 드론 플랫폼에서 실증.

---

## 8. 권장 사항 및 결론

### 8.1 솔루션 우선순위

SS500 프로젝트의 요구사항(ROS2 Jazzy, RGBD ×3, IMU, GPS 전환, MX550, 농업 환경)을 종합하면 다음 우선순위를 권장한다.

**1순위: RTAB-Map** (권장)

- 유일하게 ROS2 Jazzy를 공식 CI 지원하는 솔루션
- 다중 RGBD 카메라 지원 (`RTABMAP_SYNC_MULTI_RGBD=ON`)
- GPS NavSat 입력 네이티브 지원 → 기존 ARCH-002와 가장 자연스러운 통합
- Nav2 공식 연동 예제 포함
- 메모리 관리 모듈로 장거리 농장 매핑 가능
- 활발한 유지보수(1,860+ 커밋)
- **구현 전략**: 단일 RGBD로 시작 → 안정화 후 다중 카메라 확장 → GPS-RTAB-Map 통합

**2순위: ORB-SLAM3 (커뮤니티 래퍼)** (보완용)

- RTAB-Map의 루프 클로저가 농업 환경에서 불안정할 경우 비교 기준점으로 활용
- RGBD + 스테레오-관성 모드의 높은 정확도
- 단, 유지보수 중단 상태로 장기 프로덕션 사용은 위험
- **활용 전략**: 시뮬레이션(Gazebo Harmonic)에서 먼저 평가 후 실 필드 비교

**3순위: OpenVINS** (경량 보완)

- MX550에서 가장 안정적으로 실행 가능한 경량 VIO
- RTAB-Map의 프론트엔드 실패 시 백업 VIO로 활용
- IMU 융합 품질 우수 (v2.7, 관성 캘리브레이션 포함)
- **활용 전략**: RTAB-Map이 추적 실패할 때의 브릿지 VIO로 통합

### 8.2 단계별 구현 로드맵

#### Phase 1: 기반 통합 (4주)

```
목표: RTAB-Map + ROS2 Jazzy 환경 구축 및 Gazebo 검증

작업:
1. rtabmap_ros 빌드 (Jazzy, MULTI_RGBD 옵션 포함)
2. Gazebo Harmonic에서 RGBD 카메라 3대 시뮬레이션 설정
3. RTAB-Map 단일 RGBD 모드로 기본 SLAM 동작 확인
4. robot_localization EKF와 RTAB-Map /odom 연동 테스트
5. GPS 시뮬레이션 + EKF 글로벌 포즈 출력 확인
```

#### Phase 2: 다중 카메라 통합 (3주)

```
목표: 전방/좌/우 RGBD 3대 동시 활용

작업:
1. RTAB_SYNC_MULTI_RGBD 빌드 및 설정
2. 3카메라 포인트클라우드 병합 → 단일 SLAM 입력
3. MX550에서 처리 속도 측정 (목표: 최소 8Hz)
4. 병목 구간 프로파일링 → 카메라 해상도/처리율 최적화
5. ORB 특징점 수 최적화 (농업 환경 텍스처에 맞게 조정)
```

#### Phase 3: GPS 전환 아키텍처 (3주)

```
목표: GPS ↔ RTAB-Map VIO 원활한 전환 구현

작업:
1. GPS Quality Monitor 노드 개발 (HDOP/Fix Type 기반 공분산 조정)
2. RTAB-Map GPS NavSat 입력 연결 (지도 글로벌 앵커)
3. EKF 듀얼 인스턴스 설정 (로컬/글로벌 분리)
4. GPS 음영 시뮬레이션 테스트 (Gazebo에서 GPS 노이즈 주입)
5. 전환 시 위치 점프 < 0.3m 목표 달성 확인
```

#### Phase 4: 농업 환경 최적화 (지속)

```
목표: 실제 농업 환경(논/밭/과수원)에서 안정적 동작

작업:
1. 반복 텍스처 환경에서 루프 클로저 파라미터 튜닝
2. 계절별 외관 변화 대응 멀티맵 전략 수립
3. 인공 랜드마크(AprilTag) 필드 배치 테스트
4. 장거리(1km+) 경로 SLAM 정확도 평가
5. 과수원 수관 아래 GPS 음영 실증 테스트
```

### 8.3 주의사항 및 위험 요소

| 위험 요소 | 가능성 | 영향도 | 완화 방안 |
|-----------|--------|--------|-----------|
| MX550에서 다중 RGBD 처리 부족 | 높음 | 높음 | 카메라 해상도 축소(640×480), 처리율 제한, OpenVINS 백업 VIO |
| 반복 텍스처로 잘못된 루프 클로저 | 높음 | 높음 | GPS 기반 루프 후보 제한, 루프 클로저 임계값 높이기 |
| ROS2 Jazzy + RTAB-Map 빌드 이슈 | 중간 | 중간 | Docker 기반 개발, CI 빌드 검증 먼저 확인 |
| GPS ↔ VIO 전환 시 위치 점프 | 중간 | 높음 | 공분산 기반 부드러운 전환, 하드 스위칭 금지 |
| 계절 변화 후 지도 재국부화 실패 | 높음 | 중간 | GPS 앵커 포인트 유지, 계절별 멀티맵, 구조 기반 랜드마크 |
| Stella-VSLAM IMU 미구현으로 대안 부재 | 낮음 | 낮음 | RTAB-Map 우선 사용으로 영향 없음 |

### 8.4 결론 요약

SS500 농업용 자율주행 궤도 차량에서 Camera-Only Visual SLAM 도입을 위한 최적 경로는 다음과 같다:

1. **핵심 솔루션: RTAB-Map** — ROS2 Jazzy 공식 지원, 다중 RGBD 지원, GPS 통합, Nav2 연동, 활발한 유지보수의 조합으로 현재 시점에서 유일하게 SS500의 모든 요구사항을 충족하는 솔루션이다.

2. **GPS 전환 전략: 공분산 기반 적응적 EKF** — 기존 ARCH-002 Dual-EKF를 확장하여 GPS 품질(HDOP/Fix Type)을 공분산으로 변환, EKF가 자동으로 GPS/VIO 가중치를 조정하는 방식. 하드 스위칭 없이 자연스러운 전환.

3. **농업 환경 핵심 도전: 반복 텍스처 + 루프 클로저** — 순수 시각 루프 클로저만으로는 농업 환경에서 글로벌 일관성 유지 불가. GPS 기반 루프 클로저 제약(Cremona et al., 2023; Soncini et al., 2024)이 필수적이며, RTAB-Map의 GPS NavSat 입력이 이를 지원.

4. **단계적 접근**: 단일 카메라 → 다중 카메라 → GPS 전환 통합 → 실 필드 최적화 순으로 위험을 단계별로 관리.

---

## 9. 참고문헌

### 소프트웨어 저장소

[R1] ORB-SLAM3. UZ-SLAMLab. https://github.com/UZ-SLAMLab/ORB_SLAM3. Last commit: December 2021.

[R2] Stella-VSLAM. stella-cv. https://github.com/stella-cv/stella_vslam. Active development, 1,062+ commits.

[R3] stella_vslam_ros. stella-cv. https://github.com/stella-cv/stella_vslam_ros. ROS2 branch, 118 commits.

[R4] RTAB-Map ROS2. introlab. https://github.com/introlab/rtabmap_ros. ROS2 branch, 1,860+ commits. Jazzy CI: PASS.

[R5] VINS-Fusion. HKUST-Aerial-Robotics. https://github.com/HKUST-Aerial-Robotics/VINS-Fusion. Last activity: ~2019.

[R6] OpenVINS v2.7. RPNG, University of Delaware. https://github.com/rpng/open_vins. Released June 2024.

[R7] Kimera. MIT SPARK Lab. https://github.com/MIT-SPARK/Kimera. BSD-2 License.

[R8] Kimera-VIO. MIT SPARK Lab. https://github.com/MIT-SPARK/Kimera-VIO. 3,973 commits.

### 논문

[P1] Cremona, J., Civera, J., Kofman, E., & Pire, T. (2023). **GNSS-stereo-inertial SLAM for arable farming**. *Journal of Field Robotics*. arXiv:2307.12836.

[P2] Soncini, N., Civera, J., & Pire, T. (2024). **Addressing the challenges of loop detection in agricultural environments**. arXiv, 2024.

[P3] Qadri, M., Freeman, H., Schneider, E., & Kantor, G. (2023). **Toward Semantic Scene Understanding for Fine-Grained 3D Modeling of Plants**. arXiv, December 2023. CMU Robotics Institute.

[P4] González-Santamarta, M. Á., Rodríguez-Lera, F. J., & Matellán-Olivera, V. (2025). **VAULT: A Mobile Mapping System for ROS 2-based Autonomous Robots**. arXiv, 2025.

[P5] Dong, E., Sheriffdeen, S., Yang, S., Dong, J., De Nardi, R., Ren, C., Chang, X. W., Liu, X., & Wang, Z. (2023). **Robust, High-Precision GNSS Carrier-Phase Positioning with Visual-Inertial Fusion**. arXiv, 2023.

[P6] Pritzl, V., Yu, X., Westerlund, T., Štěpán, P., & Saska, M. (2025). **Degradation-Aware Cooperative Multi-Modal GNSS-Denied Localization Leveraging LiDAR-Based Robot Detections**. arXiv, 2025.

[P7] Abdollahi, M. R., Pourtakdoust, S. H., Nooshabadi, M. H. Y., & Pishkenari, H. N. (2022). **An Improved Multi-State Constraint Kalman Filter for Visual-Inertial Odometry**. arXiv, 2022.

[P8] Barbas Laina, S., Boche, S., Papatheodorou, S., et al. (2024). **Scalable Outdoors Autonomous Drone Flight with Visual-Inertial SLAM and Dense Submaps Built without LiDAR**. arXiv, 2024.

[P9] Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M. M., & Tardós, J. D. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multimap SLAM**. *IEEE Transactions on Robotics*, 37(6), 1874–1890.

[P10] Labbé, M., & Michaud, F. (2019). **RTAB-Map as an open-source lidar and visual simultaneous localization and mapping library for large-scale and long-term online operation**. *Journal of Field Robotics*, 36(2), 416–446.

---

*보고서 작성: Claude Sonnet 4.6 (2026-03-16)*
*권장 저장 경로: `~/projects/자율주행프로젝트_work/docs/research/visual_slam_survey_2026.md`*
