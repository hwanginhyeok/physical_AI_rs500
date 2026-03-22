# Foxglove 데모 vs RS500 프로젝트 — 스택 비교 & 갭 분석

> 작성일: 2026-03-22
> 참고 영상: YouTube `d10depJ4dEU` (Foxglove 로보틱스 데이터 시각화 & 관리 플랫폼 데모)

---

## 1. 하드웨어 비교

| 항목 | Foxglove 데모 수준 (프로덕션) | 현재 RS500 프로젝트 | 갭 |
|------|------------------------------|--------------------|----|
| **엣지 컴퓨터** | NVIDIA Jetson AGX Orin (275 TOPS, 64GB, 60W) | 개발 PC: i7-12650P + RTX 2060 (WSL2) | 실차 탑재 컴퓨터 없음 — Jetson Orin NX ($599) 필요 |
| **GPU 성능** | Jetson Orin: 2048 CUDA + 64 Tensor Core | RTX 2060: 1920 CUDA (데스크탑) | 개발용 충분. 실차 탑재용 Jetson 필요 |
| **메모리** | 32~64GB LPDDR5 | 16GB DDR4 | 시뮬레이션 대규모 월드 시 부족 가능. 32GB 권장 |
| **스토리지** | NVMe SSD (MCAP 녹화용) | SSD (용량 미확인) | MCAP 데이터 녹화 시 512GB+ NVMe 필요 |
| **센서 (실물)** | LiDAR + 카메라 + GPS-RTK + IMU | Gazebo 시뮬레이션만 | 실제 센서 미보유 |

### 실차 센서 예상 비용

| 센서 | 추천 제품 | 예상 가격 |
|------|----------|----------|
| LiDAR (16ch) | Velodyne VLP-16 또는 Ouster OS1-16 | $4,000~$8,000 |
| 카메라 (RGB-D) | Intel RealSense D455 | $300~$500 |
| GPS-RTK | u-blox F9P + 안테나 | $200~$500 |
| IMU | Xsens MTi-3 또는 BNO085 | $50~$300 |
| 엣지 컴퓨터 | Jetson Orin NX 16GB | $599 |
| **소계** | | **$5,000~$10,000** |

---

## 2. 소프트웨어 스택 비교

### A. 시각화 & 모니터링

| 기능 | Foxglove 데모 | 현재 RS500 | 상태 |
|------|-------------|-----------|------|
| 실시간 3D 시각화 | Foxglove Studio 20+ 패널 | foxglove_bridge 설치 + 레이아웃 구성 완료 | **Phase 1 완료** |
| 데이터 녹화/재생 (MCAP) | Foxglove + rosbag2 MCAP | recording_launch.py 구현 완료 | **Phase 1 완료** |
| 웹 기반 원격 모니터링 | Foxglove Web App | Foxglove 무료 플랜 사용 가능 | 설정만 하면 됨 |
| 다중 로봇 플릿 관리 | Foxglove Data Platform | 해당 없음 (단일 차량) | 불필요 |
| 이벤트 마킹 & 분석 | Foxglove Events | 없음 | Phase 4 |

### B. 인지 (Perception)

| 기능 | 프로덕션 수준 | 현재 RS500 | 상태 |
|------|------------|-----------|------|
| LiDAR 장애물 감지 | PointPillars/CenterPoint (DL) | Ray Ground Filter + Euclidean Clustering | Classical, 농업용 충분 |
| 카메라 객체 탐지 | YOLO + TensorRT | YOLO11n 설치 + 모델 다운로드 완료 | **Phase 1 완료** |
| 시맨틱 세그멘테이션 | DeepLabV3+ / SegFormer | ENet/BiSeNetV2 + PyTorch 설치 완료 | **Phase 1 완료** |
| 센서 퓨전 | Early/Mid Fusion (DL) | Late Fusion (규칙 기반) | 농업용 충분 |
| 작물 행 인식 | DL 세그멘테이션 + 행 피팅 | Classical CV (C46 완료) | Phase 2 DL 교체 |
| GPS-RTK 정밀 측위 | cm급 RTK-GNSS | 시뮬레이션 GPS (2.5m 오차) | 하드웨어 필요 |

### C. 내비게이션 & 계획

| 기능 | 상태 |
|------|------|
| 전역 경로 계획 (SmacPlannerHybrid) | **완료** |
| 지역 경로 계획 (MPPI) | **완료** |
| 커버리지 계획 (CoveragePlanner 3패턴) | **완료** |
| SLAM (LIO-SAM) | 설정 완료, 실제 구동 필요 |
| 웨이포인트 네비게이션 | **완료** |
| Behavior Tree (Nav2 BT) | **완료** |

### D. 제어

| 기능 | 상태 |
|------|------|
| 경로 추종 (Pure Pursuit + 슬립 보상) | **완료** |
| 스키드 스티어 (CalibratedSkidSteerModel) | **완료** |
| CAN 통신 (SS500 CAN 코덱) | **완료** (실차 연결만 필요) |
| 안전 시스템 (E-stop) | 소프트웨어만, 하드웨어 필요 |

### E. 데이터 & 인프라

| 기능 | 상태 |
|------|------|
| 데이터 녹화 (rosbag2 MCAP) | **Phase 1 완료** |
| CI/CD | TODO (Phase 2) |
| 테스트 커버리지 | 343건 양호 |

---

## 3. Phase 1 구현 내역 (2026-03-22)

| 작업 | 파일 | 내용 |
|------|------|------|
| Foxglove Bridge | 이미 설치됨 (`ros-jazzy-foxglove-bridge`) | simulation_launch.py에 포함 확인 |
| Foxglove 레이아웃 강화 | `config/foxglove_layout.json` | 속도 그래프 패널 + perception 로그 필터 추가 |
| rosbag2 MCAP 녹화 | `launch/recording_launch.py` (신규) | 25개 토픽 MCAP 녹화, zstd 압축 |
| 시뮬레이션 녹화 통합 | `launch/simulation_launch.py` (수정) | `use_recording:=true` 인자로 녹화 활성화 |
| YOLO11n 활성화 | `pip install ultralytics` + 모델 다운로드 | `models/yolo11n.pt`, GPU 추론 확인 |
| PyTorch 활성화 | ultralytics 설치 시 함께 설치됨 | torch 2.10.0+cu128, RTX 2060 CUDA 확인 |
| Perception 파라미터 | `config/perception_params.yaml` (수정) | YOLO 모델 경로 + 세그멘테이션 설정 추가 |
| Perception 노드 | `perception_node.py` (수정) | YOLO 모델 자동 탐색 + 파라미터 연동 |

---

## 4. 우선순위별 로드맵

### Phase 2: 소프트웨어 개발 (2~4주)

- [ ] LIO-SAM 실제 구동 테스트
- [ ] crop_row_detector DL 교체 (DeepLabV3 + 실제 데이터)
- [ ] CI/CD 파이프라인 (GitHub Actions + pytest)
- [ ] 미션 매니저 (과수원 자동 순회)

### Phase 3: 하드웨어 확보 (예산 의존)

- [ ] Jetson Orin NX 16GB ($599)
- [ ] GPS-RTK u-blox F9P ($200~500)
- [ ] LiDAR Ouster OS1-16 ($4,000~8,000)
- [ ] RealSense D455 ($300~500)
- [ ] SS500 실차 CAN 연결

### Phase 4: 프로덕션 (3~6개월)

- [ ] 실차 System Identification
- [ ] 하드웨어 안전 시스템 (E-stop, Watchdog)
- [ ] Foxglove Data Platform 클라우드 연동
- [ ] 다중 과수원 필드 테스트
- [ ] TensorRT 추론 최적화

---

## 5. 현재 달성률

```
소프트웨어 전체              ████████████████████░░░░░░  75% (Phase 1 이후)
  알고리즘/제어/계획:        ██████████████████████████  95%
  인지 (DL):                ████████████████░░░░░░░░░░  60% (YOLO+PyTorch 활성화)
  시각화/모니터링:            ██████████████████████░░░░  80% (Foxglove Bridge 구성)
  데이터 파이프라인:          ████████████████░░░░░░░░░░  50% (MCAP 녹화 구현)
  실차 연동:                ████░░░░░░░░░░░░░░░░░░░░░░  15% (CAN 코덱만)
```
