# Rosbag 녹화/분석 스킬 (rosbag-workflow)

> **트리거**: 아래 표현이 나오면 이 스킬을 즉시 실행한다.
> - "로스백 녹화" / "rosbag 녹화" / "녹화 시작"
> - "로스백 분석" / "rosbag 분석" / "백 분석"
> - "비교 분석" / "sim vs real 비교"

---

## 목적

Rosbag 녹화 → 시뮬레이션 실행 → 녹화 종료 → 분석 → 비교까지의 워크플로우를 표준화한다.

---

## 모드별 실행

### 모드 A — 녹화 모드

#### STEP A1 — 녹화 시작

```bash
ros2 launch ad_bringup record_launch.py robot:=sim
```

launch 파일이 없거나 커스텀 녹화가 필요한 경우:

```bash
# 전체 토픽 녹화
ros2 bag record -a -o {출력경로}

# 주요 토픽만 선택 녹화
ros2 bag record -o {출력경로} \
  /cmd_vel /cmd_vel_nav /cmd_vel_smoothed \
  /odom /imu/data /scan \
  /tf /tf_static \
  /local_costmap/costmap /global_costmap/costmap
```

사용자에게 녹화 대상 토픽과 출력 경로를 확인한다.

#### STEP A2 — 모니터링

녹화 중 상태를 주기적으로 확인:
- 녹화 중인 토픽 수
- 메시지 수신 여부 (`ros2 bag info` 실시간은 불가 → 파일 크기 증가로 간접 확인)

#### STEP A3 — 녹화 종료

사용자 요청 시 `Ctrl+C` 또는 프로세스 종료 안내.

```bash
# bag 파일 정보 확인
ros2 bag info {bag_path}
```

---

### 모드 B — 분석 모드

#### STEP B1 — Bag 정보 수집

```bash
ros2 bag info {bag_path}
```

출력에서 추출:
- 녹화 시간 (duration)
- 총 메시지 수
- 토픽별 메시지 수 및 주파수

#### STEP B2 — 토픽별 분석 테이블

```
📊 Rosbag 분석 — {bag_path}

녹화 시간: {duration}s
총 메시지: {count}

| 토픽 | 타입 | 메시지 수 | 평균 Hz | 비고 |
|------|------|----------|---------|------|
| /cmd_vel | Twist | 1200 | 20.0 | ✅ 정상 |
| /odom | Odometry | 600 | 10.0 | ✅ 정상 |
| /scan | LaserScan | 300 | 5.0 | ⚠️ 예상 10Hz |
| /cmd_vel_nav | Twist | 0 | 0.0 | ❌ 발행 없음 |
```

#### STEP B3 — 핵심 토픽 검증

다음 토픽의 발행 여부를 필수 확인:

| 토픽 | 예상 Hz | 없으면 |
|------|---------|--------|
| `/cmd_vel` | 10~20 | 차량 명령 미전달 |
| `/cmd_vel_nav` | 10~20 | Nav2 출력 없음 → controller_server 확인 |
| `/odom` | 10~50 | 오도메트리 미발행 → EKF 확인 |
| `/tf` | 10~50 | TF 발행 없음 → robot_state_publisher 확인 |

---

### 모드 C — 비교 분석 모드

#### STEP C1 — 두 Bag 로드

```bash
# 각각 info 확인
ros2 bag info {sim_bag_path}
ros2 bag info {real_bag_path}
```

#### STEP C2 — 동일 토픽 비교

두 bag에서 공통 토픽을 추출하여 비교:

```
🔄 Sim vs Real 비교 — {날짜}

| 토픽 | Sim Hz | Real Hz | Sim 메시지수 | Real 메시지수 | 차이 |
|------|--------|---------|-------------|-------------|------|
| /cmd_vel | 20.0 | 18.5 | 1200 | 1110 | 8% |
| /odom | 50.0 | 49.8 | 3000 | 2988 | ✅ |
```

#### STEP C3 — Foxglove 레이아웃 안내

비교 시각화가 필요하면 Foxglove Studio 사용을 안내:

```
Foxglove Studio에서 두 bag을 동시에 열어 비교할 수 있습니다:
1. File → Open → sim bag 선택
2. File → Open → real bag 선택 (두 번째 탭)
3. Layout 불러오기 (있는 경우)
```

---

## Bag 파일 네이밍 규칙

```
results/bags/YYMMDD_{sim|real}_{시나리오명}/
```

예시: `results/bags/260313_sim_waypoint_nav/`

---

## 판단 규칙

| 상황 | 행동 |
|------|------|
| 토픽 발행 없음 | 해당 노드 상태 확인 → dds-troubleshoot 스킬 연계 |
| Hz가 예상보다 낮음 | 노드 부하 또는 타이머 설정 확인 |
| sim vs real 차이 큼 | 시뮬레이션 모델 정확도 검토 |
| bag 파일 없음 | 녹화 모드(A)로 전환 제안 |

---

## 주의사항

- Bag 녹화는 디스크 공간을 빠르게 소비한다. 전체 토픽(`-a`) 녹화 시 주의
- 선택 녹화 시 `/tf`, `/tf_static`은 반드시 포함 (재생 시 TF 필요)
- `ros2 bag play` 시 시뮬레이션과 동시에 실행하면 충돌 가능 — 시뮬 종료 후 재생
- Bag 파일은 git에 커밋하지 않는다 (`.gitignore`에 `results/bags/` 확인)
