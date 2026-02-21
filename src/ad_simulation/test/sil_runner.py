#!/usr/bin/env python3
"""SIL (Software-In-the-Loop) 테스트 러너.

Gazebo 시뮬레이션을 headless로 시작하고 웨이포인트 시퀀스를 발행한 뒤
odom 데이터를 수집하여 성능 메트릭을 출력한다.

사용법:
    # 기본 웨이포인트 시퀀스 실행
    python3 -m ad_simulation.test.sil_runner

    # 커스텀 웨이포인트 파일 지정
    python3 -m ad_simulation.test.sil_runner --waypoints waypoints.csv

    # 최대 실행 시간 지정
    python3 -m ad_simulation.test.sil_runner --timeout 120

메트릭 출력:
    - 경로 추종 RMSE (Root Mean Square Error)
    - 최대 횡방향(lateral) 오차
    - 완료 시간
    - 평균 속도
    - 웨이포인트 도달 성공률
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# --------------------------------------------------------------------------- #
# ROS2 의존성 임포트
# --------------------------------------------------------------------------- #
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


# --------------------------------------------------------------------------- #
# 데이터 클래스
# --------------------------------------------------------------------------- #

@dataclass
class Waypoint:
    """웨이포인트 정의."""
    x: float
    y: float
    tolerance: float = 1.0  # 도달 판정 허용 반경 [m]


@dataclass
class OdomRecord:
    """odom 기록 엔트리."""
    timestamp: float    # 수신 시각 [s]
    x: float
    y: float
    yaw: float         # heading [rad]
    vx: float          # 선속도 [m/s]
    wz: float          # 각속도 [rad/s]


@dataclass
class SILMetrics:
    """SIL 테스트 성능 메트릭."""
    path_rmse: float = 0.0            # 경로 추종 RMSE [m]
    max_lateral_error: float = 0.0    # 최대 횡방향 오차 [m]
    completion_time: float = 0.0      # 완료 시간 [s]
    avg_speed: float = 0.0            # 평균 속도 [m/s]
    waypoints_reached: int = 0        # 도달한 웨이포인트 수
    waypoints_total: int = 0          # 전체 웨이포인트 수
    success: bool = False             # 전체 성공 여부

    def __str__(self) -> str:
        """사람이 읽기 쉬운 형태로 메트릭을 출력한다."""
        sep = '=' * 60
        return (
            f'\n{sep}\n'
            f'  SIL 테스트 결과\n'
            f'{sep}\n'
            f'  경로 추종 RMSE       : {self.path_rmse:.4f} m\n'
            f'  최대 횡방향 오차     : {self.max_lateral_error:.4f} m\n'
            f'  완료 시간            : {self.completion_time:.2f} s\n'
            f'  평균 속도            : {self.avg_speed:.3f} m/s\n'
            f'  웨이포인트 도달      : {self.waypoints_reached}/{self.waypoints_total}\n'
            f'  성공 여부            : {"성공" if self.success else "실패"}\n'
            f'{sep}\n'
        )


# --------------------------------------------------------------------------- #
# 유틸리티 함수
# --------------------------------------------------------------------------- #

def quaternion_to_yaw(orientation) -> float:
    """쿼터니언에서 yaw 각도를 추출한다."""
    q = orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """2D 유클리드 거리."""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def point_to_segment_distance(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> float:
    """점 (px, py)에서 선분 (ax,ay)-(bx,by)까지의 최단 거리."""
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab_sq = abx * abx + aby * aby
    if ab_sq < 1e-12:
        return distance_2d(px, py, ax, ay)

    t = (apx * abx + apy * aby) / ab_sq
    t = max(0.0, min(1.0, t))

    proj_x = ax + t * abx
    proj_y = ay + t * aby
    return distance_2d(px, py, proj_x, proj_y)


def compute_lateral_errors(
    odom_records: List[OdomRecord],
    waypoints: List[Waypoint],
) -> List[float]:
    """각 odom 지점에서 참조 경로(웨이포인트 연결 선분)까지의 횡방향 오차를 계산한다."""
    if len(waypoints) < 2 or not odom_records:
        return []

    # 참조 경로: 웨이포인트를 순서대로 연결한 선분들
    segments = []
    for i in range(len(waypoints) - 1):
        segments.append((
            waypoints[i].x, waypoints[i].y,
            waypoints[i + 1].x, waypoints[i + 1].y,
        ))

    errors = []
    for rec in odom_records:
        min_dist = float('inf')
        for ax, ay, bx, by in segments:
            dist = point_to_segment_distance(rec.x, rec.y, ax, ay, bx, by)
            min_dist = min(min_dist, dist)
        errors.append(min_dist)

    return errors


# --------------------------------------------------------------------------- #
# 기본 웨이포인트 시퀀스
# --------------------------------------------------------------------------- #

DEFAULT_WAYPOINTS: List[Waypoint] = [
    Waypoint(x=0.0,  y=0.0,  tolerance=1.0),   # 출발점
    Waypoint(x=10.0, y=0.0,  tolerance=1.0),   # 직진 10m
    Waypoint(x=10.0, y=10.0, tolerance=1.5),   # 좌회전 후 직진
    Waypoint(x=0.0,  y=10.0, tolerance=1.5),   # 좌회전 후 직진
    Waypoint(x=0.0,  y=0.0,  tolerance=2.0),   # 원점 복귀
]


def load_waypoints_from_csv(filepath: str) -> List[Waypoint]:
    """CSV 파일에서 웨이포인트를 로드한다.

    CSV 형식: x, y [, tolerance]
    """
    waypoints = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            x = float(row[0].strip())
            y = float(row[1].strip())
            tol = float(row[2].strip()) if len(row) > 2 else 1.0
            waypoints.append(Waypoint(x=x, y=y, tolerance=tol))
    return waypoints


# --------------------------------------------------------------------------- #
# 순수 추종 (Pure Pursuit) 간이 컨트롤러
# --------------------------------------------------------------------------- #

class PurePursuitController:
    """간이 Pure Pursuit 경로 추종 컨트롤러.

    웨이포인트 시퀀스를 순서대로 추종하며 cmd_vel을 계산한다.
    """

    def __init__(
        self,
        waypoints: List[Waypoint],
        lookahead: float = 2.0,
        max_linear: float = 1.0,
        max_angular: float = 1.0,
    ):
        self.waypoints = waypoints
        self.lookahead = lookahead
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.current_wp_idx = 1  # 첫 번째는 출발점이므로 1부터 시작
        self.finished = False

    def compute_cmd_vel(
        self, x: float, y: float, yaw: float,
    ) -> Tuple[float, float]:
        """현재 위치/heading에서 cmd_vel (linear.x, angular.z)을 계산한다."""
        if self.finished or self.current_wp_idx >= len(self.waypoints):
            self.finished = True
            return 0.0, 0.0

        target = self.waypoints[self.current_wp_idx]

        # 현재 웨이포인트까지 거리
        dist = distance_2d(x, y, target.x, target.y)

        # 웨이포인트 도달 판정
        if dist < target.tolerance:
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.finished = True
                return 0.0, 0.0
            target = self.waypoints[self.current_wp_idx]
            dist = distance_2d(x, y, target.x, target.y)

        # 목표 방향 계산
        target_yaw = math.atan2(target.y - y, target.x - x)
        yaw_error = math.atan2(
            math.sin(target_yaw - yaw),
            math.cos(target_yaw - yaw),
        )

        # 선속도: 직선 거리 기반 (가까우면 감속)
        linear_x = min(self.max_linear, dist * 0.5)
        # 각도 오차가 크면 선속도 감소 (회전 우선)
        if abs(yaw_error) > math.radians(45):
            linear_x *= 0.3

        # 각속도: 비례 제어
        kp_angular = 2.0
        angular_z = kp_angular * yaw_error
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        return linear_x, angular_z


# --------------------------------------------------------------------------- #
# Gazebo 프로세스 관리
# --------------------------------------------------------------------------- #

def _find_world_sdf() -> str:
    """SDF 월드 파일 경로를 탐색한다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('ad_simulation')
        sdf = os.path.join(pkg, 'worlds', 'agricultural_field.sdf')
        if os.path.isfile(sdf):
            return sdf
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    sdf = os.path.normpath(
        os.path.join(here, '..', 'worlds', 'agricultural_field.sdf')
    )
    if os.path.isfile(sdf):
        return sdf

    raise FileNotFoundError('agricultural_field.sdf 파일을 찾을 수 없습니다.')


def start_gazebo_headless() -> Optional[subprocess.Popen]:
    """Gazebo를 headless 서버 모드로 시작한다."""
    gz_cmd = 'gz'
    try:
        subprocess.run(
            [gz_cmd, 'sim', '--version'],
            capture_output=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print('[경고] gz sim 명령을 찾을 수 없습니다. 시뮬레이션 없이 진행합니다.')
        return None

    world_sdf = _find_world_sdf()
    model_dir = os.path.normpath(
        os.path.join(os.path.dirname(world_sdf), '..', 'models')
    )

    env = os.environ.copy()
    env['GZ_SIM_HEADLESS'] = '1'
    existing = env.get('GZ_SIM_RESOURCE_PATH', '')
    env['GZ_SIM_RESOURCE_PATH'] = (
        model_dir + os.pathsep + existing if existing else model_dir
    )

    proc = subprocess.Popen(
        [gz_cmd, 'sim', '-r', '-s', world_sdf],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # 서버 시작 대기
    print('[정보] Gazebo 서버 시작 중...')
    startup_timeout = 30.0
    start_time = time.time()
    while time.time() - start_time < startup_timeout:
        if proc.poll() is not None:
            print('[오류] Gazebo 프로세스가 시작 중 종료되었습니다.')
            return None
        try:
            result = subprocess.run(
                [gz_cmd, 'topic', '-l'],
                capture_output=True,
                text=True,
                timeout=5,
                env=env,
            )
            if '/clock' in result.stdout:
                print('[정보] Gazebo 서버 준비 완료.')
                return proc
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        time.sleep(1.0)

    print('[경고] Gazebo 서버 시작 타임아웃. 프로세스는 계속 실행 중.')
    return proc


def stop_gazebo(proc: Optional[subprocess.Popen]) -> None:
    """Gazebo 프로세스를 종료한다."""
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
    print('[정보] Gazebo 서버 종료.')


# --------------------------------------------------------------------------- #
# SIL 테스트 실행 (ROS2 연동)
# --------------------------------------------------------------------------- #

def run_sil_with_ros2(
    waypoints: List[Waypoint],
    timeout: float = 120.0,
) -> Tuple[List[OdomRecord], SILMetrics]:
    """ROS2 + Gazebo 환경에서 SIL 테스트를 실행한다."""
    if not HAS_RCLPY:
        raise RuntimeError('rclpy가 설치되어 있지 않습니다.')

    rclpy.init()
    node = rclpy.create_node('sil_runner_node')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # 퍼블리셔 / 서브스크라이버
    pub_cmd = node.create_publisher(Twist, '/cmd_vel', 10)

    odom_records: List[OdomRecord] = []
    current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'vx': 0.0, 'wz': 0.0}

    def odom_callback(msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = quaternion_to_yaw(pose.orientation)
        current_pose['x'] = pose.position.x
        current_pose['y'] = pose.position.y
        current_pose['yaw'] = yaw
        current_pose['vx'] = twist.linear.x
        current_pose['wz'] = twist.angular.z

        odom_records.append(OdomRecord(
            timestamp=time.time(),
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw,
            vx=twist.linear.x,
            wz=twist.angular.z,
        ))

    sub_odom = node.create_subscription(Odometry, '/odom', odom_callback, 10)

    # 컨트롤러 초기화
    controller = PurePursuitController(
        waypoints=waypoints,
        lookahead=2.0,
        max_linear=1.0,
        max_angular=1.0,
    )

    # 메인 루프
    print(f'[정보] SIL 실행 시작 (웨이포인트 {len(waypoints)}개, 타임아웃 {timeout}초)')
    start_time = time.time()
    rate_hz = 20  # 제어 주기 [Hz]
    dt = 1.0 / rate_hz

    while time.time() - start_time < timeout:
        executor.spin_once(timeout_sec=dt)

        if controller.finished:
            print('[정보] 모든 웨이포인트 도달 완료.')
            break

        # cmd_vel 계산 및 발행
        vx, wz = controller.compute_cmd_vel(
            current_pose['x'],
            current_pose['y'],
            current_pose['yaw'],
        )

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        pub_cmd.publish(cmd)

    # 정지 명령
    stop_cmd = Twist()
    pub_cmd.publish(stop_cmd)

    elapsed = time.time() - start_time

    # 정리
    node.destroy_publisher(pub_cmd)
    node.destroy_subscription(sub_odom)
    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

    # 메트릭 계산
    metrics = compute_metrics(odom_records, waypoints, elapsed)
    return odom_records, metrics


# --------------------------------------------------------------------------- #
# SIL 테스트 실행 (오프라인 mock 모드)
# --------------------------------------------------------------------------- #

def run_sil_offline(
    waypoints: List[Waypoint],
    timeout: float = 120.0,
) -> Tuple[List[OdomRecord], SILMetrics]:
    """시뮬레이션 없이 오프라인 mock으로 SIL 테스트를 실행한다.

    MockVehicleState 기반의 이상적 운동 모델을 사용한다.
    """
    from test_straight_drive import MockVehicleState

    vehicle = MockVehicleState(
        x=waypoints[0].x if waypoints else 0.0,
        y=waypoints[0].y if waypoints else 0.0,
        yaw=0.0,
        max_vx=2.0,
        max_wz=1.5,
        dt=0.05,  # 20Hz
    )

    controller = PurePursuitController(
        waypoints=waypoints,
        lookahead=2.0,
        max_linear=1.0,
        max_angular=1.0,
    )

    odom_records: List[OdomRecord] = []
    sim_time = 0.0
    max_sim_time = timeout

    print(f'[정보] 오프라인 SIL 실행 (웨이포인트 {len(waypoints)}개)')
    start_real = time.time()

    while sim_time < max_sim_time:
        if controller.finished:
            print('[정보] 모든 웨이포인트 도달 완료.')
            break

        vx, wz = controller.compute_cmd_vel(vehicle.x, vehicle.y, vehicle.yaw)
        vehicle.step(linear_x=vx, angular_z=wz, steps=1)
        sim_time += vehicle.dt

        odom_records.append(OdomRecord(
            timestamp=sim_time,
            x=vehicle.x,
            y=vehicle.y,
            yaw=vehicle.yaw,
            vx=vehicle.vx,
            wz=vehicle.wz,
        ))

    elapsed = sim_time
    metrics = compute_metrics(odom_records, waypoints, elapsed)
    real_elapsed = time.time() - start_real
    print(f'[정보] 오프라인 실행 완료 (실시간: {real_elapsed:.2f}초, 시뮬레이션: {sim_time:.2f}초)')

    return odom_records, metrics


# --------------------------------------------------------------------------- #
# 메트릭 계산
# --------------------------------------------------------------------------- #

def compute_metrics(
    odom_records: List[OdomRecord],
    waypoints: List[Waypoint],
    elapsed: float,
) -> SILMetrics:
    """odom 기록과 웨이포인트에서 성능 메트릭을 계산한다."""
    metrics = SILMetrics()
    metrics.waypoints_total = len(waypoints)
    metrics.completion_time = elapsed

    if not odom_records:
        return metrics

    # 횡방향 오차 계산
    lateral_errors = compute_lateral_errors(odom_records, waypoints)
    if lateral_errors:
        # RMSE
        sum_sq = sum(e * e for e in lateral_errors)
        metrics.path_rmse = math.sqrt(sum_sq / len(lateral_errors))
        # 최대 오차
        metrics.max_lateral_error = max(lateral_errors)

    # 평균 속도
    speeds = [abs(rec.vx) for rec in odom_records]
    if speeds:
        metrics.avg_speed = sum(speeds) / len(speeds)

    # 웨이포인트 도달 확인
    reached = 0
    for wp in waypoints:
        for rec in odom_records:
            if distance_2d(rec.x, rec.y, wp.x, wp.y) < wp.tolerance:
                reached += 1
                break
    metrics.waypoints_reached = reached
    metrics.success = (reached == len(waypoints))

    return metrics


# --------------------------------------------------------------------------- #
# odom 기록 CSV 저장
# --------------------------------------------------------------------------- #

def save_odom_csv(
    records: List[OdomRecord],
    filepath: str,
) -> None:
    """odom 기록을 CSV 파일로 저장한다."""
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'x', 'y', 'yaw', 'vx', 'wz'])
        for rec in records:
            writer.writerow([
                f'{rec.timestamp:.4f}',
                f'{rec.x:.6f}',
                f'{rec.y:.6f}',
                f'{rec.yaw:.6f}',
                f'{rec.vx:.6f}',
                f'{rec.wz:.6f}',
            ])
    print(f'[정보] odom 기록 저장: {filepath}')


# --------------------------------------------------------------------------- #
# 메인 엔트리포인트
# --------------------------------------------------------------------------- #

def main() -> int:
    """SIL 테스트 러너 메인 함수."""
    parser = argparse.ArgumentParser(
        description='ad_simulation SIL 테스트 러너',
    )
    parser.add_argument(
        '--waypoints', '-w',
        type=str,
        default=None,
        help='웨이포인트 CSV 파일 경로 (기본: 내장 사각형 경로)',
    )
    parser.add_argument(
        '--timeout', '-t',
        type=float,
        default=120.0,
        help='최대 실행 시간 [초] (기본: 120)',
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='odom 기록 CSV 출력 경로',
    )
    parser.add_argument(
        '--offline',
        action='store_true',
        default=False,
        help='시뮬레이션 없이 오프라인 mock 모드로 실행',
    )
    parser.add_argument(
        '--no-gazebo',
        action='store_true',
        default=False,
        help='Gazebo 자동 시작을 비활성화 (이미 실행 중인 경우)',
    )

    args = parser.parse_args()

    # 웨이포인트 로드
    if args.waypoints:
        waypoints = load_waypoints_from_csv(args.waypoints)
        print(f'[정보] 웨이포인트 파일 로드: {args.waypoints} ({len(waypoints)}개)')
    else:
        waypoints = DEFAULT_WAYPOINTS
        print(f'[정보] 기본 웨이포인트 사용 ({len(waypoints)}개)')

    # 오프라인 모드
    if args.offline:
        records, metrics = run_sil_offline(waypoints, args.timeout)
        print(metrics)
        if args.output:
            save_odom_csv(records, args.output)
        return 0 if metrics.success else 1

    # 온라인 모드 (ROS2 + Gazebo)
    if not HAS_RCLPY:
        print('[오류] rclpy가 설치되어 있지 않습니다.')
        print('[정보] --offline 옵션으로 오프라인 모드를 사용해 주세요.')
        return 1

    gz_proc = None
    if not args.no_gazebo:
        gz_proc = start_gazebo_headless()
        if gz_proc is None:
            print('[경고] Gazebo 시작 실패. --offline 모드로 전환합니다.')
            records, metrics = run_sil_offline(waypoints, args.timeout)
            print(metrics)
            if args.output:
                save_odom_csv(records, args.output)
            return 0 if metrics.success else 1

    try:
        records, metrics = run_sil_with_ros2(waypoints, args.timeout)
        print(metrics)
        if args.output:
            save_odom_csv(records, args.output)
    finally:
        if gz_proc is not None:
            stop_gazebo(gz_proc)

    return 0 if metrics.success else 1


if __name__ == '__main__':
    sys.exit(main())
