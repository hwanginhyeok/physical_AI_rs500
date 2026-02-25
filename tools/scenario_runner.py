#!/usr/bin/env python3
"""논/밭 시나리오 배치 시뮬레이션 러너.

미리 정의된 경로와 지형 조합으로 L1/L2 시뮬레이션을 자동 실행하고
비교 결과를 차트로 출력한다.

사용법:
    python tools/scenario_runner.py
    python tools/scenario_runner.py --scenario straight
    python tools/scenario_runner.py --save results/
"""

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'ad_core'))

import matplotlib.pyplot as plt
import numpy as np

from ad_core.datatypes import Pose2D
from ad_core.pure_pursuit import PurePursuitConfig, PurePursuitTracker
from ad_core.skid_steer_model import SkidSteerModel
from ad_core.vehicle_dynamics import VehicleDynamics, VehicleDynamicsConfig
from ad_core.drivetrain_model import DrivetrainModel, DrivetrainConfig
from ad_core.track_terrain_interaction import (
    TrackTerrainInteraction,
    TerrainType,
    get_terrain_config,
)
from ad_core.sensor_noise_model import (
    SensorNoiseBundle,
    SensorNoiseConfig,
    GPSNoiseConfig,
    IMUNoiseConfig,
)


# ====================================================================== #
#  상수
# ====================================================================== #

PHYSICS_DT = 0.01
CONTROL_DT = 0.05
TRACK_WIDTH = 1.4
MAX_SPEED = 1.0
VEHICLE_MASS = 200.0
MAX_SIM_TIME = 120.0  # 최대 시뮬 시간 (s)


# ====================================================================== #
#  시나리오 정의
# ====================================================================== #

def make_straight_path(length: float = 15.0, spacing: float = 0.2) -> List[Tuple[float, float]]:
    """직선 경로."""
    n = int(length / spacing)
    return [(i * spacing + 2.0, 10.0) for i in range(n + 1)]


def make_s_curve_path(spacing: float = 0.2) -> List[Tuple[float, float]]:
    """S자 커브 경로."""
    path = []
    for i in range(int(20.0 / spacing) + 1):
        x = i * spacing + 2.0
        y = 10.0 + 3.0 * math.sin(x * math.pi / 10.0)
        path.append((x, y))
    return path


def make_tight_turn_path(spacing: float = 0.2) -> List[Tuple[float, float]]:
    """급선회 포함 경로 (직진 → 90도 좌회전 → 직진 → 90도 우회전 → 직진)."""
    path = []
    # 직진 5m
    for i in range(int(5.0 / spacing) + 1):
        path.append((2.0 + i * spacing, 8.0))
    # 좌회전 (반경 3m, 90도)
    cx, cy = 7.0, 11.0
    for i in range(1, int(math.pi / 2 / (spacing / 3.0)) + 1):
        angle = -math.pi / 2 + i * (spacing / 3.0)
        path.append((cx + 3.0 * math.cos(angle), cy + 3.0 * math.sin(angle)))
    # 직진 5m (북쪽)
    last_x, last_y = path[-1]
    for i in range(1, int(5.0 / spacing) + 1):
        path.append((last_x, last_y + i * spacing))
    # 우회전 (반경 3m, 90도)
    last_x, last_y = path[-1]
    cx2, cy2 = last_x + 3.0, last_y
    for i in range(1, int(math.pi / 2 / (spacing / 3.0)) + 1):
        angle = math.pi - i * (spacing / 3.0)
        path.append((cx2 + 3.0 * math.cos(angle), cy2 + 3.0 * math.sin(angle)))
    # 직진 5m (동쪽)
    last_x, last_y = path[-1]
    for i in range(1, int(5.0 / spacing) + 1):
        path.append((last_x + i * spacing, last_y))
    return path


SCENARIOS = {
    'straight': ('직선 15m', make_straight_path),
    's_curve': ('S자 커브', make_s_curve_path),
    'tight_turn': ('급선회', make_tight_turn_path),
}

TERRAINS = [
    TerrainType.PAVED,
    TerrainType.FIELD_HARD,
    TerrainType.FIELD_SOFT,
    TerrainType.PADDY_DRY,
    TerrainType.PADDY_WET,
]

TERRAIN_LABELS = {
    TerrainType.PAVED: '포장도로',
    TerrainType.FIELD_HARD: '마른 밭',
    TerrainType.FIELD_SOFT: '부드러운 밭',
    TerrainType.PADDY_DRY: '마른 논',
    TerrainType.PADDY_WET: '젖은 논',
}

TERRAIN_COLORS = {
    TerrainType.PAVED: '#333333',
    TerrainType.FIELD_HARD: '#8B4513',
    TerrainType.FIELD_SOFT: '#D2691E',
    TerrainType.PADDY_DRY: '#DAA520',
    TerrainType.PADDY_WET: '#2E8B57',
}


# ====================================================================== #
#  시뮬 결과 기록
# ====================================================================== #

@dataclass
class RunResult:
    """단일 시뮬 실행 결과."""
    terrain: TerrainType
    level: str  # 'L1' or 'L2'
    scenario: str
    poses: List[Tuple[float, float]] = field(default_factory=list)
    speeds: List[float] = field(default_factory=list)
    slips: List[float] = field(default_factory=list)
    sinkages: List[float] = field(default_factory=list)
    cross_track_errors: List[float] = field(default_factory=list)
    front_loads: List[float] = field(default_factory=list)
    rear_loads: List[float] = field(default_factory=list)
    yaw_rates_true: List[float] = field(default_factory=list)
    yaw_rates_imu: List[float] = field(default_factory=list)
    total_time: float = 0.0
    completed: bool = False


# ====================================================================== #
#  시뮬레이션 엔진
# ====================================================================== #

def run_simulation(
    path: List[Tuple[float, float]],
    terrain: TerrainType,
    enable_physics: bool,
    scenario_name: str,
) -> RunResult:
    """경로와 지형으로 시뮬레이션을 실행하고 결과를 반환."""
    level = 'L2' if enable_physics else 'L1'
    result = RunResult(terrain=terrain, level=level, scenario=scenario_name)

    # 운동학 모델
    kinematic = SkidSteerModel(
        track_width=TRACK_WIDTH, max_speed=MAX_SPEED * 2.0, steering_efficiency=0.8,
    )

    # Pure Pursuit
    tracker = PurePursuitTracker(PurePursuitConfig(
        lookahead_gain=0.8, min_lookahead=1.0, max_lookahead=5.0,
        goal_tolerance=0.5, max_linear_speed=MAX_SPEED, track_width=TRACK_WIDTH,
    ))
    tracker.set_path(path)

    # 지면 상호작용
    terrain_interaction = TrackTerrainInteraction(
        terrain=terrain, track_width=0.3, track_length=1.2,
    )

    # 동역학 (L2) — 지면 상호작용 연결
    dynamics = VehicleDynamics(
        VehicleDynamicsConfig(
            mass=VEHICLE_MASS, Izz=50.0, cog_height=0.4,
            track_width=TRACK_WIDTH, max_speed=MAX_SPEED,
            max_accel=1.0, max_decel=2.0,
        ),
        kinematic,
        terrain_interaction=terrain_interaction,
    )

    # 구동계
    drivetrain = DrivetrainModel(DrivetrainConfig(time_constant=0.15, deadzone=0.05))

    # 센서 노이즈
    sensor_noise = SensorNoiseBundle(SensorNoiseConfig(
        gps=GPSNoiseConfig(position_stddev=0.5, dropout_probability=0.02),
        imu=IMUNoiseConfig(gyro_noise_density=0.01, gyro_bias_instability=0.001),
    ))
    sensor_noise.reset(seed=42)

    # 초기 상태
    p0, p1 = path[0], path[1]
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    pose = Pose2D(x=p0[0], y=p0[1], yaw=yaw)
    speed = 0.0
    v_left, v_right = 0.0, 0.0
    control_acc = 0.0
    sim_time = 0.0

    result.poses.append((pose.x, pose.y))

    while sim_time < MAX_SIM_TIME:
        sim_time += PHYSICS_DT

        # 제어
        control_acc += PHYSICS_DT
        if control_acc >= CONTROL_DT:
            control_acc = 0.0
            linear, angular = tracker.compute(pose, speed)
            v_left, v_right = kinematic.twist_to_tracks(linear, angular)

        if enable_physics:
            # L2: 구동계 → 동역학 (지면 상호작용 내부 반영)
            actual_left, actual_right = drivetrain.update(v_left, v_right, PHYSICS_DT)
            pose = dynamics.step(actual_left, actual_right, PHYSICS_DT, pose)

            speed = abs(dynamics.current_linear_velocity)
            yaw_rate = dynamics.current_angular_velocity

            result.slips.append(dynamics.last_slip_ratio)
            result.sinkages.append(dynamics.last_sinkage)
            result.front_loads.append(dynamics.front_normal_force)
            result.rear_loads.append(dynamics.rear_normal_force)
        else:
            # L1
            pose = kinematic.predict_pose(pose, v_left, v_right, PHYSICS_DT)
            speed = (abs(v_left) + abs(v_right)) / 2.0
            yaw_rate = (v_right - v_left) * 0.8 / TRACK_WIDTH

            result.slips.append(0.0)
            result.sinkages.append(0.0)
            half_w = VEHICLE_MASS * 9.81 / 2.0
            result.front_loads.append(half_w)
            result.rear_loads.append(half_w)

        # 센서
        imu_yaw = sensor_noise.imu.apply_gyro(yaw_rate, PHYSICS_DT)
        result.yaw_rates_true.append(yaw_rate)
        result.yaw_rates_imu.append(imu_yaw)

        # 기록
        result.poses.append((pose.x, pose.y))
        result.speeds.append(speed)

        # CTE
        cte = compute_cte(pose, path)
        result.cross_track_errors.append(cte)

        # 완료 체크
        if tracker.is_goal_reached:
            result.completed = True
            break

    result.total_time = sim_time
    return result


def compute_cte(pose: Pose2D, path: List[Tuple[float, float]]) -> float:
    """Cross-Track Error 계산."""
    min_dist = float('inf')
    px, py = pose.x, pose.y
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        abx, aby = bx - ax, by - ay
        ab_sq = abx * abx + aby * aby
        if ab_sq < 1e-12:
            dist = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab_sq))
            dist = math.hypot(px - (ax + t * abx), py - (ay + t * aby))
        min_dist = min(min_dist, dist)
    return min_dist


# ====================================================================== #
#  결과 시각화
# ====================================================================== #

def plot_scenario_results(
    scenario_name: str,
    scenario_label: str,
    path: List[Tuple[float, float]],
    results: List[RunResult],
    save_dir: str = None,
) -> None:
    """하나의 시나리오에 대한 전체 지형 비교 차트."""

    l2_results = [r for r in results if r.level == 'L2']
    l1_result = next((r for r in results if r.level == 'L1'), None)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f'시나리오: {scenario_label} — 지형별 L2 비교 (vs L1 기준)', fontsize=14)

    # ── 1. 궤적 비교 ──
    ax = axes[0, 0]
    px, py = zip(*path)
    ax.plot(px, py, 'k--', linewidth=1, alpha=0.5, label='계획 경로')
    if l1_result:
        tx, ty = zip(*l1_result.poses)
        ax.plot(tx, ty, 'gray', linewidth=1.5, alpha=0.6, label='L1 (운동학)')
    for r in l2_results:
        tx, ty = zip(*r.poses)
        color = TERRAIN_COLORS[r.terrain]
        ax.plot(tx, ty, color=color, linewidth=1.5, label=TERRAIN_LABELS[r.terrain])
    ax.set_title('궤적')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.legend(fontsize=7, loc='best')
    ax.grid(True, alpha=0.3)

    # ── 2. CTE 비교 ──
    ax = axes[0, 1]
    if l1_result and l1_result.cross_track_errors:
        t_axis = np.arange(len(l1_result.cross_track_errors)) * PHYSICS_DT
        ax.plot(t_axis, l1_result.cross_track_errors, 'gray', alpha=0.5, label='L1')
    for r in l2_results:
        if r.cross_track_errors:
            t_axis = np.arange(len(r.cross_track_errors)) * PHYSICS_DT
            ax.plot(t_axis, r.cross_track_errors, color=TERRAIN_COLORS[r.terrain],
                    label=TERRAIN_LABELS[r.terrain])
    ax.set_title('Cross-Track Error')
    ax.set_xlabel('시간 (s)')
    ax.set_ylabel('CTE (m)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 3. 속도 프로파일 ──
    ax = axes[0, 2]
    if l1_result and l1_result.speeds:
        t_axis = np.arange(len(l1_result.speeds)) * PHYSICS_DT
        ax.plot(t_axis, l1_result.speeds, 'gray', alpha=0.5, label='L1')
    for r in l2_results:
        if r.speeds:
            t_axis = np.arange(len(r.speeds)) * PHYSICS_DT
            ax.plot(t_axis, r.speeds, color=TERRAIN_COLORS[r.terrain],
                    label=TERRAIN_LABELS[r.terrain])
    ax.set_title('속도')
    ax.set_xlabel('시간 (s)')
    ax.set_ylabel('속도 (m/s)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 4. 슬립 비교 ──
    ax = axes[1, 0]
    for r in l2_results:
        if r.slips:
            t_axis = np.arange(len(r.slips)) * PHYSICS_DT
            ax.plot(t_axis, r.slips, color=TERRAIN_COLORS[r.terrain],
                    label=TERRAIN_LABELS[r.terrain])
    ax.set_title('슬립 비율')
    ax.set_xlabel('시간 (s)')
    ax.set_ylabel('Slip Ratio')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 5. 침하량 비교 ──
    ax = axes[1, 1]
    for r in l2_results:
        if r.sinkages:
            t_axis = np.arange(len(r.sinkages)) * PHYSICS_DT
            ax.plot(t_axis, r.sinkages, color=TERRAIN_COLORS[r.terrain],
                    label=TERRAIN_LABELS[r.terrain])
    ax.set_title('궤도 침하량')
    ax.set_xlabel('시간 (s)')
    ax.set_ylabel('침하량 (m)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 6. 요약 바 차트 ──
    ax = axes[1, 2]
    labels = []
    rmse_vals = []
    colors = []

    if l1_result and l1_result.cross_track_errors:
        labels.append('L1 (기준)')
        rmse_vals.append(np.sqrt(np.mean(np.array(l1_result.cross_track_errors) ** 2)))
        colors.append('gray')

    for r in l2_results:
        if r.cross_track_errors:
            labels.append(TERRAIN_LABELS[r.terrain])
            rmse_vals.append(np.sqrt(np.mean(np.array(r.cross_track_errors) ** 2)))
            colors.append(TERRAIN_COLORS[r.terrain])

    if labels:
        bars = ax.bar(labels, rmse_vals, color=colors, alpha=0.8)
        ax.set_title('CTE RMSE 비교')
        ax.set_ylabel('RMSE (m)')
        ax.tick_params(axis='x', rotation=30)
        for bar, val in zip(bars, rmse_vals):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f'{val:.3f}', ha='center', va='bottom', fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    fig.tight_layout()

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        filepath = os.path.join(save_dir, f'scenario_{scenario_name}.png')
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f'  저장: {filepath}')


def print_summary_table(all_results: List[RunResult]) -> None:
    """전체 결과 요약 테이블 출력."""
    print('\n' + '=' * 80)
    print('  시뮬레이션 결과 요약')
    print('=' * 80)
    print(f'{"시나리오":<12} {"레벨":<5} {"지형":<12} {"완료":<5} '
          f'{"시간(s)":<8} {"CTE RMSE":<10} {"CTE Max":<10} '
          f'{"평균슬립":<10} {"평균침하(mm)":<12}')
    print('-' * 80)

    for r in all_results:
        cte_arr = np.array(r.cross_track_errors) if r.cross_track_errors else np.array([0])
        rmse = np.sqrt(np.mean(cte_arr ** 2))
        max_cte = np.max(cte_arr)
        avg_slip = np.mean(r.slips) if r.slips else 0
        avg_sink = np.mean(r.sinkages) * 1000 if r.sinkages else 0  # mm

        terrain_label = TERRAIN_LABELS.get(r.terrain, r.terrain.value)
        print(f'{r.scenario:<12} {r.level:<5} {terrain_label:<12} '
              f'{"O" if r.completed else "X":<5} '
              f'{r.total_time:<8.1f} {rmse:<10.4f} {max_cte:<10.4f} '
              f'{avg_slip:<10.4f} {avg_sink:<12.2f}')

    print('=' * 80)


# ====================================================================== #
#  메인
# ====================================================================== #

def main():
    import argparse
    parser = argparse.ArgumentParser(description='논/밭 시나리오 배치 시뮬레이션')
    parser.add_argument('--scenario', choices=list(SCENARIOS.keys()),
                        help='특정 시나리오만 실행')
    parser.add_argument('--save', type=str, default=None,
                        help='결과 이미지 저장 디렉토리')
    parser.add_argument('--no-plot', action='store_true',
                        help='차트 표시 안 함 (저장만)')
    args = parser.parse_args()

    scenarios = {args.scenario: SCENARIOS[args.scenario]} if args.scenario else SCENARIOS
    all_results = []

    for sc_name, (sc_label, path_fn) in scenarios.items():
        path = path_fn()
        print(f'\n{"="*60}')
        print(f'  시나리오: {sc_label} ({sc_name})')
        print(f'  경로점: {len(path)}개')
        print(f'{"="*60}')

        sc_results = []

        # L1 기준선
        print(f'  [L1] 운동학 기준선 실행...')
        r = run_simulation(path, TerrainType.PAVED, enable_physics=False, scenario_name=sc_name)
        sc_results.append(r)
        cte_rmse = np.sqrt(np.mean(np.array(r.cross_track_errors) ** 2)) if r.cross_track_errors else 0
        print(f'       완료={r.completed}, 시간={r.total_time:.1f}s, CTE RMSE={cte_rmse:.4f}m')

        # L2 지형별
        for terrain in TERRAINS:
            label = TERRAIN_LABELS[terrain]
            print(f'  [L2] {label} 실행...')
            r = run_simulation(path, terrain, enable_physics=True, scenario_name=sc_name)
            sc_results.append(r)
            cte_rmse = np.sqrt(np.mean(np.array(r.cross_track_errors) ** 2)) if r.cross_track_errors else 0
            avg_slip = np.mean(r.slips) if r.slips else 0
            avg_sink = np.mean(r.sinkages) * 1000 if r.sinkages else 0
            print(f'       완료={r.completed}, 시간={r.total_time:.1f}s, '
                  f'CTE={cte_rmse:.4f}m, 슬립={avg_slip:.4f}, 침하={avg_sink:.2f}mm')

        all_results.extend(sc_results)

        # 차트
        plot_scenario_results(sc_name, sc_label, path, sc_results, save_dir=args.save)

    # 전체 요약
    print_summary_table(all_results)

    if not args.no_plot:
        plt.show()


if __name__ == '__main__':
    main()
