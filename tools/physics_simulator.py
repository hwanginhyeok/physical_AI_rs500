#!/usr/bin/env python3
"""Level 2 물리 시뮬레이터.

Phase 1~4 모듈(동역학, 구동계, 지면상호작용, 센서노이즈)을 조합한
통합 물리 시뮬레이션 엔진.

기존 path_follower_sim.py와 호환되며, 물리 시뮬레이션 옵션을 추가한다.

사용법:
    python tools/physics_simulator.py
    python tools/physics_simulator.py --scenario scenario.yaml
    python tools/physics_simulator.py --terrain paddy_wet

조작:
    좌클릭  - 웨이포인트 추가
    우클릭  - 마지막 웨이포인트 취소
    Enter   - 시뮬레이션 시작
    r       - 리셋
    t       - 지형 변경 (순환)
    1       - Level 1 (운동학) 전환
    2       - Level 2 (동역학) 전환
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ad_core 패키지 경로 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'ad_core'))

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
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
#  시뮬레이션 파라미터
# ====================================================================== #

PHYSICS_DT = 0.01        # 물리 루프 dt (s)
CONTROL_DT = 0.05        # 제어 루프 dt (s)
MAP_SIZE = 20.0
ANIMATION_INTERVAL = 50  # ms

# 차량 파라미터 (SS500 추정)
TRACK_WIDTH = 1.4
MAX_SPEED = 1.0
VEHICLE_MASS = 200.0

# 시각화
VEHICLE_LENGTH = 0.8
VEHICLE_WIDTH = 0.5

# 순환할 지형 목록
TERRAIN_CYCLE = [
    TerrainType.PAVED,
    TerrainType.FIELD_HARD,
    TerrainType.FIELD_SOFT,
    TerrainType.PADDY_DRY,
    TerrainType.PADDY_WET,
    TerrainType.MUD,
]


@dataclass
class SimulationState:
    """시뮬레이션 상태 기록."""

    time: float = 0.0
    poses: List[Tuple[float, float]] = field(default_factory=list)
    gps_positions: List[Tuple[float, float]] = field(default_factory=list)
    speeds: List[float] = field(default_factory=list)
    slips: List[float] = field(default_factory=list)
    sinkages: List[float] = field(default_factory=list)
    cross_track_errors: List[float] = field(default_factory=list)
    front_loads: List[float] = field(default_factory=list)
    rear_loads: List[float] = field(default_factory=list)
    yaw_rates_true: List[float] = field(default_factory=list)
    yaw_rates_imu: List[float] = field(default_factory=list)


class PhysicsSimulator:
    """Level 2 통합 물리 시뮬레이터."""

    def __init__(
        self,
        terrain: TerrainType = TerrainType.FIELD_HARD,
        enable_physics: bool = True,
        enable_sensor_noise: bool = True,
    ) -> None:
        self.terrain = terrain
        self.enable_physics = enable_physics
        self.enable_sensor_noise = enable_sensor_noise

        # 상태
        self.waypoints: List[Tuple[float, float]] = []
        self.pose = Pose2D()
        self.speed = 0.0
        self.state_str = 'drawing'  # 'drawing' | 'running' | 'finished'
        self.sim_state = SimulationState()
        self._terrain_idx = TERRAIN_CYCLE.index(terrain) if terrain in TERRAIN_CYCLE else 0
        self._control_accumulator = 0.0
        self._current_v_left = 0.0
        self._current_v_right = 0.0
        self.anim = None

        # 모듈 초기화
        self._init_modules()

        # matplotlib
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.canvas.manager.set_window_title(
            'Level 2 Physics Simulator'
        )
        self._setup_plots()

        # 이벤트 연결
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    def _init_modules(self) -> None:
        """물리 모듈을 초기화한다."""
        # 운동학 모델 (Level 1 / Level 2 공통 기반)
        self.kinematic = SkidSteerModel(
            track_width=TRACK_WIDTH,
            max_speed=MAX_SPEED * 2.0,
            steering_efficiency=0.8,
        )

        # Pure Pursuit
        pp_config = PurePursuitConfig(
            lookahead_gain=0.8,
            min_lookahead=1.0,
            max_lookahead=5.0,
            goal_tolerance=0.5,
            max_linear_speed=MAX_SPEED,
            track_width=TRACK_WIDTH,
        )
        self.tracker = PurePursuitTracker(pp_config)

        # 동역학 (Level 2)
        dyn_config = VehicleDynamicsConfig(
            mass=VEHICLE_MASS,
            Izz=50.0,
            cog_height=0.4,
            track_width=TRACK_WIDTH,
            max_speed=MAX_SPEED,
            max_accel=1.0,
            max_decel=2.0,
        )
        self.dynamics = VehicleDynamics(dyn_config, self.kinematic)

        # 구동계
        dt_config = DrivetrainConfig(
            time_constant=0.15,
            deadzone=0.05,
        )
        self.drivetrain = DrivetrainModel(dt_config)

        # 지면 상호작용
        self.terrain_interaction = TrackTerrainInteraction(
            terrain=self.terrain,
            track_width=0.3,
            track_length=1.2,
        )

        # 센서 노이즈
        gps_cfg = GPSNoiseConfig(position_stddev=0.5, dropout_probability=0.02)
        imu_cfg = IMUNoiseConfig(gyro_noise_density=0.01, gyro_bias_instability=0.001)
        noise_cfg = SensorNoiseConfig(gps=gps_cfg, imu=imu_cfg)
        self.sensor_noise = SensorNoiseBundle(noise_cfg)

    # ------------------------------------------------------------------ #
    #  UI 설정
    # ------------------------------------------------------------------ #

    def _setup_plots(self) -> None:
        ax_map = self.axes[0, 0]
        ax_map.set_xlim(-1, MAP_SIZE + 1)
        ax_map.set_ylim(-1, MAP_SIZE + 1)
        ax_map.set_aspect('equal')
        ax_map.grid(True, alpha=0.3)
        ax_map.set_title('Trajectory')
        ax_map.set_xlabel('X (m)')
        ax_map.set_ylabel('Y (m)')

        # 그래픽 요소
        self.wp_scatter, = ax_map.plot([], [], 'bo-', markersize=6, label='Plan')
        self.traj_line, = ax_map.plot([], [], 'r-', linewidth=2, label='Actual')
        self.gps_scatter, = ax_map.plot([], [], 'g.', markersize=2, alpha=0.3, label='GPS')
        self.vehicle_patch = None
        ax_map.legend(loc='upper right', fontsize=8)

        # 정보 텍스트
        self.info_text = ax_map.text(
            0.02, 0.98, '', transform=ax_map.transAxes,
            verticalalignment='top', fontsize=8, fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
        )
        self.status_text = ax_map.text(
            0.5, 0.5, self._get_status_message(),
            transform=ax_map.transAxes, ha='center', va='center',
            fontsize=11, color='gray', alpha=0.7,
        )

        # 속도/슬립 그래프
        ax_speed = self.axes[0, 1]
        ax_speed.set_title('Speed & Slip')
        ax_speed.set_xlabel('Step')
        ax_speed.set_ylabel('m/s')
        self.speed_line, = ax_speed.plot([], [], 'b-', label='Speed')
        self.slip_line, = ax_speed.plot([], [], 'r--', label='Slip')
        ax_speed.legend(fontsize=8)
        ax_speed.grid(True, alpha=0.3)

        # 하중 분포 그래프
        ax_load = self.axes[1, 0]
        ax_load.set_title('Load Distribution')
        ax_load.set_xlabel('Step')
        ax_load.set_ylabel('Force (N)')
        self.front_load_line, = ax_load.plot([], [], 'b-', label='Front')
        self.rear_load_line, = ax_load.plot([], [], 'r-', label='Rear')
        ax_load.legend(fontsize=8)
        ax_load.grid(True, alpha=0.3)

        # IMU 드리프트 그래프
        ax_imu = self.axes[1, 1]
        ax_imu.set_title('Yaw Rate (True vs IMU)')
        ax_imu.set_xlabel('Step')
        ax_imu.set_ylabel('rad/s')
        self.yaw_true_line, = ax_imu.plot([], [], 'b-', label='True')
        self.yaw_imu_line, = ax_imu.plot([], [], 'r-', alpha=0.5, label='IMU')
        ax_imu.legend(fontsize=8)
        ax_imu.grid(True, alpha=0.3)

        self.fig.tight_layout()

    def _get_status_message(self) -> str:
        level = "L2 Physics" if self.enable_physics else "L1 Kinematic"
        return (
            f'Click to add waypoints | Enter to start\n'
            f'[{level}] Terrain: {self.terrain.value}\n'
            f't=change terrain | 1/2=level switch | r=reset'
        )

    # ------------------------------------------------------------------ #
    #  이벤트 핸들러
    # ------------------------------------------------------------------ #

    def _on_click(self, event) -> None:
        if self.state_str != 'drawing':
            return
        if event.inaxes != self.axes[0, 0]:
            return

        if event.button == 1:
            self.waypoints.append((event.xdata, event.ydata))
            self._update_waypoint_display()
        elif event.button == 3:
            if self.waypoints:
                self.waypoints.pop()
                self._update_waypoint_display()

    def _on_key(self, event) -> None:
        if event.key == 'enter' and self.state_str == 'drawing':
            if len(self.waypoints) >= 2:
                self._start_simulation()
        elif event.key == 'r':
            self._reset()
        elif event.key == 't' and self.state_str == 'drawing':
            self._cycle_terrain()
        elif event.key == '1':
            self.enable_physics = False
            self.status_text.set_text(self._get_status_message())
            self.fig.canvas.draw_idle()
        elif event.key == '2':
            self.enable_physics = True
            self.status_text.set_text(self._get_status_message())
            self.fig.canvas.draw_idle()

    def _cycle_terrain(self) -> None:
        self._terrain_idx = (self._terrain_idx + 1) % len(TERRAIN_CYCLE)
        self.terrain = TERRAIN_CYCLE[self._terrain_idx]
        self.terrain_interaction.set_terrain(self.terrain)
        self.status_text.set_text(self._get_status_message())
        self.fig.canvas.draw_idle()

    def _update_waypoint_display(self) -> None:
        if self.waypoints:
            xs, ys = zip(*self.waypoints)
            self.wp_scatter.set_data(xs, ys)
        else:
            self.wp_scatter.set_data([], [])
        self.status_text.set_text(self._get_status_message())
        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------ #
    #  시뮬레이션
    # ------------------------------------------------------------------ #

    def _start_simulation(self) -> None:
        self.state_str = 'running'
        self.status_text.set_text('')
        self.sim_state = SimulationState()

        # 초기 위치
        p0 = self.waypoints[0]
        p1 = self.waypoints[1]
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        self.pose = Pose2D(x=p0[0], y=p0[1], yaw=yaw)
        self.speed = 0.0

        # 경로 설정
        dense_path = self._interpolate_path(self.waypoints, spacing=0.2)
        self.tracker = PurePursuitTracker(PurePursuitConfig(
            lookahead_gain=0.8, min_lookahead=1.0, max_lookahead=5.0,
            goal_tolerance=0.5, max_linear_speed=MAX_SPEED, track_width=TRACK_WIDTH,
        ))
        self.tracker.set_path(dense_path)

        # 모듈 리셋
        self.dynamics.reset()
        self.drivetrain.reset()
        self.sensor_noise.reset(seed=42)
        self._control_accumulator = 0.0
        self._current_v_left = 0.0
        self._current_v_right = 0.0

        self.sim_state.poses.append((self.pose.x, self.pose.y))

        self.anim = FuncAnimation(
            self.fig, self._update_frame, interval=ANIMATION_INTERVAL,
            blit=False, cache_frame_data=False,
        )
        self.fig.canvas.draw_idle()

    def _update_frame(self, frame) -> None:
        if self.state_str != 'running':
            if self.anim is not None:
                self.anim.event_source.stop()
            return

        # 제어 루프: CONTROL_DT마다 새 명령 계산
        self._control_accumulator += PHYSICS_DT * (CONTROL_DT / PHYSICS_DT)

        # 실제로는 한 프레임에 여러 물리 스텝
        steps_per_frame = max(1, int(CONTROL_DT / PHYSICS_DT))

        for _ in range(steps_per_frame):
            self._physics_step()

        # 시각화 갱신
        self._update_visuals()

        if self.tracker.is_goal_reached:
            self.state_str = 'finished'
            if self.anim is not None:
                self.anim.event_source.stop()
            self._show_result()

    def _physics_step(self) -> None:
        dt = PHYSICS_DT
        self.sim_state.time += dt

        # 제어기 (CONTROL_DT마다 갱신)
        self._control_accumulator += dt
        if self._control_accumulator >= CONTROL_DT:
            self._control_accumulator = 0.0
            linear, angular = self.tracker.compute(self.pose, self.speed)
            self._current_v_left, self._current_v_right = self.kinematic.twist_to_tracks(
                linear, angular
            )

        if self.enable_physics:
            # Level 2: 구동계 → 동역학 → 지면 상호작용
            actual_left, actual_right = self.drivetrain.update(
                self._current_v_left, self._current_v_right, dt
            )

            # 동역학 스텝
            self.pose = self.dynamics.step(actual_left, actual_right, dt, self.pose)

            # 지면 상호작용 (정보 수집)
            normal_force = self.dynamics.front_normal_force + self.dynamics.rear_normal_force
            half_normal = normal_force / 2.0
            slip_ratio = abs(actual_left - self.dynamics.current_linear_velocity) / max(
                abs(actual_left), 0.1
            )
            traction, rolling, turning, sinkage = self.terrain_interaction.compute_effective_force(
                half_normal, slip_ratio, self.dynamics.current_angular_velocity
            )

            self.speed = abs(self.dynamics.current_linear_velocity)
            yaw_rate = self.dynamics.current_angular_velocity

            # 기록
            self.sim_state.slips.append(slip_ratio)
            self.sim_state.sinkages.append(sinkage)
            self.sim_state.front_loads.append(self.dynamics.front_normal_force)
            self.sim_state.rear_loads.append(self.dynamics.rear_normal_force)

        else:
            # Level 1: 순수 운동학
            self.pose = self.kinematic.predict_pose(
                self.pose, self._current_v_left, self._current_v_right, dt
            )
            self.speed = (abs(self._current_v_left) + abs(self._current_v_right)) / 2.0
            yaw_rate = (self._current_v_right - self._current_v_left) * 0.8 / TRACK_WIDTH

            self.sim_state.slips.append(0.0)
            self.sim_state.sinkages.append(0.0)
            half_weight = VEHICLE_MASS * 9.81 / 2.0
            self.sim_state.front_loads.append(half_weight)
            self.sim_state.rear_loads.append(half_weight)

        # 센서 노이즈
        self.sim_state.yaw_rates_true.append(yaw_rate)
        if self.enable_sensor_noise:
            imu_yaw_rate = self.sensor_noise.imu.apply_gyro(yaw_rate, dt)
            gps_x, gps_y, gps_valid = self.sensor_noise.gps.apply(
                self.pose.x, self.pose.y, dt
            )
            if gps_valid:
                self.sim_state.gps_positions.append((gps_x, gps_y))
            self.sim_state.yaw_rates_imu.append(imu_yaw_rate)
        else:
            self.sim_state.yaw_rates_imu.append(yaw_rate)

        # 궤적/속도 기록
        self.sim_state.poses.append((self.pose.x, self.pose.y))
        self.sim_state.speeds.append(self.speed)

        # CTE 계산
        cte = self._compute_cte()
        self.sim_state.cross_track_errors.append(cte)

    # ------------------------------------------------------------------ #
    #  시각화
    # ------------------------------------------------------------------ #

    def _update_visuals(self) -> None:
        ax_map = self.axes[0, 0]

        # 궤적
        if self.sim_state.poses:
            xs, ys = zip(*self.sim_state.poses)
            self.traj_line.set_data(xs, ys)

        # GPS 점
        if self.sim_state.gps_positions:
            gx, gy = zip(*self.sim_state.gps_positions)
            self.gps_scatter.set_data(gx, gy)

        # 차량
        self._draw_vehicle(ax_map)

        # 정보 텍스트
        level = "L2" if self.enable_physics else "L1"
        n = len(self.sim_state.speeds)
        cte = self.sim_state.cross_track_errors[-1] if self.sim_state.cross_track_errors else 0
        slip = self.sim_state.slips[-1] if self.sim_state.slips else 0
        sink = self.sim_state.sinkages[-1] if self.sim_state.sinkages else 0
        self.info_text.set_text(
            f"[{level}] {self.terrain.value}\n"
            f"Speed: {self.speed:.2f} m/s\n"
            f"CTE:   {cte:.3f} m\n"
            f"Slip:  {slip:.3f}\n"
            f"Sink:  {sink:.4f} m\n"
            f"Step:  {n}"
        )

        # 속도/슬립 그래프
        ax_speed = self.axes[0, 1]
        self.speed_line.set_data(range(len(self.sim_state.speeds)), self.sim_state.speeds)
        self.slip_line.set_data(range(len(self.sim_state.slips)), self.sim_state.slips)
        ax_speed.relim()
        ax_speed.autoscale_view()

        # 하중 그래프
        ax_load = self.axes[1, 0]
        self.front_load_line.set_data(range(len(self.sim_state.front_loads)), self.sim_state.front_loads)
        self.rear_load_line.set_data(range(len(self.sim_state.rear_loads)), self.sim_state.rear_loads)
        ax_load.relim()
        ax_load.autoscale_view()

        # IMU 그래프
        ax_imu = self.axes[1, 1]
        self.yaw_true_line.set_data(range(len(self.sim_state.yaw_rates_true)), self.sim_state.yaw_rates_true)
        self.yaw_imu_line.set_data(range(len(self.sim_state.yaw_rates_imu)), self.sim_state.yaw_rates_imu)
        ax_imu.relim()
        ax_imu.autoscale_view()

    def _draw_vehicle(self, ax) -> None:
        if self.vehicle_patch is not None:
            self.vehicle_patch.remove()

        L, W = VEHICLE_LENGTH, VEHICLE_WIDTH
        local_pts = np.array([
            [L / 2, 0], [-L / 2, W / 2], [-L / 2, -W / 2],
        ])
        cos_y = math.cos(self.pose.yaw)
        sin_y = math.sin(self.pose.yaw)
        rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        world_pts = (rot @ local_pts.T).T + np.array([self.pose.x, self.pose.y])

        triangle = patches.Polygon(
            world_pts, closed=True,
            facecolor='lime', edgecolor='darkgreen', linewidth=2, zorder=5,
        )
        ax.add_patch(triangle)
        self.vehicle_patch = triangle

    def _show_result(self) -> None:
        errors = np.array(self.sim_state.cross_track_errors) if self.sim_state.cross_track_errors else np.array([0])
        rmse = float(np.sqrt(np.mean(errors ** 2)))
        max_err = float(np.max(errors))
        mean_err = float(np.mean(errors))

        level = "Level 2 Physics" if self.enable_physics else "Level 1 Kinematic"
        self.status_text.set_text(
            f"FINISHED [{level}]\n"
            f"Terrain: {self.terrain.value}\n"
            f"CTE RMSE: {rmse:.3f} m | Max: {max_err:.3f} m\n"
            f"Steps: {len(self.sim_state.poses)}\n"
            f"Press 'r' to reset"
        )
        self.status_text.set_fontsize(11)
        self.status_text.set_color('darkgreen')
        self.status_text.set_alpha(1.0)
        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------ #
    #  유틸리티
    # ------------------------------------------------------------------ #

    @staticmethod
    def _interpolate_path(waypoints, spacing=0.2):
        if len(waypoints) < 2:
            return list(waypoints)
        dense = [waypoints[0]]
        for i in range(len(waypoints) - 1):
            ax, ay = waypoints[i]
            bx, by = waypoints[i + 1]
            seg_len = math.hypot(bx - ax, by - ay)
            if seg_len < 1e-6:
                continue
            n_pts = max(1, int(seg_len / spacing))
            for j in range(1, n_pts + 1):
                t = j / n_pts
                dense.append((ax + t * (bx - ax), ay + t * (by - ay)))
        return dense

    def _compute_cte(self) -> float:
        if len(self.waypoints) < 2:
            return 0.0
        min_dist = float('inf')
        px, py = self.pose.x, self.pose.y
        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            abx, aby = bx - ax, by - ay
            apx, apy = px - ax, py - ay
            ab_sq = abx * abx + aby * aby
            if ab_sq < 1e-12:
                dist = math.hypot(apx, apy)
            else:
                t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
                dist = math.hypot(px - (ax + t * abx), py - (ay + t * aby))
            min_dist = min(min_dist, dist)
        return min_dist

    def _reset(self) -> None:
        if self.anim is not None:
            self.anim.event_source.stop()
            self.anim = None

        self.state_str = 'drawing'
        self.waypoints.clear()
        self.sim_state = SimulationState()
        self.pose = Pose2D()
        self.speed = 0.0

        self.wp_scatter.set_data([], [])
        self.traj_line.set_data([], [])
        self.gps_scatter.set_data([], [])
        if self.vehicle_patch is not None:
            self.vehicle_patch.remove()
            self.vehicle_patch = None

        self.info_text.set_text('')
        self.status_text.set_text(self._get_status_message())
        self.status_text.set_fontsize(11)
        self.status_text.set_color('gray')
        self.status_text.set_alpha(0.7)

        # 차트 초기화
        for ax_row in self.axes:
            for ax in ax_row:
                if ax != self.axes[0, 0]:
                    ax.clear()
        self._setup_plots()
        self.fig.canvas.draw_idle()

    def run(self) -> None:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Level 2 Physics Simulator')
    parser.add_argument(
        '--terrain', type=str, default='field_hard',
        choices=[t.value for t in TerrainType],
        help='초기 지형 유형',
    )
    parser.add_argument(
        '--no-physics', action='store_true',
        help='Level 1 (운동학 전용) 모드로 시작',
    )
    parser.add_argument(
        '--no-noise', action='store_true',
        help='센서 노이즈 비활성화',
    )
    args = parser.parse_args()

    terrain = TerrainType(args.terrain)
    sim = PhysicsSimulator(
        terrain=terrain,
        enable_physics=not args.no_physics,
        enable_sensor_noise=not args.no_noise,
    )
    sim.run()


if __name__ == '__main__':
    main()
