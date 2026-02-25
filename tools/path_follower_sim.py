#!/usr/bin/env python3
"""Pure Pursuit 경로 추종 시뮬레이터.

matplotlib GUI에서 마우스 클릭으로 경로를 그리고,
스키드 스티어 차량이 Pure Pursuit으로 해당 경로를 추종하는 것을
실시간으로 확인할 수 있다.

사용법:
    python tools/path_follower_sim.py

조작:
    좌클릭  - 웨이포인트 추가
    우클릭  - 마지막 웨이포인트 취소 (undo)
    Enter   - 시뮬레이션 시작
    r       - 리셋 (경로 그리기 모드로 복귀)
"""

import math
import sys
import os

# ad_core 패키지를 import할 수 있도록 경로 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'ad_core'))

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np

from ad_core.datatypes import Pose2D
from ad_core.pure_pursuit import PurePursuitConfig, PurePursuitTracker
from ad_core.skid_steer_model import SkidSteerModel

# ====================================================================== #
#  시뮬레이션 파라미터 (상단 상수로 조정)
# ====================================================================== #

DT = 0.05                    # 시뮬레이션 스텝 (s)
MAX_LINEAR_SPEED = 1.0       # 최대 속도 (m/s)
TRACK_WIDTH = 1.4            # 궤도 폭 (m)
LOOKAHEAD_GAIN = 0.8         # Pure Pursuit 게인
MIN_LOOKAHEAD = 1.0          # 최소 전방주시 거리 (m)
MAX_LOOKAHEAD = 5.0          # 최대 전방주시 거리 (m)
STEERING_EFFICIENCY = 0.8    # 조향 효율 (1.0=이상적)
GOAL_TOLERANCE = 0.5         # 목표 도달 판정 거리 (m)
MAP_SIZE = 20.0              # 맵 크기 (m)

# 시각화 설정
VEHICLE_LENGTH = 0.8         # 차량 삼각형 길이 (m) - 표시용
VEHICLE_WIDTH = 0.5          # 차량 삼각형 폭 (m) - 표시용
ANIMATION_INTERVAL = 50      # 애니메이션 간격 (ms)


class PathFollowerSim:
    """Pure Pursuit 경로 추종 시뮬레이터."""

    def __init__(self):
        self.waypoints = []       # 클릭한 (x, y) 리스트
        self.trajectory = []      # 실제 궤적 (x, y) 리스트
        self.cross_track_errors = []  # cross-track error 기록
        self.pose = Pose2D()
        self.speed = 0.0
        self.state = 'drawing'    # 'drawing' | 'running' | 'finished'
        self.anim = None

        # Pure Pursuit 설정
        config = PurePursuitConfig(
            lookahead_gain=LOOKAHEAD_GAIN,
            min_lookahead=MIN_LOOKAHEAD,
            max_lookahead=MAX_LOOKAHEAD,
            goal_tolerance=GOAL_TOLERANCE,
            max_linear_speed=MAX_LINEAR_SPEED,
            track_width=TRACK_WIDTH,
        )
        self.tracker = PurePursuitTracker(config)
        # model.max_speed는 tracker.max_linear_speed보다 충분히 높아야 한다.
        # 회전 시 한쪽 트랙이 linear + angular*W/2 까지 올라가므로,
        # 포화로 인한 angular 손실을 방지하기 위해 2배로 설정한다.
        self.model = SkidSteerModel(
            track_width=TRACK_WIDTH,
            max_speed=MAX_LINEAR_SPEED * 2.0,
            steering_efficiency=STEERING_EFFICIENCY,
        )

        # matplotlib 설정
        self.fig, self.ax = plt.subplots(1, 1, figsize=(9, 9))
        self.fig.canvas.manager.set_window_title('Pure Pursuit Path Follower Simulator')
        self._setup_plot()

        # 그래픽 요소
        self.wp_scatter = self.ax.plot([], [], 'bo-', markersize=8, label='Plan')[0]
        self.wp_labels = []
        self.traj_line = self.ax.plot([], [], 'r-', linewidth=2, label='Actual')[0]
        self.vehicle_patch = None
        self.info_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=10,
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
        )
        self.status_text = self.ax.text(
            0.5, 0.5, 'Click to add waypoints\nEnter to start | Right-click to undo',
            transform=self.ax.transAxes, ha='center', va='center',
            fontsize=14, color='gray', alpha=0.7,
        )

        # 이벤트 연결
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.ax.legend(loc='upper right')

    def _setup_plot(self):
        """플롯 초기 설정."""
        self.ax.set_xlim(-1, MAP_SIZE + 1)
        self.ax.set_ylim(-1, MAP_SIZE + 1)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Pure Pursuit Path Follower Simulator')

    # ------------------------------------------------------------------ #
    #  이벤트 핸들러
    # ------------------------------------------------------------------ #

    def on_click(self, event):
        """마우스 클릭 이벤트 처리."""
        if self.state != 'drawing':
            return
        if event.inaxes != self.ax:
            return

        if event.button == 1:  # 좌클릭: 웨이포인트 추가
            self.waypoints.append((event.xdata, event.ydata))
            self._update_waypoint_display()

        elif event.button == 3:  # 우클릭: undo
            if self.waypoints:
                self.waypoints.pop()
                self._update_waypoint_display()

    def on_key(self, event):
        """키보드 이벤트 처리."""
        if event.key == 'enter' and self.state == 'drawing':
            if len(self.waypoints) >= 2:
                self.start_simulation()
        elif event.key == 'r':
            self.reset()

    # ------------------------------------------------------------------ #
    #  웨이포인트 표시
    # ------------------------------------------------------------------ #

    def _update_waypoint_display(self):
        """웨이포인트 점/번호 갱신."""
        if self.waypoints:
            xs, ys = zip(*self.waypoints)
            self.wp_scatter.set_data(xs, ys)
        else:
            self.wp_scatter.set_data([], [])

        # 번호 텍스트 갱신
        for lbl in self.wp_labels:
            lbl.remove()
        self.wp_labels.clear()

        for i, (x, y) in enumerate(self.waypoints):
            lbl = self.ax.text(
                x + 0.3, y + 0.3, str(i),
                fontsize=8, color='blue', fontweight='bold',
            )
            self.wp_labels.append(lbl)

        if self.waypoints:
            self.status_text.set_text(
                f'{len(self.waypoints)} waypoints | Enter to start | Right-click to undo'
            )
        else:
            self.status_text.set_text(
                'Click to add waypoints\nEnter to start | Right-click to undo'
            )

        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------ #
    #  시뮬레이션
    # ------------------------------------------------------------------ #

    def start_simulation(self):
        """시뮬레이션을 시작한다."""
        self.state = 'running'
        self.status_text.set_text('')
        self.trajectory.clear()
        self.cross_track_errors.clear()

        # 차량 초기 위치: 첫 웨이포인트, 방향: 두 번째 웨이포인트를 향함
        p0 = self.waypoints[0]
        p1 = self.waypoints[1]
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        self.pose = Pose2D(x=p0[0], y=p0[1], yaw=yaw)
        self.speed = 0.0

        # 경로 설정: 성긴 웨이포인트를 촘촘하게 보간하여 전달
        self.tracker = PurePursuitTracker(PurePursuitConfig(
            lookahead_gain=LOOKAHEAD_GAIN,
            min_lookahead=MIN_LOOKAHEAD,
            max_lookahead=MAX_LOOKAHEAD,
            goal_tolerance=GOAL_TOLERANCE,
            max_linear_speed=MAX_LINEAR_SPEED,
            track_width=TRACK_WIDTH,
        ))
        dense_path = self._interpolate_path(self.waypoints, spacing=0.2)
        self.tracker.set_path(dense_path)

        self.trajectory.append((self.pose.x, self.pose.y))

        # 애니메이션 시작
        self.anim = FuncAnimation(
            self.fig, self.update, interval=ANIMATION_INTERVAL,
            blit=False, cache_frame_data=False,
        )
        self.fig.canvas.draw_idle()

    def update(self, frame):
        """매 프레임 시뮬레이션 업데이트."""
        if self.state != 'running':
            if self.anim is not None:
                self.anim.event_source.stop()
            return

        # Pure Pursuit 제어
        linear, angular = self.tracker.compute(self.pose, self.speed)

        # 트랙 속도 변환
        v_left, v_right = self.model.twist_to_tracks(linear, angular)

        # 다음 pose 예측
        self.pose = self.model.predict_pose(self.pose, v_left, v_right, DT)

        # 현재 속도 갱신 (트랙 속도로부터)
        self.speed = (abs(v_left) + abs(v_right)) / 2.0

        # 궤적 기록
        self.trajectory.append((self.pose.x, self.pose.y))

        # Cross-track error 계산 및 기록
        cte = self._compute_cross_track_error()
        self.cross_track_errors.append(cte)

        # 시각화 갱신
        self._draw_trajectory()
        self._draw_vehicle()
        self._draw_info(linear, angular, v_left, v_right, cte)

        # 목표 도달 확인
        if self.tracker.is_goal_reached:
            self.state = 'finished'
            if self.anim is not None:
                self.anim.event_source.stop()
            self._show_result()

    @staticmethod
    def _interpolate_path(waypoints, spacing=0.2):
        """성긴 웨이포인트를 일정 간격으로 보간하여 촘촘한 경로를 생성한다.

        마우스 클릭 웨이포인트는 2~5m 간격으로 성기기 때문에,
        Pure Pursuit의 nearest_idx 탐색과 lookahead 보간이
        안정적으로 동작하려면 0.1~0.3m 간격의 촘촘한 경로가 필요하다.
        """
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

    def _compute_cross_track_error(self):
        """현재 위치에서 경로까지의 최소 수직 거리를 계산한다."""
        if len(self.waypoints) < 2:
            return 0.0

        min_dist = float('inf')
        px, py = self.pose.x, self.pose.y

        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]

            # 선분 AB 위의 최근접점까지의 거리
            abx, aby = bx - ax, by - ay
            apx, apy = px - ax, py - ay
            ab_sq = abx * abx + aby * aby

            if ab_sq < 1e-12:
                dist = math.hypot(apx, apy)
            else:
                t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
                proj_x = ax + t * abx
                proj_y = ay + t * aby
                dist = math.hypot(px - proj_x, py - proj_y)

            min_dist = min(min_dist, dist)

        return min_dist

    # ------------------------------------------------------------------ #
    #  시각화
    # ------------------------------------------------------------------ #

    def _draw_trajectory(self):
        """실제 궤적을 그린다."""
        if self.trajectory:
            xs, ys = zip(*self.trajectory)
            self.traj_line.set_data(xs, ys)

    def _draw_vehicle(self):
        """차량을 삼각형으로 표시한다."""
        if self.vehicle_patch is not None:
            self.vehicle_patch.remove()

        # 삼각형 꼭짓점 (차량 좌표계: 전방이 +x)
        L = VEHICLE_LENGTH
        W = VEHICLE_WIDTH
        local_pts = np.array([
            [L / 2, 0],       # 앞쪽 (뾰족한 끝)
            [-L / 2, W / 2],  # 좌후방
            [-L / 2, -W / 2], # 우후방
        ])

        # 회전 변환
        cos_y = math.cos(self.pose.yaw)
        sin_y = math.sin(self.pose.yaw)
        rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        world_pts = (rot @ local_pts.T).T + np.array([self.pose.x, self.pose.y])

        triangle = patches.Polygon(
            world_pts, closed=True,
            facecolor='lime', edgecolor='darkgreen', linewidth=2, zorder=5,
        )
        self.ax.add_patch(triangle)
        self.vehicle_patch = triangle

    def _draw_info(self, linear, angular, v_left, v_right, cte):
        """상태 정보 텍스트를 갱신한다."""
        info = (
            f"Speed:  {self.speed:.2f} m/s\n"
            f"Linear: {linear:.2f} m/s\n"
            f"Angular:{angular:.3f} rad/s\n"
            f"V_L:    {v_left:.2f}  V_R: {v_right:.2f}\n"
            f"CTE:    {cte:.3f} m\n"
            f"WP idx: {self.tracker.nearest_idx}/{len(self.waypoints) - 1}"
        )
        self.info_text.set_text(info)

    def _show_result(self):
        """시뮬레이션 결과를 표시한다."""
        if self.cross_track_errors:
            errors = np.array(self.cross_track_errors)
            rmse = float(np.sqrt(np.mean(errors ** 2)))
            max_err = float(np.max(errors))
            mean_err = float(np.mean(errors))
        else:
            rmse = max_err = mean_err = 0.0

        result = (
            f"FINISHED\n"
            f"CTE RMSE: {rmse:.3f} m\n"
            f"CTE Max:  {max_err:.3f} m\n"
            f"CTE Mean: {mean_err:.3f} m\n"
            f"Steps:    {len(self.trajectory)}\n\n"
            f"Press 'r' to reset"
        )
        self.status_text.set_text(result)
        self.status_text.set_fontsize(13)
        self.status_text.set_color('darkgreen')
        self.status_text.set_alpha(1.0)

        self.info_text.set_text(
            f"RMSE: {rmse:.3f} m\n"
            f"Max:  {max_err:.3f} m\n"
            f"Mean: {mean_err:.3f} m"
        )

        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------ #
    #  리셋
    # ------------------------------------------------------------------ #

    def reset(self):
        """시뮬레이터를 초기 상태로 리셋한다."""
        if self.anim is not None:
            self.anim.event_source.stop()
            self.anim = None

        self.state = 'drawing'
        self.waypoints.clear()
        self.trajectory.clear()
        self.cross_track_errors.clear()
        self.pose = Pose2D()
        self.speed = 0.0

        # 그래픽 요소 초기화
        self.wp_scatter.set_data([], [])
        self.traj_line.set_data([], [])
        for lbl in self.wp_labels:
            lbl.remove()
        self.wp_labels.clear()
        if self.vehicle_patch is not None:
            self.vehicle_patch.remove()
            self.vehicle_patch = None

        self.info_text.set_text('')
        self.status_text.set_text(
            'Click to add waypoints\nEnter to start | Right-click to undo'
        )
        self.status_text.set_fontsize(14)
        self.status_text.set_color('gray')
        self.status_text.set_alpha(0.7)

        self.fig.canvas.draw_idle()

    def run(self):
        """시뮬레이터를 실행한다."""
        plt.show()


if __name__ == '__main__':
    sim = PathFollowerSim()
    sim.run()
