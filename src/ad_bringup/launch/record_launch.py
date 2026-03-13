"""ROS2 Bag 녹화 런치 파일.

시뮬레이션 또는 실물 로봇 주행 데이터를 rosbag으로 녹화한다.
녹화된 bag을 Foxglove에서 두 파일 동시에 열어 시뮬/실물 궤적을 비교한다.

사용법:
    ros2 launch ad_bringup record_launch.py robot:=sim
    ros2 launch ad_bringup record_launch.py robot:=real
    ros2 launch ad_bringup record_launch.py robot:=real output_dir:=/data/bags

비교 워크플로우:
    1. 시뮬 실행 + 이 런치 → ~/rosbags/sim_20260301_143000/ 녹화
    2. 실물 실행 + 이 런치 → ~/rosbags/real_20260301_150000/ 녹화
    3. Foxglove → Data Sources → Add → 두 bag 동시 오픈
    4. config/foxglove_comparison_layout.json 레이아웃 불러오기
    5. 각 패널 우상단 소스 아이콘으로 sim/real 전환 후 비교
"""

import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


# ── 녹화 토픽 목록 ──────────────────────────────────────────────────────────
_TF_TOPICS = [
    '/tf',
    '/tf_static',
    '/clock',
    '/robot_description',
]

_SENSOR_TOPICS = [
    '/odom',                    # 휠 오도메트리 (raw)
    '/sensor/imu',
    '/sensor/gps',
    '/sensor/lidar',
    '/sensor/camera/front',
    '/joint_states',
]

_LOCALIZATION_TOPICS = [
    '/odometry/local',          # EKF 로컬 (odom 프레임)
    '/odometry/global',         # EKF 글로벌 (map 프레임)
    '/odometry/gps',            # navsat_transform 출력
    '/gps/filtered',
    '/accel/local',
]

_NAV_TOPICS = [
    '/cmd_vel',
    '/cmd_vel_smoothed',
    '/plan',
    '/local_plan',
    '/goal_pose',
    '/waypoint_manager/status',
    '/waypoint_manager/markers',
]

_COSTMAP_TOPICS = [
    '/global_costmap/costmap',
    '/local_costmap/costmap',
    '/map',
]

RECORD_TOPICS = (
    _TF_TOPICS + _SENSOR_TOPICS + _LOCALIZATION_TOPICS
    + _NAV_TOPICS + _COSTMAP_TOPICS
)


def _launch_record(context, *args, **kwargs):
    """OpaqueFunction: launch 인자 값을 평가한 뒤 ExecuteProcess를 생성."""
    robot = LaunchConfiguration('robot').perform(context)      # 'sim' 또는 'real'
    output_dir = LaunchConfiguration('output_dir').perform(context)

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(output_dir, f'{robot}_{timestamp}')

    return [
        LogInfo(msg=f'[Record] robot={robot}  →  {bag_path}'),
        LogInfo(msg='[Record] 녹화 종료: Ctrl+C  (bag 디렉토리가 자동 저장됨)'),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--output', bag_path,
                '--compression-mode', 'file',
                '--compression-format', 'zstd',
            ] + RECORD_TOPICS,
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='sim',
            description='"sim" 또는 "real"',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser('~/rosbags'),
            description='bag 저장 루트 디렉토리',
        ),
        OpaqueFunction(function=_launch_record),
    ])
