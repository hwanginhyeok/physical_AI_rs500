"""rosbag2 MCAP 녹화 launch 파일.

주요 센서 토픽과 네비게이션 토픽을 MCAP 포맷으로 자동 녹화한다.
Foxglove Studio에서 직접 재생할 수 있는 포맷.

사용법:
    ros2 launch ad_bringup recording_launch.py
    ros2 launch ad_bringup recording_launch.py output_dir:=/path/to/bags
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


# 녹화 대상 토픽 목록
RECORDING_TOPICS = [
    # 센서 데이터
    '/sensor/camera/front',
    '/sensor/lidar',
    '/sensor/gps',
    '/sensor/imu',

    # 위치 추정
    '/odometry/local',
    '/odometry/global',
    '/odometry/gps',
    '/tf',
    '/tf_static',

    # 네비게이션
    '/cmd_vel',
    '/plan',
    '/goal_pose',
    '/local_costmap/costmap',
    '/global_costmap/costmap',

    # 차량 상태
    '/vehicle/status',
    '/waypoint_manager/status',
    '/waypoint_manager/markers',

    # 인지 결과 (퍼블리시 토픽이 추가되면 여기에 포함)
    '/perception/detections',
    '/perception/segmentation',

    # 시스템 로그
    '/rosout',
]


def generate_launch_description():
    # 기본 저장 경로
    default_output_dir = os.path.join(
        os.path.expanduser('~'),
        'rosbag2_data',
    )

    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='rosbag2 녹화 파일 저장 디렉토리',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='시뮬레이션 시간 사용',
    )

    declare_max_bag_duration = DeclareLaunchArgument(
        'max_bag_duration',
        default_value='300',
        description='단일 bag 파일 최대 길이 (초)',
    )

    output_dir = LaunchConfiguration('output_dir')
    max_bag_duration = LaunchConfiguration('max_bag_duration')

    # 토픽 인자 구성
    topic_args = []
    for topic in RECORDING_TOPICS:
        topic_args.extend(['--topics', topic])

    # rosbag2 record 프로세스
    rosbag2_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',              # MCAP 포맷 (Foxglove 호환)
            '--max-bag-duration', max_bag_duration,
            '--output', output_dir,
            '--compression-mode', 'message',   # 메시지 단위 압축
            '--compression-format', 'zstd',    # Zstandard 압축
            '--use-sim-time',                  # 시뮬레이션 시간 사용
        ] + topic_args,
        output='screen',
    )

    return LaunchDescription([
        declare_output_dir,
        declare_use_sim_time,
        declare_max_bag_duration,

        LogInfo(msg='[Recording] rosbag2 MCAP 녹화 시작'),
        rosbag2_record,
    ])
