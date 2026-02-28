"""웨이포인트 네비게이션 데모 런치 파일.

simulation_launch.py의 간소화 래퍼.
기본 인자로 빠르게 데모를 시작할 수 있다.

사용법:
    ros2 launch ad_bringup waypoint_nav_demo_launch.py
    # → Foxglove Studio에서 ws://localhost:8765 접속
    # → 3D 패널에서 맵 클릭하여 웨이포인트 추가
    # → Call Service 패널에서 /waypoint_manager/start 호출
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('ad_bringup')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'simulation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_foxglove': 'true',
        }.items(),
    )

    return LaunchDescription([
        LogInfo(msg='=== SS500 Waypoint Navigation Demo ==='),
        LogInfo(msg='Foxglove Studio: ws://localhost:8765'),
        LogInfo(msg='웨이포인트 추가: 3D 패널에서 맵 클릭'),
        LogInfo(msg='네비게이션 시작: /waypoint_manager/start 서비스 호출'),
        LogInfo(msg='========================================'),
        simulation_launch,
    ])
