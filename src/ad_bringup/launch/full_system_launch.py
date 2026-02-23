"""전체 자율주행 시스템 통합 launch 파일.

인지(ad_perception) + 판단(ad_planning) + 제어(ad_control) +
CAN 브릿지(ad_can_bridge) + 위치 추정(localization) 노드를 통합 실행한다.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 패키지 경로
    perception_dir = get_package_share_directory('ad_perception')
    planning_dir = get_package_share_directory('ad_planning')
    control_dir = get_package_share_directory('ad_control')
    can_bridge_dir = get_package_share_directory('ad_can_bridge')

    # Launch 인자
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부',
    )

    # 서브 launch 포함
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_dir, 'launch', 'perception_launch.py')
        ),
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_dir, 'launch', 'planning_launch.py')
        ),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_dir, 'launch', 'control_launch.py')
        ),
    )

    can_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(can_bridge_dir, 'launch', 'can_bridge_launch.py')
        ),
    )

    return LaunchDescription([
        declare_use_sim_time,
        perception_launch,
        planning_launch,
        control_launch,
        can_bridge_launch,
    ])
