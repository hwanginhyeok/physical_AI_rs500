"""ad_planning 패키지 launch 파일."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ad_planning')
    params_file = os.path.join(pkg_dir, 'config', 'planning_params.yaml')

    planning_node = Node(
        package='ad_planning',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        planning_node,
    ])
