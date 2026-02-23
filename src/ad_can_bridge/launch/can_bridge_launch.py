"""ad_can_bridge 패키지 launch 파일."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ad_can_bridge')
    params_file = os.path.join(pkg_dir, 'config', 'can_params.yaml')

    can_bridge_node = Node(
        package='ad_can_bridge',
        executable='can_bridge_node',
        name='can_bridge_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        can_bridge_node,
    ])
