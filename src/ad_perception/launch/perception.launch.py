"""Learned Perception Launch File.

Example:
    ros2 launch ad_perception perception.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description."""
    
    # 설정 파일 경로
    pkg_dir = get_package_share_directory('ad_perception')
    config_file = os.path.join(pkg_dir, 'config', 'perception_config.yaml')
    
    return LaunchDescription([
        # 파라미터 오버라이드
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Device: cuda, cpu, or empty for auto'
        ),
        
        DeclareLaunchArgument(
            'inference_rate',
            default_value='10.0',
            description='Maximum inference rate in Hz'
        ),
        
        # Learned Perception Node
        Node(
            package='ad_perception',
            executable='learned_perception_node',
            name='learned_perception',
            output='screen',
            parameters=[
                config_file,
                {
                    'device': LaunchConfiguration('device'),
                    'inference_rate': LaunchConfiguration('inference_rate'),
                }
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/camera/camera_info', '/camera/camera_info'),
            ]
        ),
    ])
