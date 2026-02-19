"""ad_simulation 통합 launch 파일.

Gazebo 서버/GUI + 궤도차량 스폰 + ros_gz_bridge + team_leader 노드를 실행한다.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('ad_simulation')
    pkg_team = get_package_share_directory('team_leader')

    # 경로 설정
    world_file = os.path.join(pkg_sim, 'worlds', 'agricultural_field.sdf')
    model_path = os.path.join(pkg_sim, 'models')
    bridge_config = os.path.join(pkg_sim, 'config', 'bridge_config.yaml')
    params_file = os.path.join(pkg_team, 'config', 'params.yaml')

    # Launch arguments
    use_gui = LaunchConfiguration('gui')
    run_team_leader = LaunchConfiguration('run_team_leader')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Gazebo GUI 실행 여부',
    )
    declare_team_leader = DeclareLaunchArgument(
        'run_team_leader', default_value='true',
        description='team_leader 노드 실행 여부',
    )

    # GZ_SIM_RESOURCE_PATH에 모델 경로 추가
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path,
    )

    # Gazebo 서버 + GUI
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 궤도차량 스폰
    spawn_vehicle = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tracked_vehicle',
            '-file', os.path.join(model_path, 'tracked_vehicle', 'model.sdf'),
            '-x', '0', '-y', '0', '-z', '0.5',
        ],
        output='screen',
    )

    # ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    # team_leader 노드 (선택적)
    team_leader_node = Node(
        package='team_leader',
        executable='leader_node',
        name='team_leader',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(run_team_leader),
    )

    return LaunchDescription([
        declare_gui,
        declare_team_leader,
        set_model_path,
        gz_sim,
        spawn_vehicle,
        bridge,
        team_leader_node,
    ])
