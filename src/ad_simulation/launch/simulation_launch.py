"""ad_simulation 통합 launch 파일.

Gazebo 서버/GUI + 궤도차량 스폰 + ros_gz_bridge + team_leader 노드를 실행한다.

사용 예시:
    # 기본 (농경지) 월드
    ros2 launch ad_simulation simulation_launch.py

    # 영월 신일리 실제 지형 월드
    ros2 launch ad_simulation simulation_launch.py world:=yeongwol_sinil
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


# 월드 이름 → SDF 상대 경로 매핑
_WORLD_MAP = {
    'agricultural_field': os.path.join('worlds', 'agricultural_field.sdf'),
    'yeongwol_sinil': os.path.join('worlds', 'real_terrain', 'yeongwol_sinil.sdf'),
    'cheongsong_orchard': os.path.join('worlds', 'real_terrain', 'cheongsong_orchard.sdf'),
}


def _resolve_world(context, *args, **kwargs):
    """OpaqueFunction: 월드 이름을 SDF 파일 경로로 해석하여 Gazebo 실행."""
    world_name = context.launch_configurations['world']
    pkg_sim = get_package_share_directory('ad_simulation')

    # 이름 매핑 또는 직접 경로
    if world_name in _WORLD_MAP:
        world_file = os.path.join(pkg_sim, _WORLD_MAP[world_name])
    else:
        world_file = world_name  # 전체 경로로 취급

    model_path = os.path.join(pkg_sim, 'models')

    return [
        IncludeLaunchDescription(
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
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'tracked_vehicle',
                '-file', os.path.join(model_path, 'tracked_vehicle', 'model.sdf'),
                '-x', '0', '-y', '0', '-z', '0.5',
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_sim = get_package_share_directory('ad_simulation')

    # 경로 설정
    model_path = os.path.join(pkg_sim, 'models')
    real_terrain_path = os.path.join(pkg_sim, 'worlds', 'real_terrain')
    bridge_config = os.path.join(pkg_sim, 'config', 'bridge_config.yaml')
    pkg_team = get_package_share_directory('team_leader')
    params_file = os.path.join(pkg_team, 'config', 'params.yaml')

    # Launch arguments
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Gazebo GUI 실행 여부',
    )
    declare_team_leader = DeclareLaunchArgument(
        'run_team_leader', default_value='true',
        description='team_leader 노드 실행 여부',
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value='agricultural_field',
        description='월드 선택: agricultural_field | yeongwol_sinil | cheongsong_orchard (또는 SDF 전체 경로)',
    )

    # 전역 설정: 시뮬레이션 시간 사용 (Nav2, robot_localization 등과 동기화 필수)
    use_sim_time = SetParameter(name='use_sim_time', value=True)

    # GZ_SIM_RESOURCE_PATH에 모델 + 지형 데이터 경로 추가
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([model_path, real_terrain_path]),
    )

    # 월드 + 차량 스폰 (OpaqueFunction으로 월드 이름 해석)
    setup_gazebo = OpaqueFunction(function=_resolve_world)

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
        condition=IfCondition(LaunchConfiguration('run_team_leader')),
    )

    return LaunchDescription([
        declare_gui,
        declare_team_leader,
        declare_world,
        use_sim_time,
        set_model_path,
        setup_gazebo,
        bridge,
        team_leader_node,
    ])
