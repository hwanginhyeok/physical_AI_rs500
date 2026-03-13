"""Gazebo Harmonic 시뮬레이션 통합 런치 파일.

실행 순서:
    1. robot_state_publisher (URDF → TF)
    2. Gazebo Harmonic (월드 + 로봇 스폰)
    3. ros_gz_bridge (Gazebo ↔ ROS2 토픽 브릿지)
    4. ekf_local + ekf_global + navsat_transform (위치 추정)
    5. Nav2 bringup (nav2_params.yaml + empty_map)
    6. waypoint_manager_node
    7. foxglove_bridge

TF 트리:
    map → odom → base_link → sensor frames
     ^      ^
     |      └── ekf_local (odom→base_link, IMU+Odom 융합)
     └── ekf_global (map→odom, GPS 융합)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── 패키지 경로 ──
    bringup_dir = get_package_share_directory('ad_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── 기본 설정 파일 경로 ──
    default_urdf_xacro = os.path.join(bringup_dir, 'urdf', 'ss500.urdf.xacro')
    default_world = os.path.join(bringup_dir, 'worlds', 'flat_field.sdf')
    default_bridge_config = os.path.join(
        bringup_dir, 'config', 'bridge_config.yaml'
    )
    default_ekf_params = os.path.join(bringup_dir, 'config', 'dual_ekf.yaml')
    default_nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_map_yaml = os.path.join(bringup_dir, 'maps', 'empty_map.yaml')
    default_sdf_model = os.path.join(
        bringup_dir, 'models', 'ss500', 'model.sdf'
    )

    # ── Launch 인자 선언 ──
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='시뮬레이션 시간 사용',
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo 월드 SDF 파일 경로',
    )
    declare_urdf_xacro = DeclareLaunchArgument(
        'urdf_xacro', default_value=default_urdf_xacro,
        description='로봇 URDF xacro 파일 경로',
    )
    declare_nav2_params = DeclareLaunchArgument(
        'params_file', default_value=default_nav2_params,
        description='Nav2 파라미터 YAML 파일',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map_yaml,
        description='Nav2 맵 YAML 파일',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Nav2 lifecycle 노드 자동 시작',
    )
    declare_foxglove = DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Foxglove Bridge 실행 여부',
    )

    # ── Launch 설정값 참조 ──
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    urdf_xacro = LaunchConfiguration('urdf_xacro')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_foxglove = LaunchConfiguration('use_foxglove')

    # ================================================================
    # 1. robot_state_publisher: URDF → TF 정적 변환 발행
    # ================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_xacro]), value_type=str
            ),
        }],
    )

    # ================================================================
    # 2. Gazebo Harmonic: 시뮬레이터 실행 + 로봇 스폰
    # ================================================================
    gazebo_server = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s',  # 서버 모드 (-s = server only)
            '-r',               # 자동 시작 (-r = run)
            world_file,
        ],
        output='screen',
    )

    # gazebo_gui: WSL2에서 Mesa 소프트웨어 렌더러 사용으로 CPU 400% 점유 → RTF 0.006x
    # Foxglove Studio(ws://localhost:8765)로 시각화하므로 GUI 비활성화
    # 필요 시 별도 터미널에서: gz sim -g
    # gazebo_gui = ExecuteProcess(
    #     cmd=['gz', 'sim', '-g'],  # GUI 모드 (-g = GUI only)
    #     output='screen',
    # )

    # 로봇 스폰
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ss500',
        output='screen',
        arguments=[
            '-name', 'ss500',
            '-file', default_sdf_model,
            '-x', '0', '-y', '0', '-z', '0.5',
        ],
    )

    # ================================================================
    # 2b. map → odom Static TF (C50: AMCL 대체)
    # AMCL은 빈 평지에서 LiDAR 특징점 부족으로 초기화 실패 → map 프레임 미발행.
    # static_transform_publisher로 map→odom identity 변환을 고정 발행한다.
    # (C51 GPS datum 고정 + ekf_global.publish_tf: true 완료 시 이 노드를 제거)
    # ================================================================
    map_to_odom_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ================================================================
    # 2c. Gazebo Scoped 센서 Frame ID 브릿지 (Static TF)
    # Gazebo Harmonic은 센서 메시지 frame_id를 "ss500/link/sensor" 형태로 발행.
    # URDF TF 트리에는 단순 이름(lidar_link, imu_link)만 있으므로,
    # 두 네임스페이스 사이에 identity transform을 추가한다.
    # ================================================================
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'ss500/lidar_link/gpu_lidar'],
    )

    imu_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_frame_bridge',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'ss500/imu_link/imu_sensor'],
    )

    # GPS 센서도 Gazebo scoped frame_id "ss500/gps_link/gps_sensor" 발행
    # navsat_transform_node가 base_footprint→gps_sensor TF를 찾으므로 브릿지 필요
    gps_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_frame_bridge',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'gps_link', 'ss500/gps_link/gps_sensor'],
    )

    # ================================================================
    # 3. ros_gz_bridge: Gazebo ↔ ROS2 토픽 브릿지
    # ================================================================
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': default_bridge_config,
        }],
    )

    # ================================================================
    # 4. Dual-EKF 위치 추정 (navigation_launch.py와 동일 구성)
    # ================================================================
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[default_ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
            ('accel/filtered', '/accel/local'),
        ],
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[default_ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/global')],
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[default_ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('gps/fix', '/sensor/gps'),
            ('imu', '/sensor/imu'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    # ================================================================
    # 5. Nav2 Bringup (C61-fix: smoother_server 파라미터 수정)
    # ================================================================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'log_level': 'info',  # C61-fix: info 로깅으로 정리
        }.items(),
    )

    # ================================================================
    # 6. Waypoint Manager Node
    # ================================================================
    waypoint_manager = Node(
        package='ad_bringup',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ================================================================
    # 7. Foxglove Bridge (조건부)
    # ================================================================
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 8765,
            'send_buffer_limit': 10000000,  # 10MB
            'num_threads': 2,
        }],
    )

    # ================================================================
    # 7b. cmd_vel Relay (C61-fix: collision_monitor 우회)
    # velocity_smoother의 cmd_vel_smoothed를 cmd_vel로 publish
    # ================================================================
    cmd_vel_relay = Node(
        package='ad_bringup',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ================================================================
    # Launch Description 조합
    # ================================================================
    return LaunchDescription([
        # 인자 선언
        declare_use_sim_time,
        declare_world,
        declare_urdf_xacro,
        declare_nav2_params,
        declare_map,
        declare_autostart,
        declare_foxglove,

        LogInfo(msg='[Simulation] SS500 Gazebo 시뮬레이션 시작'),

        # 1. TF 발행
        robot_state_publisher,

        # 2. Gazebo 시뮬레이터 (GUI 제거 — RTF 개선)
        gazebo_server,
        # gazebo_gui,  # WSL2 CPU 병목 — Foxglove로 대체
        spawn_robot,

        # 2b. map→odom 정적 변환 (AMCL 대체 — C50)
        map_to_odom_static,

        # 2c. 센서 Frame ID 브릿지 (Gazebo scoped name → URDF name)
        lidar_frame_bridge,
        imu_frame_bridge,
        gps_frame_bridge,

        # 3. 토픽 브릿지
        ros_gz_bridge,

        # 4. 위치 추정
        GroupAction(actions=[
            SetParameter('use_sim_time', use_sim_time),
            ekf_local_node,
            ekf_global_node,
            navsat_transform_node,
        ]),

        # 5~7: Gazebo 기동 대기 후 실행 (TF base_link→odom 준비 시간 확보)
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg='[Simulation] 15s 경과 — Nav2 + WaypointManager + Foxglove 시작'),

                # 5. 네비게이션 스택 (C61-fix: debug 로깅으로 smoother_server 문제 확인)
                nav2_bringup_launch,

                # 6. 웨이포인트 매니저
                waypoint_manager,

                # 7. Foxglove Bridge
                foxglove_bridge,

                # 7b. cmd_vel Relay (collision_monitor 우회)
                cmd_vel_relay,
            ],
        ),
    ])
