"""SS500 Simulation Launch File for Gazebo Harmonic.

Launches Gazebo with crop field world and SS500 robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    """Generate launch description."""
    
    pkg_ad_simulation = FindPackageShare('ad_simulation')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='crop_field.world')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-1.5')
    z_pose = LaunchConfiguration('z_pose', default='0.5')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # World file path
    world_path = PathJoinSubstitution([
        pkg_ad_simulation,
        'worlds',
        world_file
    ])
    
    # Robot description (URDF)
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                pkg_ad_simulation,
                'urdf',
                'ss500.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    
    # Gazebo Harmonic (gz sim)
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', world_path
        ],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Spawn Robot (using ros_gz spawn)
    # Note: In Gazebo Harmonic, we use gz service calls or ros_gz
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ss500',
        arguments=[
            '-name', 'ss500',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Static Transform Publishers
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Bridge for Gazebo topics (if needed)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSatFix',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz2
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            pkg_ad_simulation,
            'config',
            'simulation.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='crop_field.world',
            description='World file to load'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial X position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-1.5',
            description='Initial Y position (between crop rows)'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.5',
            description='Initial Z position'
        ),
        DeclareLaunchArgument(
            'yaw_pose',
            default_value='0.0',
            description='Initial yaw angle (facing forward)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'
        ),
        
        # Nodes
        gazebo,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        static_tf_map_odom,
        rviz_node,
    ])


# Helper for conditional launch
from launch.conditions import IfCondition
