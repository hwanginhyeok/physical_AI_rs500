from setuptools import setup
import os
from glob import glob

package_name = 'ad_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config (YAML + JSON)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
        # launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # URDF/xacro
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        # Gazebo 모델
        (os.path.join('share', package_name, 'models', 'ss500'),
            glob('models/ss500/*')),
        # Gazebo 월드
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        # Nav2 맵
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@todo.todo',
    description='자율주행 시스템 통합 런처',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_manager = ad_bringup.waypoint_manager_node:main',
            'cmd_vel_relay = ad_bringup.cmd_vel_relay:main',
        ],
    },
)
