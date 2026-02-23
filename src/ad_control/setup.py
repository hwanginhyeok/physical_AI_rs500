from setuptools import setup
import os
from glob import glob

package_name = 'ad_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@todo.todo',
    description='자율주행 제어 ROS2 노드 패키지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_node = ad_control.control_node:main',
        ],
    },
)
