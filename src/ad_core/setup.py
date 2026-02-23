from setuptools import find_packages, setup

package_name = 'ad_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='순수 알고리즘 코어 (ROS2 독립, adtpc 이식 대상)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
