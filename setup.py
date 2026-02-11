from setuptools import find_packages, setup
from glob import glob

package_name = 'hand_tracking_sdk_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        # Core upstream SDK dependency for parsing/transport/frame assembly.
        'hand-tracking-sdk>=1.0.0,<2.0.0',
    ],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='ROS 2 bridge for Hand Tracking SDK streams',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_tracking_bridge = hand_tracking_sdk_ros2.bridge_node:main',
        ],
    },
)
