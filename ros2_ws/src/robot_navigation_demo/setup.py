from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walle',
    maintainer_email='walle@example.com',
    description='Robot navigation package for WALL-E with SLAM mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_odom_publisher = robot_navigation.fake_odom_publisher:main',
            'logged_waypoint_follower = nav2_gps_waypoint_follower_demo.logged_waypoint_follower:main',
            'interactive_waypoint_follower = nav2_gps_waypoint_follower_demo.interactive_waypoint_follower:main',
            'gps_waypoint_logger = nav2_gps_waypoint_follower_demo.gps_waypoint_logger:main'
        ],
    },
)
