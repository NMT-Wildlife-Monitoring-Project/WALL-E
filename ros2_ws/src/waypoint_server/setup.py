from setuptools import setup

package_name = 'waypoint_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_waypoint_system.launch.py']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Simple ROS2 Python example',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'waypoint_follower_node = waypoint_server.waypoint_follower_node:main'
            'gps_waypoint_handler_node = waypoint_server.gps_waypoint_handler_node:main',
        ],
    },
)

