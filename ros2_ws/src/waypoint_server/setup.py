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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walle',
    maintainer_email='you@example.com',
    description='Waypoint management and following nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_server_node = waypoint_server.waypoint_server_node:main',
            'waypoint_follower_node = waypoint_server.waypoint_follower_node:main',
        ],
    },
)

