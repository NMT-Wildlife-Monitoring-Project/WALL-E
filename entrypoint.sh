#!/bin/bash
set -e

# Source the ROS environment
source /opt/ros/humble/setup.bash
source /home/walle/ros2_ws/install/setup.bash
sudo chmod 666 /dev/serial0
sudo chmod 666 /dev/ttyUSB0

ros2 launch teleop teleop_launch.py &
NODE1_PID=$!
ros2 launch motor_control motor_launch.py &
NODE2_PID=$!
ros2 launch rplidar_ros view_rplidar_a1_launch.py scan_mode:=Standard &
NODE3_PID=$!

exec "$@"

