#!/bin/bash
set -e

# Source the ROS environment
source /opt/ros/humble/setup.bash
source /home/luna/ros2_ws/install/setup.bash

# Set up the RealSense device permissions
# sudo udevadm control --reload-rules && sudo udevadm trigger

# Launch the ROS nodes, including the RealSense D455 camera
exec "$@"

