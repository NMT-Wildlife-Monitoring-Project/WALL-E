#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/ros2_ws/install/setup.bash

# Execute the passed commandex
exec "$@"