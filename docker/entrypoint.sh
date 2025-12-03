#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/ros2_ws/install/setup.bash

# Execute the passed command
# If single argument, treat as shell command string (for proper argument parsing)
# If multiple arguments, pass through directly
if [ $# -eq 1 ]; then
    exec bash -c "$1"
else
    exec "$@"
fi