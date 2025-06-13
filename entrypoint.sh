#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

source /home/$USER/ros2_roboclaw_driver/install/setup.bash

# Source workspace if it exists
if [ -f "/home/$USER/ros2_ws/install/local_setup.bash" ]; then
    source /home/$USER/ros2_ws/install/local_setup.bash
fi

# Execute the passed command
exec "$@"