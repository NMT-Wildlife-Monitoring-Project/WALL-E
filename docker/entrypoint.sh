#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/ros2_roboclaw_driver/install/setup.bash
source $HOME/ros2_ws/install/setup.bash

# Set group and permissions for GPIO devices
chgrp gpio /dev/gpiochip* /dev/gpiomem* || true
chmod g+rw /dev/gpiochip* /dev/gpiomem* || true

# Execute the passed commandex
exec "$@"