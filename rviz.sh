#!/bin/bash
docker exec -it --privileged luna \
  bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && exec rviz2"

