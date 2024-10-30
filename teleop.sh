#!/bin/bash

docker build -t ros2_luna .
docker run -it --rm \
  -v /dev:/dev \
  --network=host \
  --privileged \
  --name \
  luna \
  ros2_luna \
  ros2 launch teleop teleop_launch.py

