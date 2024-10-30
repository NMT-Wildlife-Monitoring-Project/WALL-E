#!/bin/bash

docker build -t ros2_luna .
docker run -it --rm \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --network=host \
  --privileged \
  --device-cgroup-rule='c 13:* rmw' \
  --device-cgroup-rule "c 81:* rmw" \
  --device-cgroup-rule "c 189:* rmw" \
  --name \
  luna \
  ros2_luna \
  bash

