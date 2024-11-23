#!/bin/bash

cd $PWD
export HOME="$HOME"
export USER="$USER"

. /opt/ros/noetic/setup.bash
. $HOME/catkin_ws/devel/setup.bash

exec "$@"
