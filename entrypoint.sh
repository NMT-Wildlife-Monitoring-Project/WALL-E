#!/bin/bash

cd $PWD
export HOME="$HOME"
export USER="$USER"

. /opt/ros/noetic/setup.bash
[ -e $HOME/$USER/catkin_ws/devel/setup.bash ] && . $HOME/$USER/catkin_ws/devel/setup.bash

bash
