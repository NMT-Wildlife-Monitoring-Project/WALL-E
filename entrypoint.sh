#!/bin/bash

cd $PWD
export HOME="$HOME"
export USER="$USER"

. /opt/ros/noetic/setup.bash
. $HOME/catkin_ws/devel/setup.bash

sudo dbus-daemon --system --fork

# Start Avahi Daemon
sudo avahi-daemon --daemonize

sudo rm /run/dbus/pid

exec "$@"
