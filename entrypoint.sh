#!/bin/bash

cd $PWD
export HOME="$HOME"
export USER="$USER"

. /opt/ros/$ROS_DISTRO/setup.bash
. $HOME/catkin_ws/devel/setup.bash

# Run the web app in the background
echo "Starting web app..."
cd $HOME/web_app && python3 app.py &

# Wait for the web app to start
sleep 2

# sudo pigpiod
# sudo dbus-daemon --system --fork

# Start Avahi Daemon
# sudo avahi-daemon --daemonize

# sudo rm /run/dbus/pid

exec "$@"
