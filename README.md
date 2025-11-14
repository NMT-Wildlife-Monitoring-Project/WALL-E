# WALL-E
Wildlife Activity Life Explorer
The main repository for the New Mexico Tech Wildlife Monitoring Project

# INSTALLATION

This system is designed to run in Docker on linux.  

Install docker  
<https://docs.docker.com/engine/install/ubuntu/>  

To use docker without sudo, run the following commands  
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Ensure you have an SSH key set up with Github (optional)
<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>  

Clone this repository. To commit to the repository, clone with the ssh address. Don't push to main unless you helped write this README (make your own branch).
```
git clone https://github.com/NMT-Wildlife-Monitoring-Project/WALL-E.git
cd WALL-E
```

Install udev rules
```
cd scripts
sudo cp 99-walle-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Build the docker image  
This may take a while as several large packages are installed.  
```
cd docker
./start_docker.sh -b
```  

# USAGE

## Docker start script -- start_docker.sh
This script handles building and running the docker container, running commands inside the container, and setting up ros networking and environment.
WARNING: The following options are not implemented yet.
```
--start (-s)
--teleop (-t)
--usb-cam (-u)
--video-stream (-v)
--mapping (-M)
--view-map (-w)
--motors (-g)
--rosbridge (-B)
```
```
Usage: ./start_docker.sh [ --start (-s) | --teleop (-t) | --usb-cam (-u)     | --video-stream (-v) | --mapping (-M) | --view-map (-w)     | --motors (-g) | --rosbridge (-B) --command (-c) <command>]      [ --ros-domain-id (-i) <id> | --copy (-C) <from> <to> | --display (-d)     | --build (-b) | --stop (-x) | --restart (-R) | --quiet (-q) | --help (-h) ]
This script is used to start and manage a Docker container for WALL-E the wildlife monitoring robot.
If no IP addresses are specified, the script will attempt to determine them from the hostname. If this fails, try setting the hostname or IP.
If no action is specified, the script will open an interactive bash terminal in the container.
Actions (pick ONE):
  --start (-s)                Start all processes on the robot
  --teleop (-t)               Run joystick control
  --usb-cam (-u)              Run usb camera node
  --video-stream (-v)         View the video stream
  --mapping (-M)              Run mapping
  --view-map (-w)             Run map view
  --motors (-m)               Run motor control
  --rosbridge (-B)            Run rosbridge server
  --command (-c) <command>    Pass a command to be run in the container
Options:
  --ros-domain-id (-i) <id>   Set the ROS domain ID (default: 62)
  --copy (-C) <from> <to>     Copy files from the container to the host
  --display (-d)              Enable display support (forward X11 display)
  --build (-b)                Build the Docker container (will stop the running container if any)
  --stop (-x)                 Stop the running Docker container
  --restart (-R)              Restart the Docker container if it is running
  --quiet (-q)                Suppress output
  --help (-h)                 Show this help message
  ```

# Ros Workspace
TODO: Basically all of these

## ros2_roboclaw_driver
TODO: Configure this
Separate ros workspace

## robot_bringup

## robot_description

## robot_motors
we might not need this one

## robot_navigation

## robot_teleop
Run the teleop node
'ros2 launch teleop teleop_launch.py'
See <https://wiki.ros.org/joy> and <https://wiki.ros.org/teleop_twist_joy>. The documentation is for ros 1 but the parameters are the same for the most part. Use
`ros2 param list` when running the node to see available parameters.

## robot_web_interface

## sllidar_ros2
This package is for rplidar laserscan sensors.





# Cellular
<https://www.waveshare.com/wiki/SIM7600E-H_4G_HAT>


sudo nmcli dev disconnect wlan0
sudo zerotier-cli peers
ipv4 - trying wifi
sudo systemctl restart zerotier-one.service
sudo zerotier-cli peers 
should be ipv6
may need to reboot
