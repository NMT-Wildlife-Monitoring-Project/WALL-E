# WALL-E
Wildlife Activity Life Explorer
The main repository for the New Mexico Tech Wildlife Monitoring Project

# Installation - Linux

To use ros kinetic in docker to control WALLE remotely, see the follwing sections
in the Raspberry Pi Installation Section

1. Install Docker
2. Install Zerotier and join network <https://www.zerotier.com/download/>
2. Clone repository with SSH
3. Build with Docker

# Installation - Raspberry Pi

These instructions are for seting up ros kinetic in docker on a raspberry pi for WALLE

## Install docker  
<https://docs.docker.com/engine/install/ubuntu/>  

To use docker without sudo, run the following commands  
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## Install dependencies  
### Install Zerotier  
`curl -s https://install.zerotier.com | sudo bash`  
### Join network
`sudo zerotier-cli join 6ab565387a6fafa0`  
After joining, be sure to authorize the new device on the ZeroTier website.
### Raspberry pi installed packages
`sudo apt install gpsd gpsd-client python3-gps python3-flask python3-gps python3-opencv python3-numpy python3-netifaces python3-pigpio`  

## Clone this repository  
Make an SSH key with `ssh-keygen`.  
Ensure you have your SSH key set up with your Github account.  
<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>  

Clone this repository with SSH and cd into it  
```
git clone git@github.com:NMT-Wildlife-Monitoring-Project/WALL-E.git
cd WALL-E
```

## Build with Docker
`start_docker.sh -b` or `--build` will handle building the Docker container. The Docker
container will copy in the catkin workspace and build with catkin. If any changes are made
to the catkin workspace or dockerfile, the container must be rebuilt. It can also be run
with other options. 
`walle@raspberrypi:~/WALL-E $ ./start_docker.sh -b`  

## Setup Waveshare cellular shield  

### Disable Bluetooth  
1. `sudo nano /boot/firmware/config.txt`  
2. Add `dtoverlay=disable-bt` before or inside `[all`. Ensure `enable_uart=1` is present.  
3. `sudo systemctl disable hciuart`  
4. Reboot  
5. `sudo reboot`  

## Setup GPS
1. Edit `sudo nano /etc/default/gpsd` to match
```
START_DAEMON="true"
GPSD_OPTIONS=""
DEVICES="/dev/ttyUSB1"
USBAUTO="false"
GPSD_SOCKET="/var/run/gpsd.sock"
```
2. Start the service
3. Test with `cgps`


## Installing Services

To set up the services for automatic startup:

1. Copy the service files to the systemd directory:  
  ```
  sudo cp *.service /etc/systemd/system/
  ```

2. Reload the systemd manager configuration  
  ```
  sudo systemctl daemon-reload
  ```

3. Enable the services to start on boot:  
  ```
  sudo systemctl enable robot_start.service
  sudo systemctl enable gps_start.service
  sudo systemctl enable web_app.service
  sudo systemctl enable waveshare.service
  sudo systemctl enable gpsd
  sudo systemctl enable pigpiod
  ```

4. Start the services manually or restart:  
  ```
  sudo systemctl start robot_start.service
  sudo systemctl start gps_start.service
  sudo systemctl start web_app.service
  sudo systemctl start waveshare.service
  sudo systemctl start gpsd
  sudo systemctl start pigpiod
  ```

5. Check the status of the service:  
  `sudo systemctl status service_name`  
  `journalctl -eu service_name`  

# USAGE

## Website Control Panel
To visit the website to view WALLE's location, map, and camera feed or control WALLE remotely (IN DEVELOPMENT)
go to http://raspberrypi_zerotier_ip:5000 (<http://172.27.32.111:5000>). You must have joined the zerotier
network and been authorized to view the website.

## start_docker.sh
```
walle@raspberrypi:~/WALL-E $ ./start_docker.sh -h
Usage: ./start_docker.sh [--start (-s) | --teleop (-t) | --usb-cam (-u) | --video-stream (-v) |
           --mapping (-M) | --view-map (-w) | --motors (-m) | --command (-c) <command> | --roscore (-r) | --build (-b) | --stop (-x) |
           --restart (-R)] [--port (-p) <port>] [--ip (-i) <local_ip>] [--master-ip (-m) <master_ip>]
           [--master-hostname (-n) <master_hostname>] [--display (-d)] [--quiet (-q)] [--help (-h)]
This script is used to start and manage a Docker container for WALL-E the wildlife monitoring robot.
If no IP addresses are specified, the script will attempt to determine them from the hostname. If this fails, try setting the hostname or IP.
If no action is specified, the script will open an interactive bash terminal in the container.
Actions (pick ONE):
  --start (-s)                Start all processes on the robot
  --teleop (-t)               Run joystick control using teleop.launch
  --usb-cam (-u)              Run usb camera node using usb_cam.launch
  --video-stream (-v)         View the video stream using view_camera.launch
  --mapping (-M)              Run mapping process using the slamtec mapper
  --view-map (-w)             Run map view using view_slamware_ros_sdk_server_node.launch
  --motors (-g)               Run motor control using motor_control.launch
  --rosbridge (-B)            Run rosbridge server
  --roscore (-r)              Run roscore
  --command (-c) <command>    Pass a command to be run in the container
Options:
  --port (-p) <port>          Specify custom ROS master port (default is 11311)
  --ip (-i) <local_ip>         Specify local IP
  --master-ip (-m) <master_ip> Specify master IP
  --master-hostname (-n) <master_hostname> Specify master hostname (default is raspberrypi.local)
  --display (-d)              Enable display support (forward X11 display)
  --build (-b)                Build the Docker container (will stop the running container if any)
  --stop (-x)                 Stop the running Docker container
  --restart (-R)              Restart the Docker container if it is running
  --quiet (-q)                Suppress output
  --help (-h)                 Show this help message
```

Without the proper IP addresses, ROS 1 will not work across machines. Normally, ROS over wifi
would also only work if all machines are on the same network and can ping each other. Ros
requires two IP addresses for this, the IP of the local machine, and the IP address of the machine
running the ROS master, which should normally be the Raspberry Pi through a systemctl service.
Zerotier will connect the machines across networks so that they can ping each other with their zerotier
IP address.

These IP addresses are set at the top of `start_docker.sh` (recommended for permanent IP addresses)
or passed as command line arguments with `--ip` and `--master-ip`. Unless the cellular shield is not
being used, these should be the zerotier IP address.

# Spare Docs

## mjpg_streaner

To start an http video stream from the raspberry pi
```mjpg_streamer -i "input_uvc.so -d /dev/video0" -o "output_http.so"```

To view the stream from the pi
<http://raspberrypi.local:8080/?action=stream>

To view the stream from another computer, replace raspberrypi.local with the pi's ip address.

## Slamtec Ros SDK Documentation
<https://developer.slamtec.com/docs/slamware/ros-sdk-en/2.8.2_rtm/slamware_ros_sdk_server_node/>

## Cellular shield
<https://www.waveshare.com/wiki/SIM7600E-H_4G_HAT?
