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

Ensure you have an SSH key set up with Github  
<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>  

Clone this repository with SSH and cd into it  
```
git clone git@github.com:NMT-Wildlife-Monitoring-Project/WALL-E.git
cd WALL-E
```

# USAGE

## start_docker.sh
```
pi5-walle@raspberrypi:~/WALL-E $ ./start_docker.sh -h
Usage: ./start_docker.sh [--start (-s) | --teleop (-t) | --usb-cam (-u) | --command (-c) <command> | --roscore (-r) | --build (-b) | --stop (-x)] [--port (-p) <port>] [--ip (-i) <host_ip>] [--display (-d)] [--help (-h)]
  --start (-s)                Run robot_start.launch from package control
  --teleop (-t)               Run teleop.launch from package control
  --usb-cam (-u)              Run rosrun usb_cam usb_cam_node
  --command (-c) <command>    Pass a command to be run in the container
  --roscore (-r)              Run roscore
  --port (-p) <port>          Specify custom ROS master port (default is 11311)
  --ip (-i) <host_ip>         Specify host IP
  --display (-d)              Enable display support (forward X11 display)
  --build (-b)                Build the Docker container
  --stop (-x)                 Stop the Docker container if it is running
  --help (-h)                 Show this help message
```

## mjpg_streaner

To start an http video stream from the raspberry pi
```mjpg_streamer -i "input_uvc.so -d /dev/video0" -o "output_http.so"```

To view the stream from the pi
<http://raspberrypi.local:8080/?action=stream>

To view the stream from another computer, replace raspberrypi.local with the pi's ip address.
