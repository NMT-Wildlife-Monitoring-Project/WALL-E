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
```

Build the docker image  
`./start_docker.sh`  

# USAGE

Run rtabmap and rvis
```
./rtabmap.sh
```
open a new terminal window and run
`./rvis.sh` 

To run teleop
`./teleop.sh`

To open another terminal in the same container run  
`docker exec -it --privileged luna bash`  
then:
`source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`


Change parameters in the launch file

Run the motor control node
`ros2 launch motor_control motor_launch.py'
Parameters:
- cmd_vel_topic: cmd_vel
- wheel_base: 0.5
- wheel_diameter: 0.1
- max_rpm: 100
- min_rpm: 10
- motor_serial_device: /dev/serial0

Run the teleop node
'ros2 launch teleop teleop_launch.py'
See <https://wiki.ros.org/joy> and <https://wiki.ros.org/teleop_twist_joy>. The documentation is for ros 1 but the parameters are the same for the most part. Use
`ros2 param list` when running the node to see available parameters.

# Cellular
<https://www.waveshare.com/wiki/SIM7600E-H_4G_HAT>
