### Dana's Notes
-entrypoint.sh its the first thing that runs when the container starts; DOES NOT RUN when docker exec

-ros2 launch rplidar_ros view_rplidar_a1_launch.py   #Runs rviz

sudo ./start_docker.sh   #Start WALL-E image (might have different name)
#####

#Run the motor control node
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
`ros2 param list' when running the node to see available parameters.
  



