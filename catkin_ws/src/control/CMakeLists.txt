cmake_minimum_required(VERSION 3.0.2)
project(control)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  sensor_msgs
  actionlib
  move_base_msgs
  tf
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp roslaunch sensor_msgs actionlib move_base_msgs tf geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
