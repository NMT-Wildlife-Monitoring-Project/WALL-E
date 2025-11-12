cmake_minimum_required(VERSION 3.5)
project(waypoint_server)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  waypoint_server/waypoint_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

