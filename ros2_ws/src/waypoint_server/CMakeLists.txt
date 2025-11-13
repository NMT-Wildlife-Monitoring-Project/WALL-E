cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
