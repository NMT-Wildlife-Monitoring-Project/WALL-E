cmake_minimum_required(VERSION 3.8)
project(robot_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RoboclawStatus.msg"
)

ament_package()