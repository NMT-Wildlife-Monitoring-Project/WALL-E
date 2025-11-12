cmake_minimum_required(VERSION 3.5)
project(waypoint_server)

# Ensure we can find the ament macros
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files (if you have any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource files (optional)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install executables (Python entrypoints)
install(PROGRAMS
  src/waypoint_server/waypoint_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

