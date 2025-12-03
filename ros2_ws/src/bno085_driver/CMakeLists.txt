cmake_minimum_required(VERSION 3.8)
project(bno085_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CalibrateMagnetometer.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python modules manually to avoid duplicate custom targets
install(
  DIRECTORY bno085_driver
  DESTINATION ${PYTHON_INSTALL_DIR}
)

# Install Python executables
# Install Python executable with expected name (no .py extension)
install(PROGRAMS
  bno085_driver/bno085_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bno085_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
