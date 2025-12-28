cmake_minimum_required(VERSION 3.5)
project(my_ros2_assignment)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Consume the CATKIN_INSTALL_INTO_PREFIX_ROOT cache entry if it is present in
# the environment (some ROS 1/ROS 2 bridge setups export it). This prevents
# CMake from warning about an unused manually-specified variable while having
# no functional impact on this pure ROS 2 package.
option(CATKIN_INSTALL_INTO_PREFIX_ROOT "Compatibility shim; ignored" OFF)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    my_ros2_assignment/my_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
