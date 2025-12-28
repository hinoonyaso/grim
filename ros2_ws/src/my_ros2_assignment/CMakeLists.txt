cmake_minimum_required(VERSION 3.5)
project(my_ros2_assignment)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    my_ros2_assignment/my_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
