# ROS 2 Humble Doosan E0509 GUI Simulation

This repository provides a ROS 2 Humble package skeleton for the Doosan E0509 robot arm assignment. It includes a PyQt5 GUI for queueing Cartesian targets (absolute or relative), configurable velocity/acceleration limits, and simulated joint/pose feedback.

## Layout
```
ros2_ws/
  requirements.txt
  src/my_ros2_assignment/
    package.xml
    setup.py
    setup.cfg
    resource/my_ros2_assignment
    my_ros2_assignment/__init__.py
    my_ros2_assignment/my_node.py
```

## Running (Ubuntu 22.04 + ROS 2 Humble)
1. Install apt dependencies (the requirements file contains only package names, so `xargs -a` works without extra filtering):
   ```bash
   cd ros2_ws
   sudo apt-get update
   xargs -a requirements.txt sudo apt-get install -y
   ```
2. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```
3. Launch the GUI-driven simulator:
   ```bash
   ros2 run my_ros2_assignment my_node
   ```

The GUI uses a separate thread to run ROS 2, lets you queue targets, and updates simulated joint angles and base-frame pose in real time.
