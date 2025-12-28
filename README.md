# Doosan E0509 ROS2 simulation scaffold

This repository contains a ROS 2 package skeleton and PyQt5 GUI for the Doosan E0509 robot arm simulation described in the assignment. The package is intentionally lightweight so it can run in environments without a full ROS 2 install while keeping the directory structure and entry points expected by `colcon`.

## Repository layout
- `ros2_ws/src/my_ros2_assignment`: ROS 2 Python package with GUI and simulator code.
- `ros2_ws/requirements.txt`: Suggested apt dependencies for ROS 2 Humble, MoveIt 2, and PyQt5.

## Running
1. Install ROS 2 Humble (or Foxy) and PyQt5 on Ubuntu 22.04/20.04.
2. Build the workspace:
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```
3. Launch the GUI:
   ```bash
   ros2 run my_ros2_assignment my_node
   ```

## GUI features
- Enter one or more Cartesian targets (x,y,z per line) and choose relative mode.
- Provide maximum speed and acceleration values.
- Execute and stop trajectory playback in a background worker thread.
- Live updates for connection status, motion state, joint angles, and end-effector pose along with a scrolling log.

## Notes
- The `FakeDoosanArm` class supplies a deterministic kinematic stub so the GUI can be demonstrated without hardware.
- When ROS 2 libraries are unavailable, the GUI still loads but publishes nothing; the `Ros2Bridge` class is ready for extension to real command publishers.
