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

 The GUI uses a separate thread to run ROS 2, lets you queue targets, and updates simulated joint angles and base-frame pose in real time. It publishes `/joint_states`, `/display_planned_path` (MoveIt/RViz) and a configurable `FollowJointTrajectory` topic so Gazebo or RViz can mirror the motion.

### Integrating MoveIt 2 + RViz
1. Clone the Doosan E-series MoveIt 2 support into the workspace alongside this package (example):
   ```bash
   cd ros2_ws/src
   git clone https://github.com/DoosanRobotics/doosan-robot2.git
   git clone https://github.com/moveit/moveit2.git --branch humble
   cd .. && rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```
2. Launch the Doosan E0509 MoveIt bringup and RViz2. In the official `doosan-robot2` repo the package names follow the pattern `dsr_moveit_config_<model>`, so for the E0509 model the tested command is:
   ```bash
   ros2 launch dsr_moveit_config_e0509 start.launch.py
   ```
   If you are using a different model or branch, run `ros2 pkg list | grep dsr_moveit_config` to confirm the package name before launching.
3. Run the GUI node from this package. The queued targets will broadcast `DisplayTrajectory` and `JointTrajectory` messages so RViz shows the path and controllers can follow it. The controller topic defaults to the Doosan MoveIt2 scaled controller (`/dsr01/scaled_joint_trajectory_controller/joint_trajectory`) and the MoveIt model id defaults to `e0509`. Override them (and the joint names if your MoveIt config uses different names) to match your bringup:
   ```bash
   ros2 run my_ros2_assignment my_node \
    --ros-args \
    -p controller_topic:=/dsr_moveit_controller/joint_trajectory \
    -p robot_model_id:=e0509 \
    -p joint_names:="['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']"
   ```

### Integrating Gazebo
1. Start a Gazebo simulation with the Doosan E0509 model and a `FollowJointTrajectory`-compatible controller (see the `doosan-robot2` gazebo launch files for the exact command). Ensure the controller subscribes to `/doosan_arm_controller/joint_trajectory` or update the topic in `my_node.py` if your setup uses a different controller name.
2. Run the GUI node. Joint states and trajectories published by the GUI will drive the simulated robot while the GUI displays the same end-effector motion.
