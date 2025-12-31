from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayTrajectory, RobotState as MoveItRobotState, RobotTrajectory
from qtpy import QtCore, QtWidgets
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class MotionTarget:
    """Represents a single end-effector target."""

    point: Point
    relative: bool = False
    velocity: float = 0.1
    acceleration: float = 0.1


@dataclass
class RobotSimState:
    joint_positions: List[float] = field(default_factory=list)
    ee_position: np.ndarray = field(default_factory=lambda: np.zeros(3))


class RobotControlNode(Node):
    """ROS 2 node that simulates a Doosan E0509 robot arm."""

    def __init__(self) -> None:
        super().__init__('doosan_e0509_gui_sim')
        self.state = RobotSimState()
        self.joint_names = [f'joint{i}' for i in range(1, 7)]
        self.declare_parameter('robot_model_id', 'e0509')
        self.declare_parameter('joint_names', self.joint_names)
        self.declare_parameter(
            'controller_topic', '/dsr01/scaled_joint_trajectory_controller/joint_trajectory'
        )
        self.joint_names = list(
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        self.dof = len(self.joint_names)
        if not self.state.joint_positions:
            self.state.joint_positions = [0.0] * self.dof
        controller_topic = self.get_parameter('controller_topic').get_parameter_value().string_value
        self.robot_model_id = (
            self.get_parameter('robot_model_id').get_parameter_value().string_value
        )
        self._status_pub = self.create_publisher(String, '/sim/status', 10)
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 50)
        self._trajectory_pub = self.create_publisher(JointTrajectory, controller_topic, 10)
        self._display_traj_pub = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        self._status_pub.publish(String(data='Robot control node initialized.'))

    def update_state(self, target: MotionTarget, progress: float) -> None:
        """Interpolate robot state toward a target and publish status."""
        direction = np.array([target.point.x, target.point.y, target.point.z])
        if target.relative:
            direction = self.state.ee_position + direction
        new_pose = (1.0 - progress) * self.state.ee_position + progress * direction
        self.state.ee_position = new_pose
        self.state.joint_positions = self.ik_placeholder(new_pose)
        message = f"Moving to ({new_pose[0]:.3f}, {new_pose[1]:.3f}, {new_pose[2]:.3f})"
        self._status_pub.publish(String(data=message))
        self.publish_joint_state()

    def ik_placeholder(self, ee_position: np.ndarray) -> List[float]:
        """A placeholder inverse kinematics calculation.

        MoveIt or the Doosan SDK should replace this logic in a real deployment.
        """
        scales = np.linspace(0.5, 1.0, num=self.dof)
        return [float(ee_position.mean() * scale) for scale in scales]

    def publish_joint_state(self) -> None:
        """Publish the current joint state for RViz/Gazebo."""

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.state.joint_positions)
        self._joint_state_pub.publish(msg)

    def publish_preview_trajectory(
        self, start: List[float], goal: List[float], duration: float = 5.0
    ) -> JointTrajectory:
        """Publish a simple joint-space trajectory that RViz and controllers can replay."""

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        start_point = JointTrajectoryPoint()
        start_point.positions = start
        start_point.time_from_start.sec = 0
        traj.points.append(start_point)

        goal_point = JointTrajectoryPoint()
        goal_point.positions = goal
        goal_point.time_from_start.sec = int(duration)
        traj.points.append(goal_point)

        self._trajectory_pub.publish(traj)

        display = DisplayTrajectory()
        display.model_id = self.robot_model_id
        display.trajectory_start = MoveItRobotState()
        display.trajectory_start.joint_state.name = self.joint_names
        display.trajectory_start.joint_state.position = start
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        display.trajectory.append(robot_traj)
        self._display_traj_pub.publish(display)
        return traj


class ROSExecutorThread(QtCore.QThread):
    """Runs the ROS 2 executor on a background thread to keep Qt responsive."""

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.node = node
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def run(self) -> None:
        self.executor.spin()

    def stop(self) -> None:
        self.executor.shutdown()
        self.executor.remove_node(self.node)


class MotionWorker(QtCore.QThread):
    """Executes queued motion targets on behalf of the GUI."""

    status_changed = QtCore.Signal(str)
    pose_changed = QtCore.Signal(float, float, float)
    joints_changed = QtCore.Signal(list)

    def __init__(self, node: RobotControlNode, targets: List[MotionTarget]) -> None:
        super().__init__()
        self.node = node
        self.targets = targets
        self._stop_requested = False

    def request_stop(self) -> None:
        self._stop_requested = True

    def run(self) -> None:
        for target in self.targets:
            if self._stop_requested:
                self.status_changed.emit('Motion cancelled by user.')
                return
            self.status_changed.emit('Executing target...')
            target_pose = np.array([target.point.x, target.point.y, target.point.z])
            start_joints = self.node.state.joint_positions.copy()
            goal_joints = self.node.ik_placeholder(
                self.node.state.ee_position + target_pose if target.relative else target_pose
            )
            self.node.publish_preview_trajectory(start_joints, goal_joints, duration=5.0)
            steps = max(5, int(target.velocity * 100))
            for i in range(steps + 1):
                if self._stop_requested:
                    self.status_changed.emit('Motion cancelled by user.')
                    return
                progress = i / steps
                self.node.update_state(target, progress)
                pose = self.node.state.ee_position
                self.pose_changed.emit(float(pose[0]), float(pose[1]), float(pose[2]))
                self.joints_changed.emit(self.node.state.joint_positions)
                self.msleep(max(10, int(1000 / steps)))
            self.status_changed.emit('Target complete.')
        self.status_changed.emit('All targets complete.')


class ControlWindow(QtWidgets.QMainWindow):
    """Qt GUI that gathers user input and displays robot state."""

    def __init__(self, node: RobotControlNode, executor_thread: ROSExecutorThread) -> None:
        super().__init__()
        self.setWindowTitle('Doosan E0509 GUI Simulator (ROS 2 Humble)')
        self.node = node
        self.executor_thread = executor_thread
        self.motion_worker: Optional[MotionWorker] = None
        self.targets: List[MotionTarget] = []

        self._build_layout()

    def _build_layout(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        controls = QtWidgets.QGroupBox('Control')
        controls_layout = QtWidgets.QFormLayout(controls)

        self.relative_checkbox = QtWidgets.QCheckBox('Use relative coordinates')
        self.relative_checkbox.setChecked(False)
        controls_layout.addRow(self.relative_checkbox)

        self.x_input = QtWidgets.QLineEdit('0.3')
        self.y_input = QtWidgets.QLineEdit('0.0')
        self.z_input = QtWidgets.QLineEdit('0.3')
        controls_layout.addRow('Target X (m)', self.x_input)
        controls_layout.addRow('Target Y (m)', self.y_input)
        controls_layout.addRow('Target Z (m)', self.z_input)

        self.velocity_input = QtWidgets.QLineEdit('0.2')
        self.acc_input = QtWidgets.QLineEdit('0.2')
        controls_layout.addRow('Max velocity (m/s)', self.velocity_input)
        controls_layout.addRow('Max acceleration (m/s^2)', self.acc_input)

        self.add_target_button = QtWidgets.QPushButton('Add target')
        self.add_target_button.clicked.connect(self._on_add_target)
        self.run_button = QtWidgets.QPushButton('Execute queue')
        self.run_button.clicked.connect(self._on_execute)
        self.stop_button = QtWidgets.QPushButton('Stop')
        self.stop_button.clicked.connect(self._on_stop)
        controls_layout.addRow(self.add_target_button)
        controls_layout.addRow(self.run_button)
        controls_layout.addRow(self.stop_button)

        self.queue_view = QtWidgets.QListWidget()
        controls_layout.addRow('Queued targets', self.queue_view)

        status_box = QtWidgets.QGroupBox('Status')
        status_layout = QtWidgets.QFormLayout(status_box)
        self.connection_label = QtWidgets.QLabel('Connected (simulated)')
        self.motion_label = QtWidgets.QLabel('Idle')
        self.pose_label = QtWidgets.QLabel('(0.000, 0.000, 0.000)')
        self.joints_label = QtWidgets.QLabel('0, 0, 0, 0, 0, 0')
        self.log_view = QtWidgets.QTextEdit()
        self.log_view.setReadOnly(True)
        status_layout.addRow('Robot connection', self.connection_label)
        status_layout.addRow('Motion state', self.motion_label)
        status_layout.addRow('EE position', self.pose_label)
        status_layout.addRow('Joint angles', self.joints_label)
        status_layout.addRow('Log', self.log_view)

        layout.addWidget(controls)
        layout.addWidget(status_box)

    def _on_add_target(self) -> None:
        try:
            point = Point(x=float(self.x_input.text()), y=float(self.y_input.text()), z=float(self.z_input.text()))
            velocity = max(0.05, float(self.velocity_input.text()))
            acceleration = max(0.05, float(self.acc_input.text()))
        except ValueError:
            self._log('Invalid numeric input for target or motion parameters.')
            return

        target = MotionTarget(point=point, relative=self.relative_checkbox.isChecked(), velocity=velocity, acceleration=acceleration)
        self.targets.append(target)
        display = f"({'rel' if target.relative else 'abs'}) x={point.x:.3f} y={point.y:.3f} z={point.z:.3f} v={velocity:.2f} a={acceleration:.2f}"
        self.queue_view.addItem(display)
        self._log('Queued new target: ' + display)

    def _on_execute(self) -> None:
        if not self.targets:
            self._log('No targets to execute.')
            return
        if self.motion_worker and self.motion_worker.isRunning():
            self._log('Motion already running.')
            return

        self.motion_worker = MotionWorker(self.node, self.targets.copy())
        self.motion_worker.status_changed.connect(self._on_status)
        self.motion_worker.pose_changed.connect(self._on_pose_update)
        self.motion_worker.joints_changed.connect(self._on_joint_update)
        self.motion_worker.finished.connect(self._on_motion_finished)
        self.motion_label.setText('Executing')
        self.motion_worker.start()
        self._log('Started motion queue with %d targets.' % len(self.targets))

    def _on_stop(self) -> None:
        if self.motion_worker and self.motion_worker.isRunning():
            self.motion_worker.request_stop()
            self.motion_label.setText('Stop requested')
            self._log('Stop requested by user.')

    def _on_status(self, text: str) -> None:
        self.motion_label.setText(text)
        self._log(text)

    def _on_pose_update(self, x: float, y: float, z: float) -> None:
        self.pose_label.setText(f'({x:.3f}, {y:.3f}, {z:.3f})')

    def _on_joint_update(self, joints: List[float]) -> None:
        joint_text = ', '.join(f'{value:.3f}' for value in joints)
        self.joints_label.setText(joint_text)

    def _on_motion_finished(self) -> None:
        self.motion_label.setText('Idle')
        self.targets.clear()
        self.queue_view.clear()
        self._log('Motion queue finished.')

    def _log(self, text: str) -> None:
        self.log_view.append(text)
        self.node.get_logger().info(text)


def main() -> None:
    rclpy.init()
    node = RobotControlNode()
    executor_thread = ROSExecutorThread(node)
    executor_thread.start()

    app = QtWidgets.QApplication([])
    window = ControlWindow(node, executor_thread)
    window.resize(900, 500)
    window.show()
    app.exec_()

    if window.motion_worker and window.motion_worker.isRunning():
        window.motion_worker.request_stop()
        window.motion_worker.wait()
    executor_thread.stop()
    executor_thread.wait()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
