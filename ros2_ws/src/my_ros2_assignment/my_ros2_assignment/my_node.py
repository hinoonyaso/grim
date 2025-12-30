"""ROS2-based simulation scaffolding for the Doosan E0509 arm with a PyQt5 GUI."""
from __future__ import annotations

import importlib.util
import math
import sys
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

QtSpec = importlib.util.find_spec("PyQt5")
if QtSpec:
    from PyQt5 import QtCore, QtWidgets
else:  # pragma: no cover - headless environments
    QtCore = None  # type: ignore
    QtWidgets = None  # type: ignore

RclpySpec = importlib.util.find_spec("rclpy")
if RclpySpec:
    import rclpy
    from rclpy.node import Node
else:  # pragma: no cover - simplifies running without ROS2
    rclpy = None  # type: ignore
    Node = object  # type: ignore


@dataclass
class TargetPose:
    """Container describing a single end-effector target."""

    x: float
    y: float
    z: float
    is_relative: bool
    speed: float
    acceleration: float


@dataclass
class RobotState:
    """In-memory representation of the simulated robot."""

    joint_positions: List[float] = field(default_factory=lambda: [0.0] * 6)
    ee_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    moving: bool = False
    connected: bool = True


class FakeDoosanArm:
    """Small deterministic simulator for the Doosan E0509 workspace."""

    def __init__(self) -> None:
        self.state = RobotState()
        self._lock = threading.Lock()

    def move_to(self, pose: TargetPose, status_callback=None) -> None:
        """Step the end-effector toward the target while updating joint guesses."""
        with self._lock:
            self.state.moving = True
        steps = max(5, int(max(abs(coord) for coord in (pose.x, pose.y, pose.z)) * 10))
        origin = self.state.ee_pose
        target = (
            origin[0] + pose.x if pose.is_relative else pose.x,
            origin[1] + pose.y if pose.is_relative else pose.y,
            origin[2] + pose.z if pose.is_relative else pose.z,
        )
        for i in range(1, steps + 1):
            blend = i / steps
            interpolated = tuple(origin[idx] + (target[idx] - origin[idx]) * blend for idx in range(3))
            with self._lock:
                self.state.ee_pose = interpolated
                self.state.joint_positions = self._inverse_kinematics_guess(interpolated)
            if status_callback:
                status_callback(f"Moving to {target} ({i}/{steps})")
            QtCore.QThread.msleep(int(1000 * max(0.001, 1.0 / (pose.speed + 1e-3))))
        with self._lock:
            self.state.moving = False
            self.state.ee_pose = target
            self.state.joint_positions = self._inverse_kinematics_guess(target)

    def _inverse_kinematics_guess(self, xyz: Tuple[float, float, float]) -> List[float]:
        """Fake IK that maps xyz to joint space in a reproducible way."""
        x, y, z = xyz
        base = math.atan2(y, x)
        shoulder = math.atan2(z, math.hypot(x, y))
        elbow = -shoulder / 2
        wrist_pitch = shoulder / 2
        wrist_roll = base / 2
        tool = 0.0
        return [math.degrees(angle) for angle in (base, shoulder, elbow, wrist_pitch, wrist_roll, tool)]


class Ros2Bridge(Node):
    """Lightweight ROS2 node that can be expanded for real robots."""

    def __init__(self) -> None:
        if rclpy:
            super().__init__("doosan_arm_gui")
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False


class MotionWorker(QtCore.QThread):
    update_status = QtCore.pyqtSignal(str)
    update_log = QtCore.pyqtSignal(str)
    update_joint = QtCore.pyqtSignal(list)
    update_pose = QtCore.pyqtSignal(tuple)
    finished = QtCore.pyqtSignal()

    def __init__(self, controller: FakeDoosanArm, queue: List[TargetPose]) -> None:
        super().__init__()
        self.controller = controller
        self.queue = queue
        self._cancelled = False

    def cancel(self) -> None:
        self._cancelled = True

    def run(self) -> None:  # type: ignore[override]
        for pose in self.queue:
            if self._cancelled:
                self.update_log.emit("Motion cancelled")
                break
            self.update_status.emit("Moving")
            self.controller.move_to(pose, status_callback=self.update_log.emit)
            self.update_joint.emit(self.controller.state.joint_positions)
            self.update_pose.emit(self.controller.state.ee_pose)
        self.update_status.emit("Idle")
        self.finished.emit()


class ControlWindow(QtWidgets.QMainWindow):
    """PyQt5 control panel for the Doosan arm."""

    def __init__(self, controller: FakeDoosanArm, ros_node: Optional[Ros2Bridge]) -> None:
        super().__init__()
        self.controller = controller
        self.ros_node = ros_node
        self.worker: Optional[MotionWorker] = None
        self.setWindowTitle("Doosan E0509 Simulator")
        self.resize(600, 480)
        self._build_ui()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        layout.addLayout(self._build_control_panel())
        layout.addLayout(self._build_status_panel())
        self.setCentralWidget(central)

    def _build_control_panel(self) -> QtWidgets.QVBoxLayout:
        controls = QtWidgets.QVBoxLayout()

        self.rel_checkbox = QtWidgets.QCheckBox("Relative coordinates")
        controls.addWidget(self.rel_checkbox)

        self.target_edit = QtWidgets.QPlainTextEdit()
        self.target_edit.setPlaceholderText("Enter targets: x,y,z per line")
        controls.addWidget(self._labeled_widget("Targets", self.target_edit))

        self.speed_edit = QtWidgets.QLineEdit("0.2")
        self.accel_edit = QtWidgets.QLineEdit("0.1")
        controls.addWidget(self._labeled_widget("Max speed (m/s)", self.speed_edit))
        controls.addWidget(self._labeled_widget("Max acceleration (m/s^2)", self.accel_edit))

        self.run_button = QtWidgets.QPushButton("Execute trajectory")
        self.run_button.clicked.connect(self.start_motion)
        controls.addWidget(self.run_button)

        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_motion)
        controls.addWidget(self.stop_button)

        controls.addStretch(1)
        return controls

    def _build_status_panel(self) -> QtWidgets.QVBoxLayout:
        status = QtWidgets.QVBoxLayout()

        self.conn_label = QtWidgets.QLabel("ROS2: disconnected")
        status.addWidget(self.conn_label)

        self.motion_label = QtWidgets.QLabel("State: Idle")
        status.addWidget(self.motion_label)

        self.pose_label = QtWidgets.QLabel("EE pose: (0, 0, 0)")
        status.addWidget(self.pose_label)

        self.joint_label = QtWidgets.QLabel("Joints: 0, 0, 0, 0, 0, 0")
        status.addWidget(self.joint_label)

        self.log_box = QtWidgets.QTextEdit()
        self.log_box.setReadOnly(True)
        status.addWidget(self._labeled_widget("Log", self.log_box))

        return status

    def _labeled_widget(self, label: str, widget: QtWidgets.QWidget) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.addWidget(QtWidgets.QLabel(label))
        layout.addWidget(widget)
        return container

    def _parse_targets(self) -> List[TargetPose]:
        lines = [line.strip() for line in self.target_edit.toPlainText().splitlines() if line.strip()]
        targets: List[TargetPose] = []
        speed = float(self.speed_edit.text() or 0)
        accel = float(self.accel_edit.text() or 0)
        for line in lines:
            parts = line.split(',')
            if len(parts) != 3:
                self._append_log(f"Skipping invalid target: {line}")
                continue
            x, y, z = map(float, parts)
            targets.append(
                TargetPose(
                    x=x,
                    y=y,
                    z=z,
                    is_relative=self.rel_checkbox.isChecked(),
                    speed=speed,
                    acceleration=accel,
                )
            )
        return targets

    def start_motion(self) -> None:
        targets = self._parse_targets()
        if not targets:
            self._append_log("No valid targets specified")
            return
        if self.worker and self.worker.isRunning():
            self._append_log("Already moving")
            return
        self.worker = MotionWorker(self.controller, targets)
        self.worker.update_status.connect(self._update_status)
        self.worker.update_joint.connect(lambda joints: self.joint_label.setText(f"Joints: {', '.join(f'{j:.1f}' for j in joints)}"))
        self.worker.update_pose.connect(lambda pose: self.pose_label.setText(f"EE pose: ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})"))
        self.worker.update_log.connect(self._append_log)
        self.worker.finished.connect(lambda: self.run_button.setEnabled(True))
        self.run_button.setEnabled(False)
        self.worker.start()

    def stop_motion(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.cancel()
            self._append_log("Stop requested")

    def _append_log(self, message: str) -> None:
        self.log_box.append(message)

    def _update_status(self, state: str) -> None:
        self.motion_label.setText(f"State: {state}")


def init_ros() -> Optional[Ros2Bridge]:
    if not rclpy:
        return None
    rclpy.init()
    node = Ros2Bridge()
    node.connect()
    return node


def main(argv: Optional[List[str]] = None) -> None:
    if not QtWidgets:
        sys.stderr.write("PyQt5 is required to run this GUI\n")
        return
    argv = argv if argv is not None else sys.argv
    app = QtWidgets.QApplication(argv)
    controller = FakeDoosanArm()
    ros_node = init_ros()
    window = ControlWindow(controller, ros_node)
    if ros_node:
        window.conn_label.setText("ROS2: connected")
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
