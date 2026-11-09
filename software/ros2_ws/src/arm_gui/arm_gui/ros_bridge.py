"""The only module that talks to ROS. Widgets see Qt signals and plain Python values.

rclpy spins in its own thread; every callback hands its data to the GUI thread through a
Qt signal (queued connection), so widgets are only ever touched from the GUI thread.
"""

import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .qt import QtCore, Signal

ARM_CONTROLLER = "arm_controller"
JOINT_STATE_MAX_RATE_HZ = 20.0


class RosBridge(QtCore.QObject):
    description_received = Signal(str)  # URDF xml
    joint_state = Signal(object)  # {name: (position, velocity, effort)}
    controllers_changed = Signal(object)  # [(name, state)] or None if controller_manager is not running
    nodes_changed = Signal(object)  # sorted list of node names
    goal_status = Signal(str)  # human readable status of the last motion command
    log = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        rclpy.init()
        self._node = Node("arm_gui")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self._node.create_subscription(String, "/robot_description", self._on_description, latched)
        self._node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._action = ActionClient(self._node, FollowJointTrajectory, f"/{ARM_CONTROLLER}/follow_joint_trajectory")
        self._trajectory_pub = self._node.create_publisher(JointTrajectory, f"/{ARM_CONTROLLER}/joint_trajectory", 10)
        self._list_controllers = self._node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._node.create_timer(1.0, self._poll_graph)

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._last_joint_emit = 0.0

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, name="rclpy", daemon=True)
        self._thread.start()

    # ---------------------------------------------------------------- ROS thread
    def _spin(self):
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def _on_description(self, msg):
        self.description_received.emit(msg.data)

    def _on_joint_state(self, msg):
        now = time.monotonic()
        if now - self._last_joint_emit < 1.0 / JOINT_STATE_MAX_RATE_HZ:
            return
        self._last_joint_emit = now
        n = len(msg.name)
        vel = list(msg.velocity) + [float("nan")] * (n - len(msg.velocity))
        eff = list(msg.effort) + [float("nan")] * (n - len(msg.effort))
        self.joint_state.emit({name: (msg.position[i], vel[i], eff[i]) for i, name in enumerate(msg.name)})

    def _poll_graph(self):
        self.nodes_changed.emit(sorted(self._node.get_node_names()))
        if not self._list_controllers.service_is_ready():
            self.controllers_changed.emit(None)
            return
        future = self._list_controllers.call_async(ListControllers.Request())
        future.add_done_callback(self._on_controllers)

    def _on_controllers(self, future):
        result = future.result()
        if result is not None:
            self.controllers_changed.emit([(c.name, c.state) for c in result.controller])

    # ---------------------------------------------------------------- commands (GUI thread)
    def send_joint_goal(self, targets: dict, duration_s: float):
        """Move the given joints (SI units) to their targets in duration_s, all arriving together."""
        if not self._action.server_is_ready():
            self.goal_status.emit(f"Ο {ARM_CONTROLLER} δεν είναι διαθέσιμος")
            return
        point = JointTrajectoryPoint(positions=[float(v) for v in targets.values()])
        point.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1.0) * 1e9))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=list(targets), points=[point])
        self.goal_status.emit(f"Αποστολή κίνησης ({duration_s:.1f} s)…")
        self._action.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def stop(self):
        """Software stop: cancel the running trajectory; the controller holds the current position.
        This is NOT a safety function: the hardware E-stop cuts the servo power."""
        with self._goal_lock:
            handle, self._goal_handle = self._goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        # An empty trajectory also makes the controller stop and hold (covers goals sent by others).
        self._trajectory_pub.publish(JointTrajectory())
        self.goal_status.emit("STOP: η κίνηση ακυρώθηκε")
        self.log.emit("STOP από τον χρήστη")

    def shutdown(self):
        self._executor.shutdown()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ---------------------------------------------------------------- action callbacks (ROS thread)
    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_status.emit("Η κίνηση απορρίφθηκε από τον controller")
            return
        with self._goal_lock:
            self._goal_handle = handle
        self.goal_status.emit("Σε κίνηση…")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        response = future.result()
        status = response.status if response is not None else GoalStatus.STATUS_UNKNOWN
        text = {
            GoalStatus.STATUS_SUCCEEDED: "Η κίνηση ολοκληρώθηκε",
            GoalStatus.STATUS_CANCELED: "Η κίνηση ακυρώθηκε",
            GoalStatus.STATUS_ABORTED: "Η κίνηση διακόπηκε από τον controller",
        }.get(status, f"Η κίνηση τερμάτισε (κατάσταση {status})")
        if response is not None and response.result.error_code != 0:
            text += f": {response.result.error_string or response.result.error_code}"
        self.goal_status.emit(text)
