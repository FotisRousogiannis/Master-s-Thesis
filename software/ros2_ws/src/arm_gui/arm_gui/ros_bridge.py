"""The only module that talks to ROS. Widgets see Qt signals and plain Python values.

rclpy spins in its own thread; every callback hands its data to the GUI thread through a
Qt signal (queued connection), so widgets are only ever touched from the GUI thread.
"""

import json
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .qt import QtCore, Signal

ARM_CONTROLLER = "arm_controller"
JOINT_STATE_MAX_RATE_HZ = 20.0
SERVICE_TIMEOUT_S = 3.0
PARAM_TYPES = {
    ParameterType.PARAMETER_BOOL: "bool",
    ParameterType.PARAMETER_INTEGER: "integer",
    ParameterType.PARAMETER_DOUBLE: "double",
    ParameterType.PARAMETER_STRING: "string",
}
ACTION_SUFFIX = "/_action/send_goal"


def _full_name(name, namespace):
    return f"{namespace.rstrip('/')}/{name}"


def _split_name(full_name):
    namespace, _, name = full_name.rpartition("/")
    return name, namespace or "/"


def diagnostic_dict(status):
    level = status.level  # `byte` in the message: rclpy gives bytes
    return {
        "name": status.name,
        "level": level[0] if isinstance(level, (bytes, bytearray)) else int(level),
        "message": status.message,
        "hardware_id": status.hardware_id,
        "values": {kv.key: kv.value for kv in status.values},
    }


class RosBridge(QtCore.QObject):
    description_received = Signal(str)  # URDF xml
    joint_state = Signal(object)  # {name: (position, velocity, effort)}
    controllers_changed = Signal(object)  # [(name, state)] or None if controller_manager is not running
    nodes_changed = Signal(object)  # sorted list of node names
    goal_status = Signal(str)  # human readable status of the last motion command
    log = Signal(str)
    graph_changed = Signal(object)  # {"nodes": [full names], "topics": [(name, types, n_pub, n_sub)]}
    parameters_loaded = Signal(str, object)  # node, [param dicts] (see _param_dict)
    parameters_set = Signal(str, object)  # node, [(name, ok, reason)]
    service_result = Signal(str, bool, str)  # service, success, message
    diagnostics = Signal(object)  # [{"name", "level" 0 ok/1 warn/2 error/3 stale, "message", "hardware_id", "values"}]

    def __init__(self, parent=None):
        super().__init__(parent)
        rclpy.init()
        self._node = Node("arm_gui")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self._node.create_subscription(String, "/robot_description", self._on_description, latched)
        self._node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self._action = ActionClient(self._node, FollowJointTrajectory, f"/{ARM_CONTROLLER}/follow_joint_trajectory")
        self._logbook_pub = self._node.create_publisher(String, "/logbook/event", 10)  # ADR-0020
        self._trajectory_pub = self._node.create_publisher(JointTrajectory, f"/{ARM_CONTROLLER}/joint_trajectory", 10)
        self._list_controllers = self._node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._node.create_timer(1.0, self._poll_graph)

        self._param_clients = {}
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

    def _on_diagnostics(self, msg):
        self.diagnostics.emit([diagnostic_dict(s) for s in msg.status])

    def _poll_graph(self):
        nodes = sorted(_full_name(n, ns) for n, ns in self._node.get_node_names_and_namespaces())
        self.nodes_changed.emit(nodes)
        topics = [
            (name, types, self._node.count_publishers(name), self._node.count_subscribers(name))
            for name, types in sorted(self._node.get_topic_names_and_types())
        ]
        self.graph_changed.emit({"nodes": nodes, "topics": topics})
        if not self._list_controllers.service_is_ready():
            self.controllers_changed.emit(None)
            return
        future = self._list_controllers.call_async(ListControllers.Request())
        future.add_done_callback(self._on_controllers)

    def _on_controllers(self, future):
        result = future.result()
        if result is not None:
            self.controllers_changed.emit([(c.name, c.state) for c in result.controller])

    # ---------------------------------------------------------------- graph (any thread)
    def node_info(self, full_name: str) -> dict:
        """Publishers, subscriptions, services, clients and actions of a node, with their types."""
        name, namespace = _split_name(full_name)
        n = self._node
        try:
            services = n.get_service_names_and_types_by_node(name, namespace)
            info = {
                "Δημοσιεύει": n.get_publisher_names_and_types_by_node(name, namespace),
                "Συνδρομές": n.get_subscriber_names_and_types_by_node(name, namespace),
                "Actions": [
                    (s[: -len(ACTION_SUFFIX)], [t.replace("_SendGoal", "") for t in types])
                    for s, types in services
                    if s.endswith(ACTION_SUFFIX)
                ],
                "Services": [(s, t) for s, t in services if "/_action/" not in s],
                "Clients": [(s, t) for s, t in n.get_client_names_and_types_by_node(name, namespace) if "/_action/" not in s],
            }
        except Exception as exc:  # node disappeared between listing and query
            return {"Σφάλμα": [(str(exc), [])]}
        return {k: sorted(v) for k, v in info.items()}

    # ---------------------------------------------------------------- parameters (GUI thread starts, ROS thread finishes)
    def _client(self, srv_type, service):
        key = (srv_type, service)
        if key not in self._param_clients:
            self._param_clients[key] = self._node.create_client(srv_type, service)
        return self._param_clients[key]

    def _call(self, srv_type, service, request, on_done, on_error):
        client = self._client(srv_type, service)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            on_error(f"Η υπηρεσία {service} δεν απαντά")
            return

        def done(future):
            exc = future.exception()
            on_error(str(exc)) if exc else on_done(future.result())

        client.call_async(request).add_done_callback(done)

    def request_parameters(self, node: str):
        def fail(msg):
            self.parameters_loaded.emit(node, msg)

        def on_list(result):
            names = sorted(result.result.names)
            self._call(
                DescribeParameters, f"{node}/describe_parameters", DescribeParameters.Request(names=names),
                lambda d: on_describe(names, d.descriptors), fail,
            )

        def on_describe(names, descriptors):
            self._call(
                GetParameters, f"{node}/get_parameters", GetParameters.Request(names=names),
                lambda g: self.parameters_loaded.emit(
                    node, [self._param_dict(n, d, v) for n, d, v in zip(names, descriptors, g.values)]
                ),
                fail,
            )

        threading.Thread(
            target=self._call,
            args=(ListParameters, f"{node}/list_parameters", ListParameters.Request(depth=0), on_list, fail),
            daemon=True,
        ).start()

    @staticmethod
    def _param_dict(name, descriptor, value):
        item = {
            "name": name,
            "type": PARAM_TYPES.get(value.type, "other"),
            "value": parameter_value_to_python(value),
            "description": descriptor.description,
            "read_only": descriptor.read_only,
            "min": None,
            "max": None,
        }
        if descriptor.integer_range:
            item["min"], item["max"] = descriptor.integer_range[0].from_value, descriptor.integer_range[0].to_value
        elif descriptor.floating_point_range:
            r = descriptor.floating_point_range[0]
            item["min"], item["max"] = r.from_value, r.to_value
        return item

    def set_parameters(self, node: str, values: dict):
        """values: {name: python value of the parameter's own type}. Each is accepted or rejected by the node."""
        names = list(values)
        request = SetParameters.Request(parameters=[Parameter(n, value=v).to_parameter_msg() for n, v in values.items()])
        threading.Thread(
            target=self._call,
            args=(
                SetParameters, f"{node}/set_parameters", request,
                lambda r: self.parameters_set.emit(
                    node, [(n, res.successful, res.reason) for n, res in zip(names, r.results)]
                ),
                lambda msg: self.parameters_set.emit(node, [(n, False, msg) for n in names]),
            ),
            daemon=True,
        ).start()

    def call_trigger(self, service: str):
        threading.Thread(
            target=self._call,
            args=(
                Trigger, service, Trigger.Request(),
                lambda r: self.service_result.emit(service, r.success, r.message),
                lambda msg: self.service_result.emit(service, False, msg),
            ),
            daemon=True,
        ).start()

    # ---------------------------------------------------------------- commands (GUI thread)
    def log_event(self, event: dict):
        """One entry for the logbook (arm_logbook node, ADR-0020)."""
        self._logbook_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

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
