"""arm_calibration node: serves the one arm file (arm.yaml, ADR-0023) as ROS 2 parameters.

    ros2 run arm_calibration calibration_node
    ros2 param set /arm_calibration servos.servo_1.neutral_us 1605
    ros2 service call /arm_calibration/save std_srvs/srv/Trigger
    ros2 service call /arm_calibration/set_zero std_srvs/srv/Trigger   # arm in its reference pose
    ros2 service call /arm_calibration/set_zero/joint_1 std_srvs/srv/Trigger   # one joint only

Parameters are the file's flat keys (joints.joint_1.zero_deg); a null in the file has no
parameter until it is given a value (e.g. ros2 param set ... encoders.enc_3.channel 4).
Every change is validated against the whole file (arm_config.validate: ranges, unique channels,
references, signs) before it is accepted. ~/save writes the file, ~/reload discards unsaved changes.
~/set_zero takes the encoder readings of the arm in its reference pose as the joint zeros
(ADR-0019): all joints or none, from the encoder/<joint> entries of /diagnostics.
~/set_zero/<joint> does the same for one joint (while not all encoders are connected, or to
redo one joint); Arm Studio calls it for the joints the user ticks.
The hardware layer (arm_hardware -> ESP32) reads these parameters when it connects.
"""

import copy
import json
import os
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import arm_config, encoders


def _descriptor(name: str) -> ParameterDescriptor:
    leaf = name.rsplit(".", 1)[-1]
    return ParameterDescriptor(description=arm_config.DESCRIPTIONS.get(leaf, ""))


def default_file() -> Path:
    env = os.environ.get("ARM_CALIBRATION_FILE")
    if env:
        return Path(env)
    return Path(get_package_share_directory("arm_calibration")) / "config" / "arm.yaml"


class CalibrationNode(Node):
    def __init__(self):
        # Undeclared names are allowed so that a null of the file can get a value; _on_set refuses
        # every name the file cannot have.
        super().__init__("arm_calibration", allow_undeclared_parameters=True)
        self._file = Path(
            self.declare_parameter(
                "file",
                str(default_file()),
                ParameterDescriptor(description="Το αρχείο του βραχίονα (arm.yaml)", read_only=True),
            ).value
        )
        self._data = arm_config.load(self._file)
        self._values = arm_config.parameters(self._data)
        for name, value in self._values.items():
            self.declare_parameter(name, value, _descriptor(name))
        self.add_on_set_parameters_callback(self._on_set)
        self.create_service(Trigger, "~/save", self._on_save)
        self.create_service(Trigger, "~/reload", self._on_reload)
        self._encoder_reports = {}  # joint -> (stamp s, level, raw deg or None)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self.create_service(Trigger, "~/set_zero", self._on_set_zero)
        for joint in self._zero_joints():
            self.create_service(Trigger, f"~/set_zero/{joint}",
                                lambda _req, resp, j=joint: self._set_zeros([j], resp))
        self._logbook = self.create_publisher(String, "/logbook/event", 10)  # history (ADR-0020)
        self.get_logger().info(f"{len(self._values)} παράμετροι από {self._file}")

    def _zero_joints(self):
        """Joints with an encoder: each can be zeroed (~/set_zero/<joint>)."""
        return [name for name, joint in self._data["joints"].items() if joint.get("encoder")]

    def _on_set(self, params):
        if all(p.name == "file" for p in params):
            return SetParametersResult(successful=True)
        candidate = copy.deepcopy(self._data)
        try:
            for p in params:
                if p.name != "file":
                    arm_config.set_value(candidate, p.name, p.value)
        except (KeyError, TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        errors = arm_config.validate(candidate)
        if errors:
            return SetParametersResult(successful=False, reason="; ".join(errors))
        self._data = candidate
        self._values = arm_config.parameters(candidate)
        return SetParametersResult(successful=True)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if not status.name.startswith("encoder/"):
                continue
            level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)
            values = {kv.key: kv.value for kv in status.values}
            try:
                raw = float(values[encoders.RAW_KEY])
            except (KeyError, ValueError):
                raw = None
            self._encoder_reports[status.name.split("/", 1)[1]] = (self._now(), level, raw)

    def _on_set_zero(self, _request, response):
        return self._set_zeros(self._zero_joints(), response)

    def _set_zeros(self, joints, response):
        """Zeros of these joints from their latest encoder reports: all of them or none."""
        zeros, problems = encoders.zeros_from_reports(joints, self._encoder_reports, self._now())
        if problems:
            response.success = False
            response.message = "Δεν άλλαξε κανένα μηδέν: " + "; ".join(problems)
            return response
        result = self.set_parameters_atomically(
            [rclpy.parameter.Parameter(f"joints.{j}.zero_deg", value=float(z)) for j, z in zeros.items()]
        )
        response.success = result.successful
        if result.successful:
            for joint, zero in zeros.items():
                event = {"kind": "calibration", "joint": joint, "zero_deg": zero, "reference_deg": 0.0,
                         "invert": self._values[f"joints.{joint}.invert"]}
                self._logbook.publish(String(data=json.dumps(event)))
        response.message = (
            "Νέα μηδενικά: " + ", ".join(f"{j} {z:.2f}°" for j, z in zeros.items())
            + ". Πάτα «Αποθήκευση στο αρχείο» για να μείνουν."
            if result.successful
            else result.reason
        )
        return response

    def _on_save(self, _request, response):
        try:
            arm_config.save(self._data, self._file.resolve())  # --symlink-install: the repo file
            response.success, response.message = True, f"Αποθηκεύτηκε στο {self._file.resolve()}"
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _on_reload(self, _request, response):
        try:
            values = arm_config.parameters(arm_config.load(self._file))
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
            return response
        gone = set(self._values) - set(values)
        if gone:
            response.success = False
            response.message = f"Τιμές που έγιναν null στο αρχείο ({', '.join(sorted(gone))}): απαιτείται επανεκκίνηση"
            return response
        result = self.set_parameters_atomically([rclpy.parameter.Parameter(n, value=v) for n, v in values.items()])
        response.success, response.message = result.successful, result.reason or "Επαναφορτώθηκε από το αρχείο"
        return response


def main():
    rclpy.init()
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
