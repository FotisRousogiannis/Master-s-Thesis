"""arm_calibration node: serves the calibration file as ROS 2 parameters (ADR-0017).

    ros2 run arm_calibration calibration_node
    ros2 param set /arm_calibration servos.wrist_diff_a.neutral_us 1605
    ros2 service call /arm_calibration/save std_srvs/srv/Trigger
    ros2 service call /arm_calibration/set_zero std_srvs/srv/Trigger   # arm in its reference pose

Every change is validated against the whole calibration (ranges, pulse limits, unique
channels) before it is accepted. ~/save writes the file, ~/reload discards unsaved changes.
~/set_zero takes the encoder readings of the arm in its reference pose as the joint zeros
(ADR-0019): all joints or none, from the encoder/<joint> entries of /diagnostics.
The hardware layer (arm_hardware -> ESP32) reads these parameters when it connects.
"""

import os
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from std_srvs.srv import Trigger

from . import calibration, encoders


def _descriptor(name: str) -> ParameterDescriptor:
    f = calibration.field_for(name)
    d = ParameterDescriptor(description=f.description)
    if f.type is int and f.min is not None:
        d.integer_range = [IntegerRange(from_value=int(f.min), to_value=int(f.max), step=1)]
    elif f.type is float and f.min is not None:
        d.floating_point_range = [FloatingPointRange(from_value=float(f.min), to_value=float(f.max), step=0.0)]
    return d


def default_file() -> Path:
    env = os.environ.get("ARM_CALIBRATION_FILE")
    if env:
        return Path(env)
    return Path(get_package_share_directory("arm_calibration")) / "config" / "calibration.yaml"


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("arm_calibration")
        self._file = Path(
            self.declare_parameter(
                "file",
                str(default_file()),
                ParameterDescriptor(description="Αρχείο βαθμονόμησης", read_only=True),
            ).value
        )
        self._values = calibration.load(self._file)
        for name, value in self._values.items():
            self.declare_parameter(name, value, _descriptor(name))
        self.add_on_set_parameters_callback(self._on_set)
        self.create_service(Trigger, "~/save", self._on_save)
        self.create_service(Trigger, "~/reload", self._on_reload)
        self._encoder_reports = {}  # joint -> (stamp s, level, raw deg or None)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self.create_service(Trigger, "~/set_zero", self._on_set_zero)
        self.get_logger().info(f"{len(self._values)} παράμετροι από {self._file}")

    def _on_set(self, params):
        candidate = dict(self._values)
        try:
            for p in params:
                candidate[p.name] = calibration.coerce(p.name, p.value)
        except (KeyError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        errors = calibration.validate(candidate)
        if errors:
            return SetParametersResult(successful=False, reason="; ".join(errors))
        self._values = candidate
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
        joints = calibration.encoder_names(self._values)
        zeros, problems = encoders.zeros_from_reports(joints, self._encoder_reports, self._now())
        if problems:
            response.success = False
            response.message = "Δεν άλλαξε κανένα μηδέν: " + "; ".join(problems)
            return response
        result = self.set_parameters_atomically(
            [rclpy.parameter.Parameter(f"encoders.{j}.zero_deg", value=float(z)) for j, z in zeros.items()]
        )
        response.success = result.successful
        response.message = (
            "Νέα μηδενικά: " + ", ".join(f"{j} {z:.2f}°" for j, z in zeros.items())
            + ". Πάτα «Αποθήκευση στο αρχείο» για να μείνουν."
            if result.successful
            else result.reason
        )
        return response

    def _on_save(self, _request, response):
        try:
            calibration.save(self._file, self._values)
            response.success, response.message = True, f"Αποθηκεύτηκε στο {self._file.resolve()}"
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _on_reload(self, _request, response):
        try:
            values = calibration.load(self._file)
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
            return response
        unknown = set(values) - set(self._values)
        if unknown:
            response.success = False
            response.message = f"Νέες παράμετροι στο αρχείο ({', '.join(sorted(unknown))}): απαιτείται επανεκκίνηση"
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
