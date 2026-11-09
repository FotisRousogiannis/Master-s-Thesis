"""arm_logbook node: the only program that opens the logbook database (ADR-0020).

    ros2 run arm_logbook logbook_node
    ros2 service call /arm_logbook/last_position std_srvs/srv/Trigger

- /joint_states: when the arm stops at a new position, the position is saved. Only with
  track_position:=true (the real arm); bringup sets it, so simulation never overwrites it.
- /logbook/event (std_msgs/String, JSON): calibrations, recordings, faults from other nodes and Arm Studio.
- ~/last_position (Trigger): the saved positions as JSON, for the start-up of the hardware (ADR-0019).

String + JSON is temporary: it becomes typed messages when arm_interfaces exists (ADR-0016).
"""

import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .store import Logbook, StillDetector

EVENT_TOPIC = "/logbook/event"


def data_dir() -> Path:
    """<repo>/.local in Docker (compose.yaml: ARM_DATA_DIR): on the host disk, outside git."""
    return Path(os.environ.get("ARM_DATA_DIR", Path.home() / ".local" / "share" / "arm_studio"))


class LogbookNode(Node):
    def __init__(self):
        super().__init__("arm_logbook")
        path = self.declare_parameter("database", str(data_dir() / "arm_logbook.db")).value
        self._book = Logbook(path)
        self._still = StillDetector()
        if self.declare_parameter("track_position", False).value:
            self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(String, EVENT_TOPIC, self._on_event, 10)
        self.create_service(Trigger, "~/last_position", self._on_last_position)
        self.get_logger().info(f"Ημερολόγιο: {self._book.path}")

    def _on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        stopped = self._still.update(positions, time.monotonic())
        if stopped:
            self._book.save_positions(stopped, time.time())

    def _on_event(self, msg):
        try:
            self._book.handle_event(json.loads(msg.data), time.time())
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f"Άκυρο γεγονός στο {EVENT_TOPIC}: {exc}: {msg.data}")

    def _on_last_position(self, _request, response):
        saved = self._book.last_positions()
        response.success = bool(saved)
        response.message = json.dumps(saved) if saved else "Δεν υπάρχει αποθηκευμένη θέση"
        return response

    def destroy_node(self):
        self._book.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LogbookNode()
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
