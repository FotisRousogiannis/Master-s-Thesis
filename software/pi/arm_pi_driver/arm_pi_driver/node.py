"""Το node ROS 2 του Pi: συνδέει τον Driver (driver.py) με τα topics (ADR-0007, ADR-0021).

    /<segment>/joint_states    (δημοσιεύει)  θέσεις των αρθρώσεων, σε ακτίνια
    /<segment>/joint_commands  (ακούει)      στόχοι θέσης, σε ακτίνια· χωρίς νέα εντολή για
                                             watchdog_s τα μοτέρ σταματούν
    /diagnostics               (δημοσιεύει)  encoder/<άρθρωση>, pi_driver/<segment>, force/…, 1 Hz
    /parameter_events          (ακούει)      αλλαγές βαθμονόμησης του /arm_calibration (PC)

Παράμετροι: fake (χωρίς υλικό: προσομοίωση), rate_hz, watchdog_s, config (αρχείο YAML),
calibration (calibration.yaml· κενό → του πακέτου arm_calibration).
    ros2 run arm_pi_driver driver --ros-args -p fake:=true
"""

import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from arm_pi_driver.calibration import apply_calibration, load_calibration, parameter_change
from arm_pi_driver.control import joint_configs
from arm_pi_driver.driver import Driver, ERROR, IDLE, USABLE, WARNING
from arm_pi_driver.hardware import ADS1115, AS5600, PCA9685, TCA9548A, ZD10
from arm_pi_driver.hardware.fake import FakeForceSensor, SimulatedArm
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterEvent, ParameterType
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

LEVELS = {IDLE: DiagnosticStatus.OK, WARNING: DiagnosticStatus.WARN, ERROR: DiagnosticStatus.ERROR}
CALIBRATION_NODE = '/arm_calibration'       # το node του PC που κρατά το calibration.yaml


def load_config(path: str) -> dict:
    """Το YAML της ρύθμισης· κενό → το config/base.yaml του πακέτου."""
    if not path:
        path = str(Path(get_package_share_directory('arm_pi_driver')) / 'config' / 'base.yaml')
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def calibration_file(path: str) -> str:
    """Το calibration.yaml· κενό → αυτό που εγκατέστησε το colcon από το repo (arm_calibration)."""
    if path:
        return path
    share = Path(get_package_share_directory('arm_calibration'))
    return str(share / 'config' / 'calibration.yaml')


def parameter_value(value):
    """Η τιμή ενός rcl_interfaces/ParameterValue (μόνο όσοι τύποι μας αφορούν)."""
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    return None


def real_hardware(config: dict):
    """Το αληθινό υλικό στο I²C του Pi: (encoders, pwm, force_sensors)."""
    from smbus2 import SMBus     # μόνο εδώ: με fake:=true δεν χρειάζεται καθόλου

    bus = SMBus(config['i2c_bus'])
    mux = TCA9548A(bus, config['tca_address'])
    encoders = {name: AS5600(bus, mux, joint['mux_channel'])
                for name, joint in joint_configs(config).items()}
    pca = config['pca']
    pwm = PCA9685(bus, mux, pca['mux_channel'], pca['address'], pca['freq_hz'], pca['osc_hz'],
                  pca['pulse_min_us'], pca['pulse_max_us'])
    force_sensors = {}
    for group in config['groups'].values():
        if 'force_sensors' in group:
            adc_cfg = group['adc']
            adc = ADS1115(bus, mux, adc_cfg['mux_channel'], adc_cfg.get('address', 0x48),
                          adc_cfg.get('full_scale_v', 4.096), adc_cfg.get('rate_sps', 860))
            for name, sensor in group['force_sensors'].items():
                force_sensors[name] = ZD10(adc, sensor['input'])
    return encoders, pwm, force_sensors


def fake_force_sensors(config: dict) -> dict:
    """Ψεύτικοι αισθητήρες πίεσης (χωρίς πίεση) για την προσομοίωση."""
    return {name: FakeForceSensor() for group in config['groups'].values()
            for name in group.get('force_sensors', {})}


class PiDriverNode(Node):
    """Το arm_pi_driver: όσα κάνει ο Driver, πάνω από ROS 2."""

    def __init__(self):
        super().__init__('arm_pi_driver')
        fake = self.declare_parameter('fake', False).value
        rate_hz = self.declare_parameter('rate_hz', 50.0).value
        watchdog_s = self.declare_parameter('watchdog_s', 0.2).value
        config = load_config(self.declare_parameter('config', '').value)
        calibration = calibration_file(self.declare_parameter('calibration', '').value)
        apply_calibration(config, load_calibration(calibration))    # μηδέν, φορά, άνοιγμα αρπάγης
        self.segment = config['segment']

        if fake:
            self.sim = SimulatedArm(config)
            encoders, pwm, sensors = self.sim.encoders, self.sim.pwm, fake_force_sensors(config)
        else:
            self.sim = None
            encoders, pwm, sensors = real_hardware(config)
        joints = joint_configs(config)
        self.channels = {name: joint['mux_channel'] for name, joint in joints.items()}
        self.driver = Driver(config, encoders, pwm, watchdog_s, force_sensors=sensors)
        self.report = None

        self.states = self.create_publisher(JointState, f'/{self.segment}/joint_states', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(JointState, f'/{self.segment}/joint_commands',
                                 self.on_command, 10)
        self.create_subscription(ParameterEvent, '/parameter_events', self.on_parameters, 10)
        self.period = 1.0 / rate_hz
        self.create_timer(self.period, self.tick)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f"{'ΠΡΟΣΟΜΟΙΩΣΗ (fake)' if fake else 'υλικό I²C'} · {rate_hz:g} Hz · "
            f'/{self.segment}/joint_states, /{self.segment}/joint_commands · '
            f'ομάδες {", ".join(config["groups"])} · αρθρώσεις {", ".join(joints)} · '
            f'βαθμονόμηση {calibration}')

    def on_command(self, msg: JointState) -> None:
        """Στόχοι θέσης: ακτίνια από το ROS 2 → μοίρες για τον Driver."""
        targets = {name: math.degrees(p) for name, p in zip(msg.name, msg.position)}
        self.driver.command(targets, time.monotonic())

    def on_parameters(self, event: ParameterEvent) -> None:
        """Αλλαγή βαθμονόμησης στο PC (π.χ. «Μηδέν = τρέχουσα θέση») → αμέσως στον Driver."""
        if event.node != CALIBRATION_NODE:
            return
        for parameter in list(event.new_parameters) + list(event.changed_parameters):
            change = parameter_change(parameter.name, parameter_value(parameter.value))
            if change:
                self.driver.update_joint(*change)

    def tick(self) -> None:
        """Ένας κύκλος του Driver και δημοσίευση των θέσεων."""
        if self.sim:
            self.sim.step(self.period)
        self.report = self.driver.tick(time.monotonic())
        for level, text in self.report.events:
            getattr(self.get_logger(), level)(text)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, joint in self.report.joints.items():
            if joint.position_deg is not None:
                msg.name.append(name)
                msg.position.append(math.radians(joint.position_deg))
        self.states.publish(msg)

    def publish_diagnostics(self) -> None:
        """Μία κατάσταση ανά encoder και μία για τις εξόδους (ADR-0018)."""
        if self.report is None:
            return
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        for name, joint in self.report.joints.items():
            status = DiagnosticStatus()
            status.name = f'encoder/{name}'
            status.hardware_id = f'AS5600 · TCA9548A κανάλι {self.channels[name]}'
            if joint.status == 'OK':
                status.level = DiagnosticStatus.OK
            elif joint.status in USABLE:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR
            status.message = joint.status
            if not math.isnan(joint.raw_deg):
                status.values.append(KeyValue(key='raw_deg', value=f'{joint.raw_deg:.2f}'))
            if joint.position_deg is not None:
                status.values.append(KeyValue(key='position_deg',
                                              value=f'{joint.position_deg:.2f}'))
            array.status.append(status)
        outputs = DiagnosticStatus()
        outputs.name = f'pi_driver/{self.segment}'
        outputs.hardware_id = 'PCA9685'
        outputs.level = LEVELS[self.report.level]
        outputs.message = self.report.reason
        array.status.append(outputs)
        for name, level, message, values in self.report.extra:     # π.χ. αισθητήρες αρπάγης
            status = DiagnosticStatus()
            status.name, status.level, status.message = name, LEVELS[level], message
            status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
            array.status.append(status)
        self.diagnostics.publish(array)


def main(args=None):
    """Σημείο εκκίνησης: ros2 run arm_pi_driver driver."""
    rclpy.init(args=args)
    node = PiDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.stop()      # ό,τι κι αν γίνει: κανένας παλμός στα servo
        node.destroy_node()
        rclpy.try_shutdown()
