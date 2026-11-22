"""Το node ROS 2 του Pi: συνδέει τον Driver (driver.py) με τα topics (ADR-0007, ADR-0021).

    /<segment>/joint_states    (δημοσιεύει)  θέσεις των αρθρώσεων, σε ακτίνια
    /<segment>/joint_commands  (ακούει)      στόχοι θέσης, σε ακτίνια· χωρίς νέα εντολή για
                                             watchdog_s τα μοτέρ σταματούν
    /diagnostics               (δημοσιεύει)  encoder/<άρθρωση>, servo/<servo>, pi_driver/<segment>, force/…, 1 Hz
    /parameter_events          (ακούει)      αλλαγές βαθμονόμησης του /arm_calibration (PC)
    ~/identify/request         (ακούει)      αναγνώριση (ADR-0023): JSON {'id', 'op', ...}
    ~/identify/reply           (δημοσιεύει)  οι απαντήσεις, JSON (identify.Session)

Αναγνώριση: με το πρώτο αίτημα ο κανονικός έλεγχος σταματά (φρένο) και μόνο η αναγνώριση γράφει στα
servo· πίσω με {'op': 'exit'} ή μόνο του αν δεν έρθει αίτημα για 5 s.

Ρύθμιση: το ΕΝΑ αρχείο arm.yaml (ADR-0023), αντίγραφο του repo στο Pi. Μόλις απαντήσει το
/arm_calibration του PC, παίρνει τις τιμές ΤΟΥ (ίσως νεότερες από το αρχείο του Pi) και μετά κάθε
αλλαγή του ζωντανά. Εφαρμόζονται μόνο όσες τιμές διαφέρουν από αυτές που ισχύουν· όσες αλλάζουν τη
δομή (κανάλια, servo/encoder μιας άρθρωσης, enabled) ισχύουν μετά από επανεκκίνηση.

Παράμετροι: fake (χωρίς υλικό: προσομοίωση), rate_hz, watchdog_s, config (arm.yaml· κενό → του
πακέτου arm_calibration).
    ros2 run arm_pi_driver driver --ros-args -p fake:=true
"""

import json
import math
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from arm_calibration import arm_config
from arm_pi_driver import arm_file
from arm_pi_driver.control import joint_configs
from arm_pi_driver.driver import Driver, ERROR, IDLE, USABLE, WARNING
from arm_pi_driver.hardware import ADS1115, AS5600, PCA9685, TCA9548A, ZD10
from arm_pi_driver.hardware.fake import FakeEncoder, FakeForceSensor, SimulatedArm
from arm_pi_driver.identify import DEVICES, Identifier, Session, TCA_CHANNELS
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

LEVELS = {IDLE: DiagnosticStatus.OK, WARNING: DiagnosticStatus.WARN, ERROR: DiagnosticStatus.ERROR}
CALIBRATION_NODE = '/arm_calibration'       # το node του PC που κρατά το arm.yaml
CALIBRATION_PREFIXES = ('servos.', 'encoders.', 'force_sensors.', 'joints.', 'controller.')
SYNC_PERIOD_S = 2.0                         # πόσο συχνά ξαναρωτά το PC μέχρι να απαντήσει
SYNC_TIMEOUT_S = 6.0                        # χωρίς απάντηση τόσο: χάθηκε (DDS), ξαναρωτά


def arm_file_path(path: str) -> str:
    """Το arm.yaml· κενό → αυτό που εγκατέστησε το colcon από το repo (arm_calibration)."""
    if path:
        return path
    return str(Path(get_package_share_directory('arm_calibration')) / 'config' / 'arm.yaml')


def parameter_value(value):
    """Η τιμή ενός rcl_interfaces/ParameterValue (None για τύπο που δεν μας αφορά)."""
    kinds = {
        ParameterType.PARAMETER_DOUBLE: lambda: value.double_value,
        ParameterType.PARAMETER_BOOL: lambda: value.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: value.integer_value,
        ParameterType.PARAMETER_STRING: lambda: value.string_value,
        ParameterType.PARAMETER_DOUBLE_ARRAY: lambda: list(value.double_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: lambda: list(value.integer_array_value),
        ParameterType.PARAMETER_STRING_ARRAY: lambda: list(value.string_array_value),
    }
    return kinds[value.type]() if value.type in kinds else None


def real_hardware(config: dict):
    """Το αληθινό υλικό στο I²C του Pi: (encoders, pwm, force_sensors, mux).

    Πριν από αυτό ο κόμβος κρατά την κλειδαριά του I²C (hardware/i2c_lock.py).
    """
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
    return encoders, pwm, force_sensors, mux


def real_identifier(config: dict, data: dict, pwm, mux) -> Identifier:
    """Αναγνώριση στο αληθινό I²C: ένας AS5600 σε ΚΑΘΕ κανάλι (ίδιος mux με τον driver)."""
    encoders = {ch: AS5600(mux.bus, mux, ch) for ch in TCA_CHANNELS}
    return Identifier(encoders, pwm, lambda ch: mux.probe(ch, DEVICES), config['pca'],
                      data['controller']['bench'])


def fake_identifier(config: dict, data: dict, sim: SimulatedArm) -> Identifier:
    """Αναγνώριση στην προσομοίωση: encoders όπου έχει η προσομοίωση, αλλού «δεν απαντά»."""
    by_channel = {joint['mux_channel']: sim.encoders[name] for name, joint in sim.joints.items()}
    missing = FakeEncoder(math.nan, 'NO_RESPONSE')
    encoders = {ch: by_channel.get(ch, missing) for ch in TCA_CHANNELS}

    def probe(channel):
        found = [0x40] if channel == config['pca']['mux_channel'] else []
        return found + ([0x36] if channel in by_channel else [])

    return Identifier(encoders, sim.pwm, probe, config['pca'], data['controller']['bench'])


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
        path = arm_file_path(self.declare_parameter('config', '').value)
        self.arm = arm_file.load(path)                              # τα δεδομένα του arm.yaml
        self.values = arm_config.parameters(self.arm)               # οι τιμές που ισχύουν τώρα
        config, disabled = arm_file.driver_config(self.arm)
        self.segment = config['segment']

        if fake:
            self.sim = SimulatedArm(config)
            encoders, pwm, sensors = self.sim.encoders, self.sim.pwm, fake_force_sensors(config)
            identifier = fake_identifier(config, self.arm, self.sim)
        else:
            self.sim = None
            self.i2c_lock = self.wait_for_i2c()
            encoders, pwm, sensors, mux = real_hardware(config)
            identifier = real_identifier(config, self.arm, pwm, mux)
        # self.arm είναι ζωντανό: κανάλια που δίνει τώρα το UI ισχύουν και στην αναγνώριση
        self.session = Session(identifier, self.arm)
        joints = joint_configs(config)
        self.channels = {name: joint['mux_channel'] for name, joint in joints.items()}
        self.driver = Driver(config, encoders, pwm, watchdog_s, force_sensors=sensors)
        self.report = None

        self.states = self.create_publisher(JointState, f'/{self.segment}/joint_states', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(JointState, f'/{self.segment}/joint_commands',
                                 self.on_command, 10)
        self.create_subscription(ParameterEvent, '/parameter_events', self.on_parameters, 10)
        self.identify_reply = self.create_publisher(String, '~/identify/reply', 10)
        self.create_subscription(String, '~/identify/request', self.on_identify, 10)
        self.list_client = self.create_client(ListParameters,
                                              f'{CALIBRATION_NODE}/list_parameters')
        self.get_client = self.create_client(GetParameters, f'{CALIBRATION_NODE}/get_parameters')
        self.synced, self.syncing, self.sync_started = False, False, 0.0
        self.sync_timer = self.create_timer(SYNC_PERIOD_S, self.sync_calibration)
        self.period = 1.0 / rate_hz
        self.create_timer(self.period, self.tick)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f"{'ΠΡΟΣΟΜΟΙΩΣΗ (fake)' if fake else 'υλικό I²C'} · {rate_hz:g} Hz · "
            f'/{self.segment}/joint_states, /{self.segment}/joint_commands · '
            f'ομάδες {", ".join(config["groups"])} · αρθρώσεις {", ".join(joints)} · '
            f'ανενεργές: {", ".join(disabled) or "καμία"} · {path}')

    def wait_for_i2c(self) -> int:
        """Η κλειδαριά του I²C· όσο την κρατά το teleop.py ή το wrist_bench.py, περιμένει."""
        from arm_pi_driver.hardware.i2c_lock import hold, I2CBusy
        path = os.environ.get('ARM_I2C_LOCK', '/root/master-s-thesis/.local/i2c.lock')
        said = False
        while True:
            try:
                return hold(path)
            except I2CBusy:
                if not said:
                    self.get_logger().error('το I²C το χρησιμοποιεί άλλο πρόγραμμα (teleop.py ή '
                                            'wrist_bench.py;)· περιμένω να κλείσει')
                    said = True
                time.sleep(1.0)

    def on_identify(self, msg: String) -> None:
        """Ένα αίτημα αναγνώρισης (JSON)· η πρώτη φορά σταματά τον κανονικό έλεγχο."""
        try:
            request = json.loads(msg.data)
        except ValueError:
            request = {'op': None}
        if not isinstance(request, dict):
            request = {'op': None}
        was_active = self.session.active
        if not was_active and request.get('op') != 'exit':
            self.driver.last_command = None               # ο driver σταματά: φρένο, χωρίς στόχους
            for controller in self.driver.controllers:
                controller.clear_targets()
            self.driver.tick(time.monotonic())
            self.get_logger().warning('αναγνώριση: ο κανονικός έλεγχος σταμάτησε')
        self.send_identify(self.session.handle(request, time.monotonic()))
        if was_active and not self.session.active:
            self.leave_identify()

    def send_identify(self, reply: dict) -> None:
        """Μία απάντηση της αναγνώρισης στο ~/identify/reply."""
        self.identify_reply.publish(String(data=json.dumps(reply, ensure_ascii=False)))

    def leave_identify(self) -> None:
        """Πίσω στον κανονικό έλεγχο: ο driver ξαναγράφει τους παλμούς του (φρένο)."""
        self.driver.written = {}
        self.get_logger().info('αναγνώριση: τέλος, κανονικός έλεγχος')

    def on_command(self, msg: JointState) -> None:
        """Στόχοι θέσης: ακτίνια από το ROS 2 → μοίρες για τον Driver (όχι σε αναγνώριση)."""
        if self.session.active:
            return
        targets = {name: math.degrees(p) for name, p in zip(msg.name, msg.position)}
        self.driver.command(targets, time.monotonic())

    def on_parameters(self, event: ParameterEvent) -> None:
        """Αλλαγή βαθμονόμησης στο PC (π.χ. «Μηδέν = τρέχουσα θέση») → αμέσως στον Driver."""
        if event.node != CALIBRATION_NODE:
            return
        for parameter in list(event.new_parameters) + list(event.changed_parameters):
            self.calibrate(parameter.name, parameter_value(parameter.value))

    def calibrate(self, name: str, value) -> bool:
        """Μία τιμή του arm.yaml· εφαρμόζεται μόνο αν διαφέρει από αυτή που ισχύει."""
        if value is None or self.values.get(name) == value:
            return False
        change = arm_file.parameter_change(self.arm, name, value)
        if change is None:
            return False
        self.values[name] = arm_config.coerce(name, value)
        return self.driver.apply_change(change)

    def sync_calibration(self) -> None:
        """Ζητά όλες τις τιμές από το /arm_calibration του PC, μέχρι να απαντήσει μία φορά."""
        if self.syncing and time.monotonic() - self.sync_started > SYNC_TIMEOUT_S:
            self.syncing = False        # η απάντηση χάθηκε («failed to send response» στο PC)
        if self.synced or self.syncing or not self.list_client.service_is_ready():
            return
        self.syncing, self.sync_started = True, time.monotonic()
        request = ListParameters.Request()
        request.prefixes = [p.rstrip('.') for p in CALIBRATION_PREFIXES]
        self.list_client.call_async(request).add_done_callback(self._on_names)

    def _on_names(self, future) -> None:
        if future.result() is None:
            self.syncing = False
            return
        names = [n for n in future.result().result.names if n.startswith(CALIBRATION_PREFIXES)]
        request = GetParameters.Request()
        request.names = names
        self.get_client.call_async(request).add_done_callback(
            lambda f: self._on_values(names, f))

    def _on_values(self, names: list, future) -> None:
        self.syncing = False
        if future.result() is None:
            return
        changed = sum(self.calibrate(name, parameter_value(value))
                      for name, value in zip(names, future.result().values))
        self.synced = True
        self.get_logger().info(f'βαθμονόμηση από το {CALIBRATION_NODE} του PC: {len(names)} '
                               f'τιμές, {changed} διαφορετικές από το αρχείο του Pi')

    def log(self, level: str, text: str) -> None:
        """Ένα γεγονός του Driver στο log του ROS 2.

        Μία γραμμή κώδικα ανά επίπεδο: ο logger του rclpy θυμάται το επίπεδο ΑΝΑ ΣΗΜΕΙΟ κλήσης
        και σκάει («Logger severity cannot be changed between calls») αν η ίδια γραμμή γράψει
        μία φορά info και μετά warning.
        """
        logger = self.get_logger()
        if level == ERROR:
            logger.error(text)
        elif level == WARNING:
            logger.warning(text)
        else:
            logger.info(text)

    def tick(self) -> None:
        """Ένας κύκλος του Driver (ή της αναγνώρισης) και δημοσίευση των θέσεων."""
        if self.sim:
            self.sim.step(self.period)
        if self.session.active:
            for reply in self.session.tick(time.monotonic()):
                self.send_identify(reply)
            if not self.session.active:
                self.leave_identify()
            return
        self.report = self.driver.tick(time.monotonic())
        for level, text in self.report.events:
            self.log(level, text)
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
        outputs.message = 'αναγνώριση' if self.session.active else self.report.reason
        array.status.append(outputs)
        for name, level, message, values in self.report.extra:     # π.χ. αισθητήρες αρπάγης
            status = DiagnosticStatus()
            status.name, status.level, status.message = name, LEVELS[level], message
            if name.startswith('servo/'):
                status.hardware_id = f"PCA9685 κανάλι {values['channel']}"
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
