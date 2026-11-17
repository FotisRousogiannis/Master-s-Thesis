import copy
import math

from arm_pi_driver.calibration import gripper_limits
from arm_pi_driver.control import build_controllers, GripperController
from arm_pi_driver.driver import Driver
from arm_pi_driver.hardware import ADS1115, ForceSensor, NO_RESPONSE, TCA9548A, ZD10
from arm_pi_driver.hardware.fake import FakeBus, FakeForceSensor, SimulatedArm
from arm_pi_driver.hardware.zd10 import NO_SENSOR
import pytest

DT = 0.02
GRIPPER = {
    'controller': 'gripper',
    'joints': {'gripper': {'mux_channel': 7, 'zero_deg': 0.0, 'sign': 1,
                           'law': {'type': 'pid', 'kp': 0.02}, **gripper_limits(60.0)}},
    'motors': {'a': {'channel': 8, 'neutral_us': 1600, 'deadband_edges_us': [1560, 1640],
                     'mix': {'gripper': 1}}},
    'contact': 1.3,
    'adc': {'mux_channel': 7},
    'force_sensors': {'finger_1': {'input': 0}, 'finger_2': {'input': 1}},
}


class AdsBus(FakeBus):
    """FakeBus με τους καταχωρητές 16 bit του ADS1115: μετατροπή = τάση της επιλεγμένης εισόδου."""

    def __init__(self, volts):
        super().__init__()
        self.volts, self.config_word = volts, 0

    def write_i2c_block_data(self, address, register, data):
        super().write_i2c_block_data(address, register, data)
        if address == 0x48 and register == 0x01:
            self.config_word = (data[0] << 8) | data[1]

    def read_i2c_block_data(self, address, register, length):
        self.calls.append(('read_i2c_block_data', address, register, length))
        self._ack(address)
        if register == 0x01:
            return [0x80 | (self.config_word >> 8) & 0x7F, self.config_word & 0xFF]   # OS = 1
        analog_input = ((self.config_word >> 12) & 0b111) - 0b100
        raw = round(self.volts[analog_input] / 4.096 * 32768) & 0xFFFF
        return [raw >> 8, raw & 0xFF]


@pytest.fixture
def arm(config):
    config = copy.deepcopy(config)
    config['groups']['gripper'] = copy.deepcopy(GRIPPER)
    return config


def fingers(value=0.94):
    return {'finger_1': FakeForceSensor(value), 'finger_2': FakeForceSensor(value)}


def run(sim, driver, seconds, targets, start=0.0):
    now, report = start, None
    for _ in range(round(seconds / DT)):
        driver.command(targets, now)
        sim.step(DT)
        report = driver.tick(now)
        now += DT
    return report, now


def test_ads1115_reads_the_two_fingers():
    bus = AdsBus({0: 0.94, 1: 2.5})
    adc = ADS1115(bus, TCA9548A(bus), mux_channel=7, rate_sps=860)
    assert adc.read_volts(0) == pytest.approx(0.94, abs=1e-3)
    assert adc.read_volts(1) == pytest.approx(2.5, abs=1e-3)
    assert bus.writes[0] == (0x70, 1 << 7)                    # πρώτα το κανάλι του mux


def test_zd10_voltage_status_and_resistance():
    bus = AdsBus({0: 0.94, 1: 0.01})
    adc = ADS1115(bus, TCA9548A(bus), 7)
    pressed, unplugged = ZD10(adc, 0), ZD10(adc, 1)
    assert isinstance(pressed, ForceSensor)
    assert pressed.read() == (pytest.approx(0.94, abs=1e-3), 'OK')
    assert pressed.resistance_ohm(0.94) == pytest.approx(17_000, rel=0.01)   # χωρίς πίεση
    assert unplugged.read()[1] == NO_SENSOR                  # ~0 V: ο αισθητήρας λείπει
    bus.absent = {0x48}
    value, status = pressed.read()
    assert math.isnan(value) and status == NO_RESPONSE


def test_gripper_opens_and_closes_by_position(arm):
    sim = SimulatedArm(arm)
    driver = Driver(arm, sim.encoders, sim.pwm, force_sensors=fingers())
    assert any(isinstance(c, GripperController) for c in driver.controllers)
    run(sim, driver, 4.0, {'gripper': 40.0})
    assert sim.angles['gripper'] == pytest.approx(40.0, abs=1.5)
    run(sim, driver, 4.0, {'gripper': 0.0}, start=10.0)
    assert sim.angles['gripper'] == pytest.approx(0.0, abs=1.5)


def test_gripper_stops_closing_on_contact_but_can_open(arm):
    sim = SimulatedArm(arm)
    sensors = fingers()
    driver = Driver(arm, sim.encoders, sim.pwm, force_sensors=sensors)
    run(sim, driver, 4.0, {'gripper': 50.0})
    sensors['finger_2'].value = 1.8                          # το δάχτυλο 2 ακούμπησε
    before = sim.angles['gripper']
    run(sim, driver, 2.0, {'gripper': 0.0}, start=10.0)      # «κλείσε»
    assert sim.angles['gripper'] == pytest.approx(before, abs=0.5)
    [gripper] = [c for c in driver.controllers if isinstance(c, GripperController)]
    assert gripper.state == 'επαφή'
    run(sim, driver, 2.0, {'gripper': 60.0}, start=20.0)     # το άνοιγμα επιτρέπεται
    assert sim.angles['gripper'] > before + 5


def test_gripper_does_not_close_with_a_faulty_sensor(arm):
    sim = SimulatedArm(arm)
    sensors = fingers()
    driver = Driver(arm, sim.encoders, sim.pwm, force_sensors=sensors)
    run(sim, driver, 4.0, {'gripper': 50.0})
    sensors['finger_1'].status = NO_SENSOR                   # ξεκόλλησε: δεν θα «ένιωθε» επαφή
    report, _ = run(sim, driver, 2.0, {'gripper': 0.0}, start=10.0)
    assert sim.angles['gripper'] > 45
    names = [name for name, *_ in report.extra]
    assert 'force/finger_1' in names and 'gripper/gripper' in names


def test_uncalibrated_gripper_does_not_open(arm):
    arm['groups']['gripper']['joints']['gripper'].update(gripper_limits(0.0))   # open_deg = 0
    sim = SimulatedArm(arm)
    driver = Driver(arm, sim.encoders, sim.pwm, force_sensors=fingers())
    run(sim, driver, 2.0, {'gripper': 40.0})
    assert sim.angles['gripper'] == pytest.approx(0.0, abs=0.5)


def test_missing_force_sensors_are_refused(arm):
    with pytest.raises(ValueError):
        build_controllers(arm, {'finger_1': FakeForceSensor()})
