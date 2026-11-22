import math

from arm_pi_driver.hardware import AS5600, Encoder, NO_RESPONSE, TCA9548A
from arm_pi_driver.hardware.as5600 import AS5600_ADDRESS, RAW_ANGLE, STATUS
from arm_pi_driver.hardware.fake import FakeBus, FakeEncoder
import pytest


def encoder_registers(raw: int, status: int) -> dict:
    """Οι καταχωρητές ενός AS5600 που μετρά `raw` (0–4095) με αυτό το byte STATUS."""
    return {
        (AS5600_ADDRESS, RAW_ANGLE): raw >> 8,
        (AS5600_ADDRESS, RAW_ANGLE + 1): raw & 0xFF,
        (AS5600_ADDRESS, STATUS): status,
    }


def test_mux_opens_a_channel_once():
    bus = FakeBus()
    mux = TCA9548A(bus)
    mux.select(2)
    mux.select(2)
    mux.select(5)
    assert bus.writes == [(0x70, 0b0000_0100), (0x70, 0b0010_0000)]


def test_mux_rejects_a_channel_that_does_not_exist():
    with pytest.raises(ValueError):
        TCA9548A(FakeBus()).select(8)


def test_as5600_angle_from_two_bytes():
    bus = FakeBus(encoder_registers(2755, 0x20))  # 0x0A, 0xC3: το παράδειγμα με το χέρι
    angle, status = AS5600(bus, TCA9548A(bus), 2).read()
    assert angle == pytest.approx(242.1387, abs=1e-3)
    assert status == 'OK'
    assert bus.writes == [(0x70, 0b0000_0100)]  # πρώτα άνοιξε το κανάλι του


def test_as5600_ignores_the_unused_high_bits():
    bus = FakeBus(encoder_registers(4095, 0x20))
    bus.registers[(AS5600_ADDRESS, RAW_ANGLE)] = 0xFF  # τα 4 πάνω bits δεν ανήκουν στη γωνία
    angle, _ = AS5600(bus, TCA9548A(bus), 0).read()
    assert angle == pytest.approx(4095 * 360 / 4096)


@pytest.mark.parametrize('status, text', [
    (0x00, 'NO_MAGNET'), (0x20, 'OK'), (0x30, 'TOO_WEAK'), (0x28, 'TOO_STRONG'),
])
def test_magnet_status(status, text):
    assert AS5600.magnet_status(status) == text


def test_encoders_share_one_contract():
    bus = FakeBus(encoder_registers(1024, 0x20))
    for encoder in (AS5600(bus, TCA9548A(bus), 1), FakeEncoder(90.0)):
        assert isinstance(encoder, Encoder)
        assert encoder.read() == (pytest.approx(90.0), 'OK')
    with pytest.raises(TypeError):
        Encoder()


def test_nothing_connected_is_reported_not_raised():
    bus = FakeBus(encoder_registers(1024, 0x20))
    bus.absent = {0x70}                      # ούτε ο mux: όπως στο Pi χωρίς τίποτα συνδεδεμένο
    angle, status = AS5600(bus, TCA9548A(bus), 2).read()
    assert math.isnan(angle) and status == NO_RESPONSE


def test_encoder_unplugged_then_plugged_back():
    bus = FakeBus(encoder_registers(1024, 0x20))
    mux = TCA9548A(bus)
    encoder = AS5600(bus, mux, 2)
    bus.absent = {AS5600_ADDRESS}           # ο mux απαντά, ο encoder όχι
    assert encoder.read()[1] == NO_RESPONSE
    bus.absent = set()                       # ξανασυνδέθηκε: χωρίς επανεκκίνηση του προγράμματος
    assert encoder.read() == (pytest.approx(90.0), 'OK')


def test_mux_resends_the_channel_after_a_failure():
    bus = FakeBus()
    mux = TCA9548A(bus)
    bus.absent = {0x70}
    with pytest.raises(OSError):
        mux.select(2)
    bus.absent = set()
    mux.select(2)                            # πρέπει να ξαναστείλει, δεν ξέρει τι είναι ανοιχτό
    assert bus.writes == [(0x70, 0b0000_0100)]


def test_mux_probe_finds_only_what_answers():
    bus = FakeBus()
    bus.absent = {0x48}
    mux = TCA9548A(bus)
    assert mux.probe(3, [0x40, 0x36, 0x48]) == [0x40, 0x36]
    assert bus.writes[0] == (0x70, 0b0000_1000)


def test_one_program_at_a_time_on_the_i2c(tmp_path):
    from arm_pi_driver.hardware.i2c_lock import hold, I2CBusy, release
    lock = tmp_path / '.local' / 'i2c.lock'
    first = hold(lock)
    with pytest.raises(I2CBusy):
        hold(lock)                      # e.g. teleop.py while the driver runs
    release(first)
    release(hold(lock))                 # free again
