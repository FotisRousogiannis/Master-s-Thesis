from arm_pi_driver.hardware.fake import FakeEncoder, SimulatedArm
from arm_pi_driver.identify import DONE, Identifier, LOST, TIMEOUT, wrap
import pytest

DT = 0.02
BENCH = {'max_duration_s': 3.0, 'max_travel_deg': 30.0}
SERVO_1 = (0, 1630, [1420, 1840])          # (κανάλι PCA, νεκρό σημείο, νεκρή ζώνη) της βάσης
SERVO_2 = (1, 1605, [1510, 1700])


def make_bench(config):
    """Η προσομοίωση της βάσης, με encoders σε ΟΛΑ τα κανάλια (όσα δεν υπάρχουν: NO_RESPONSE)."""
    sim = SimulatedArm(config)
    by_channel = {joint['mux_channel']: sim.encoders[name] for name, joint in sim.joints.items()}
    missing = FakeEncoder(float('nan'), 'NO_RESPONSE')
    encoders = {ch: by_channel.get(ch, missing) for ch in range(8)}

    def probe(channel):
        found = [0x40] if channel == config['pca']['mux_channel'] else []
        return found + ([0x36] if channel in by_channel else [])

    return sim, Identifier(encoders, sim.pwm, probe, config['pca'], BENCH)


def jog(sim, ident, servos, duration_s=0.5, stop_deg=3.0):
    """Τρέχει την κίνηση ως το τέλος της (προσομοίωση + tick)."""
    now = 0.0
    ident.start_jog([(*servo, speed) for servo, speed in servos], duration_s, stop_deg, now)
    while True:
        sim.step(DT)
        now += DT
        result = ident.tick(now)
        if result is not None:
            return result


def test_wrap():
    assert wrap(359.0 - 1.0) == pytest.approx(-2.0) and wrap(1.0 - 359.0) == pytest.approx(2.0)


def test_scan_finds_the_bench(config):
    _, ident = make_bench(config)
    scan = ident.scan()
    assert scan[0]['devices'] == ['PCA9685'] and scan[0]['encoder'] is None
    assert scan[1]['devices'] == ['AS5600'] and scan[1]['encoder']['status'] == 'OK'
    assert scan[2]['encoder']['raw_deg'] == pytest.approx(267.891, abs=0.01)    # μηδέν joint_1
    assert scan[5] == {'devices': [], 'encoder': None}
    assert ident.read()[5] == {'raw_deg': None, 'status': 'NO_RESPONSE'}


def test_one_servo_of_a_differential_moves_both_encoders(config):
    sim, ident = make_bench(config)
    result = jog(sim, ident, [(SERVO_1, 0.4)])
    assert result.reason == DONE and result.channels == [0]
    assert set(result.moved) == {1, 2}                          # joint_2 (κανάλι 1), joint_1 (2)
    assert result.moved[2] > 0 and result.moved[1] < 0          # joint_2: invert στο arm.yaml


def test_signs_of_one_joint_move_only_its_encoder(config):
    """joint_1 = {servo_1: +1, servo_2: −1}: μόνο ο encoder του joint_1 (κανάλι 2) κινείται."""
    sim, ident = make_bench(config)
    result = jog(sim, ident, [(SERVO_1, 0.4), (SERVO_2, -0.4)])
    assert result.reason == DONE and list(result.moved) == [2] and result.moved[2] >= 3.0


def test_stops_at_the_first_change_and_brakes(config):
    sim, ident = make_bench(config)
    jog(sim, ident, [(SERVO_1, 0.4)], stop_deg=3.0)
    assert sim.pwm.pulses == {0: 1630}                          # φρένο στο νεκρό σημείο
    assert abs(sim.angles['joint_1']) < 6.0                     # σταμάτησε νωρίς
    ident.release()
    assert sim.pwm.pulses == {0: None}


def test_inside_the_deadband_nothing_moves_until_timeout(config):
    sim, ident = make_bench(config)
    result = jog(sim, ident, [((0, 1630, None), 0.1)], duration_s=0.3)   # 1630 + 87 µs < 1840
    assert result.reason == TIMEOUT and result.moved == {}
    assert result.duration_s == pytest.approx(0.3, abs=DT)


def test_a_lost_encoder_stops_at_once(config):
    sim, ident = make_bench(config)
    ident.start_jog([(0, 1630, None, 0.4)], 1.0, 3.0, 0.0)
    ident.encoders[1].status = 'NO_RESPONSE'
    assert ident.tick(DT).reason == LOST and sim.pwm.pulses[0] == 1630


UNSAFE = [(5.0, 3.0, 0.4), (0.5, 45.0, 0.4), (0.5, 3.0, 1.5), (0.0, 3.0, 0.4)]


@pytest.mark.parametrize('duration_s, stop_deg, speed', UNSAFE)
def test_unsafe_requests_are_refused(config, duration_s, stop_deg, speed):
    sim, ident = make_bench(config)
    with pytest.raises(ValueError):
        ident.start_jog([(0, 1630, None, speed)], duration_s, stop_deg, 0.0)
    assert sim.pwm.pulses == {}                                 # τίποτα δεν γράφτηκε


def test_one_jog_at_a_time(config):
    _, ident = make_bench(config)
    ident.start_jog([(0, 1630, None, 0.4)], 0.5, 3.0, 0.0)
    with pytest.raises(ValueError):
        ident.start_jog([(1, 1605, None, 0.4)], 0.5, 3.0, 0.0)
    ident.stop()
    assert not ident.jogging


def test_pulse_stays_inside_the_safety_offset(config):
    sim, ident = make_bench(config)
    ident.start_jog([(0, 1630, None, 1.0)], 0.5, 3.0, 0.0)
    assert sim.pwm.pulses[0] <= 1630 + config['pca']['max_offset_us']


@pytest.fixture
def session(config, data):
    from arm_pi_driver.identify import Session
    sim, ident = make_bench(config)
    return sim, Session(ident, data, timeout_s=5.0)


def test_session_scan_read_and_jog_by_servo_name(session):
    sim, s = session
    assert s.handle({'id': 1, 'op': 'scan'}, 0.0)['result'][0]['devices'] == ['PCA9685']
    assert s.active
    reply = s.handle({'id': 2, 'op': 'jog', 'servos': {'servo_1': 0.4, 'servo_2': -0.4}}, 0.0)
    assert reply['ok'], reply
    now, done = 0.0, []
    while not done:
        sim.step(DT)
        now += DT
        done = [r for r in s.tick(now) if r['op'] != 'live']
    assert done[0]['id'] == 2 and done[0]['ok'] and list(done[0]['result']['moved']) == ['2']


def test_session_refuses_servos_without_a_channel(session):
    _, s = session
    reply = s.handle({'id': 1, 'op': 'jog', 'servos': {'servo_5': 0.4}}, 0.0)
    assert not reply['ok'] and 'κανάλι' in reply['error']
    assert not s.handle({'op': 'jog', 'servos': {'nothing': 0.4}}, 0.0)['ok']
    assert not s.handle({'op': 'dance'}, 0.0)['ok']


def test_session_exits_on_request_and_on_silence(session):
    sim, s = session
    s.handle({'op': 'jog', 'servos': {'servo_1': 0.4}, 'duration_s': 3.0, 'stop_deg': 30.0}, 0.0)
    s.handle({'op': 'exit'}, 0.1)
    assert not s.active and sim.pwm.pulses[0] is None
    s.handle({'op': 'read'}, 1.0)
    assert [r for r in s.tick(5.9) if r['op'] != 'live'] == [] and s.active
    [notice] = s.tick(6.1)
    assert notice['op'] == 'exit' and not s.active


def test_command_line_requests():
    from arm_pi_driver.identify_cli import parse
    assert parse(['scan'])['op'] == 'scan'
    jog = parse(['jog', 'servo_1=0.4', 'servo_2=-0.4', '--duration', '1'])
    assert jog['servos'] == {'servo_1': 0.4, 'servo_2': -0.4} and jog['duration_s'] == 1.0
    with pytest.raises(SystemExit):
        parse(['jog', 'servo_1'])
