"""Οι διαδικασίες του teleop ([6], [9], [10]) και το «κράτα το κουμπί», στην προσομοίωση."""

from arm_pi_driver.hardware.fake import FakeEncoder, SimulatedArm
from arm_pi_driver.identify import Identifier, Session
import pytest

DT = 0.02


@pytest.fixture
def bench(config, data):
    sim = SimulatedArm(config)
    by_channel = {j['mux_channel']: sim.encoders[n] for n, j in sim.joints.items()}
    missing = FakeEncoder(float('nan'), 'NO_RESPONSE')
    encoders = {ch: by_channel.get(ch, missing) for ch in range(8)}
    bench = data['controller']['bench']
    ident = Identifier(encoders, sim.pwm, lambda ch: [], config['pca'], bench)
    return sim, Session(ident, data)


def run(sim, session, request, seconds=60.0):
    """Ένα αίτημα routine ως το τέλος: (αποτέλεσμα, μηνύματα προόδου)."""
    reply = session.handle({'id': 7, **request}, 0.0)
    assert reply['ok'], reply
    now, notes = 0.0, []
    while now < seconds:
        sim.step(DT)
        now += DT
        session.last_request = now                    # το Arm Studio στέλνει ping
        for r in session.tick(now):
            if r['op'] == 'progress':
                notes.append(r['result'])
            if r['op'] == 'routine':
                return r, notes
    raise AssertionError('η διαδικασία δεν τελείωσε')


def test_identify_moves_finds_the_encoder_and_returns(bench):
    sim, session = bench
    reply, notes = run(sim, session, {'op': 'routine', 'name': 'identify',
                                      'servos': {'servo_1': 1, 'servo_2': -1}})
    result = reply['result']
    assert reply['ok'] and result['main'] == 2 and abs(result['moved']['2']) >= 3.0
    assert result['returned'] and abs(sim.angles['joint_1']) < 1.5          # πίσω στην αρχή
    assert abs(result['moved']['1']) < 1.0 and notes                         # μόνο το joint_1
    assert sim.pwm.pulses[0] is None and sim.pwm.pulses[1] is None           # τέλος: off


def test_servo_deadband_like_teleop_10(bench):
    sim, session = bench
    reply, notes = run(sim, session, {'op': 'routine', 'name': 'servo_deadband',
                                      'servo': 'servo_1',
                                      'brake': ['servo_2']})
    down, up = reply['result']['down_us'], reply['result']['up_us']
    assert 1420 - 40 <= down < 1420 and 1840 < up <= 1840 + 40               # sim: 1420/1840
    assert len(notes) == 2


def test_joint_deadband_like_teleop_9(bench):
    sim, session = bench
    reply, _ = run(sim, session, {'op': 'routine', 'name': 'joint_deadband',
                                  'servos': {'servo_1': 1, 'servo_2': 1}, 'channel': 1,
                                  'invert': True})
    assert reply['ok'] and 0 < reply['result']['pos'] <= 0.6 and 0 < reply['result']['neg'] <= 0.6


def test_a_lost_encoder_stops_the_routine(bench):
    sim, session = bench
    session.handle({'id': 1, 'op': 'routine', 'name': 'identify',
                    'servos': {'servo_1': 1, 'servo_2': -1}}, 0.0)
    session.tick(DT)
    session.identifier.encoders[2].status = 'NO_RESPONSE'
    replies = session.tick(2 * DT)
    end = [r for r in replies if r['op'] == 'routine'][0]
    assert not end['ok'] and 'κανάλι 2' in end['error'] and sim.pwm.pulses[0] is None


def test_drive_moves_only_while_refreshed(bench):
    sim, session = bench
    now = 0.0
    for _ in range(50):                                # «κρατάει το W» 1 s
        session.handle({'op': 'drive', 'servos': {'servo_1': 0.4, 'servo_2': -0.4}}, now)
        sim.step(DT)
        now += DT
        session.tick(now)
    moved = sim.angles['joint_1']
    assert moved > 2.0 and sim.pwm.pulses[0] is not None
    for _ in range(25):                                # «άφησε το πλήκτρο»
        sim.step(DT)
        now += DT
        session.last_request = now
        session.tick(now)
    assert sim.pwm.pulses[0] is None and sim.pwm.pulses[1] is None


def test_live_readings_and_stop(bench):
    sim, session = bench
    session.handle({'op': 'ping'}, 0.0)
    live = [r for r in session.tick(0.0) if r['op'] == 'live']
    assert live and live[0]['result'][2]['status'] == 'OK'
    session.handle({'id': 3, 'op': 'routine', 'name': 'servo_deadband', 'servo': 'servo_1'}, 0.0)
    session.tick(DT)
    session.handle({'op': 'stop'}, DT)
    end = [r for r in session.tick(2 * DT) if r['op'] == 'routine'][0]
    assert not end['ok'] and 'διακοπή' in end['error']


# ---------------------------------------------------------------------------------------- 8β
class Jaw:
    """Αρπάγη χωρίς υλικό: κλείνει όσο έχει παλμό κάτω από το νεκρό σημείο· από τα 20° και κάτω
    υπάρχει αντικείμενο, και η τάση των ZD10 ανεβαίνει όσο σφίγγει."""

    def __init__(self, angle=60.0, contact=20.0):
        self.angle, self.contact, self.pulses = angle, contact, {}
        jaw = self

        class Encoder:
            def read(self):
                return jaw.angle % 360.0, 'OK'

        class Sensor:
            def read(self):
                return 0.94 + max(0.0, jaw.contact - jaw.angle) * 0.2, 'OK'

        self.encoder, self.sensor = Encoder(), Sensor()

    def set_pulse(self, channel, pulse):
        self.pulses[channel] = pulse

    def off(self, channel):
        self.pulses[channel] = None

    def step(self):
        pulse = self.pulses.get(4)
        if pulse is not None and pulse < 1600:
            self.angle = max(0.0, self.angle - 0.5)


def squeeze(jaw, safety_v=2.4, stop_at=None):
    from arm_pi_driver import routines
    routine = routines.gripper_pressure(lambda speed: {4: 1600 + 400 * speed}, 3, 1, 0.0, 0.0,
                                        {'finger_1': jaw.sensor, 'finger_2': jaw.sensor}, safety_v)
    runner, now = routines.Runner(routine, {3: jaw.encoder}, jaw, 65.0), 0.0
    while not runner.done and now < 60:
        jaw.step()
        runner.tick(now)
        now += DT
        if stop_at is not None and jaw.angle <= stop_at:
            runner.stop()
    return runner


def test_gripper_pressure_stop_gives_the_pressure_of_that_moment():
    jaw = Jaw()
    runner = squeeze(jaw, stop_at=18.0)
    assert runner.error is None and runner.result['reason'] == 'stop'
    assert runner.result['volts']['finger_1'] == pytest.approx(0.94 + 0.4, abs=0.25)
    assert jaw.pulses[4] is None                                        # off μετά το Stop


def test_gripper_pressure_stops_at_the_sensor_safety_limit():
    runner = squeeze(Jaw())
    assert runner.result['reason'] == 'safety' and runner.result['volts']['finger_1'] >= 2.4


def test_gripper_pressure_without_an_object_stops_closed():
    runner = squeeze(Jaw(contact=-1.0))
    assert runner.result['reason'] == 'closed' and runner.result['position_deg'] <= 0.0


def test_routines_without_a_result_on_stop_still_end_with_an_error():
    from arm_pi_driver import routines
    runner = routines.Runner(routines.servo_deadband(0, 1600, {}), {3: Jaw().encoder}, Jaw(), 30)
    runner.tick(0.0)
    runner.stop()
    assert runner.done and runner.result is None and 'διακοπή' in runner.error


def test_gripper_pressure_needs_8a_and_the_sensors(bench):
    _, session = bench
    reply = session.handle({'op': 'routine', 'name': 'gripper_pressure'}, 0.0)
    assert not reply['ok'] and '8α' in reply['error']                   # όρια [0, 0]
    gripper = session.data['joints']['gripper']
    gripper['limits_deg'] = [0.0, 60.0]
    session.data['encoders'][gripper['encoder']]['channel'] = 3
    reply = session.handle({'op': 'routine', 'name': 'gripper_pressure'}, 0.0)
    assert not reply['ok'] and 'αισθητήρες πίεσης σε αυτόν' in reply['error']
