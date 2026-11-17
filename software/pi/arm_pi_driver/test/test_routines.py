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
