"""Η διαδικασία βαθμονόμησης (arm_calibration.procedure, PC) πάνω στην αναγνώριση (Pi) και την
προσομοίωση: το «φυσικό» βραχίονα τον ορίζει το arm.yaml του repo, ενώ η διαδικασία ξεκινά από ένα
αρχείο με σκόπιμα λάθη και πρέπει να τα βρει."""

import copy

from arm_calibration import arm_config, procedure
from arm_pi_driver.hardware.fake import FakeEncoder, SimulatedArm
from arm_pi_driver.identify import Identifier, Session
import pytest

DT = 0.02


@pytest.fixture
def bench(config, data):
    """(προσομοίωση, Session με το ΛΑΘΟΣ αρχείο, το λάθος αρχείο)."""
    sim = SimulatedArm(config)
    by_channel = {j['mux_channel']: sim.encoders[n] for n, j in sim.joints.items()}
    missing = FakeEncoder(float('nan'), 'NO_RESPONSE')
    encoders = {ch: by_channel.get(ch, missing) for ch in range(8)}

    def probe(channel):
        return ([0x40] if channel == 0 else []) + ([0x36] if channel in by_channel else [])

    wrong = copy.deepcopy(data)
    wrong['encoders']['enc_1']['channel'], wrong['encoders']['enc_2']['channel'] = 1, 2  # ανάποδα
    wrong['joints']['joint_1']['servos']['servo_2'] = 1                 # λάθος πρόσημο
    wrong['joints']['joint_2']['servos']['servo_2'] = -1                # (για να μείνει έγκυρο)
    identifier = Identifier(encoders, sim.pwm, probe, config['pca'], data['controller']['bench'])
    return sim, Session(identifier, wrong), wrong


def ask(sim, session, request):
    """Ένα αίτημα στο Pi· για jog, περιμένει το αποτέλεσμα (όπως ο οδηγός του Arm Studio)."""
    reply = session.handle(request, 0.0)
    assert reply['ok'], reply
    if request['op'] != 'jog':
        return reply['result']
    now = 0.0
    while True:
        sim.step(DT)
        now += DT
        done = session.tick(now)
        if done:
            assert done[0]['ok'], done
            return done[0]['result']


def mapping(sim, session, data, joint):
    """Βήμα 3: κίνηση της άρθρωσης με τα πρόσημα του αρχείου και ανάλυση."""
    return procedure.analyze_jog(data, joint, ask(sim, session, procedure.joint_jog(data, joint)))


def test_procedure_finds_swapped_encoders_and_a_wrong_sign(bench):
    sim, session, wrong = bench
    assert not [f for f in procedure.check_scan(wrong, ask(sim, session, {'op': 'scan'}))
                if f.level == procedure.ERROR]
    # Με το λάθος πρόσημο, το «joint_1» του αρχείου κινεί στην πραγματικότητα το joint_2.
    m1, m2 = (mapping(sim, session, wrong, j) for j in ('joint_1', 'joint_2'))
    assert (m1.channel, m2.channel) == (1, 2)       # ο encoder του joint_2 είναι στο κανάλι 1
    # Οι μετρήσεις συμφωνούν με το λάθος αρχείο (σαν να άλλαξαν όνομα οι δύο αρθρώσεις): η
    # διαδικασία ΔΕΝ προτείνει τυφλές αλλαγές. Ο χρήστης βλέπει ποια κίνηση έγινε και λέει «αυτό
    # ήταν το joint_2» → ανταλλαγή servo/encoder των δύο αρθρώσεων → το σωστό αρχείο.
    assert m1.proposals == [] and m2.proposals == []
    swap = procedure.relabel_proposals(wrong, 'joint_1', 'joint_2')
    fixed, errors = procedure.apply(wrong, swap)
    assert errors == []
    for joint in ('joint_1', 'joint_2'):
        real = arm_config.load()['joints'][joint]
        assert fixed['joints'][joint]['servos'] == real['servos']
        channel = fixed['encoders'][fixed['joints'][joint]['encoder']]['channel']
        assert channel == arm_config.load()['encoders'][real['encoder']]['channel']


def test_procedure_finds_an_encoder_on_another_channel(bench, data):
    sim, session, _ = bench
    session.data = moved = copy.deepcopy(data)
    moved['encoders']['enc_1']['channel'] = 5                           # το αρχείο λέει 5
    m = mapping(sim, session, moved, 'joint_1')
    assert {p.name: p.new for p in m.proposals} == {'encoders.enc_1.channel': 2}
    new, errors = procedure.apply(moved, m.proposals)
    assert errors == [] and new['encoders']['enc_1']['channel'] == 2


def test_servo_deadband_is_measured_close_to_the_real_one(bench, data):
    """Η προσομοίωση έχει νεκρή ζώνη 1420–1840 µs στο servo_1 (από το arm.yaml)."""
    sim, session, _ = bench
    session.data = data
    sweep = procedure.ServoDeadbandSweep(data, 'servo_1')
    while (request := sweep.next_request()) is not None:
        sweep.record(ask(sim, session, request))
    down, up = {p.name: p.new for p in sweep.proposals(data)}.get(
        'servos.servo_1.deadband_us', data['servos']['servo_1']['deadband_us'])
    assert 1420 - 30 <= down <= 1420 and 1840 <= up <= 1840 + 30


def test_joint_direction_then_valid_file(bench, data):
    sim, session, _ = bench
    session.data = data
    m = mapping(sim, session, data, 'joint_2')
    proposals = procedure.direction_proposals(data, 'joint_2', m.delta, positive=True)
    assert proposals == []                          # το αρχείο του repo είναι ήδη σωστό
    flipped = procedure.direction_proposals(data, 'joint_2', m.delta, False)
    new, errors = procedure.apply(data, flipped)
    assert errors == [] and arm_config.validate(new) == []
