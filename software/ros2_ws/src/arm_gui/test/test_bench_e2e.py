"""Arm Studio's calibration panel against the Pi's identification mode on the simulated arm.

The requests and replies go through JSON as on the network. Catches what unit tests of each side
miss: on the bench (2026-11-17) the Pi's «started» was taken for a result and Arm Studio closed.
"""

import json
import os
import sys
import tempfile
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
PI = Path(__file__).resolve().parents[4] / "pi" / "arm_pi_driver"
if not PI.is_dir():
    pytest.skip("arm_pi_driver not found", allow_module_level=True)
sys.path.insert(0, str(PI))
import test_calibration as tc  # noqa: E402
import test_gui_smoke as smoke  # noqa: E402

app = smoke.app
DT = 0.02


@pytest.fixture
def rig(app):
    from arm_calibration import arm_config
    from arm_gui.main_window import MainWindow
    from arm_pi_driver import arm_file
    from arm_pi_driver.hardware.fake import FakeEncoder, SimulatedArm
    from arm_pi_driver.identify import Identifier, Session

    data = arm_config.load()
    config, _ = arm_file.driver_config(data)
    sim = SimulatedArm(config)
    by_channel = {j["mux_channel"]: sim.encoders[n] for n, j in sim.joints.items()}
    encoders = {ch: by_channel.get(ch, FakeEncoder(float("nan"), "NO_RESPONSE")) for ch in range(8)}
    session = Session(Identifier(encoders, sim.pwm, lambda ch: [], config["pca"], data["controller"]["bench"]),
                      data)
    window = MainWindow(smoke.FakeBridge(), smoke.FakeProcesses(), smoke.FakePlacer(), tempfile.mktemp())
    window.calibration._confirm = lambda text: True
    window._bridge.parameters_loaded.emit("/arm_calibration", tc.params())
    clock = [0.0]

    def pump(seconds):
        bridge = window._bridge
        for _ in range(int(seconds / DT)):
            while bridge.identify_requests:
                request = json.loads(json.dumps(bridge.identify_requests.pop(0)))
                bridge.identify_reply.emit(json.loads(json.dumps(session.handle(request, clock[0]))))
            sim.step(DT)
            clock[0] += DT
            session.last_request = clock[0]                     # Arm Studio's ping
            for reply in session.tick(clock[0]):
                bridge.identify_reply.emit(json.loads(json.dumps(reply)))

    yield window.calibration, sim, pump
    window._processes.running = False
    window.close()


def until_done(panel, pump, seconds=60.0):
    for _ in range(int(seconds / 0.5)):
        pump(0.5)
        if not panel.bench.running:
            return
    raise AssertionError("η διαδικασία δεν τελείωσε")


def test_identify_both_joints_of_the_base(rig):
    panel, sim, pump = rig
    panel.do_identify()
    until_done(panel, pump)
    texts = [t for _, t in panel.bench.log]
    assert any("joint_1 → encoder στο κανάλι 2" in t for t in texts)
    assert any("joint_2 → encoder στο κανάλι 1" in t for t in texts)
    assert panel.bench.changes == {}                              # the file matches the arm
    assert abs(sim.angles["joint_1"]) < 1.5 and abs(sim.angles["joint_2"]) < 1.5   # came back


def test_servo_and_joint_deadbands(rig):
    panel, _, pump = rig
    panel._routine(panel.bench.servo_deadbands)
    until_done(panel, pump)
    down, up = panel.bench.changes["servos.servo_1.deadband_us"]
    assert 1380 <= down <= 1420 and 1840 <= up <= 1880               # the simulation: 1420 / 1840
    panel._routine(panel.bench.joint_deadbands)
    until_done(panel, pump)
    assert set(panel.bench.changes) == {"joints.joint_1.deadband", "joints.joint_2.deadband"}


def test_arrows_move_and_stop_when_released(rig):
    panel, sim, pump = rig
    panel.move_joint.setCurrentText("joint_2")
    panel.hold(1)
    for _ in range(10):
        panel._send_drive()
        pump(0.1)
    panel.release()
    pump(0.5)
    assert sim.angles["joint_2"] > 5 and abs(sim.angles["joint_1"]) < 0.5
    assert sim.pwm.pulses[0] is None and sim.pwm.pulses[1] is None
    assert panel.live_table.rowCount() == 2
