"""The calibration bench (no Qt) and the «Βαθμονόμηση» panel, with the arm.yaml of the repo."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
arm_config = pytest.importorskip("arm_calibration.arm_config")
from arm_gui.calibration import Bench, nest, zero_for  # noqa: E402
import test_gui_smoke as smoke  # noqa: E402

app = smoke.app          # the same fixtures (window with a fake ROS bridge) as the smoke tests
window = smoke.window


def params():
    """/arm_calibration as the bridge gives it: [{'name', 'value', ...}] of arm.yaml."""
    return [{"name": n, "value": v, "type": "other", "description": "", "read_only": False, "min": None,
             "max": None} for n, v in arm_config.parameters(arm_config.load()).items()]


@pytest.fixture
def bench():
    b = Bench()
    b.data = nest({p["name"]: p["value"] for p in params()})
    return b


SCAN = {"0": {"devices": ["PCA9685"], "encoder": None},
        "1": {"devices": ["AS5600"], "encoder": {"raw_deg": 112.7, "status": "OK"}},
        "2": {"devices": ["AS5600"], "encoder": {"raw_deg": 267.9, "status": "OK"}}}


def test_nest_gives_back_the_file(bench):
    data = arm_config.load()
    assert bench.data["joints"]["joint_1"] == data["joints"]["joint_1"]
    assert bench.data["encoders"]["enc_3"] == {"channel": None}          # -1 → null
    assert arm_config.validate(bench.data) == []


def routine_done(bench, request, result):
    return bench.handle({"id": request["id"], "op": "routine", "ok": True, "result": result})


def test_identify_runs_every_joint_of_the_group_then_says_what_it_found(bench):
    group = bench.groups()[0]
    assert group.joints == ("joint_1", "joint_2")
    first = bench.identify(group)
    assert first["name"] == "identify" and first["servos"] == {"servo_1": 1, "servo_2": -1}
    bench.handle({"id": first["id"], "op": "progress", "ok": True, "result": "0.5 s: κανάλι 2 +3.2°"})
    [second] = routine_done(bench, first, {"moved": {"1": 0.2, "2": 3.4}, "main": 2})
    assert second["servos"] == {"servo_1": 1, "servo_2": 1}
    assert routine_done(bench, second, {"moved": {"1": -3.1, "2": 0.3}, "main": 1}) == []
    assert bench.changes == {}                                             # the file is right already
    assert any("joint_2 → encoder στο κανάλι 1, η γωνία κατεβαίνει" in line for line in bench.summary)


def test_the_pis_started_is_not_a_result(bench):
    """Regression (bench 2026-11-17): the immediate «started» was taken for joint_1's result, joint_2
    was asked while joint_1 still ran, and the real result crashed Arm Studio."""
    first = bench.identify(bench.groups()[0])
    assert bench.handle({"id": first["id"], "op": "routine", "ok": True, "result": "ξεκίνησε"}) == []
    assert bench.running == "Αναγνώριση" and bench._current == "joint_1"
    bench.stop()
    assert bench.handle({"id": first["id"], "op": "routine", "ok": True, "result": {"moved": {"2": 3}}}) == []


def test_identify_finds_swapped_encoders_and_fixes_the_names(bench):
    group = bench.groups()[0]
    first = bench.identify(group)
    [second] = routine_done(bench, first, {"moved": {"1": 3.3, "2": 0.1}, "main": 1})
    routine_done(bench, second, {"moved": {"1": 0.0, "2": -3.0}, "main": 2})
    assert bench.changes["joints.joint_1.encoder"] == "enc_2" and bench.changes["joints.joint_2.encoder"] == "enc_1"
    bench.swap_names()                                                     # «Ανάποδα ονόματα»
    assert "joints.joint_1.encoder" not in bench.changes


def test_identify_refuses_too_little_or_two_encoders(bench):
    group = bench.groups()[0]
    first = bench.identify(group)
    [second] = routine_done(bench, first, {"moved": {"1": 0.4, "2": 0.9}, "main": 2})
    routine_done(bench, second, {"moved": {"1": 3.0, "2": 2.5}, "main": 1})
    assert bench.changes == {} and len([s for s in bench.summary if s.startswith("✖")]) == 2


def test_flip_reverses_the_signs_of_a_joint(bench):
    group = bench.groups()[0]
    first = bench.identify(group)
    [second] = routine_done(bench, first, {"moved": {"2": 3.0}, "main": 2})
    routine_done(bench, second, {"moved": {"1": -3.0}, "main": 1})
    bench.flip("joint_1")
    assert bench.changes["joints.joint_1.servos.servo_1"] == -1 and bench.changes["joints.joint_1.invert"] is True


def test_servo_and_joint_deadbands(bench):
    group = bench.groups()[0]
    first = bench.servo_deadbands(group)
    assert first["servo"] == "servo_1" and first["brake"] == ["servo_2"]
    [second] = routine_done(bench, first, {"down_us": 1410, "up_us": 1850})
    routine_done(bench, second, {"down_us": 1500, "up_us": 1710})
    assert bench.changes["servos.servo_1.deadband_us"] == [1410, 1850]
    assert bench.changes["servos.servo_1.neutral_us"] == 1630
    first = bench.joint_deadbands(group)
    assert first["channel"] == 2 and first["invert"] is False
    [second] = routine_done(bench, first, {"pos": 0.07, "neg": 0.09})
    routine_done(bench, second, {"pos": None, "neg": 0.1})
    assert bench.changes == {"joints.joint_1.deadband": [0.07, 0.09]}
    assert any(s.startswith("✖ joint_2") for s in bench.summary)


def test_live_position_and_zero(bench):
    bench.handle({"op": "live", "ok": True, "result": {"2": {"raw_deg": 277.891, "status": "TOO_WEAK"}}})
    assert bench.position("joint_1") == 10.0
    assert bench.zero("joint_1", 90.0, -30, 50)
    assert bench.changes == {"joints.joint_1.zero_deg": zero_for(277.891, False, 90.0),
                             "joints.joint_1.limits_deg": [-30.0, 50.0]}
    assert not bench.zero("joint_2", 0.0, -90, 90)                        # no reading of channel 1


def test_drive_mixes_the_joints_with_the_signs_of_the_file(bench):
    request = bench.drive({"joint_1": 0.4, "joint_2": 0.0})
    assert request["servos"] == {"servo_1": 0.4, "servo_2": -0.4}
    request = bench.drive({"joint_1": 0.4, "joint_2": 0.4})
    assert request["servos"] == {"servo_1": 0.8, "servo_2": 0.0}


def test_errors_and_the_pi_leaving_on_its_own(bench):
    first = bench.identify(bench.groups()[0])
    bench.handle({"id": first["id"], "op": "routine", "ok": False, "error": "κανάλι 2: σταμάτησε να μετρά"})
    assert bench.running is None and bench.log[-1][0] == "error"
    bench.handle({"id": None, "op": "exit", "ok": True, "result": "καμία εντολή για > 5 s"})
    assert not bench.active and bench.ping() is None


@pytest.fixture
def panel(window):
    window.calibration._confirm = lambda text: True
    window._bridge.parameters_loaded.emit("/arm_calibration", params())
    return window.calibration


def test_pages_show_the_file(panel):
    assert panel.servos.rowCount() == 11 and panel.joints.rowCount() == 7
    row = [panel.servos.item(0, c).text() for c in range(7)]
    assert row == ["servo_1", "0", "1630", "1420", "1840", "βάση, κινητήρας A", "joint_1, joint_2"]
    joint_1 = [panel.joints.item(0, c).text() for c in range(5)]
    assert joint_1 == ["joint_1", "ναι", "βάση, στροφή", "servo_1:+1  servo_2:-1", "enc_1"]
    assert panel.encoders.item(0, 2).text() == "joint_1"


def test_edits_are_sent_all_at_once(panel, window):
    panel.servos.item(0, 2).setText("1625")                               # neutral
    panel.joints.item(1, 8).setText("80")                                 # joint_2 upper limit
    panel.apply_edits()
    node, values = window._bridge.atomic_sets[-1]
    assert node == "/arm_calibration"
    assert values == {"servos.servo_1.neutral_us": 1625, "joints.joint_2.limits_deg": [-90.0, 80.0]}
    panel.servos.item(0, 1).setText("κάτι")
    panel.apply_edits()
    assert "Δεν στάλθηκαν" in panel.status.text() and len(window._bridge.atomic_sets) == 1


def test_rejected_changes_are_shown(panel, window):
    window._bridge.parameters_set.emit("/arm_calibration", [("a", False, "κανάλι PCA 0: το έχουν servo_1, servo_2")])
    assert "κανάλι PCA 0" in panel.status.text()


def test_live_values_from_diagnostics(panel, window):
    window._bridge.diagnostics.emit([{"name": "encoder/joint_1", "level": 0, "message": "OK", "hardware_id": "",
                                      "values": {"raw_deg": "267.90", "position_deg": "0.01"}}])
    assert panel.joints.item(0, 11).text() == "0.01" and panel.encoders.item(0, 3).text() == "267.90"


def test_values_are_asked_again_while_none_came(window):
    """A reply of /arm_calibration can be lost (DDS): while the node runs, the panel asks again."""
    panel, b = window.calibration, window._bridge
    now = [100.0]
    panel._clock = lambda: now[0]
    panel.set_nodes(["/arm_calibration"])
    asked = b.param_requests.count("/arm_calibration")                     # the Parameters panel's
    now[0] += 1.0
    panel.set_nodes(["/arm_calibration"])
    assert b.param_requests.count("/arm_calibration") == asked             # not every second
    now[0] += 7.0
    panel.set_nodes(["/arm_calibration"])
    assert b.param_requests.count("/arm_calibration") == asked + 1
    b.parameters_loaded.emit("/arm_calibration", params())
    now[0] += 7.0
    panel.set_nodes(["/arm_calibration"])
    assert b.param_requests.count("/arm_calibration") == asked + 1         # loaded: no more


def test_joints_page_hides_the_details_until_asked(panel):
    assert panel.joints.isColumnHidden(5) and not panel.joints.isColumnHidden(11)
    panel.details.setChecked(True)
    assert not panel.joints.isColumnHidden(5)


def test_arrows_move_while_held_and_keys_do_the_same(panel, window):
    b = window._bridge
    panel.move_joint.setCurrentText("joint_2")
    panel.hold(1)
    assert b.identify_requests[-1] == {**b.identify_requests[-1], "op": "drive",
                                       "servos": {"servo_1": 0.4, "servo_2": 0.4}}
    assert panel._drive_timer.isActive()
    panel.release()
    assert not panel._drive_timer.isActive()
    from arm_gui.qt import QtCore, QtGui
    panel.move_joint.setCurrentText("joint_1")
    key = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_A, QtCore.Qt.NoModifier)
    panel.keyPressEvent(key)
    assert b.identify_requests[-1]["servos"] == {"servo_1": -0.4, "servo_2": 0.4}
    panel.keyReleaseEvent(QtGui.QKeyEvent(QtCore.QEvent.KeyRelease, QtCore.Qt.Key_A, QtCore.Qt.NoModifier))
    assert not panel._drive_timer.isActive()


def test_identify_then_save_from_the_panel(panel, window):
    b = window._bridge
    panel.go_step(1)
    panel.do_identify()
    first = b.identify_requests[-1]
    assert first["op"] == "routine" and "Αναγνώριση" in panel.mode.text()
    b.identify_reply.emit({"id": first["id"], "op": "routine", "ok": True, "result": {"moved": {"1": 3.3}}})
    second = b.identify_requests[-1]
    b.identify_reply.emit({"id": second["id"], "op": "routine", "ok": True, "result": {"moved": {"2": 3.0}}})
    assert panel.save_button.isEnabled()
    panel.save_changes()
    node, values = b.atomic_sets[-1]
    assert values["joints.joint_1.encoder"] == "enc_2"
    b.parameters_set.emit(node, [(n, True, "") for n in values])
    assert b.triggers[-1] == "/arm_calibration/save" and not panel.save_button.isEnabled()


def test_live_table_and_zero_from_the_panel(panel, window):
    b = window._bridge
    b.identify_reply.emit({"id": None, "op": "live", "ok": True,
                           "result": {"1": {"raw_deg": 112.676, "status": "OK"},
                                      "2": {"raw_deg": 277.891, "status": "TOO_WEAK"}}})
    rows = {panel.live_table.item(r, 0).text(): panel.live_table.item(r, 3).text()
            for r in range(panel.live_table.rowCount())}
    assert rows == {"joint_1": "+10.0", "joint_2": "-0.0"} or rows["joint_1"] == "+10.0"
    panel.go_step(2)
    panel.zero_joint.setCurrentText("joint_2")
    assert (panel.lower.value(), panel.upper.value()) == (-90.0, 90.0)     # joint_2's limits
    panel.zero_joint.setCurrentText("joint_1")
    assert (panel.lower.value(), panel.upper.value()) == (-30.0, 50.0)
    panel.zero_angle.setValue(0.0)
    panel.do_zero()
    assert panel.bench.changes["joints.joint_1.zero_deg"] == 277.891


def test_wizard_steps_and_power(panel):
    assert panel.tabs.tabText(0) == "Οδηγός" and panel.step == 0 and "σβηστό" in panel.power.text()
    panel.go_step(1)
    assert "ΑΝΑΜΜΕΝΟ" in panel.power.text()
    panel.go_step(99)
    assert panel.step == len(panel.STEPS) - 1
