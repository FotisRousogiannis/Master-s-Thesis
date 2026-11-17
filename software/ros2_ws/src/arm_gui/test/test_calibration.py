"""The calibration wizard (no Qt) and the «Βαθμονόμηση» panel, with the arm.yaml of the repo."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
arm_config = pytest.importorskip("arm_calibration.arm_config")
from arm_gui.calibration import Wizard, nest  # noqa: E402
import test_gui_smoke as smoke  # noqa: E402

app = smoke.app          # the same fixtures (window with a fake ROS bridge) as the smoke tests
window = smoke.window


def params():
    """/arm_calibration as the bridge gives it: [{'name', 'value', ...}] of arm.yaml."""
    return [{"name": n, "value": v, "type": "other", "description": "", "read_only": False, "min": None,
             "max": None} for n, v in arm_config.parameters(arm_config.load()).items()]


@pytest.fixture
def wizard():
    w = Wizard()
    w.data = nest({p["name"]: p["value"] for p in params()})
    return w


SCAN = {"0": {"devices": ["PCA9685"], "encoder": None},
        "1": {"devices": ["AS5600"], "encoder": {"raw_deg": 112.7, "status": "OK"}},
        "2": {"devices": ["AS5600"], "encoder": {"raw_deg": 267.9, "status": "OK"}}}


def test_nest_gives_back_the_file(wizard):
    data = arm_config.load()
    assert wizard.data["joints"]["joint_1"] == data["joints"]["joint_1"]
    assert wizard.data["encoders"]["enc_3"] == {"channel": None}          # null: no parameter
    assert arm_config.validate(wizard.data) == []


def test_scan_reply_gives_findings_and_readings(wizard):
    request = wizard.scan()
    assert wizard.active and request["op"] == "scan"
    wizard.handle({"id": request["id"], "op": "scan", "ok": True, "result": SCAN})
    assert wizard.readings[2]["raw_deg"] == 267.9
    assert any("joint_1: encoder OK" in f.text for f in wizard.findings)


def test_jog_then_relabel_direction_and_approval(wizard):
    request = wizard.jog_joint("joint_1")
    assert request["servos"] == {"servo_1": 0.4, "servo_2": -0.4} and wizard.busy
    assert wizard.keepalive() is None                                     # not during a motion
    wizard.handle({"id": request["id"], "op": "jog", "ok": True, "result": "ξεκίνησε"})
    assert wizard.busy
    wizard.handle({"id": request["id"], "op": "jog", "ok": True,
                   "result": {"moved": {"5": 3.1}, "reason": "κινήθηκε"}})
    assert not wizard.busy and wizard.mapping.channel == 5
    assert [p.name for p, _ in wizard.proposals] == ["encoders.enc_1.channel"]
    wizard.direction(False)
    names = {p.name for p, _ in wizard.proposals}
    assert {"joints.joint_1.servos.servo_1", "joints.joint_1.servos.servo_2"} <= names
    assert wizard.approved() == ({}, [])                                  # nothing approved yet
    wizard.proposals[0][1] = True
    assert wizard.approved()[0] == {"encoders.enc_1.channel": 5}
    wizard.applied({"encoders.enc_1.channel"})
    assert "encoders.enc_1.channel" not in {p.name for p, _ in wizard.proposals}


def test_deadband_sweep_asks_until_it_finds_motion(wizard):
    request = wizard.servo_deadband("servo_1")
    follow = []
    for _ in range(3):                                                    # no motion at 10, 20, 30 µs
        follow = wizard.handle({"id": request["id"], "op": "jog", "ok": True, "result": {"moved": {}}})
        request = follow[0]
    assert request["plain"] and request["servos"]["servo_1"] > 0
    follow = wizard.handle({"id": request["id"], "op": "jog", "ok": True, "result": {"moved": {"2": 1.0}}})
    assert follow[0]["servos"]["servo_1"] < 0                             # now the other direction


def test_errors_and_the_pi_leaving_on_its_own(wizard):
    request = wizard.jog_joint("joint_3")
    wizard.handle({"id": request["id"], "op": "jog", "ok": False, "error": "το servo_3 δεν έχει κανάλι"})
    assert not wizard.busy and wizard.findings[-1].level == "error"
    wizard.handle({"id": None, "op": "exit", "ok": True, "result": "καμία εντολή για > 5 s"})
    assert not wizard.active and wizard.keepalive() is None


def test_zero_and_gripper_from_a_read(wizard):
    assert not wizard.zero("joint_1")                                     # no reading yet
    request = wizard.read()
    wizard.handle({"id": request["id"], "op": "read", "ok": True,
                   "result": {"2": {"raw_deg": 270.0, "status": "OK"}}})
    assert wizard.zero("joint_1")
    assert ("joints.joint_1.zero_deg", 270.0) in [(p.name, p.new) for p, _ in wizard.proposals]


# ---------------------------------------------------------------------------------------------- panel


@pytest.fixture
def panel(window):
    window.calibration._confirm = lambda text: True
    window._bridge.parameters_loaded.emit("/arm_calibration", params())
    return window.calibration


def test_order_of_the_file_and_moved_joint_follows_the_choice(panel):
    assert [panel.joints.item(r, 0).text() for r in range(3)] == ["joint_1", "joint_2", "joint_3"]
    assert panel.joints.item(6, 0).text() == "gripper"
    assert [panel.servos.item(r, 0).text() for r in (1, 9, 10)] == ["servo_2", "servo_10", "servo_11"]
    panel.joint_choice.setCurrentText("joint_2")
    assert panel.actual_choice.currentText() == "joint_2"


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


def test_wizard_from_the_panel(panel, window):
    b = window._bridge
    panel.do_scan()
    scan = b.identify_requests[-1]
    b.identify_reply.emit({"id": scan["id"], "op": "scan", "ok": True, "result": SCAN})
    assert panel.findings.count() > 0 and panel.mode.text() == "Pi: αναγνώριση"
    panel.joint_choice.setCurrentText("joint_1")
    panel.do_jog()
    jog = b.identify_requests[-1]
    assert jog["op"] == "jog"
    b.identify_reply.emit({"id": jog["id"], "op": "jog", "ok": True, "result": {"moved": {"5": 3.0}}})
    assert panel.proposals.rowCount() == 1
    panel.proposals.item(0, 0).setCheckState(2)                           # Qt.Checked
    panel.apply_proposals()
    assert b.atomic_sets[-1] == ("/arm_calibration", {"encoders.enc_1.channel": 5})
    b.parameters_set.emit("/arm_calibration", [("encoders.enc_1.channel", True, "")])
    assert panel.proposals.rowCount() == 0 and b.param_requests[-1] == "/arm_calibration"
    panel.do_exit()
    assert b.identify_requests[-1]["op"] == "exit" and panel.mode.text() == "Pi: κανονικός έλεγχος"


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


def test_wizard_shows_one_step_at_a_time(panel):
    assert panel.tabs.tabText(0) == "Οδηγός" and panel.step == 0
    assert not panel.joint_choice.isVisibleTo(panel) and "σβηστό" in panel.power.text()
    panel.go_step(2)                                                      # mapping: moves the arm
    assert panel.step_pages.currentIndex() == 2 and "ΑΝΑΜΜΕΝΟ" in panel.power.text()
    assert panel.joint_choice.isVisibleTo(panel) and not panel.servo_choice.isVisibleTo(panel)
    panel.go_step(5)
    assert panel.servo_choice.isVisibleTo(panel)
    panel.go_step(99)
    assert panel.step == 7
    panel.go_step(-1)
    assert panel.step == 0


def test_joints_page_hides_the_details_until_asked(panel):
    assert panel.joints.isColumnHidden(5) and not panel.joints.isColumnHidden(11)
    panel.details.setChecked(True)
    assert not panel.joints.isColumnHidden(5)


def test_apply_button_counts_the_approved(panel, window):
    b = window._bridge
    panel.do_scan()
    panel.joint_choice.setCurrentText("joint_1")
    panel.do_jog()
    jog = b.identify_requests[-1]
    b.identify_reply.emit({"id": jog["id"], "op": "jog", "ok": True, "result": {"moved": {"5": 3.0}}})
    assert panel.step_buttons[3].text() == "4 · Έλεγχος (1)"
    panel.proposals.item(0, 0).setCheckState(2)
    assert panel.apply_proposals_button.text() == "Εφαρμογή (1)"
