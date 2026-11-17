"""Builds the whole window with a fake ROS bridge (no ROS needed) and drives it like a user would."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
qt = pytest.importorskip("arm_gui.qt")
from arm_gui.layouts import Rect  # noqa: E402
from arm_gui.qt import QtCore, QtWidgets, Signal  # noqa: E402

URDF = """<robot name="r"><link name="a"/><link name="b"/><link name="c"/>
  <joint name="j1" type="revolute"><parent link="a"/><child link="b"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="0.5"/></joint>
  <joint name="j2" type="revolute"><parent link="b"/><child link="c"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="0.5"/></joint></robot>"""


class FakeBridge(QtCore.QObject):
    description_received = Signal(str)
    joint_state = Signal(object)
    controllers_changed = Signal(object)
    nodes_changed = Signal(object)
    goal_status = Signal(str)
    log = Signal(str)
    graph_changed = Signal(object)
    parameters_loaded = Signal(str, object)
    parameters_set = Signal(str, object)
    service_result = Signal(str, bool, str)
    diagnostics = Signal(object)
    identify_reply = Signal(object)

    def __init__(self):
        super().__init__()
        self.goals, self.stops = [], 0
        self.param_requests, self.param_sets, self.triggers = [], [], []
        self.events = []
        self.identify_requests, self.atomic_sets = [], []

    def identify(self, request):
        self.identify_requests.append(request)

    def set_parameters_atomically(self, node, values):
        self.atomic_sets.append((node, values))

    def log_event(self, event):
        self.events.append(event)

    def node_info(self, name):
        return {"Δημοσιεύει": [("/joint_states", ["sensor_msgs/msg/JointState"])], "Συνδρομές": []} if name else {}

    def request_parameters(self, node):
        self.param_requests.append(node)

    def set_parameters(self, node, values):
        self.param_sets.append((node, values))

    def call_trigger(self, service):
        self.triggers.append(service)

    def send_joint_goal(self, targets, duration):
        self.goals.append((targets, duration))

    def stop(self):
        self.stops += 1


class FakeProcesses(QtCore.QObject):
    output = Signal(str, str)
    running_changed = Signal(str, bool)

    def __init__(self):
        super().__init__()
        self.started, self.alive = [], set()

    @property
    def running(self):
        return bool(self.alive)

    @running.setter
    def running(self, value):
        assert not value
        self.alive.clear()

    def is_running(self, name):
        return name in self.alive

    def start(self, name, program, args):
        self.started.append([program, *args])
        self.alive.add(name)
        self.running_changed.emit(name, True)

    def stop(self, name):
        if name in self.alive:
            self.alive.discard(name)
            self.running_changed.emit(name, False)

    def any_running(self):
        return self.running

    def stop_all(self, wait_ms=0):
        self.alive.clear()


class FakePlacer:
    available = True

    def __init__(self):
        self.moves = []
        self.open = {}

    def windows(self):
        return self.open

    def move(self, win_id, rect):
        self.moves.append((win_id, rect))
        return True


@pytest.fixture
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def window(app, tmp_path):
    from arm_gui.main_window import MainWindow

    w = MainWindow(FakeBridge(), FakeProcesses(), FakePlacer(), str(tmp_path / "settings.ini"))
    w.show()
    yield w
    w._processes.running = False
    w.close()


def test_joint_rows_are_built_from_the_urdf_and_send_a_goal(window):
    window._bridge.description_received.emit(URDF)
    window._bridge.joint_state.emit({"j1": (0.0, 0.0, 0.0), "j2": (0.0, 0.0, 0.0)})
    assert list(window.joints.joints) == ["j1", "j2"]
    window.joints._rows["j1"].spin.setValue(30.0)
    window.joints.send()
    targets, duration = window._bridge.goals[-1]
    assert targets["j1"] == pytest.approx(0.5236, abs=1e-3) and targets["j2"] == 0.0
    assert duration > 0.5


def test_stop_button_and_escape_reach_the_bridge(window):
    window.stop_button.click()
    assert window._bridge.stops == 1


def test_start_runs_bringup_without_windows_and_opens_them_separately(window):
    window.system._hardware.setCurrentIndex(1)  # gazebo
    window.system._start.click()
    launch = ["ros2", "launch", "arm_bringup"]
    assert window._processes.started == [
        [*launch, "bringup.launch.py", "hardware:=gazebo", "rviz:=false", "gui:=false"],
        [*launch, "rviz.launch.py", "use_sim_time:=true"],
        [*launch, "gazebo_gui.launch.py"],
    ]
    assert not window.system._start.isEnabled()


def test_windows_open_and_close_without_stopping_the_system(window):
    system, procs = window.system, window._processes
    rviz, gazebo = system._windows["rviz"], system._windows["gazebo"]
    assert not rviz.isEnabled()  # nothing to show before the system runs
    system._auto_open.setChecked(False)
    system._start.click()  # mock
    assert procs.alive == {"system"} and rviz.isEnabled() and not gazebo.isEnabled()

    rviz.click()
    assert procs.started[-1][-2:] == ["rviz.launch.py", "use_sim_time:=false"] and rviz.isChecked()
    procs.stop("rviz")  # the user closed the RViz window itself
    assert not rviz.isChecked() and procs.alive == {"system"}

    rviz.click()
    system._stop.click()  # stopping the system closes its windows
    assert procs.alive == set() and not rviz.isChecked() and not rviz.isEnabled()


def test_real_hardware_is_not_selectable_yet(window):
    combo = window.system._hardware
    assert not combo.model().item(2).isEnabled()


def test_external_windows_are_placed_once_when_they_appear(window):
    window.apply_targets({"gazebo": Rect(0, 0, 400, 300), "rviz": Rect(400, 0, 400, 300)}, move_app=False)
    window._placer.open = {"gazebo": ("0x1", Rect(9, 9, 9, 9))}
    window._place_external_windows()
    window._place_external_windows()
    assert [m[0] for m in window._placer.moves] == ["0x1"]


def test_saved_layout_round_trip(window):
    window._targets = {"rviz": Rect(10, 20, 300, 200)}
    window.save_layout("δοκιμή")
    assert "δοκιμή" in window._saved_layout_names()
    window._targets = {}
    window.load_layout("δοκιμή")
    assert "rviz" in window._targets
    window.delete_layout("δοκιμή")
    assert window._saved_layout_names() == []


CAL_PARAMS = [
    {"name": "file", "type": "string", "value": "/x/arm.yaml", "description": "Αρχείο", "read_only": True,
     "min": None, "max": None},
    {"name": "servos.servo_7.neutral_us", "type": "integer", "value": 1600, "description": "Νεκρό σημείο",
     "read_only": False, "min": 500, "max": 2500},
    {"name": "servos.servo_7.description", "type": "string", "value": "καρπός", "description": "Περιγραφή",
     "read_only": False, "min": None, "max": None},
    {"name": "safety.max_duration_s", "type": "double", "value": 3.0, "description": "Διάρκεια",
     "read_only": False, "min": 0.1, "max": 30.0},
    *({"name": f"joints.{j}.zero_deg", "type": "double", "value": 0.0, "description": "Μηδέν",
       "read_only": False, "min": 0.0, "max": 360.0} for j in ("joint_1", "joint_2", "joint_3")),
    *({"name": f"joints.{j}.encoder", "type": "string", "value": f"enc_{j[-1]}", "description": "Encoder",
       "read_only": False, "min": None, "max": None} for j in ("joint_1", "joint_2", "joint_3")),
]


def test_calibration_node_is_opened_automatically_and_edits_are_applied(window):
    b = window._bridge
    b.nodes_changed.emit(["/arm_calibration", "/arm_gui", "/controller_manager"])
    assert b.param_requests == ["/arm_calibration"]
    b.parameters_loaded.emit("/arm_calibration", CAL_PARAMS)
    panel = window.parameters
    editor = panel._editors["servos.servo_7.neutral_us"]
    assert editor.widget.suffix() == " µs" and editor.widget.maximum() == 2500
    assert not panel._editors["file"].widget.isEnabled()  # read-only
    editor.widget.setValue(1605)
    assert panel._apply.isEnabled() and not panel._save.isEnabled()  # apply before saving
    panel.apply()
    assert b.param_sets[-1] == ("/arm_calibration", {"servos.servo_7.neutral_us": 1605})
    b.parameters_set.emit("/arm_calibration", [("servos.servo_7.neutral_us", True, "")])
    assert not panel._apply.isEnabled() and panel._save.isEnabled()
    panel._save.click()
    assert b.triggers == ["/arm_calibration/save"]


def test_encoder_zeros_for_the_ticked_joints_then_reloaded(window):
    b, panel = window._bridge, window.parameters
    b.nodes_changed.emit(["/arm_calibration"])
    b.parameters_loaded.emit("/arm_calibration", CAL_PARAMS)
    assert panel._set_zero.isVisibleTo(panel)
    joints = panel.zero_joints()
    assert joints and all(not j.startswith("servos") for j in joints)
    b.diagnostics.emit([{"name": f"encoder/{joints[0]}", "level": 0, "message": "OK", "hardware_id": "",
                         "values": {"raw_deg": "10.0"}}])
    seen = {}
    choices = [None, joints, [joints[0]]]
    panel.choose_joints = lambda js, states: seen.update(states) or choices.pop(0)
    panel._set_zero.click()  # cancelled: nothing is sent
    assert b.triggers == []
    assert seen[joints[0]] == "ok" and all(seen[j] == "silent" for j in joints[1:])
    panel._set_zero.click()  # every joint: one all-or-none call
    assert b.triggers == ["/arm_calibration/set_zero"]
    panel.show_service_result("/arm_calibration/set_zero", False, "Δεν άλλαξε κανένα μηδέν: joint_1: ο encoder δεν έχει αναφέρει")
    assert "joint_1" in panel._status.text() and b.param_requests == ["/arm_calibration"]
    panel._set_zero.click()  # only the joints that report (before every encoder is connected)
    assert b.triggers[-1] == f"/arm_calibration/set_zero/{joints[0]}"
    panel.show_service_result(f"/arm_calibration/set_zero/{joints[0]}", True, "Νέα μηδενικά: …")
    assert panel._status.text().startswith(f"✔ {joints[0]}:")
    assert b.param_requests == ["/arm_calibration", "/arm_calibration"]  # shows the new zeros
    panel._editors["servos.servo_7.neutral_us"].widget.setValue(1605)
    assert not panel._set_zero.isEnabled()  # unapplied edits first


def test_rejected_parameter_stays_modified_and_shows_the_reason(window):
    b = window._bridge
    b.nodes_changed.emit(["/arm_calibration"])
    b.parameters_loaded.emit("/arm_calibration", CAL_PARAMS)
    window.parameters._editors["safety.max_duration_s"].widget.setValue(5.0)
    window.parameters.apply()
    b.parameters_set.emit("/arm_calibration", [("safety.max_duration_s", False, "εκτός ορίων")])
    assert window.parameters._editors["safety.max_duration_s"].modified
    assert "εκτός ορίων" in window.parameters._status.text()


def test_graph_panel_lists_nodes_topics_and_node_details(window):
    window._bridge.graph_changed.emit({
        "nodes": ["/arm_calibration", "/controller_manager"],
        "topics": [("/joint_states", ["sensor_msgs/msg/JointState"], 1, 2)],
    })
    g = window.graph
    assert g._nodes.count() == 2 and g._topics.rowCount() == 1
    g._nodes.setCurrentRow(1)
    assert g._info.topLevelItem(0).text(0).startswith("Δημοσιεύει")
    g._filter.setText("calib")
    assert g._nodes.count() == 1 and g._topics.rowCount() == 0


def test_theme_can_be_switched_and_is_remembered(window):
    from arm_gui import theme

    window.set_theme("light")
    assert theme.current().name == "light"
    assert window._settings.value("ui/theme") == "light"
    window.set_theme("dark")


def _status(name, level=0, message="", **values):
    return {"name": name, "level": level, "message": message, "hardware_id": "", "values": values}


def test_devices_panel_shows_which_devices_are_active(window):
    b, panel = window._bridge, window.devices
    now = [100.0]
    panel._clock = lambda: now[0]
    params = [
        {"name": f"servos.{s}.channel", "type": "integer", "value": 9, "description": "", "read_only": False,
         "min": 0, "max": 15}
        for s in ("wrist_diff_a", "wrist_diff_b")
    ] + [{"name": "joints.joint_5.encoder", "type": "string", "value": "enc_5", "description": "", "read_only": False,
          "min": None, "max": None},
         {"name": "joints.joint_5.enabled", "type": "bool", "value": True, "description": "", "read_only": False,
          "min": None, "max": None}]
    window.parameters._node = "/arm_calibration"
    b.parameters_loaded.emit("/arm_calibration", params)
    assert panel.counts() == {"Servos": (0, 2), "Encoders": (0, 1)}  # configured, never reported
    assert panel.state_of("servo/wrist_diff_a") == "expected"

    b.diagnostics.emit([
        _status("servo/wrist_diff_a", pulse_us="1600"),
        _status("encoder/joint_5", 1, "ασθενής μαγνήτης"),
        _status("sensor/ina219", 2, "δεν απαντά στο I2C"),
        _status("controller_manager: arm_controller"),
    ])
    assert panel.counts() == {"Servos": (1, 2), "Encoders": (1, 1), "Αισθητήρες": (1, 1), "Άλλα": (1, 1)}
    assert [panel.state_of(n) for n in ("servo/wrist_diff_a", "encoder/joint_5", "sensor/ina219")] == [
        "ok", "warn", "error"]
    assert "Servos</b> 1/2" in panel._summary.text()

    now[0] += 10  # reports stopped: the devices stay listed, as silent
    b.diagnostics.emit([_status("servo/wrist_diff_a")])
    assert panel.state_of("servo/wrist_diff_a") == "ok"
    assert panel.state_of("encoder/joint_5") == "silent"
    assert panel.counts()["Encoders"] == (0, 1)

    panel._only_active.setChecked(True)
    shown = [panel._tree.topLevelItem(i).child(j).text(0)
             for i in range(panel._tree.topLevelItemCount()) for j in range(panel._tree.topLevelItem(i).childCount())]
    assert shown == ["wrist_diff_a"]


def test_recording_runs_only_with_the_system_and_is_logged(window, tmp_path, monkeypatch):
    monkeypatch.setenv("ARM_DATA_DIR", str(tmp_path))
    procs, b, rec = window._processes, window._bridge, window.rec_button
    assert not rec.isEnabled()  # nothing to record yet
    window.system._auto_open.setChecked(False)
    window.system._start.click()
    assert rec.isEnabled()

    window.ask_note = lambda: None  # cancelled: nothing starts
    rec.click()
    assert not rec.isChecked() and "record" not in procs.alive

    window.ask_note = lambda: "PID joint_2"
    rec.click()
    assert rec.isChecked() and "record" in procs.alive
    cmd = procs.started[-1]
    assert cmd[:4] == ["ros2", "bag", "record", "-o"] and "/joint_states" in cmd
    path = cmd[4]
    assert path.startswith(str(tmp_path / "recordings")) and (tmp_path / "recordings").is_dir()
    assert b.events[-1]["kind"] == "recording_start" and b.events[-1]["note"] == "PID joint_2"

    procs.stop("system")  # stopping the system also ends the recording
    assert not rec.isChecked() and not rec.isEnabled() and "record" not in procs.alive
    assert b.events[-1] == {"kind": "recording_stop", "path": path}


def test_tools_catalog_is_valid_and_pi_tools_go_through_ssh(tmp_path):
    from arm_gui.tools import PiTarget, command_for, load_catalog
    tools = {t.id: t for t in load_catalog()}
    assert {"i2c_scan", "outputs_off", "controllers", "logbook_position"} <= set(tools)
    assert tools["probe_channels"].moves and not tools["i2c_scan"].moves
    assert command_for(tools["controllers"], PiTarget(), tmp_path) == ("bash", ["-c", "ros2 control list_controllers"])
    with pytest.raises(ValueError):
        command_for(tools["i2c_scan"], PiTarget(), tmp_path)  # no Pi configured
    program, args = command_for(tools["i2c_scan"], PiTarget("pi.local", "fotis", "Master-s-Thesis"), tmp_path)
    assert program == "ssh" and "BatchMode=yes" in args and "fotis@pi.local" in args
    assert args[-1] == "cd ~/Master-s-Thesis && python3 -u software/tools/wrist_bench/wrist_bench.py scan"
    assert command_for(tools["i2c_scan"], PiTarget("h", "u", "/opt/repo"), tmp_path)[1][-1].startswith("cd /opt/repo && ")


def test_tools_panel_runs_the_selected_tool_and_asks_before_moving(window, tmp_path, monkeypatch):
    monkeypatch.setenv("ARM_DATA_DIR", str(tmp_path))
    panel, procs = window.tools, window._processes
    panel._filter.setText("i2c")
    panel.select("i2c_scan")
    assert panel.selected().id == "i2c_scan"
    conn = window.connection
    conn._user.setText("")
    conn._on_edited()
    panel.run_selected()  # no Pi user yet: nothing starts
    assert "tool" not in procs.alive and "Σύνδεση" in panel._output.toPlainText()

    conn._host.setText("pi.local")
    conn._user.setText("fotis")
    conn._on_edited()  # set once in the Connection panel, used by the Tools panel
    assert window._settings.value("pi/host") == "pi.local"  # remembered for next time
    panel.run_selected()
    assert procs.started[-1][0] == "ssh" and "tool" in procs.alive and not panel._run.isEnabled()
    procs.output.emit("tool", "mux7: 0x36 0x48")
    assert "0x48" in panel._output.toPlainText()
    procs.stop("tool")
    assert panel._run.isEnabled()

    panel._filter.setText("")
    panel.select("probe_channels")
    panel.confirm = lambda *_: False
    n = len(procs.started)
    panel.run_selected()
    assert len(procs.started) == n  # moving tool refused
    panel.confirm = lambda *_: True
    panel.run_selected()
    assert procs.started[-1][-1].endswith("wrist_bench.py probe")


def test_connection_panel_reports_each_step(window, monkeypatch):
    from arm_gui import link
    panel = window.connection
    panel._run_async = False
    assert panel._host.text() == ""  # found by "Εύρεση και έλεγχος" (UDP discovery)
    monkeypatch.setattr(link, "has_local_address", lambda ip: True)
    monkeypatch.setattr(link, "port_open", lambda ip, port=22, timeout=1.0: False)
    monkeypatch.setattr(link, "find_pi", lambda name="armpi": ("192.168.1.20", "δίκτυο (armpi.local)"))
    monkeypatch.setattr(link, "check_ssh", lambda host, user, d: (True, f"SSH χωρίς κωδικό προς {host}"))
    monkeypatch.setattr(link, "measure_link", lambda host, count, timeout_s: (FakeStats([300_000] * count), ""))
    panel._user.setText("fotis")
    panel._on_edited()
    panel.check_all()
    assert panel._pills["cable"].property("state") == "ok"
    assert panel._pills["pi"].text() == "192.168.1.20" and panel._host.text() == "192.168.1.20"
    assert window.tools.pi.host == "192.168.1.20"  # the Tools panel follows
    assert panel._pills["udp"].property("state") == "ok" and "0 % απώλειες" in panel._texts["udp"].text()
    assert panel._pills["ssh"].property("state") == "ok"

    monkeypatch.setattr(link, "find_pi", lambda name="armpi": (None, ""))
    panel._host.setText("")
    panel._on_edited()
    panel.check_all()
    assert panel._pills["pi"].property("state") == "warn"
    assert "εύρεση UDP" in panel._texts["pi"].text() and panel._pills["udp"].text() == "—"

    calls = []
    monkeypatch.setattr(link, "install_key", lambda h, u, p, d: calls.append((h, u, p)) or (True, "ok"))
    panel._host.setText("10.42.0.2")
    panel._on_edited()
    panel.ask_password = lambda: None  # cancelled
    panel.setup_key()
    panel.ask_password = lambda: "secret"
    panel.setup_key()
    assert calls == [("10.42.0.2", "fotis", "secret")]

    window._bridge.nodes_changed.emit(["/arm_pi_driver", "/arm_gui"])
    assert panel._pills["ros"].property("state") == "ok"


class FakeStats:
    """Stands in for arm_link.stats.LinkStats: rtts in ns, `lost` pings without an answer."""

    def __init__(self, rtts_ns, lost=0):
        self.rtts_ns = list(rtts_ns)
        self.sent_count = len(self.rtts_ns) + lost
        self.pending = set(range(lost))
        self.loss_percent = 100.0 * lost / self.sent_count if self.sent_count else 0.0
        ms = [r / 1e6 for r in self.rtts_ns] or [0.0]
        self.avg_ms, self.max_ms, self.jitter_ms = sum(ms) / len(ms), max(ms), 0.0


def test_connection_panel_measures_the_link(window, monkeypatch):
    from arm_gui import link
    panel = window.connection
    panel._run_async = False
    panel._host.setText("")
    panel._on_edited()
    panel.measure()
    assert panel._pills["udp"].property("state") == "warn"  # no host yet

    panel._host.setText("10.42.0.116")
    panel._on_edited()
    results = iter([
        (FakeStats([170_000, 400_000]), ""),
        (FakeStats([170_000], lost=1), ""),
        (FakeStats([170_000, 25_000_000]), ""),
        (FakeStats([], lost=5), ""),
        (None, link.NO_ARM_LINK),
    ])
    monkeypatch.setattr(link, "measure_link", lambda host, count, timeout_s: next(results))
    states = []
    for _ in range(5):
        panel.measure()
        states.append((panel._pills["udp"].property("state"), panel._pills["udp"].text()))
        assert panel._measure.isEnabled()
    assert states == [("ok", "καλή"), ("warn", "ασταθής"), ("warn", "ασταθής"),
                      ("warn", "δεν απαντά"), ("warn", "όχι")]
    assert "arm_link" in panel._texts["udp"].text()



def test_five_sections_in_the_side_bar(window):
    assert window.section == "run"
    assert window.nav.count() == 5
    window.show_section("cal")
    assert window.stack.currentWidget() is window.calibration
    window.show_section("adv")
    titles = [window.advanced.tabText(i) for i in range(window.advanced.count())]
    assert titles == ["Παράμετροι", "ROS 2", "Εργαλεία", "Log", "Αισθητήρες (raw)"]
    window.save_layout("x")
    window.show_section("run")
    window.load_layout("x")
    assert window.section == "adv"                                       # the section is part of a layout


def test_messages_show_only_changes_of_devices(window):
    b, strip = window._bridge, window.messages
    status = {"name": "encoder/joint_1", "level": 2, "message": "NO_MAGNET", "hardware_id": "", "values": {}}
    b.diagnostics.emit([status])
    b.diagnostics.emit([status])                                          # same state again: no new message
    assert sum("encoder/joint_1: NO_MAGNET" in t for t in strip.texts()) == 1
    assert window._warn_pill.text() == "1 προειδοποίηση"
    b.diagnostics.emit([{**status, "level": 0, "message": "OK"}])
    assert "encoder/joint_1: εντάξει" in strip.texts()[-1]
    assert window._warn_pill.text() == "Καμία προειδοποίηση"
