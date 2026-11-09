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

    def __init__(self):
        super().__init__()
        self.goals, self.stops = [], 0
        self.param_requests, self.param_sets, self.triggers = [], [], []
        self.events = []

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
    {"name": "file", "type": "string", "value": "/x/calibration.yaml", "description": "Αρχείο", "read_only": True,
     "min": None, "max": None},
    {"name": "servos.wrist_diff_a.neutral_us", "type": "integer", "value": 1600, "description": "Νεκρό σημείο",
     "read_only": False, "min": 500, "max": 2500},
    {"name": "servos.wrist_diff_a.invert", "type": "bool", "value": False, "description": "Αντίστροφη φορά",
     "read_only": False, "min": None, "max": None},
    {"name": "safety.max_duration_s", "type": "double", "value": 3.0, "description": "Διάρκεια",
     "read_only": False, "min": 0.1, "max": 30.0},
]


def test_calibration_node_is_opened_automatically_and_edits_are_applied(window):
    b = window._bridge
    b.nodes_changed.emit(["/arm_calibration", "/arm_gui", "/controller_manager"])
    assert b.param_requests == ["/arm_calibration"]
    b.parameters_loaded.emit("/arm_calibration", CAL_PARAMS)
    panel = window.parameters
    editor = panel._editors["servos.wrist_diff_a.neutral_us"]
    assert editor.widget.suffix() == " µs" and editor.widget.maximum() == 2500
    assert not panel._editors["file"].widget.isEnabled()  # read-only
    editor.widget.setValue(1605)
    assert panel._apply.isEnabled() and not panel._save.isEnabled()  # apply before saving
    panel.apply()
    assert b.param_sets[-1] == ("/arm_calibration", {"servos.wrist_diff_a.neutral_us": 1605})
    b.parameters_set.emit("/arm_calibration", [("servos.wrist_diff_a.neutral_us", True, "")])
    assert not panel._apply.isEnabled() and panel._save.isEnabled()
    panel._save.click()
    assert b.triggers == ["/arm_calibration/save"]


def test_encoder_zeros_are_set_only_after_confirmation_and_then_reloaded(window):
    b, panel = window._bridge, window.parameters
    b.nodes_changed.emit(["/arm_calibration"])
    b.parameters_loaded.emit("/arm_calibration", CAL_PARAMS)
    assert panel._set_zero.isVisibleTo(panel)
    answers = [False, True]
    panel.confirm = lambda _title, _text: answers.pop(0)
    panel._set_zero.click()  # "no": nothing is sent
    assert b.triggers == []
    panel._set_zero.click()
    assert b.triggers == ["/arm_calibration/set_zero"]
    panel.show_service_result("/arm_calibration/set_zero", False, "Δεν άλλαξε κανένα μηδέν: joint_1: ο encoder δεν έχει αναφέρει")
    assert "joint_1" in panel._status.text() and b.param_requests == ["/arm_calibration"]
    panel.show_service_result("/arm_calibration/set_zero", True, "Νέα μηδενικά: …")
    assert b.param_requests == ["/arm_calibration", "/arm_calibration"]  # shows the new zeros
    panel._editors["servos.wrist_diff_a.neutral_us"].widget.setValue(1605)
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
    ] + [{"name": "encoders.joint_5.zero_deg", "type": "double", "value": 0.0, "description": "", "read_only": False,
          "min": 0.0, "max": 360.0}]
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
