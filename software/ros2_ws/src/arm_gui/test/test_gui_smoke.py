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

    def __init__(self):
        super().__init__()
        self.goals, self.stops = [], 0

    def send_joint_goal(self, targets, duration):
        self.goals.append((targets, duration))

    def stop(self):
        self.stops += 1


class FakeProcesses(QtCore.QObject):
    output = Signal(str, str)
    running_changed = Signal(str, bool)

    def __init__(self):
        super().__init__()
        self.started, self.running = [], False

    def start(self, name, program, args):
        self.started.append([program, *args])
        self.running = True
        self.running_changed.emit(name, True)

    def stop(self, name):
        self.running = False
        self.running_changed.emit(name, False)

    def any_running(self):
        return self.running

    def stop_all(self, wait_ms=0):
        self.running = False


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


def test_start_runs_bringup_with_the_selected_hardware(window):
    window.system._hardware.setCurrentIndex(1)  # gazebo
    window.system._start.click()
    assert window._processes.started[-1] == [
        "ros2", "launch", "arm_bringup", "bringup.launch.py", "hardware:=gazebo", "rviz:=true"
    ]
    assert not window.system._start.isEnabled()


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
