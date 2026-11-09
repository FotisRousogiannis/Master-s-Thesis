"""Main window: dockable panels, toolbar with STOP, and the layout of the three windows (ADR-0016)."""

import json
import os
import time
from pathlib import Path

from . import APP_NAME, theme
from .layouts import APP, PRESETS, Rect, WindowPlacer, fit_to_screens, preset_layout
from .model import parse_joints
from .panels.devices_panel import DevicesPanel
from .panels.graph_panel import GraphPanel
from .panels.joints_panel import JointsPanel
from .panels.log_panel import LogPanel
from .panels.parameters_panel import CALIBRATION_NODE, ParametersPanel
from .panels.sensors_panel import SensorsPanel
from .panels.system_panel import SystemPanel
from .panels.tools_panel import ToolsPanel
from .processes import ProcessManager
from .qt import QtCore, QtGui, QtWidgets, Qt
from .tools import PiTarget, command_for

SYSTEM_PROCESS = "system"
WINDOW_PROCESSES = ("rviz", "gazebo")  # same keys as the window layouts
RECORD_PROCESS = "record"
TOOL_PROCESS = "tool"
# What a recording keeps (ADR-0020): enough to replay the motion in RViz and plot it.
RECORD_TOPICS = [
    "/joint_states", "/tf", "/tf_static", "/robot_description", "/diagnostics",
    "/arm_controller/joint_trajectory", "/arm_controller/controller_state", "/clock",
]
ICON = Path(__file__).resolve().parent / "resources" / "arm_studio.png"
PLACE_POLL_MS = 1000
STOP_WAIT_ON_EXIT_MS = 10000


def config_dir() -> Path:
    """Outside the container image when ARM_GUI_CONFIG_DIR is set (compose.yaml), so layouts survive `docker compose down`."""
    return Path(os.environ.get("ARM_GUI_CONFIG_DIR", Path.home() / ".config" / "arm_gui"))


def data_dir() -> Path:
    """Logbook and recordings (ADR-0020): <repo>/.local in Docker (ARM_DATA_DIR), on the host disk, outside git."""
    return Path(os.environ.get("ARM_DATA_DIR", Path.home() / ".local" / "share" / "arm_studio"))


def _rect(q) -> Rect:
    return Rect(q.x(), q.y(), q.width(), q.height())


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, bridge, processes=None, placer=None, settings_file=None):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} — ρομποτικός βραχίονας")
        self.setWindowIcon(QtGui.QIcon(str(ICON)))
        self._bridge = bridge
        self._processes = processes or ProcessManager(self)
        self._placer = placer or WindowPlacer()
        settings_file = settings_file or str(config_dir() / "settings.ini")
        self._settings = QtCore.QSettings(settings_file, QtCore.QSettings.IniFormat)
        theme.apply(self._settings.value("ui/theme", "dark"))
        self._targets: dict[str, Rect] = {}  # where the external windows should go
        self._placed: set[str] = set()  # window ids already placed for the current targets
        self._hardware = "mock"
        self._recording = None  # path of the running recording

        self.system = SystemPanel()
        self.joints = JointsPanel()
        self.sensors = SensorsPanel()
        self.devices = DevicesPanel()
        self.parameters = ParametersPanel()
        self.graph = GraphPanel(bridge.node_info)
        self.log = LogPanel()
        self.tools = ToolsPanel()
        self._docks = {}
        self._build_docks()
        self._build_toolbar()
        self._build_menus()
        self._connect()

        self._place_timer = QtCore.QTimer(self)
        self._place_timer.timeout.connect(self._place_external_windows)
        self._place_timer.start(PLACE_POLL_MS)
        self._restore_session()
        self._load_pi()

    # ------------------------------------------------------------------ construction
    def _build_docks(self):
        self.setDockOptions(
            QtWidgets.QMainWindow.AnimatedDocks
            | QtWidgets.QMainWindow.AllowNestedDocks
            | QtWidgets.QMainWindow.AllowTabbedDocks
            | QtWidgets.QMainWindow.GroupedDragging
        )
        for key, title, widget in [
            ("system", "Σύστημα", self.system),
            ("joints", "Αρθρώσεις", self.joints),
            ("sensors", "Αισθητήρες", self.sensors),
            ("devices", "Συσκευές", self.devices),
            ("parameters", "Παράμετροι", self.parameters),
            ("graph", "ROS 2", self.graph),
            ("tools", "Εργαλεία", self.tools),
            ("log", "Log", self.log),
        ]:
            dock = QtWidgets.QDockWidget(title, self)
            dock.setObjectName(f"dock_{key}")  # needed by saveState/restoreState
            dock.setWidget(widget)
            self._docks[key] = dock
        self._default_dock_arrangement()

    def _default_dock_arrangement(self):
        d = self._docks
        for dock in d.values():
            dock.setFloating(False)
            dock.show()
            self.removeDockWidget(dock)
        self.addDockWidget(Qt.LeftDockWidgetArea, d["system"])
        self.splitDockWidget(d["system"], d["joints"], Qt.Horizontal)
        self.splitDockWidget(d["joints"], d["sensors"], Qt.Horizontal)
        for key in ("devices", "parameters", "graph", "tools", "log"):
            self.tabifyDockWidget(d["sensors"], d[key])
        for dock in d.values():
            dock.show()
        d["sensors"].raise_()

    def _build_toolbar(self):
        bar = QtWidgets.QToolBar("Κύρια")
        bar.setObjectName("toolbar_main")
        bar.setMovable(False)
        bar.setFloatable(False)
        bar.toggleViewAction().setEnabled(False)  # STOP must always be visible
        self.addToolBar(Qt.TopToolBarArea, bar)

        logo = QtWidgets.QLabel()
        logo.setPixmap(QtGui.QIcon(str(ICON)).pixmap(36, 36))
        bar.addWidget(logo)
        titles = QtWidgets.QWidget()
        tl = QtWidgets.QVBoxLayout(titles)
        tl.setContentsMargins(0, 0, 12, 0)
        tl.setSpacing(0)
        title = QtWidgets.QLabel(APP_NAME)
        title.setObjectName("app_title")
        subtitle = QtWidgets.QLabel("Ρομποτικός βραχίονας 6 DOF")
        subtitle.setObjectName("app_subtitle")
        tl.addWidget(title)
        tl.addWidget(subtitle)
        bar.addWidget(titles)

        self._system_pill = self._pill()
        self._ros_pill = self._pill()
        bar.addWidget(self._system_pill)
        bar.addWidget(self._ros_pill)
        self._motion_status = QtWidgets.QLabel("")
        self._motion_status.setProperty("muted", True)
        bar.addWidget(self._motion_status)

        spacer = QtWidgets.QWidget()
        spacer.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        bar.addWidget(spacer)

        self.rec_button = QtWidgets.QPushButton("●  REC")
        self.rec_button.setObjectName("rec_button")
        self.rec_button.setCheckable(True)
        self.rec_button.setEnabled(False)
        self.rec_button.setToolTip("Καταγραφή (rosbag) των αρθρώσεων, του tf και των διαγνωστικών· μόνο όσο τρέχει το σύστημα")
        self.rec_button.toggled.connect(self.set_recording)
        bar.addWidget(self.rec_button)

        self.stop_button = QtWidgets.QPushButton("■  STOP  (Esc)")
        self.stop_button.setObjectName("stop_button")
        self.stop_button.setToolTip(
            "Ακύρωση κίνησης και κράτημα θέσης. Λογισμικό — ΔΕΝ αντικαθιστά το E-stop της τροφοδοσίας."
        )
        self.stop_button.clicked.connect(self._bridge.stop)
        QtWidgets.QShortcut(QtGui.QKeySequence(Qt.Key_Escape), self, activated=self._bridge.stop)
        bar.addWidget(self.stop_button)
        self._set_pill(self._system_pill, "off", "Σύστημα σταματημένο")
        self._set_pill(self._ros_pill, "off", "ROS 2: —")

    @staticmethod
    def _pill():
        label = QtWidgets.QLabel()
        label.setProperty("pill", True)
        return label

    @staticmethod
    def _set_pill(label, state, text):
        label.setText(text)
        if label.property("state") != state:
            label.setProperty("state", state)
            theme.repolish(label)

    def _build_menus(self):
        file_menu = self.menuBar().addMenu("&Αρχείο")
        file_menu.addAction("Έξοδος", self.close, QtGui.QKeySequence.Quit)

        view = self.menuBar().addMenu("&Προβολή")
        for dock in self._docks.values():
            view.addAction(dock.toggleViewAction())
        view.addSeparator()
        view.addAction("Επαναφορά πάνελ", self._default_dock_arrangement)
        view.addSeparator()
        themes = QtWidgets.QActionGroup(self)
        for key, label in (("dark", "Σκούρο θέμα"), ("light", "Ανοιχτό θέμα")):
            action = view.addAction(label, lambda k=key: self.set_theme(k))
            action.setCheckable(True)
            action.setChecked(theme.current().name == key)
            themes.addAction(action)

        self._layout_menu = self.menuBar().addMenu("&Διάταξη")
        self._rebuild_layout_menu()

    def _rebuild_layout_menu(self):
        menu = self._layout_menu
        menu.clear()
        for key, label in PRESETS.items():
            menu.addAction(label, lambda k=key: self.apply_preset(k))
        saved = self._saved_layout_names()
        if saved:
            menu.addSeparator()
            for name in saved:
                menu.addAction(f"★ {name}", lambda n=name: self.load_layout(n))
        menu.addSeparator()
        menu.addAction("Αποθήκευση τρέχουσας διάταξης…", self._ask_save_layout)
        delete = menu.addMenu("Διαγραφή διάταξης")
        delete.setEnabled(bool(saved))
        for name in saved:
            delete.addAction(name, lambda n=name: self.delete_layout(n))
        if not self._placer.available:
            note = menu.addAction("(wmctrl δεν βρέθηκε: Gazebo/RViz δεν μετακινούνται)")
            note.setEnabled(False)

    def _connect(self):
        b = self._bridge
        b.description_received.connect(self._on_description)
        b.joint_state.connect(self.joints.set_current)
        b.joint_state.connect(self.sensors.set_joint_state)
        b.controllers_changed.connect(self.system.set_controllers)
        b.nodes_changed.connect(self.parameters.set_nodes)
        b.nodes_changed.connect(self._on_nodes)
        b.graph_changed.connect(self.graph.set_graph)
        b.parameters_loaded.connect(self.parameters.show_parameters)
        b.parameters_loaded.connect(self._on_parameters)
        b.diagnostics.connect(self.devices.update_diagnostics)
        b.parameters_set.connect(self.parameters.show_set_results)
        b.service_result.connect(self.parameters.show_service_result)
        b.service_result.connect(lambda srv, ok, msg: self.log.append("υπηρεσία", f"{srv}: {msg}"))
        self.parameters.load_requested.connect(b.request_parameters)
        self.parameters.apply_requested.connect(b.set_parameters)
        self.parameters.service_requested.connect(b.call_trigger)
        b.goal_status.connect(self._on_goal_status)
        b.log.connect(lambda line: self.log.append("app", line))
        self.joints.goal_requested.connect(b.send_joint_goal)
        self.system.start_requested.connect(self.start_system)
        self.system.window_requested.connect(self.set_window)
        self.tools.run_requested.connect(self.run_tool)
        self.tools.stop_requested.connect(lambda: self._processes.stop(TOOL_PROCESS))
        self.tools.pi_changed.connect(self._save_pi)
        self._processes.output.connect(lambda name, line: self.tools.append(line) if name == TOOL_PROCESS else None)
        self.system.stop_requested.connect(lambda: self._processes.stop(SYSTEM_PROCESS))
        self._processes.output.connect(self.log.append)
        self._processes.running_changed.connect(self._on_process_running)

    # ------------------------------------------------------------------ system
    def start_system(self, hardware: str, open_windows: bool):
        """The stack starts without windows; RViz and Gazebo are separate programs, opened and closed on demand."""
        self._hardware = hardware
        self._processes.start(
            SYSTEM_PROCESS,
            "ros2",
            ["launch", "arm_bringup", "bringup.launch.py", f"hardware:={hardware}", "rviz:=false", "gui:=false"],
        )
        if open_windows:
            self.set_window("rviz", True)
            if hardware == "gazebo":
                self.set_window("gazebo", True)

    def set_window(self, key: str, wanted: bool):
        if not wanted:
            self._processes.stop(key)
        elif not self._processes.is_running(key):
            self._placed.clear()  # a new window: place it again
            launch = {
                "rviz": ["rviz.launch.py", f"use_sim_time:={str(self._hardware == 'gazebo').lower()}"],
                "gazebo": ["gazebo_gui.launch.py"],
            }[key]
            self._processes.start(key, "ros2", ["launch", "arm_bringup", *launch])

    # ------------------------------------------------------------------ tools
    def run_tool(self, tool):
        try:
            program, args = command_for(tool, self.tools.pi, data_dir() / "ssh")
        except ValueError as exc:
            self.tools.append(f"✖ {exc}")
            return
        self._processes.start(TOOL_PROCESS, program, args)

    def _save_pi(self, pi: PiTarget):
        for key in ("host", "user", "repo"):
            self._settings.setValue(f"pi/{key}", getattr(pi, key))

    def _load_pi(self):
        s = self._settings
        self.tools.set_pi(PiTarget(s.value("pi/host", PiTarget.host), s.value("pi/user", ""), s.value("pi/repo", PiTarget.repo)))

    # ------------------------------------------------------------------ recording (ADR-0020)
    def set_recording(self, wanted: bool):
        if not wanted:
            self._processes.stop(RECORD_PROCESS)
            return
        if self._processes.is_running(RECORD_PROCESS):
            return
        note = self.ask_note()
        if note is None:
            self._set_rec_checked(False)
            return
        path = data_dir() / "recordings" / time.strftime("%Y-%m-%d_%H-%M-%S")
        path.parent.mkdir(parents=True, exist_ok=True)
        self._recording = str(path)
        self._processes.start(RECORD_PROCESS, "ros2", ["bag", "record", "-o", str(path), *RECORD_TOPICS])
        self._bridge.log_event({"kind": "recording_start", "path": str(path), "note": note, "topics": RECORD_TOPICS})

    def ask_note(self):
        """What is being tested; None = cancelled."""
        text, ok = QtWidgets.QInputDialog.getText(self, "Καταγραφή", "Σημείωση (τι δοκιμάζεις, ρυθμίσεις):")
        return text.strip() if ok else None

    def _set_rec_checked(self, checked: bool):
        self.rec_button.blockSignals(True)
        self.rec_button.setChecked(checked)
        self.rec_button.blockSignals(False)

    def _on_process_running(self, name, running):
        if name == TOOL_PROCESS:
            self.tools.set_running(running)
            if not running:
                self.tools.append("— τέλος —")
        elif name == RECORD_PROCESS:
            self._set_rec_checked(running)
            if not running and self._recording:
                self._bridge.log_event({"kind": "recording_stop", "path": self._recording})
                self.log.append("app", f"Καταγραφή αποθηκεύτηκε: {self._recording}")
                self._recording = None
        elif name == SYSTEM_PROCESS:
            self.system.set_running(running)
            self._set_pill(self._system_pill, "ok" if running else "off",
                           "Σύστημα σε λειτουργία" if running else "Σύστημα σταματημένο")
            self.rec_button.setEnabled(running)
            if not running:
                for key in (*WINDOW_PROCESSES, RECORD_PROCESS):
                    self._processes.stop(key)
        elif name in WINDOW_PROCESSES:
            self.system.set_window_open(name, running)

    def _on_nodes(self, nodes):
        others = [n for n in nodes if n != "/arm_gui"]
        self._set_pill(self._ros_pill, "ok" if others else "off", f"ROS 2: {len(others)} nodes")

    def _on_parameters(self, node, params):
        """The servos and encoders of the calibration are devices we expect to hear from."""
        if node == CALIBRATION_NODE and not isinstance(params, str):
            expected = set()
            for p in params:
                group, *rest = p["name"].split(".")
                if group in ("servos", "encoders") and rest:
                    expected.add(f"{group[:-1]}/{rest[0]}")
            self.devices.set_expected(sorted(expected))

    def set_theme(self, name: str):
        theme.apply(name)
        self._settings.setValue("ui/theme", name)
        self.system.set_controllers(self.system.controllers)

    def _on_description(self, urdf):
        try:
            self.joints.set_joints(parse_joints(urdf))
        except Exception as exc:  # malformed model: report, keep running
            self.log.append("app", f"Μη έγκυρο /robot_description: {exc}")

    def _on_goal_status(self, text):
        self._motion_status.setText(text)
        self.joints.set_status(text)
        self.log.append("κίνηση", text)

    # ------------------------------------------------------------------ layouts
    def screens(self) -> list[Rect]:
        primary = QtGui.QGuiApplication.primaryScreen()
        others = [s for s in QtGui.QGuiApplication.screens() if s is not primary]
        return [_rect(s.availableGeometry()) for s in [primary, *others]]

    def apply_preset(self, key: str):
        self.apply_targets(preset_layout(key, self.screens()))

    def apply_targets(self, targets: dict[str, Rect], move_app: bool = True):
        screens = self.screens()
        targets = {k: fit_to_screens(r, screens) for k, r in targets.items()}
        if move_app and APP in targets:
            self._set_frame_geometry(targets[APP])
        self._targets = {k: r for k, r in targets.items() if k != APP}
        self._placed.clear()
        self._place_external_windows()

    def _set_frame_geometry(self, rect: Rect):
        """Position the window including its title bar, like wmctrl does for the others."""
        self.showNormal()
        frame, inner = self.frameGeometry(), self.geometry()
        left, top = inner.x() - frame.x(), inner.y() - frame.y()
        right, bottom = frame.width() - inner.width() - left, frame.height() - inner.height() - top
        self.setGeometry(rect.x + left, rect.y + top, rect.w - left - right, rect.h - top - bottom)

    def _place_external_windows(self):
        if not self._targets:
            return
        for key, (win_id, _geometry) in self._placer.windows().items():
            if key in self._targets and win_id not in self._placed:
                self._placer.move(win_id, self._targets[key])
                self._placed.add(win_id)

    def _current_layout(self) -> dict:
        external = {k: g.to_dict() for k, (_id, g) in self._placer.windows().items()}
        # A window that is not open keeps its last known target.
        for key, rect in self._targets.items():
            external.setdefault(key, rect.to_dict())
        return {
            "app": _rect(self.frameGeometry()).to_dict(),
            "external": external,
            "docks": bytes(self.saveState().toBase64()).decode(),
        }

    def _apply_layout(self, data: dict):
        if data.get("docks"):
            self.restoreState(QtCore.QByteArray.fromBase64(data["docks"].encode()))
        targets = {k: Rect.from_dict(v) for k, v in data.get("external", {}).items()}
        if "app" in data:
            targets[APP] = Rect.from_dict(data["app"])
        self.apply_targets(targets)

    def _saved_layout_names(self) -> list[str]:
        self._settings.beginGroup("layouts")
        names = sorted(self._settings.childKeys())
        self._settings.endGroup()
        return names

    def save_layout(self, name: str):
        self._settings.setValue(f"layouts/{name}", json.dumps(self._current_layout()))
        self._settings.sync()
        self._rebuild_layout_menu()

    def load_layout(self, name: str):
        raw = self._settings.value(f"layouts/{name}")
        if raw:
            self._apply_layout(json.loads(raw))

    def delete_layout(self, name: str):
        self._settings.remove(f"layouts/{name}")
        self._rebuild_layout_menu()

    def _ask_save_layout(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "Αποθήκευση διάταξης", "Όνομα:")
        name = name.strip().replace("/", "_")
        if ok and name:
            self.save_layout(name)

    def _restore_session(self):
        raw = self._settings.value("session/last")
        if raw:
            self._apply_layout(json.loads(raw))
        else:
            self.apply_preset("two_screens" if len(self.screens()) >= 2 else "laptop")

    def save_session(self):
        self._settings.setValue("session/last", json.dumps(self._current_layout()))
        self._settings.sync()

    # ------------------------------------------------------------------ exit
    def closeEvent(self, event):
        if self._processes.any_running():
            answer = QtWidgets.QMessageBox.question(
                self,
                APP_NAME,
                "Το σύστημα τρέχει. Να τερματιστεί και να κλείσει η εφαρμογή;",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        self.save_session()  # before stopping, while Gazebo/RViz windows still exist
        self._processes.stop_all(wait_ms=STOP_WAIT_ON_EXIT_MS)
        event.accept()
