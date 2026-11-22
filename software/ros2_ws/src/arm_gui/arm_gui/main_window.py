"""Main window: five sections in a side bar, toolbar with STOP, messages strip, and the layout of the
three windows (ADR-0016; sections instead of dockable panels: ADR-0016 supplement 2026-11-17).

    Χειρισμός          Σύστημα + Αρθρώσεις
    Βαθμονόμηση        the calibration panel (ADR-0023) and the devices that are alive
    Pi                 connection to the Pi
    Για προχωρημένους  Παράμετροι, ROS 2, Εργαλεία, Log
"""

import json
import os
import time
from pathlib import Path

from . import APP_NAME, theme
from .layouts import APP, PRESETS, Rect, WindowPlacer, fit_to_screens, preset_layout
from .model import parse_joints
from .panels.calibration_panel import CalibrationPanel
from .panels.connection_panel import ConnectionPanel
from .panels.devices_panel import DevicesPanel
from .panels.graph_panel import GraphPanel
from .panels.joints_panel import JointsPanel
from .panels.log_panel import LogPanel
from .panels.messages import MessagesStrip
from .panels.parameters_panel import CALIBRATION_NODE, ParametersPanel
from .panels.system_panel import SystemPanel
from .panels.tools_panel import ToolsPanel
from .processes import ProcessManager
from .qt import QtCore, QtGui, QtWidgets, Qt, Signal
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


class SectionBar(QtWidgets.QWidget):
    """The side bar: one button per section (title + a line of what is in it), one checked."""

    currentRowChanged = Signal(int)

    def __init__(self, sections, parent=None):
        super().__init__(parent)
        self.setObjectName("section_bar")
        self.setFixedWidth(240)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 10, 8, 10)
        layout.setSpacing(4)
        self._group = QtWidgets.QButtonGroup(self)
        self._group.setExclusive(True)
        for i, (label, sub) in enumerate(sections):
            button = QtWidgets.QPushButton(f"{label}\n{sub}")
            button.setObjectName("section_button")
            button.setCheckable(True)
            button.setToolTip(f"{label} (Ctrl+{i + 1})")
            self._group.addButton(button, i)
            layout.addWidget(button)
        layout.addStretch(1)
        self._group.idClicked.connect(self.setCurrentRow)
        self._current = -1

    def count(self) -> int:
        return len(self._group.buttons())

    def currentRow(self) -> int:
        return self._current

    def setCurrentRow(self, row: int):
        self._group.button(row).setChecked(True)
        if row != self._current:
            self._current = row
            self.currentRowChanged.emit(row)


def _rect(q) -> Rect:
    return Rect(q.x(), q.y(), q.width(), q.height())


def expected_devices(values: dict) -> list:
    """servo/<servo> and encoder/<joint> of the enabled joints, from the /arm_calibration parameters.
    A servo of a disabled joint, or one without a channel (-1), is not driven, so it never reports."""
    expected = set()
    for name, value in values.items():
        parts = name.split(".")
        if parts[0] != "joints" or not values.get(f"joints.{parts[1]}.enabled", True):
            continue
        if len(parts) == 3 and parts[2] == "encoder":
            expected.add(f"encoder/{parts[1]}")
        if len(parts) == 4 and parts[2] == "servos" and values.get(f"servos.{parts[3]}.channel", -1) != -1:
            expected.add(f"servo/{parts[3]}")
    return sorted(expected)


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
        self.devices = DevicesPanel()
        self.parameters = ParametersPanel()
        self.calibration = CalibrationPanel()
        self.graph = GraphPanel(bridge.node_info)
        self.log = LogPanel()
        self.tools = ToolsPanel()
        self.connection = ConnectionPanel(data_dir() / "ssh")
        self.messages = MessagesStrip()
        self._build_sections()
        self._build_toolbar()
        self._build_menus()
        self._connect()

        self._place_timer = QtCore.QTimer(self)
        self._place_timer.timeout.connect(self._place_external_windows)
        self._place_timer.start(PLACE_POLL_MS)
        self._restore_session()
        self._load_pi()

    # ------------------------------------------------------------------ construction
    SECTIONS = (
        ("run", "Χειρισμός", "σύστημα, αρθρώσεις"),
        ("cal", "Βαθμονόμηση", "οδηγός, ρυθμίσεις"),
        ("pi", "Pi", "σύνδεση με το Pi"),
        ("adv", "Για προχωρημένους", "παράμετροι, ROS 2, log"),
    )

    def _build_sections(self):
        run = QtWidgets.QSplitter(Qt.Horizontal)
        run.addWidget(self.system)
        run.addWidget(self.joints)
        run.setStretchFactor(1, 1)
        self.advanced = QtWidgets.QTabWidget()
        for widget, title in ((self.parameters, "Παράμετροι"), (self.graph, "ROS 2"), (self.tools, "Εργαλεία"),
                              (self.log, "Log")):
            self.advanced.addTab(widget, title)
        self.calibration.tabs.addTab(self.devices, "Συσκευές")       # what is alive: part of setting up
        pages = {"run": run, "cal": self.calibration, "pi": self.connection, "adv": self.advanced}

        self.stack = QtWidgets.QStackedWidget()
        self.nav = SectionBar([(label, sub) for _key, label, sub in self.SECTIONS])
        self._section_keys = [key for key, _label, _sub in self.SECTIONS]
        for key in self._section_keys:
            self.stack.addWidget(pages[key])
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.nav.setCurrentRow(0)

        body = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(body)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(self.nav)
        h.addWidget(self.stack, 1)
        central = QtWidgets.QSplitter(Qt.Vertical)
        central.addWidget(body)
        central.addWidget(self.messages)
        central.setStretchFactor(0, 1)
        central.setCollapsible(0, False)
        self.setCentralWidget(central)

    @property
    def section(self) -> str:
        return self._section_keys[self.nav.currentRow()]

    def show_section(self, key: str):
        self.nav.setCurrentRow(self._section_keys.index(key))

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
        self._warn_pill = self._pill()
        bar.addWidget(self._system_pill)
        bar.addWidget(self._ros_pill)
        bar.addWidget(self._warn_pill)
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
        self._set_warnings(0)

    def _set_warnings(self, count: int):
        self._set_pill(self._warn_pill, "warn" if count else "ok",
                       ("1 προειδοποίηση" if count == 1 else f"{count} προειδοποιήσεις") if count
                       else "Καμία προειδοποίηση")

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
        for number, (key, label, _sub) in enumerate(self.SECTIONS, start=1):
            view.addAction(label, lambda k=key: self.show_section(k), QtGui.QKeySequence(f"Ctrl+{number}"))
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
        b.controllers_changed.connect(self.system.set_controllers)
        b.nodes_changed.connect(self.parameters.set_nodes)
        b.nodes_changed.connect(self._on_nodes)
        b.graph_changed.connect(self.graph.set_graph)
        b.parameters_loaded.connect(self.parameters.show_parameters)
        b.parameters_loaded.connect(self._on_parameters)
        b.diagnostics.connect(self.devices.update_diagnostics)
        b.diagnostics.connect(self.parameters.update_diagnostics)
        b.parameters_set.connect(self.parameters.show_set_results)
        b.parameters_loaded.connect(self.calibration.show_parameters)
        b.parameters_set.connect(self.calibration.show_set_results)
        b.diagnostics.connect(self.calibration.update_diagnostics)
        b.identify_reply.connect(self.calibration.on_identify_reply)
        b.nodes_changed.connect(self.calibration.set_nodes)
        self.calibration.load_requested.connect(b.request_parameters)
        self.calibration.apply_requested.connect(b.set_parameters_atomically)
        self.calibration.identify_requested.connect(b.identify)
        self.calibration.service_requested.connect(b.call_trigger)
        b.service_result.connect(self.calibration.show_service_result)
        b.service_result.connect(self.parameters.show_service_result)
        b.service_result.connect(lambda srv, ok, msg: self.log.append("υπηρεσία", f"{srv}: {msg}"))
        b.service_result.connect(lambda srv, ok, msg: self.messages.add("info" if ok else "error", f"{srv}: {msg}"))
        b.diagnostics.connect(self.messages.update_diagnostics)
        self.messages.warnings_changed.connect(self._set_warnings)
        self.parameters.load_requested.connect(b.request_parameters)
        self.parameters.apply_requested.connect(b.set_parameters)
        self.parameters.service_requested.connect(b.call_trigger)
        b.goal_status.connect(self._on_goal_status)
        b.log.connect(lambda line: self.log.append("app", line))
        b.log.connect(lambda line: self.messages.add("info", line))
        self.joints.goal_requested.connect(b.send_joint_goal)
        self.system.start_requested.connect(self.start_system)
        self.system.window_requested.connect(self.set_window)
        self.tools.run_requested.connect(self.run_tool)
        self.tools.stop_requested.connect(lambda: self._processes.stop(TOOL_PROCESS))
        self.connection.pi_changed.connect(self._save_pi)
        self.connection.pi_changed.connect(self.tools.set_pi)
        b.nodes_changed.connect(self.connection.set_nodes)
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
        pi = PiTarget(s.value("pi/host", PiTarget.host), s.value("pi/user", ""), s.value("pi/repo", PiTarget.repo))
        self.connection.set_pi(pi)
        self.tools.set_pi(pi)

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
            self.messages.add("info", "Σύστημα σε λειτουργία" if running else "Σύστημα σταματημένο")
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
        """Devices we expect to hear from (arm.yaml, ADR-0023): the servos and the encoder of every
        enabled joint (the Pi reports them as servo/<servo> and encoder/<joint>)."""
        if node == CALIBRATION_NODE and not isinstance(params, str):
            self.devices.set_expected(expected_devices({p["name"]: p["value"] for p in params}))

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
        self.messages.add("info", text)

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
            "section": self.section,
        }

    def _apply_layout(self, data: dict):
        if data.get("section") in self._section_keys:   # older layouts had "docks": ignored
            self.show_section(data["section"])
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
