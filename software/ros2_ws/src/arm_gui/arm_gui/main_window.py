"""Main window: dockable panels, toolbar with STOP, and the layout of the three windows (ADR-0016)."""

import json
import os
from pathlib import Path

from . import APP_NAME
from .layouts import APP, PRESETS, Rect, WindowPlacer, fit_to_screens, preset_layout
from .model import parse_joints
from .panels.joints_panel import JointsPanel
from .panels.log_panel import LogPanel
from .panels.sensors_panel import SensorsPanel
from .panels.system_panel import SystemPanel
from .processes import ProcessManager
from .qt import QtCore, QtGui, QtWidgets, Qt

SYSTEM_PROCESS = "system"
PLACE_POLL_MS = 1000
STOP_WAIT_ON_EXIT_MS = 10000


def config_dir() -> Path:
    """Outside the container image when ARM_GUI_CONFIG_DIR is set (compose.yaml), so layouts survive `docker compose down`."""
    return Path(os.environ.get("ARM_GUI_CONFIG_DIR", Path.home() / ".config" / "arm_gui"))


def _rect(q) -> Rect:
    return Rect(q.x(), q.y(), q.width(), q.height())


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, bridge, processes=None, placer=None, settings_file=None):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} — ρομποτικός βραχίονας")
        self._bridge = bridge
        self._processes = processes or ProcessManager(self)
        self._placer = placer or WindowPlacer()
        settings_file = settings_file or str(config_dir() / "settings.ini")
        self._settings = QtCore.QSettings(settings_file, QtCore.QSettings.IniFormat)
        self._targets: dict[str, Rect] = {}  # where the external windows should go
        self._placed: set[str] = set()  # window ids already placed for the current targets

        self.system = SystemPanel()
        self.joints = JointsPanel()
        self.sensors = SensorsPanel()
        self.log = LogPanel()
        self._docks = {}
        self._build_docks()
        self._build_toolbar()
        self._build_menus()
        self._connect()

        self._place_timer = QtCore.QTimer(self)
        self._place_timer.timeout.connect(self._place_external_windows)
        self._place_timer.start(PLACE_POLL_MS)
        self._restore_session()

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
        self.tabifyDockWidget(d["sensors"], d["log"])
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
        self.stop_button = QtWidgets.QPushButton("STOP (Esc)")
        self.stop_button.setToolTip(
            "Ακύρωση κίνησης και κράτημα θέσης. Λογισμικό — ΔΕΝ αντικαθιστά το E-stop της τροφοδοσίας."
        )
        self.stop_button.setStyleSheet(
            "QPushButton { background: #c62828; color: white; font-weight: bold; padding: 6px 24px; }"
            "QPushButton:pressed { background: #8e0000; }"
        )
        self.stop_button.clicked.connect(self._bridge.stop)
        QtWidgets.QShortcut(QtGui.QKeySequence(Qt.Key_Escape), self, activated=self._bridge.stop)
        bar.addWidget(self.stop_button)
        bar.addSeparator()
        self._motion_status = QtWidgets.QLabel("")
        bar.addWidget(self._motion_status)

    def _build_menus(self):
        file_menu = self.menuBar().addMenu("&Αρχείο")
        file_menu.addAction("Έξοδος", self.close, QtGui.QKeySequence.Quit)

        view = self.menuBar().addMenu("&Προβολή")
        for dock in self._docks.values():
            view.addAction(dock.toggleViewAction())
        view.addSeparator()
        view.addAction("Επαναφορά πάνελ", self._default_dock_arrangement)

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
        b.nodes_changed.connect(self.system.set_nodes)
        b.goal_status.connect(self._on_goal_status)
        b.log.connect(lambda line: self.log.append("app", line))
        self.joints.goal_requested.connect(b.send_joint_goal)
        self.system.start_requested.connect(self.start_system)
        self.system.stop_requested.connect(lambda: self._processes.stop(SYSTEM_PROCESS))
        self._processes.output.connect(self.log.append)
        self._processes.running_changed.connect(self._on_process_running)

    # ------------------------------------------------------------------ system
    def start_system(self, hardware: str, rviz: bool):
        self._placed.clear()  # new Gazebo/RViz windows: place them again
        self._processes.start(
            SYSTEM_PROCESS,
            "ros2",
            ["launch", "arm_bringup", "bringup.launch.py", f"hardware:={hardware}", f"rviz:={str(rviz).lower()}"],
        )

    def _on_process_running(self, name, running):
        if name == SYSTEM_PROCESS:
            self.system.set_running(running)

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
