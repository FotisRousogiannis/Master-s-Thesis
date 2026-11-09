"""Joints panel: one slider per joint, built from the URDF limits."""

from ..model import JointInfo, move_duration
from ..qt import Qt, QtWidgets, Signal

SLIDER_STEPS_PER_UNIT = 10  # 0.1° (or 0.1 mm) resolution


class _JointRow:
    def __init__(self, joint: JointInfo, grid: QtWidgets.QGridLayout, row: int, on_released):
        self.joint = joint
        lo, hi = joint.to_display(joint.lower), joint.to_display(joint.upper)
        self.slider = QtWidgets.QSlider(Qt.Horizontal)
        self.slider.setRange(round(lo * SLIDER_STEPS_PER_UNIT), round(hi * SLIDER_STEPS_PER_UNIT))
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setDecimals(1)
        self.spin.setSuffix(f" {joint.unit}")
        self.current = QtWidgets.QLabel("—")
        self.current.setMinimumWidth(70)
        self.current.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.slider.valueChanged.connect(lambda v: self._set_spin(v / SLIDER_STEPS_PER_UNIT))
        self.spin.valueChanged.connect(lambda v: self._set_slider(v))
        self.slider.sliderReleased.connect(on_released)

        grid.addWidget(QtWidgets.QLabel(joint.name), row, 0)
        grid.addWidget(QtWidgets.QLabel(f"{lo:.0f}"), row, 1)
        grid.addWidget(self.slider, row, 2)
        grid.addWidget(QtWidgets.QLabel(f"{hi:.0f}"), row, 3)
        grid.addWidget(self.spin, row, 4)
        grid.addWidget(self.current, row, 5)

    def _set_spin(self, value):
        if abs(self.spin.value() - value) > 1e-6:
            self.spin.blockSignals(True)
            self.spin.setValue(value)
            self.spin.blockSignals(False)

    def _set_slider(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(round(value * SLIDER_STEPS_PER_UNIT))
        self.slider.blockSignals(False)

    @property
    def target(self) -> float:
        """Target in SI units."""
        return self.joint.clamp(self.joint.from_display(self.spin.value()))

    def set_target(self, si_value: float):
        self.spin.setValue(self.joint.to_display(self.joint.clamp(si_value)))

    def show_current(self, si_value: float):
        self.current.setText(f"{self.joint.to_display(si_value):.1f} {self.joint.unit}")


class JointsPanel(QtWidgets.QWidget):
    goal_requested = Signal(object, float)  # {joint: target SI}, duration s

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows: dict[str, _JointRow] = {}
        self._current: dict[str, float] = {}

        layout = QtWidgets.QVBoxLayout(self)
        self._placeholder = QtWidgets.QLabel("Αναμονή για το μοντέλο του βραχίονα (/robot_description)…")
        layout.addWidget(self._placeholder)
        self._grid_host = QtWidgets.QWidget()
        self._grid = QtWidgets.QGridLayout(self._grid_host)
        self._grid.setColumnStretch(2, 1)
        layout.addWidget(self._grid_host)

        options = QtWidgets.QHBoxLayout()
        options.addWidget(QtWidgets.QLabel("Ταχύτητα"))
        self._speed = QtWidgets.QSpinBox()
        self._speed.setRange(5, 100)
        self._speed.setValue(50)
        self._speed.setSuffix(" % του ορίου")
        options.addWidget(self._speed)
        self._live = QtWidgets.QCheckBox("Αποστολή μόλις αφήσεις τη μπάρα")
        options.addWidget(self._live)
        options.addStretch(1)
        layout.addLayout(options)

        buttons = QtWidgets.QHBoxLayout()
        self._send = QtWidgets.QPushButton("Αποστολή")
        self._home = QtWidgets.QPushButton("Home (όλα 0)")
        self._sync = QtWidgets.QPushButton("Στόχοι = τρέχουσα θέση")
        self._send.clicked.connect(self.send)
        self._home.clicked.connect(self.home)
        self._sync.clicked.connect(self.sync_to_current)
        for b in (self._send, self._home, self._sync):
            buttons.addWidget(b)
        layout.addLayout(buttons)
        self._status = QtWidgets.QLabel()
        self._status.setWordWrap(True)
        layout.addWidget(self._status)
        layout.addStretch(1)
        self._set_enabled(False)

    def set_joints(self, joints: list[JointInfo]):
        if [r.joint for r in self._rows.values()] == joints:
            return
        while self._grid.count():
            self._grid.takeAt(0).widget().deleteLater()
        self._rows = {}
        for i, joint in enumerate(joints):
            self._rows[joint.name] = _JointRow(joint, self._grid, i, self._on_slider_released)
        self._placeholder.setVisible(not joints)
        self._set_enabled(bool(joints))
        self.sync_to_current()

    def set_current(self, state: dict):
        for name, (position, _vel, _eff) in state.items():
            self._current[name] = position
            if name in self._rows:
                self._rows[name].show_current(position)

    def set_status(self, text: str):
        self._status.setText(text)

    @property
    def joints(self):
        return {name: row.joint for name, row in self._rows.items()}

    def send(self):
        targets = {name: row.target for name, row in self._rows.items()}
        if targets:
            duration = move_duration(self._current, targets, self.joints, self._speed.value() / 100.0)
            self.goal_requested.emit(targets, duration)

    def home(self):
        for row in self._rows.values():
            row.set_target(0.0)
        self.send()

    def sync_to_current(self):
        for name, row in self._rows.items():
            if name in self._current:
                row.set_target(self._current[name])

    def _on_slider_released(self):
        if self._live.isChecked():
            self.send()

    def _set_enabled(self, enabled):
        for widget in (self._send, self._home, self._sync, self._speed, self._live):
            widget.setEnabled(enabled)
