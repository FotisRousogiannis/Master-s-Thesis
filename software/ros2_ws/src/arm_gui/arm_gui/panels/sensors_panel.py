"""Sensors panel: live joint feedback from /joint_states.

Which devices (servos, encoders, sensors, boards) are alive is the Devices panel (ADR-0018).
"""

import math

from ..qt import QtCore, QtWidgets

REFRESH_MS = 100  # 10 Hz is enough for reading numbers


def _fmt(value, scale=1.0, digits=1):
    return "—" if value is None or math.isnan(value) else f"{value * scale:.{digits}f}"


class SensorsPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._latest = {}
        self._dirty = False
        layout = QtWidgets.QVBoxLayout(self)

        joints_box = QtWidgets.QGroupBox("Αρθρώσεις (/joint_states)")
        jl = QtWidgets.QVBoxLayout(joints_box)
        self._table = QtWidgets.QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(["Άρθρωση", "Θέση [°]", "Ταχύτητα [°/s]", "Ροπή [N·m]"])
        self._table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._table.setMinimumHeight(self._table.verticalHeader().defaultSectionSize() * 7)
        jl.addWidget(self._table)
        self._age = QtWidgets.QLabel("Δεν έχουν ληφθεί δεδομένα")
        jl.addWidget(self._age)
        layout.addWidget(joints_box, 1)

        self._last_rx = None
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(REFRESH_MS)

    def set_joint_state(self, state: dict):
        self._latest = state
        self._dirty = True
        self._last_rx = QtCore.QElapsedTimer()
        self._last_rx.start()

    def _refresh(self):
        if self._last_rx is not None:
            age = self._last_rx.elapsed() / 1000.0
            self._age.setText("Ζωντανά" if age < 1.0 else f"Χωρίς δεδομένα εδώ και {age:.0f} s")
        if not self._dirty:
            return
        self._dirty = False
        names = sorted(self._latest)
        if self._table.rowCount() != len(names):
            self._table.setRowCount(len(names))
        for row, name in enumerate(names):
            pos, vel, eff = self._latest[name]
            for col, text in enumerate([name, _fmt(pos, 180 / math.pi), _fmt(vel, 180 / math.pi), _fmt(eff, 1, 2)]):
                item = self._table.item(row, col)
                if item is None:
                    self._table.setItem(row, col, QtWidgets.QTableWidgetItem(text))
                elif item.text() != text:
                    item.setText(text)
