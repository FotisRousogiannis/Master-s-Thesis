"""Devices panel: which servos, encoders, sensors and boards are alive right now (ADR-0018).

Source: the standard /diagnostics topic. A device is "active" while it keeps reporting; when its
reports stop it stays in the list as silent, so a cable that fell off is visible. Devices that
are configured but never reported (e.g. the servos of calibration.yaml) are listed as expected.
"""

import time

from .. import theme
from ..qt import QtCore, QtGui, QtWidgets

SILENT_AFTER_S = 3.0
REFRESH_MS = 500
OK, WARN, ERROR, STALE = 0, 1, 2, 3
# Prefix of the diagnostic name ("servo/wrist_diff_a") -> group title, in display order.
GROUPS = {"servo": "Servos", "encoder": "Encoders", "sensor": "Αισθητήρες", "board": "Πλακέτες"}
OTHER = "Άλλα"


def group_of(name: str) -> str:
    prefix, sep, _ = name.partition("/")
    return GROUPS.get(prefix, OTHER) if sep else OTHER


def short_name(name: str) -> str:
    prefix, sep, rest = name.partition("/")
    return rest if sep and prefix in GROUPS else name


class DevicesPanel(QtWidgets.QWidget):
    def __init__(self, parent=None, clock=time.monotonic):
        super().__init__(parent)
        self._clock = clock
        self._devices = {}  # name -> status dict + "seen" (None: expected, never reported)
        layout = QtWidgets.QVBoxLayout(self)

        self._summary = QtWidgets.QLabel()
        self._summary.setWordWrap(True)
        layout.addWidget(self._summary)
        self._only_active = QtWidgets.QCheckBox("Μόνο ενεργές")
        self._only_active.toggled.connect(self._refresh)
        layout.addWidget(self._only_active)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setHeaderLabels(["Συσκευή", "Κατάσταση", "Μήνυμα", "Τιμές"])
        self._tree.setColumnWidth(0, 190)
        self._tree.setColumnWidth(1, 170)
        self._tree.setColumnWidth(2, 170)
        self._tree.setAlternatingRowColors(True)
        layout.addWidget(self._tree, 1)

        note = QtWidgets.QLabel(
            "Πηγή: /diagnostics. «Ενεργή» = ανέφερε τα τελευταία 3 s. Στην προσομοίωση δεν υπάρχουν "
            "servos/encoders, άρα δεν αναφέρουν."
        )
        note.setWordWrap(True)
        note.setProperty("muted", True)
        layout.addWidget(note)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(REFRESH_MS)
        self._refresh()

    # ------------------------------------------------------------------ inputs
    def set_expected(self, names):
        """Devices that should exist (from configuration); shown even before they report."""
        for name in names:
            self._devices.setdefault(name, {"level": STALE, "message": "", "hardware_id": "", "values": {}, "seen": None})
        self._refresh()

    def update_diagnostics(self, statuses):
        now = self._clock()
        for s in statuses:
            self._devices[s["name"]] = {**s, "seen": now}
        self._refresh()

    # ------------------------------------------------------------------ state
    def state_of(self, name: str) -> str:
        """expected | silent | ok | warn | error"""
        d = self._devices[name]
        if d["seen"] is None:
            return "expected"
        if self._clock() - d["seen"] > SILENT_AFTER_S or d["level"] == STALE:
            return "silent"
        return {OK: "ok", WARN: "warn"}.get(d["level"], "error")

    def counts(self) -> dict:
        """{group title: (active, total)}"""
        out = {}
        for name in self._devices:
            active, total = out.get(group_of(name), (0, 0))
            out[group_of(name)] = (active + (self.state_of(name) in ("ok", "warn", "error")), total + 1)
        return out

    def _refresh(self):
        t = theme.current()
        labels = {
            "ok": ("● Ενεργή", t.ok),
            "warn": ("● Προειδοποίηση", t.warn),
            "error": ("● Σφάλμα", t.error),
            "silent": ("○ Σιωπηλή", t.muted),
            "expected": ("○ Δεν έχει αναφέρει", t.muted),
        }
        counts = self.counts()
        order = [g for g in [*GROUPS.values(), OTHER] if g in counts]
        self._summary.setText(
            "   ".join(f"<b>{g}</b> {counts[g][0]}/{counts[g][1]}" for g in order)
            or "Καμία συσκευή δεν έχει αναφέρει κατάσταση."
        )
        rows = []
        for group in order:
            names = sorted(n for n in self._devices if group_of(n) == group)
            if self._only_active.isChecked():
                names = [n for n in names if self.state_of(n) in ("ok", "warn", "error")]
            rows.append((group, [(n, self.state_of(n)) for n in names]))

        expanded = {
            self._tree.topLevelItem(i).data(0, QtCore.Qt.UserRole): self._tree.topLevelItem(i).isExpanded()
            for i in range(self._tree.topLevelItemCount())
        }
        self._tree.clear()
        for group, devices in rows:
            top = QtWidgets.QTreeWidgetItem(self._tree, [f"{group} ({counts[group][0]}/{counts[group][1]})"])
            top.setData(0, QtCore.Qt.UserRole, group)
            for name, state in devices:
                d = self._devices[name]
                text, colour = labels[state]
                values = "  ".join(f"{k}={v}" for k, v in d["values"].items())
                item = QtWidgets.QTreeWidgetItem(top, [short_name(name), text, d["message"], values])
                item.setForeground(1, QtGui.QBrush(QtGui.QColor(colour)))
                item.setToolTip(0, f"{name}\nhardware_id: {d['hardware_id'] or '—'}")
                item.setToolTip(3, values)
            top.setExpanded(expanded.get(group, True))
