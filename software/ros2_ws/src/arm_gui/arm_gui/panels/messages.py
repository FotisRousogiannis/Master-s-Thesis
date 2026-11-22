"""Messages strip: only what the user must notice (warnings, errors, results), under every section.

The full output of every process stays in the Log (Για προχωρημένους).
"""

import time

from ..qt import QtWidgets, Signal

MAX_ITEMS = 200
MARKS = {"info": "•", "warning": "⚠", "error": "✖", "ok": "✓"}


class MessagesStrip(QtWidgets.QWidget):
    warnings_changed = Signal(int)  # how many devices are in warning/error now

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        title = QtWidgets.QLabel("Μηνύματα · ολόκληρο το log στα «Για προχωρημένους»")
        title.setProperty("muted", True)
        layout.addWidget(title)
        self.list = QtWidgets.QListWidget()
        self.list.setMaximumHeight(96)
        self.setMaximumHeight(130)
        layout.addWidget(self.list)
        self._levels = {}  # diagnostic name -> last level (0 ok, 1 warn, 2 error, 3 stale)

    def add(self, level: str, text: str):
        self.list.addItem(f"{time.strftime('%H:%M:%S')}  {MARKS.get(level, '•')} {text}")
        while self.list.count() > MAX_ITEMS:
            self.list.takeItem(0)
        self.list.scrollToBottom()

    def texts(self) -> list:
        return [self.list.item(i).text() for i in range(self.list.count())]

    def update_diagnostics(self, statuses):
        """A device entering or leaving warning/error becomes one message (not one per second)."""
        for s in statuses:
            name, level = s["name"], s["level"]
            before = self._levels.get(name)
            self._levels[name] = level
            if before == level or (before is None and level == 0):
                continue
            if level >= 2:
                self.add("error", f"{name}: {s['message']}")
            elif level == 1:
                self.add("warning", f"{name}: {s['message']}")
            elif before is not None:
                self.add("ok", f"{name}: εντάξει")
        self.warnings_changed.emit(sum(1 for level in self._levels.values() if level in (1, 2)))
