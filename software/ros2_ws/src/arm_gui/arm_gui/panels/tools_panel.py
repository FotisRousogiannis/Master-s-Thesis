"""Tools panel: a searchable list of useful commands (tools.yaml), run on this computer or on the Pi."""

from ..qt import QtGui, QtWidgets, Signal
from ..tools import PiTarget, Tool, load_catalog

MAX_LINES = 3000


def _searchable(text: str) -> str:
    """Lower case, and I²C found by typing i2c."""
    return text.lower().replace("²", "2")


class ToolsPanel(QtWidgets.QWidget):
    run_requested = Signal(object)  # Tool
    stop_requested = Signal()
    pi_changed = Signal(object)  # PiTarget

    def __init__(self, tools: list[Tool] | None = None, parent=None):
        super().__init__(parent)
        self._tools = {t.id: t for t in (tools if tools is not None else load_catalog())}
        self._pi = PiTarget()
        self._running = False
        layout = QtWidgets.QVBoxLayout(self)

        self._filter = QtWidgets.QLineEdit()
        self._filter.setPlaceholderText("Αναζήτηση (π.χ. i2c, encoder, controllers)…")
        self._filter.setClearButtonEnabled(True)
        self._filter.textChanged.connect(self._refresh)
        layout.addWidget(self._filter)

        split = QtWidgets.QSplitter()
        self._list = QtWidgets.QTreeWidget()
        self._list.setHeaderHidden(True)
        self._list.currentItemChanged.connect(lambda *_: self._show_selected())
        self._list.itemDoubleClicked.connect(lambda *_: self.run_selected())
        split.addWidget(self._list)

        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        self._title = QtWidgets.QLabel("Διάλεξε εργαλείο")
        self._title.setStyleSheet("font-weight: 600;")
        self._about = QtWidgets.QLabel()
        self._about.setWordWrap(True)
        self._about.setProperty("muted", True)
        self._command = QtWidgets.QLabel()
        self._command.setWordWrap(True)
        self._command.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        buttons = QtWidgets.QHBoxLayout()
        self._run = QtWidgets.QPushButton("Εκτέλεση")
        self._run.setProperty("primary", True)
        self._run.clicked.connect(self.run_selected)
        self._stop = QtWidgets.QPushButton("Διακοπή")
        self._stop.clicked.connect(self.stop_requested.emit)
        buttons.addWidget(self._run)
        buttons.addWidget(self._stop)
        buttons.addStretch(1)
        self._output = QtWidgets.QPlainTextEdit()
        self._output.setReadOnly(True)
        self._output.setMaximumBlockCount(MAX_LINES)
        self._output.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        for w in (self._title, self._about, self._command):
            rl.addWidget(w)
        rl.addLayout(buttons)
        rl.addWidget(self._output, 1)
        split.addWidget(right)
        split.setStretchFactor(1, 2)
        layout.addWidget(split, 1)

        pi_box = QtWidgets.QGroupBox("Raspberry Pi (SSH)")
        form = QtWidgets.QHBoxLayout(pi_box)
        self._host = QtWidgets.QLineEdit()
        self._host.setPlaceholderText("π.χ. raspberrypi.local ή 192.168.1.20")
        self._user = QtWidgets.QLineEdit()
        self._user.setPlaceholderText("χρήστης")
        self._repo = QtWidgets.QLineEdit(self._pi.repo)
        self._repo.setToolTip("Φάκελος του repo στο Pi (σχετικά με το home ή απόλυτος)")
        for label, w in (("Host", self._host), ("Χρήστης", self._user), ("Repo", self._repo)):
            form.addWidget(QtWidgets.QLabel(label))
            form.addWidget(w, 1)
            w.editingFinished.connect(self._on_pi_edited)
        layout.addWidget(pi_box)
        self._refresh()

    # ------------------------------------------------------------------ inputs
    def set_pi(self, pi: PiTarget):
        self._pi = pi
        self._host.setText(pi.host)
        self._user.setText(pi.user)
        self._repo.setText(pi.repo)
        self._update_buttons()

    @property
    def pi(self) -> PiTarget:
        return self._pi

    def set_running(self, running: bool):
        self._running = running
        self._update_buttons()

    def append(self, line: str):
        self._output.appendPlainText(line)

    # ------------------------------------------------------------------ actions
    def select(self, tool_id: str):
        for i in range(self._list.topLevelItemCount()):
            group = self._list.topLevelItem(i)
            for j in range(group.childCount()):
                if group.child(j).data(0, 256) == tool_id:  # Qt.UserRole
                    self._list.setCurrentItem(group.child(j))
                    return

    def selected(self) -> Tool | None:
        item = self._list.currentItem()
        return self._tools.get(item.data(0, 256)) if item else None

    def run_selected(self):
        tool = self.selected()
        if tool is None or self._running:
            return
        if tool.where == "pi" and not self._pi.ready:
            self.append("✖ Συμπλήρωσε host και χρήστη του Raspberry Pi (κάτω).")
            return
        if tool.moves and not self.confirm(
            tool.name, f"Το εργαλείο «{tool.name}» κινεί servo.\n\nΕίναι ο βραχίονας ελεύθερος να κινηθεί;"
        ):
            return
        self._output.clear()
        self.run_requested.emit(tool)

    def confirm(self, title, text) -> bool:
        return QtWidgets.QMessageBox.question(self, title, text) == QtWidgets.QMessageBox.Yes

    # ------------------------------------------------------------------ internals
    def _on_pi_edited(self):
        self._pi = PiTarget(self._host.text().strip(), self._user.text().strip(), self._repo.text().strip())
        self.pi_changed.emit(self._pi)
        self._update_buttons()

    def _refresh(self):
        current = self.selected()
        f = _searchable(self._filter.text().strip())
        self._list.clear()
        groups = {}
        for tool in self._tools.values():
            text = _searchable(" ".join((tool.name, tool.group, tool.description, tool.command)))
            if f and f not in text:
                continue
            if tool.group not in groups:
                groups[tool.group] = QtWidgets.QTreeWidgetItem(self._list, [tool.group])
                groups[tool.group].setExpanded(True)
            item = QtWidgets.QTreeWidgetItem(groups[tool.group], [("⚠ " if tool.moves else "") + tool.name])
            item.setData(0, 256, tool.id)
            item.setToolTip(0, tool.description)
        if current:
            self.select(current.id)
        self._show_selected()

    def _show_selected(self):
        tool = self.selected()
        self._title.setText(tool.name if tool else "Διάλεξε εργαλείο")
        if tool:
            where = "στο Raspberry Pi (SSH)" if tool.where == "pi" else "σε αυτόν τον υπολογιστή"
            self._about.setText(f"{tool.description}\nΤρέχει {where}." + ("\n⚠ Κινεί servo." if tool.moves else ""))
            self._command.setText(f"$ {tool.command}")
        else:
            self._about.setText("")
            self._command.setText("")
        self._update_buttons()

    def _update_buttons(self):
        tool = self.selected()
        self._run.setEnabled(tool is not None and not self._running)
        self._stop.setEnabled(self._running)
        if tool is not None and tool.where == "pi" and not self._pi.ready:
            self._run.setToolTip("Συμπλήρωσε host και χρήστη του Raspberry Pi")
        else:
            self._run.setToolTip("")
