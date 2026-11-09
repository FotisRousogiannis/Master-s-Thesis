"""Parameters panel: read and change the ROS 2 parameters of any node (ADR-0017).

For /arm_calibration (servo pulse widths, channels, safety limits) it also saves the values
to the calibration file in the repository, or reloads them from it.
"""

from .. import theme
from ..qt import Qt, QtWidgets, Signal

CALIBRATION_NODE = "/arm_calibration"
INT_LIMIT = 2**31 - 1
UNITS = {"_us": " µs", "_hz": " Hz", "_s": " s", "_deg": " °", "_mm": " mm", "_m": " m"}


def unit_of(name: str) -> str:
    leaf = name.rsplit(".", 1)[-1]
    return next((u for suffix, u in UNITS.items() if leaf.endswith(suffix)), "")


class _Editor:
    """One value editor; reports whether it differs from the value the node holds."""

    def __init__(self, param: dict, on_change):
        self.param = param
        self.original = param["value"]
        kind, value = param["type"], param["value"]
        if kind == "bool":
            w = QtWidgets.QCheckBox()
            w.setChecked(bool(value))
            w.toggled.connect(on_change)
        elif kind == "integer":
            w = QtWidgets.QSpinBox()
            lo, hi = param["min"], param["max"]
            w.setRange(int(lo) if lo is not None else -INT_LIMIT, int(hi) if hi is not None else INT_LIMIT)
            w.setValue(int(value))
            w.setSuffix(unit_of(param["name"]))
            w.valueChanged.connect(on_change)
        elif kind == "double":
            w = QtWidgets.QDoubleSpinBox()
            w.setDecimals(3)
            lo, hi = param["min"], param["max"]
            w.setRange(lo if lo is not None else -1e9, hi if hi is not None else 1e9)
            w.setValue(float(value))
            w.setSuffix(unit_of(param["name"]))
            w.valueChanged.connect(on_change)
        elif kind == "string":
            w = QtWidgets.QLineEdit(str(value))
            w.textChanged.connect(on_change)
        else:
            w = QtWidgets.QLabel(str(value))
        if param["read_only"] or kind == "other":
            w.setEnabled(False)
        tip = param["description"]
        if param["min"] is not None:
            tip += f"\nΕπιτρεπτά: {param['min']} … {param['max']}"
        w.setToolTip(tip.strip())
        self.widget = w

    @property
    def value(self):
        w, kind = self.widget, self.param["type"]
        if kind == "bool":
            return w.isChecked()
        if kind in ("integer", "double"):
            return w.value()
        if kind == "string":
            return w.text()
        return self.original

    @property
    def modified(self) -> bool:
        return self.value != self.original

    def accept(self):
        self.original = self.value

    def revert(self):
        w, kind = self.widget, self.param["type"]
        if kind == "bool":
            w.setChecked(bool(self.original))
        elif kind in ("integer", "double"):
            w.setValue(self.original)
        elif kind == "string":
            w.setText(str(self.original))

    def mark(self):
        self.widget.setProperty("modified", self.modified)
        theme.repolish(self.widget)


class ParametersPanel(QtWidgets.QWidget):
    load_requested = Signal(str)  # node
    apply_requested = Signal(str, object)  # node, {name: value}
    service_requested = Signal(str)  # Trigger service

    def __init__(self, parent=None):
        super().__init__(parent)
        self._editors: dict[str, _Editor] = {}
        self._node = None
        layout = QtWidgets.QVBoxLayout(self)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Node"))
        self._nodes = QtWidgets.QComboBox()
        self._nodes.setMinimumContentsLength(20)
        self._nodes.activated.connect(lambda _i: self.reload())
        top.addWidget(self._nodes, 1)
        self._reload = QtWidgets.QPushButton("Ανάγνωση")
        self._reload.clicked.connect(self.reload)
        top.addWidget(self._reload)
        layout.addLayout(top)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setColumnCount(3)
        self._tree.setHeaderLabels(["Παράμετρος", "Τιμή", "Περιγραφή"])
        self._tree.setAlternatingRowColors(True)
        self._tree.header().setStretchLastSection(True)
        self._tree.setColumnWidth(0, 230)
        self._tree.setColumnWidth(1, 160)
        layout.addWidget(self._tree, 1)

        buttons = QtWidgets.QHBoxLayout()
        self._apply = QtWidgets.QPushButton("Εφαρμογή")
        self._apply.setProperty("primary", True)
        self._apply.clicked.connect(self.apply)
        self._revert = QtWidgets.QPushButton("Αναίρεση αλλαγών")
        self._revert.clicked.connect(self.revert)
        self._save = QtWidgets.QPushButton("Αποθήκευση στο αρχείο")
        self._save.setToolTip("Γράφει τις τρέχουσες τιμές στο arm_calibration/config/calibration.yaml")
        self._save.clicked.connect(lambda: self.service_requested.emit(f"{CALIBRATION_NODE}/save"))
        self._load_file = QtWidgets.QPushButton("Επαναφόρτωση από αρχείο")
        self._load_file.clicked.connect(lambda: self.service_requested.emit(f"{CALIBRATION_NODE}/reload"))
        for b in (self._apply, self._revert):
            buttons.addWidget(b)
        buttons.addStretch(1)
        for b in (self._save, self._load_file):
            buttons.addWidget(b)
        layout.addLayout(buttons)

        self._status = QtWidgets.QLabel("Διάλεξε node και πάτα «Ανάγνωση».")
        self._status.setWordWrap(True)
        self._status.setProperty("muted", True)
        layout.addWidget(self._status)
        self._update_buttons()

    # ------------------------------------------------------------------ inputs from the bridge
    def set_nodes(self, nodes):
        current = self._nodes.currentText()
        if [self._nodes.itemText(i) for i in range(self._nodes.count())] == list(nodes):
            return
        self._nodes.blockSignals(True)
        self._nodes.clear()
        self._nodes.addItems(nodes)
        wanted = current or (CALIBRATION_NODE if CALIBRATION_NODE in nodes else "")
        if wanted in nodes:
            self._nodes.setCurrentText(wanted)
        self._nodes.blockSignals(False)
        if not current and wanted == CALIBRATION_NODE:
            self.reload()  # calibration is what people open this panel for

    def show_parameters(self, node, params):
        if node != self._node:
            return
        if isinstance(params, str):
            self._status.setText(f"Σφάλμα: {params}")
            return
        self._tree.clear()
        self._editors = {}
        groups = {}
        for p in params:
            *path, leaf = p["name"].split(".")
            parent = self._tree.invisibleRootItem()
            for depth in range(len(path)):
                key = ".".join(path[: depth + 1])
                if key not in groups:
                    groups[key] = QtWidgets.QTreeWidgetItem(parent, [path[depth]])
                    groups[key].setFirstColumnSpanned(False)
                parent = groups[key]
            item = QtWidgets.QTreeWidgetItem(parent, [leaf, "", p["description"]])
            editor = _Editor(p, self._update_buttons)
            self._editors[p["name"]] = editor
            self._tree.setItemWidget(item, 1, editor.widget)
        for key, item in groups.items():
            item.setExpanded(node == CALIBRATION_NODE or "." not in key and key not in ("qos_overrides",))
        self._status.setText(f"{len(params)} παράμετροι του {node}")
        self._update_buttons()

    def show_set_results(self, node, results):
        if node != self._node:
            return
        rejected = []
        for name, ok, reason in results:
            editor = self._editors.get(name)
            if ok and editor:
                editor.accept()
            elif not ok:
                rejected.append(f"{name}: {reason}")
        self._status.setText("Εφαρμόστηκαν." if not rejected else "Απορρίφθηκαν:\n" + "\n".join(rejected))
        self._update_buttons()

    def show_service_result(self, service, ok, message):
        if service.startswith(CALIBRATION_NODE):
            self._status.setText(("✔ " if ok else "✖ ") + message)
            if ok and service.endswith("/reload"):
                self.reload()

    # ------------------------------------------------------------------ actions
    def reload(self):
        self._node = self._nodes.currentText() or None
        if self._node:
            self._status.setText(f"Ανάγνωση παραμέτρων του {self._node}…")
            self.load_requested.emit(self._node)
        self._update_buttons()

    def apply(self):
        changed = {n: e.value for n, e in self._editors.items() if e.modified}
        if changed and self._node:
            self._status.setText(f"Εφαρμογή {len(changed)} αλλαγών…")
            self.apply_requested.emit(self._node, changed)

    def revert(self):
        for editor in self._editors.values():
            editor.revert()
        self._update_buttons()

    def _update_buttons(self, *_):
        for editor in self._editors.values():
            editor.mark()
        n = sum(e.modified for e in self._editors.values())
        self._apply.setText(f"Εφαρμογή ({n})" if n else "Εφαρμογή")
        self._apply.setEnabled(n > 0)
        self._revert.setEnabled(n > 0)
        is_cal = self._node == CALIBRATION_NODE
        self._save.setVisible(is_cal)
        self._load_file.setVisible(is_cal)
        self._save.setEnabled(n == 0)  # apply first, so the file gets what the node really holds
        self._save.setToolTip(
            "Γράφει τις τρέχουσες τιμές στο calibration.yaml" if n == 0 else "Πρώτα «Εφαρμογή» των αλλαγών"
        )
        self._reload.setEnabled(self._nodes.count() > 0)
        theme.repolish(self._apply)
