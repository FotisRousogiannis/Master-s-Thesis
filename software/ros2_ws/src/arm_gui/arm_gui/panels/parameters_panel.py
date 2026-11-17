"""Parameters panel: read and change the ROS 2 parameters of any node (ADR-0017).

For /arm_calibration (servo pulse widths, channels, encoder zeros, safety limits) it also saves
the values to the calibration file in the repository, reloads them from it, and sets the encoder
zeros from the arm in its reference pose (ADR-0019): for the joints the user ticks, by default the
ones whose encoder reports now; all of them at once through ~/set_zero (all or none), a subset
through ~/set_zero/<joint> (so it works before every encoder is connected).
"""

import time

from .. import theme
from ..qt import Qt, QtWidgets, Signal

CALIBRATION_NODE = "/arm_calibration"
REPORT_MAX_AGE_S = 2.0  # an encoder report older than this does not count as "reporting now"
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

    def __init__(self, parent=None, clock=time.monotonic):
        super().__init__(parent)
        self._clock = clock
        self._editors: dict[str, _Editor] = {}
        self._encoders = {}  # joint -> (time, level, has raw_deg) of its last encoder/<joint> report
        self._keep_status = False  # a service result stays visible while the values are re-read
        self._node = None
        layout = QtWidgets.QVBoxLayout(self)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Node"))
        self._nodes = QtWidgets.QComboBox()
        self._nodes.setMinimumContentsLength(20)
        self._nodes.activated.connect(lambda _i: self.reload())
        top.addWidget(self._nodes, 1)
        self._reload = QtWidgets.QPushButton("Ανάγνωση")
        self._reload.clicked.connect(lambda: self.reload())
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
        self._set_zero = QtWidgets.QPushButton("Μηδέν = τρέχουσα θέση")
        self._set_zero.setToolTip(
            "Ο βραχίονας στη στάση αναφοράς. Οι ενδείξεις των encoders που επιλέγεις γίνονται τα νέα μηδενικά."
        )
        self._set_zero.clicked.connect(self.set_zero)
        for b in (self._apply, self._revert):
            buttons.addWidget(b)
        buttons.addStretch(1)
        for b in (self._set_zero, self._save, self._load_file):
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
        if not self._keep_status:
            self._status.setText(f"{len(params)} παράμετροι του {node}")
        self._keep_status = False
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

    def update_diagnostics(self, statuses):
        """Remember which encoders report now (encoder/<joint> in /diagnostics), for «Μηδέν»."""
        now = self._clock()
        for s in statuses:
            if s["name"].startswith("encoder/"):
                self._encoders[s["name"].split("/", 1)[1]] = (now, s["level"], "raw_deg" in s["values"])

    def encoder_state(self, joint) -> str:
        """'ok', 'problem' or 'silent' for the encoder of this joint, from the last reports."""
        report = self._encoders.get(joint)
        if report is None or self._clock() - report[0] > REPORT_MAX_AGE_S:
            return "silent"
        _, level, has_raw = report
        return "ok" if level == 0 and has_raw else "problem"

    def show_service_result(self, service, ok, message):
        if service.startswith(CALIBRATION_NODE):
            prefix = "" if "/set_zero/" not in service else f"{service.rsplit('/', 1)[1]}: "
            self._status.setText(("✔ " if ok else "✖ ") + prefix + message)
            if ok and (service.endswith("/reload") or "/set_zero" in service):
                self.reload(keep_status=True)

    # ------------------------------------------------------------------ actions
    def reload(self, keep_status=False):
        self._node = self._nodes.currentText() or None
        self._keep_status = keep_status
        if self._node:
            if not keep_status:
                self._status.setText(f"Ανάγνωση παραμέτρων του {self._node}…")
            self.load_requested.emit(self._node)
        self._update_buttons()

    def apply(self):
        changed = {n: e.value for n, e in self._editors.items() if e.modified}
        if changed and self._node:
            self._status.setText(f"Εφαρμογή {len(changed)} αλλαγών…")
            self.apply_requested.emit(self._node, changed)

    def zero_joints(self) -> list:
        """The joints that have an encoder zero in the calibration (encoders.<joint>.zero_deg)."""
        return sorted(n.split(".")[1] for n in self._editors if n.startswith("encoders.") and n.endswith(".zero_deg"))

    def set_zero(self):
        joints = self.zero_joints()
        chosen = self.choose_joints(joints, {j: self.encoder_state(j) for j in joints})
        if not chosen:
            return
        self._status.setText("Ορισμός μηδενικών: " + ", ".join(chosen) + "…")
        if sorted(chosen) == joints:
            self.service_requested.emit(f"{CALIBRATION_NODE}/set_zero")  # all or none, in one step
        else:
            for joint in chosen:
                self.service_requested.emit(f"{CALIBRATION_NODE}/set_zero/{joint}")

    def choose_joints(self, joints, states):
        """Dialog: tick the joints whose current reading becomes their zero. None = cancelled."""
        labels = {"ok": "μετρά", "problem": "πρόβλημα στον encoder", "silent": "δεν αναφέρει"}
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Μηδέν = τρέχουσα θέση")
        layout = QtWidgets.QVBoxLayout(dialog)
        text = QtWidgets.QLabel(
            "Ο βραχίονας στη στάση αναφοράς: οι επιλεγμένες αρθρώσεις στο 0.\n"
            "Η τωρινή ένδειξη του encoder τους γίνεται το νέο μηδέν· οι άλλες δεν αλλάζουν.\n"
            "Αν κάποια επιλεγμένη δεν μετρά, δεν αλλάζει (μήνυμα ανά άρθρωση).")
        text.setWordWrap(True)
        layout.addWidget(text)
        boxes = {}
        for joint in joints:
            box = QtWidgets.QCheckBox(f"{joint}  ({labels[states[joint]]})")
            box.setChecked(states[joint] == "ok")
            layout.addWidget(box)
            boxes[joint] = box
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return None
        return [j for j, box in boxes.items() if box.isChecked()]

    def confirm(self, title, text) -> bool:
        answer = QtWidgets.QMessageBox.question(self, title, text)
        return answer == QtWidgets.QMessageBox.Yes

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
        self._set_zero.setVisible(is_cal)
        self._set_zero.setEnabled(n == 0)
        self._save.setEnabled(n == 0)  # apply first, so the file gets what the node really holds
        self._save.setToolTip(
            "Γράφει τις τρέχουσες τιμές στο calibration.yaml" if n == 0 else "Πρώτα «Εφαρμογή» των αλλαγών"
        )
        self._reload.setEnabled(self._nodes.count() > 0)
        theme.repolish(self._apply)
