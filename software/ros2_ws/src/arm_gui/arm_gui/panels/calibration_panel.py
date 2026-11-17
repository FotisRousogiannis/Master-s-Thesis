"""Calibration panel (ADR-0023): the one arm file in three pages, and the wizard on top of them.

    Servo        servos:         channel, neutral, deadband, description   + where it is used
    Αισθητήρες   encoders, force_sensors: where each is                    + live reading / status
    Αρθρώσεις    joints:         enabled, servos with sign, encoder, zero, direction, limits, deadband
                                                                           + live position / status
    Οδηγός       the procedure: scan, jog per joint, direction, deadbands, zero, gripper -> proposals

Values come from /arm_calibration (flat parameters of arm.yaml); «Εφαρμογή» sends the edited ones
all at once, and the node validates the whole file. «Αποθήκευση» is the Parameters panel's.
"""

import time

from ..calibration import Wizard, nest
from ..qt import QtCore, QtWidgets, Signal

CALIBRATION_NODE = "/arm_calibration"
KEEPALIVE_MS = 2000      # the Pi leaves identification mode after 5 s without a request
RELOAD_AFTER_S = 6.0     # no values yet while /arm_calibration runs: ask again (a reply can be lost)
NONE = "—"


def _text(value) -> str:
    if value is None:
        return NONE
    if isinstance(value, bool):
        return "ναι" if value else "όχι"
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)


def parse(text: str, like):
    """Cell text -> value of the same type as `like` (the value in the file); ValueError if impossible."""
    text = text.strip()
    if text in ("", NONE):
        raise ValueError("κενό")
    if isinstance(like, bool):
        if text.lower() in ("ναι", "true", "1", "yes"):
            return True
        if text.lower() in ("όχι", "οχι", "false", "0", "no"):
            return False
        raise ValueError(f"«{text}»: ναι ή όχι")
    if isinstance(like, int):
        return int(text)
    if isinstance(like, float):
        return float(text)
    if like is None:                                   # a null of the file: a number if it looks like one
        try:
            return int(text)
        except ValueError:
            return text
    return text


def signs_text(servos: dict) -> str:
    return "  ".join(f"{name}:{sign:+d}" for name, sign in servos.items())


class _Table(QtWidgets.QTableWidget):
    """Rows of one section of the file; editable cells know their parameter (name, element of a pair)."""

    def __init__(self, headers):
        super().__init__(0, len(headers))
        self.setHorizontalHeaderLabels(headers)
        self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.horizontalHeader().setStretchLastSection(True)
        self.verticalHeader().setVisible(False)
        self.cells = {}                    # (row, col) -> (parameter, index or None, value in the file)
        self.live = {}                     # (row key, live column name) -> item

    def set_rows(self, rows):
        """rows: [(key, [cell])], cell = ('edit', parameter, index, value) | ('show', text)."""
        self.blockSignals(True)
        self.setRowCount(len(rows))
        self.cells, self.live = {}, {}
        for r, (key, cells) in enumerate(rows):
            for c, cell in enumerate(cells):
                if cell[0] == "edit":
                    _, name, index, value = cell
                    item = QtWidgets.QTableWidgetItem(_text(value))
                    self.cells[(r, c)] = (name, index, value)
                else:
                    item = QtWidgets.QTableWidgetItem(cell[1])
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                    if cell[0] == "live":
                        self.live[(key, cell[2])] = item
                self.setItem(r, c, item)
        self.blockSignals(False)

    def set_live(self, key, column, text):
        item = self.live.get((key, column))
        if item is not None and item.text() != text:
            item.setText(text)

    def edits(self, values: dict, errors: list) -> None:
        """Cells that differ from the file -> {parameter: value}; pairs are sent whole."""
        for (r, c), (name, index, value) in self.cells.items():
            text = self.item(r, c).text()
            if text == _text(value):
                continue
            try:
                if index is None:
                    values[name] = parse(text, value)
                else:
                    pair = list(values.get(name, self._pair(r, name)))
                    pair[index] = parse(text, value)
                    values[name] = pair
            except ValueError as exc:
                errors.append(f"{name}: {exc}")

    def _pair(self, row, name):
        pair = [None, None]
        for (r, c), (n, index, value) in self.cells.items():
            if r == row and n == name:
                pair[index] = value
        return pair


class CalibrationPanel(QtWidgets.QWidget):
    load_requested = Signal(str)                  # node
    apply_requested = Signal(str, object)         # node, {parameter: value}: all at once
    identify_requested = Signal(object)           # request to the Pi (dict)

    def __init__(self, parent=None, confirm=None, clock=time.monotonic):
        super().__init__(parent)
        self.data = None
        self._clock = clock
        self._asked = None                         # when values were last requested
        self.wizard = Wizard()
        self._confirm = confirm or self._ask
        layout = QtWidgets.QVBoxLayout(self)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)

        self.servos = _Table(["Servo", "Κανάλι PCA", "Νεκρό σημείο [µs]", "Ζώνη κάτω [µs]", "Ζώνη πάνω [µs]",
                              "Περιγραφή", "Αρθρώσεις"])
        self.encoders = _Table(["Encoder", "Κανάλι TCA", "Άρθρωση", "Ένδειξη [°]", "Μαγνήτης"])
        self.forces = _Table(["Αισθητήρας", "Κανάλι ADS1115", "Είσοδος", "Μέγ. πίεση [V]", "Τάση [V]",
                              "Κατάσταση"])
        self.joints = _Table(["Άρθρωση", "Ενεργή", "Περιγραφή", "Servo (πρόσημο)", "Encoder", "Μηδέν [°]",
                              "Αντίθετη φορά", "Όριο κάτω [°]", "Όριο πάνω [°]", "Ζώνη +", "Ζώνη −",
                              "Θέση [°]", "Κατάσταση"])
        sensors = QtWidgets.QWidget()
        sl = QtWidgets.QVBoxLayout(sensors)
        sl.addWidget(QtWidgets.QLabel("Encoders (AS5600)"))
        sl.addWidget(self.encoders)
        sl.addWidget(QtWidgets.QLabel("Αισθητήρες πίεσης (ZD10 μέσω ADS1115)"))
        sl.addWidget(self.forces)
        self.tabs.addTab(self.servos, "Servo")
        self.tabs.addTab(sensors, "Αισθητήρες")
        self.tabs.addTab(self.joints, "Αρθρώσεις")
        self.tabs.addTab(self._build_wizard(), "Οδηγός")

        buttons = QtWidgets.QHBoxLayout()
        self.apply_button = QtWidgets.QPushButton("Εφαρμογή αλλαγών")
        self.apply_button.setToolTip("Στέλνει όσα κελιά άλλαξες στο /arm_calibration (όλα μαζί)· "
                                     "«Αποθήκευση στο αρχείο» από το πάνελ «Παράμετροι»")
        self.apply_button.clicked.connect(self.apply_edits)
        reload_button = QtWidgets.QPushButton("Ανανέωση")
        reload_button.clicked.connect(lambda: self.load_requested.emit(CALIBRATION_NODE))
        buttons.addWidget(self.apply_button)
        buttons.addWidget(reload_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        self.status = QtWidgets.QLabel("Περιμένω το /arm_calibration…")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        self._keepalive = QtCore.QTimer(self)
        self._keepalive.timeout.connect(self._send_keepalive)
        self._keepalive.start(KEEPALIVE_MS)

    # ---------------------------------------------------------------- wizard page
    def _build_wizard(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        warning = QtWidgets.QLabel("⚠ Τα βήματα με κίνηση κινούν το υλικό: E-stop στο χέρι. Όσο τρέχει ο οδηγός, "
                                   "ο κανονικός έλεγχος του Pi είναι σταματημένος (αναγνώριση).")
        warning.setWordWrap(True)
        v.addWidget(warning)
        self.mode = QtWidgets.QLabel("Pi: κανονικός έλεγχος")
        v.addWidget(self.mode)

        row = QtWidgets.QHBoxLayout()
        for text, slot in (("1–2 · Σάρωση I²C", self.do_scan), ("Ανάγνωση encoders", self.do_read),
                           ("Έξοδος από αναγνώριση", self.do_exit)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            row.addWidget(button)
        v.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        self.joint_choice = QtWidgets.QComboBox()
        self.jog_button = QtWidgets.QPushButton("3 · Κίνηση άρθρωσης")
        self.jog_button.clicked.connect(self.do_jog)
        self.actual_choice = QtWidgets.QComboBox()
        relabel = QtWidgets.QPushButton("Κινήθηκε αυτή")
        relabel.setToolTip("Αν κινήθηκε άλλη άρθρωση από αυτή που διάλεξες (π.χ. διαφορικό με ανάποδα ονόματα)")
        relabel.clicked.connect(lambda: self._answer(lambda: self.wizard.relabel(self.actual_choice.currentText())))
        row.addWidget(QtWidgets.QLabel("Άρθρωση:"))
        row.addWidget(self.joint_choice)
        row.addWidget(self.jog_button)
        row.addWidget(QtWidgets.QLabel("Κινήθηκε η:"))
        row.addWidget(self.actual_choice)
        row.addWidget(relabel)
        v.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("5 · Η κίνηση ήταν προς τα:"))
        for text, positive in (("θετικά", True), ("αρνητικά", False)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda _=False, p=positive: self._answer(lambda: self.wizard.direction(p)))
            row.addWidget(button)
        row.addStretch(1)
        v.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        self.servo_choice = QtWidgets.QComboBox()
        servo_sweep = QtWidgets.QPushButton("6 · Νεκρή ζώνη servo")
        servo_sweep.clicked.connect(lambda: self._move(self.wizard.servo_deadband, self.servo_choice.currentText()))
        joint_sweep = QtWidgets.QPushButton("Νεκρή ζώνη άρθρωσης")
        joint_sweep.clicked.connect(lambda: self._move(self.wizard.joint_deadband, self.joint_choice.currentText()))
        stop = QtWidgets.QPushButton("Διακοπή")
        stop.clicked.connect(lambda: self.identify_requested.emit(self.wizard.stop()))
        for w in (QtWidgets.QLabel("Servo:"), self.servo_choice, servo_sweep, joint_sweep, stop):
            row.addWidget(w)
        v.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        zero = QtWidgets.QPushButton("7 · Μηδέν = τωρινή θέση")
        zero.setToolTip("Από την τελευταία «Ανάγνωση encoders», για την άρθρωση που επέλεξες")
        zero.clicked.connect(lambda: self._reading(self.wizard.zero, self.joint_choice.currentText()))
        closed = QtWidgets.QPushButton("8α · Αρπάγη κλειστή")
        closed.clicked.connect(lambda: self._reading(lambda j: self.wizard.gripper(j, True), "gripper"))
        opened = QtWidgets.QPushButton("Αρπάγη ανοιχτή")
        opened.clicked.connect(lambda: self._reading(lambda j: self.wizard.gripper(j, False), "gripper"))
        pressure = QtWidgets.QPushButton("8β · Μέγιστη πίεση")
        pressure.setEnabled(False)
        pressure.setToolTip("Θέλει αργό κλείσιμο με ζωντανή πίεση στο Pi: δεν υπάρχει ακόμα")
        for w in (zero, closed, opened, pressure):
            row.addWidget(w)
        v.addLayout(row)

        self.findings = QtWidgets.QListWidget()
        v.addWidget(QtWidgets.QLabel("Ευρήματα"))
        v.addWidget(self.findings, 1)
        self.proposals = QtWidgets.QTableWidget(0, 5)
        self.proposals.setHorizontalHeaderLabels(["Έγκριση", "Παράμετρος", "Αρχείο", "Μετρήθηκε", "Γιατί"])
        self.proposals.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.proposals.horizontalHeader().setStretchLastSection(True)
        self.proposals.verticalHeader().setVisible(False)
        self.proposals.itemChanged.connect(self._on_approval)
        v.addWidget(QtWidgets.QLabel("Προτάσεις (αρχείο → μετρήθηκε)"))
        v.addWidget(self.proposals, 1)
        row = QtWidgets.QHBoxLayout()
        apply = QtWidgets.QPushButton("Εφαρμογή εγκεκριμένων")
        apply.clicked.connect(self.apply_proposals)
        clear = QtWidgets.QPushButton("Καθαρισμός")
        clear.clicked.connect(self._clear)
        row.addWidget(apply)
        row.addWidget(clear)
        row.addStretch(1)
        v.addLayout(row)
        return page

    def _ask(self, text) -> bool:
        answer = QtWidgets.QMessageBox.question(self, "Κίνηση", text)
        return answer == QtWidgets.QMessageBox.Yes

    def _ready(self) -> bool:
        if self.data is None:
            self.status.setText("Δεν έχουν φορτωθεί τιμές από το /arm_calibration.")
        elif self.wizard.busy:
            self.status.setText("Τρέχει ήδη κίνηση· περίμενε ή «Διακοπή».")
        else:
            return True
        return False

    def _send(self, request) -> None:
        self.identify_requested.emit(request)
        self._refresh_wizard()

    def do_scan(self):
        if self._ready():
            self._send(self.wizard.scan())

    def do_read(self):
        if self._ready():
            self._send(self.wizard.read())

    def do_exit(self):
        self._send(self.wizard.exit())

    def do_jog(self):
        self._move(self.wizard.jog_joint, self.joint_choice.currentText())

    def _move(self, make_request, name):
        if not name or not self._ready():
            return
        if not self._confirm(f"Θα κινηθεί λίγο ({name}). E-stop στο χέρι, ελεύθερη διαδρομή;"):
            return
        try:
            request = make_request(name)
        except (KeyError, ValueError) as exc:
            self.status.setText(f"Δεν ξεκίνησε: {exc}")
            return
        self._send(request)

    def _reading(self, action, joint):
        if self.data is None or joint not in self.data.get("joints", {}):
            return
        if not action(joint):
            self.status.setText(f"{joint}: καμία έγκυρη ένδειξη· πρώτα «Ανάγνωση encoders».")
        self._refresh_wizard()

    def _answer(self, action):
        action()
        self._refresh_wizard()

    def _clear(self):
        self.wizard.findings, self.wizard.proposals = [], []
        self._refresh_wizard()

    def _send_keepalive(self):
        request = self.wizard.keepalive()
        if request is not None:
            self.identify_requested.emit(request)

    def on_identify_reply(self, reply: dict):
        for request in self.wizard.handle(reply):
            self.identify_requested.emit(request)
        if self.wizard.readings:
            self._show_readings()
        self._refresh_wizard()

    def _on_approval(self, item):
        if item.column() == 0 and item.row() < len(self.wizard.proposals):
            self.wizard.proposals[item.row()][1] = item.checkState() == QtCore.Qt.Checked

    def apply_proposals(self):
        values, manual = self.wizard.approved()
        if manual:
            self.status.setText("Θέλουν αλλαγή στο αρχείο με το χέρι (αφαίρεση): " + ", ".join(manual))
        if values:
            self.apply_requested.emit(CALIBRATION_NODE, values)
        elif not manual:
            self.status.setText("Καμία εγκεκριμένη πρόταση.")

    def _refresh_wizard(self):
        self.mode.setText("Pi: αναγνώριση" + (" · κίνηση…" if self.wizard.busy else "") if self.wizard.active
                          else "Pi: κανονικός έλεγχος")
        self.findings.clear()
        marks = {"info": "•", "warning": "⚠", "error": "✖"}
        for f in self.wizard.findings:
            self.findings.addItem(f"{marks.get(f.level, '•')} {f.text}")
        self.proposals.blockSignals(True)
        self.proposals.setRowCount(len(self.wizard.proposals))
        for r, (p, ok) in enumerate(self.wizard.proposals):
            check = QtWidgets.QTableWidgetItem()
            check.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
            check.setCheckState(QtCore.Qt.Checked if ok else QtCore.Qt.Unchecked)
            self.proposals.setItem(r, 0, check)
            for c, text in enumerate((p.name, _text(p.old), _text(p.new), p.why), start=1):
                item = QtWidgets.QTableWidgetItem(text)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.proposals.setItem(r, c, item)
        self.proposals.blockSignals(False)

    # ---------------------------------------------------------------- pages
    def set_nodes(self, nodes):
        """While /arm_calibration runs and no values came (e.g. a lost reply), ask again."""
        if CALIBRATION_NODE not in nodes or self.data is not None:
            return
        now = self._clock()
        if self._asked is None:
            self._asked = now          # the Parameters panel asks as soon as the node appears
        elif now - self._asked > RELOAD_AFTER_S:
            self._asked = now
            self.load_requested.emit(CALIBRATION_NODE)

    def show_parameters(self, node, params):
        if node != CALIBRATION_NODE:
            return
        if isinstance(params, str):
            self.status.setText(params)
            return
        values = {p["name"]: p["value"] for p in params if p["name"] != "file"}
        if not any(name.startswith("joints.") for name in values):
            return
        self.data = nest(values)
        self.wizard.data = self.data
        self._fill()
        self.status.setText(f"{len(values)} τιμές από το {CALIBRATION_NODE}")

    def _fill(self):
        d = self.data
        users = {}
        for joint, j in d["joints"].items():
            for servo in j.get("servos", {}):
                users.setdefault(servo, []).append(joint)
        rows = []
        for name, s in d["servos"].items():
            band = s.get("deadband_us") or [None, None]
            key = f"servos.{name}"
            rows.append((name, [("show", name), ("edit", f"{key}.channel", None, s.get("channel")),
                                ("edit", f"{key}.neutral_us", None, s.get("neutral_us")),
                                ("edit", f"{key}.deadband_us", 0, band[0]), ("edit", f"{key}.deadband_us", 1, band[1]),
                                ("edit", f"{key}.description", None, s.get("description", "")),
                                ("show", ", ".join(users.get(name, [])) or NONE)]))
        self.servos.set_rows(rows)

        owner = {j.get("encoder"): joint for joint, j in d["joints"].items() if j.get("encoder")}
        self.encoders.set_rows([(name, [("show", name), ("edit", f"encoders.{name}.channel", None, e.get("channel")),
                                        ("show", owner.get(name, NONE)), ("live", NONE, "raw"),
                                        ("live", NONE, "status")]) for name, e in d["encoders"].items()])
        self.forces.set_rows([(name, [("show", name),
                                      ("edit", f"force_sensors.{name}.adc_channel", None, f.get("adc_channel")),
                                      ("edit", f"force_sensors.{name}.input", None, f.get("input")),
                                      ("edit", f"force_sensors.{name}.max_v", None, f.get("max_v")),
                                      ("live", NONE, "value"), ("live", NONE, "status")])
                              for name, f in d["force_sensors"].items()])
        rows = []
        for name, j in d["joints"].items():
            key = f"joints.{name}"
            limits = j.get("limits_deg") or [None, None]
            band = j.get("deadband") or [None, None]
            rows.append((name, [("show", name), ("edit", f"{key}.enabled", None, j.get("enabled")),
                                ("edit", f"{key}.description", None, j.get("description", "")),
                                ("show", signs_text(j.get("servos", {}))),
                                ("edit", f"{key}.encoder", None, j.get("encoder")),
                                ("edit", f"{key}.zero_deg", None, j.get("zero_deg")),
                                ("edit", f"{key}.invert", None, j.get("invert")),
                                ("edit", f"{key}.limits_deg", 0, limits[0]),
                                ("edit", f"{key}.limits_deg", 1, limits[1]),
                                ("edit", f"{key}.deadband", 0, band[0]), ("edit", f"{key}.deadband", 1, band[1]),
                                ("live", NONE, "position"), ("live", NONE, "status")]))
        self.joints.set_rows(rows)
        for combo, names in ((self.joint_choice, list(d["joints"])), (self.actual_choice, list(d["joints"])),
                             (self.servo_choice, list(d["servos"]))):
            current = combo.currentText()
            combo.clear()
            combo.addItems(names)
            if current in names:
                combo.setCurrentText(current)

    def apply_edits(self):
        values, errors = {}, []
        for table in (self.servos, self.encoders, self.forces, self.joints):
            table.edits(values, errors)
        if errors:
            self.status.setText("Δεν στάλθηκαν:\n" + "\n".join(errors))
            return
        if not values:
            self.status.setText("Καμία αλλαγή.")
            return
        self.apply_requested.emit(CALIBRATION_NODE, values)
        self.status.setText(f"Στάλθηκαν {len(values)} αλλαγές…")

    def show_set_results(self, node, results):
        if node != CALIBRATION_NODE:
            return
        rejected = sorted({reason for _, ok, reason in results if not ok})
        if rejected:
            self.status.setText("Απορρίφθηκαν (καμία δεν εφαρμόστηκε):\n" + "\n".join(rejected))
            return
        self.wizard.applied({name for name, ok, _ in results if ok})
        self.status.setText(f"Εφαρμόστηκαν {len(results)}. «Αποθήκευση στο αρχείο» από το πάνελ «Παράμετροι».")
        self._refresh_wizard()
        self.load_requested.emit(CALIBRATION_NODE)

    # ---------------------------------------------------------------- live values
    def update_diagnostics(self, statuses):
        if self.data is None:
            return
        encoder_of = {joint: j.get("encoder") for joint, j in self.data["joints"].items()}
        for s in statuses:
            kind, _, name = s["name"].partition("/")
            values = s["values"]
            if kind == "encoder" and name in encoder_of:
                self.joints.set_live(name, "position", values.get("position_deg", NONE))
                self.joints.set_live(name, "status", s["message"])
                if encoder_of[name]:
                    self.encoders.set_live(encoder_of[name], "raw", values.get("raw_deg", NONE))
                    self.encoders.set_live(encoder_of[name], "status", s["message"])
            elif kind == "force":
                self.forces.set_live(name, "value", values.get("value", NONE))
                self.forces.set_live(name, "status", s["message"])

    def _show_readings(self):
        """In identification mode the readings come by channel (all encoders, not only the joints')."""
        for name, e in self.data["encoders"].items():
            reading = self.wizard.readings.get(e.get("channel"))
            if reading:
                raw = reading.get("raw_deg")
                self.encoders.set_live(name, "raw", NONE if raw is None else f"{raw:.2f}")
                self.encoders.set_live(name, "status", reading.get("status", NONE))
