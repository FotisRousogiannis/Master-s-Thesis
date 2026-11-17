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
from arm_calibration import procedure

from .. import theme
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
    service_requested = Signal(str)               # Trigger service (save the file)

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
        joints = QtWidgets.QWidget()
        jl = QtWidgets.QVBoxLayout(joints)
        jl.setContentsMargins(0, 0, 0, 0)
        self.details = QtWidgets.QCheckBox("Λεπτομέρειες (μηδέν, φορά, όρια, νεκρή ζώνη)")
        self.details.toggled.connect(self._show_details)
        jl.addWidget(self.details)
        jl.addWidget(self.joints)
        self.tabs.addTab(self._build_wizard(), "Οδηγός")
        self.tabs.addTab(self.servos, "Servo")
        self.tabs.addTab(sensors, "Αισθητήρες")
        self.tabs.addTab(joints, "Αρθρώσεις")
        self._show_details(False)

        self.page_buttons = QtWidgets.QWidget()          # for the three pages, not the wizard
        buttons = QtWidgets.QHBoxLayout(self.page_buttons)
        buttons.setContentsMargins(0, 0, 0, 0)
        self.apply_button = QtWidgets.QPushButton("Εφαρμογή αλλαγών")
        self.apply_button.setToolTip("Στέλνει όσα κελιά άλλαξες στο /arm_calibration (όλα μαζί)· "
                                     "«Αποθήκευση στο αρχείο» από το πάνελ «Παράμετροι»")
        self.apply_button.clicked.connect(self.apply_edits)
        reload_button = QtWidgets.QPushButton("Ανανέωση")
        reload_button.clicked.connect(lambda: self.load_requested.emit(CALIBRATION_NODE))
        save_button = QtWidgets.QPushButton("Αποθήκευση στο αρχείο")
        save_button.setToolTip("Γράφει τις τιμές που ισχύουν στο arm.yaml")
        save_button.clicked.connect(self.save_file)
        buttons.addWidget(self.apply_button)
        buttons.addWidget(save_button)
        buttons.addWidget(reload_button)
        buttons.addStretch(1)
        layout.addWidget(self.page_buttons)
        self.tabs.currentChanged.connect(lambda i: self.page_buttons.setVisible(i != 0))
        self.page_buttons.setVisible(False)
        self.status = QtWidgets.QLabel("Περιμένω το /arm_calibration…")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        self._keepalive = QtCore.QTimer(self)
        self._keepalive.timeout.connect(self._send_keepalive)
        self._keepalive.start(KEEPALIVE_MS)

    # ---------------------------------------------------------------- wizard page
    # (short name, title, what to do, servo power on, shows the joint choice, shows the servo choice)
    STEPS = (
        ("Κανάλια", "Σε ποιο κανάλι είναι κάθε servo",
         "Γράψε το κανάλι του PCA9685 για κάθε servo στη σελίδα «Servo» και «Εφαρμογή αλλαγών». "
         "Τίποτα δεν κινείται σε αυτό το βήμα.", False, False, False),
        ("Σάρωση", "Τι υπάρχει στο I²C",
         "Ελέγχει τι απαντά σε κάθε κανάλι και αν κάθε encoder βλέπει μαγνήτη.", False, False, False),
        ("Αντιστοίχιση", "Ποιος encoder ανήκει σε ποια άρθρωση",
         "Μία άρθρωση τη φορά: μικρή κίνηση, στάση με την πρώτη αλλαγή encoder. Αν κινήθηκε άλλη άρθρωση από "
         "αυτή που διάλεξες, πες ποια.", True, True, False),
        ("Έλεγχος", "Αρχείο → μετρήθηκε",
         "Ό,τι μετρήθηκε και διαφέρει από το αρχείο. Τσέκαρε όσα θέλεις να αλλάξουν και «Εφαρμογή».",
         False, False, False),
        ("Φορά", "Θετική φορά",
         "Το λογισμικό δεν ξέρει ποια κατεύθυνση λες «θετική»: κίνησε την άρθρωση και πες προς τα πού πήγε.",
         True, True, False),
        ("Νεκρές ζώνες", "Νεκρή ζώνη servo και άρθρωσης",
         "Ο παλμός ανεβαίνει σιγά-σιγά μέχρι να κινηθεί ο encoder, και προς τις δύο φορές.", True, True, True),
        ("Μηδέν", "Θέση αναφοράς",
         "Φέρε την άρθρωση στη θέση αναφοράς της (με το χέρι, ρεύμα σβηστό), «Ανάγνωση» και «Μηδέν εδώ».",
         False, True, False),
        ("Αρπάγη", "Όρια και πίεση της αρπάγης",
         "Κλειστή = 0°. Κινείται στο μεσαίο 90 % της διαδρομής και σταματά στο 90 % της μέγιστης πίεσης.",
         True, False, False),
    )

    def _button(self, text, slot, primary=False):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(slot)
        if primary:
            button.setProperty("primary", True)
        return button

    def _build_wizard(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)

        steps = QtWidgets.QHBoxLayout()
        self.step_buttons = []
        for i, step in enumerate(self.STEPS):
            button = self._button(f"{i + 1} · {step[0]}", lambda _=False, n=i: self.go_step(n))
            self.step_buttons.append(button)
            steps.addWidget(button)
        steps.addStretch(1)
        v.addLayout(steps)

        head = QtWidgets.QHBoxLayout()
        texts = QtWidgets.QVBoxLayout()
        self.step_number = QtWidgets.QLabel()
        self.step_number.setObjectName("step_number")
        self.step_title = QtWidgets.QLabel()
        self.step_title.setObjectName("step_title")
        self.step_hint = QtWidgets.QLabel()
        self.step_hint.setWordWrap(True)
        for w in (self.step_number, self.step_title, self.step_hint):
            texts.addWidget(w)
        head.addLayout(texts, 1)
        chips = QtWidgets.QVBoxLayout()
        self.power = QtWidgets.QLabel()
        self.power.setProperty("pill", True)
        self.mode = QtWidgets.QLabel("Pi: κανονικός έλεγχος")
        self.mode.setProperty("pill", True)
        chips.addWidget(self.power)
        chips.addWidget(self.mode)
        chips.addStretch(1)
        head.addLayout(chips)
        v.addLayout(head)

        choices = QtWidgets.QHBoxLayout()
        self.joint_choice = QtWidgets.QComboBox()
        self.servo_choice = QtWidgets.QComboBox()
        self.joint_label, self.servo_label = QtWidgets.QLabel("Άρθρωση:"), QtWidgets.QLabel("Servo:")
        # A real arm may need more than the defaults (bench: 0.4 for 0.5 s moved the base < 1.5°)
        self.speed = QtWidgets.QDoubleSpinBox()
        self.speed.setRange(0.05, 1.0)
        self.speed.setSingleStep(0.05)
        self.speed.setValue(0.4)
        self.speed.setToolTip("Ταχύτητα της κίνησης (κλάσμα του μέγιστου)")
        self.duration = QtWidgets.QDoubleSpinBox()
        self.duration.setRange(0.1, 3.0)
        self.duration.setSingleStep(0.1)
        self.duration.setValue(0.5)
        self.duration.setSuffix(" s")
        self.duration.setToolTip("Το πολύ τόσο· σταματά νωρίτερα με την πρώτη αλλαγή 3° σε encoder (όριο Pi: 3 s)")
        self.motion_label, self.duration_label = QtWidgets.QLabel("Ταχύτητα:"), QtWidgets.QLabel("Διάρκεια έως:")
        for w in (self.joint_label, self.joint_choice, self.servo_label, self.servo_choice,
                  self.motion_label, self.speed, self.duration_label, self.duration):
            choices.addWidget(w)
        choices.addStretch(1)
        v.addLayout(choices)

        self.step_pages = QtWidgets.QStackedWidget()
        self.actual_choice = QtWidgets.QComboBox()
        # «Κινήθηκε η» starts as the joint that was moved: the user changes it only if another moved
        self.joint_choice.currentTextChanged.connect(self.actual_choice.setCurrentText)
        self.jog_button = self._button("Κίνηση", self.do_jog, primary=True)
        self.proposals = QtWidgets.QTableWidget(0, 5)
        self.proposals.setHorizontalHeaderLabels(["Έγκριση", "Τι", "Αρχείο", "Μετρήθηκε", "Γιατί"])
        self.proposals.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.proposals.horizontalHeader().setStretchLastSection(True)
        self.proposals.verticalHeader().setVisible(False)
        self.proposals.itemChanged.connect(self._on_approval)
        pressure = QtWidgets.QPushButton("Μέγιστη πίεση")
        pressure.setEnabled(False)
        pressure.setToolTip("Θέλει αργό κλείσιμο με ζωντανή πίεση στο Pi: δεν υπάρχει ακόμα")
        relabel = self._button("Ήταν αυτή",
                               lambda: self._answer(lambda: self.wizard.relabel(self.actual_choice.currentText())))
        pages = (
            [self._button("Άνοιγμα της σελίδας «Servo»", lambda: self.tabs.setCurrentWidget(self.servos))],
            [self._button("Σάρωση I²C", self.do_scan, primary=True), self._button("Ανάγνωση encoders", self.do_read)],
            [self.jog_button, QtWidgets.QLabel("Κινήθηκε η:"), self.actual_choice, relabel],
            [self.proposals],
            [self._button("Κίνηση", self.do_jog, primary=True),
             self._button("Ήταν προς τα θετικά", lambda: self._answer(lambda: self.wizard.direction(True))),
             self._button("Ήταν προς τα αρνητικά", lambda: self._answer(lambda: self.wizard.direction(False)))],
            [self._button("Νεκρή ζώνη servo",
                          lambda: self._move(self.wizard.servo_deadband, self.servo_choice.currentText()),
                          primary=True),
             self._button("Νεκρή ζώνη άρθρωσης",
                          lambda: self._move(self.wizard.joint_deadband, self.joint_choice.currentText())),
             self._button("Διακοπή", lambda: self.identify_requested.emit(self.wizard.stop()))],
            [self._button("Ανάγνωση encoders", self.do_read),
             self._button("Μηδέν εδώ", lambda: self._reading(self.wizard.zero, self.joint_choice.currentText()),
                          primary=True)],
            [self._button("Ανάγνωση encoders", self.do_read),
             self._button("Κλειστή: κράτα ένδειξη",
                          lambda: self._reading(lambda j: self.wizard.gripper(j, True), "gripper")),
             self._button("Ανοιχτή: κράτα ένδειξη",
                          lambda: self._reading(lambda j: self.wizard.gripper(j, False), "gripper")),
             pressure],
        )
        for widgets in pages:
            w = QtWidgets.QWidget()
            box = QtWidgets.QVBoxLayout(w) if widgets[0] is self.proposals else QtWidgets.QHBoxLayout(w)
            box.setContentsMargins(0, 0, 0, 0)
            for item in widgets:
                box.addWidget(item)
            if widgets[0] is not self.proposals:
                box.addStretch(1)
            self.step_pages.addWidget(w)
        v.addWidget(self.step_pages, 1)

        v.addWidget(QtWidgets.QLabel("Ευρήματα"))
        self.findings = QtWidgets.QListWidget()
        self.findings.setMaximumHeight(130)
        v.addWidget(self.findings)

        foot = QtWidgets.QHBoxLayout()
        foot.addWidget(self._button("◀ Πίσω", lambda: self.go_step(self.step - 1)))
        foot.addStretch(1)
        foot.addWidget(self._button("Έξοδος από αναγνώριση", self.do_exit))
        foot.addWidget(self._button("Καθαρισμός", self._clear))
        self.apply_proposals_button = self._button("Εφαρμογή (0)", self.apply_proposals)
        foot.addWidget(self.apply_proposals_button)
        foot.addWidget(self._button("Αποθήκευση στο αρχείο", self.save_file))
        foot.addWidget(self._button("Επόμενο ▶", lambda: self.go_step(self.step + 1), primary=True))
        v.addLayout(foot)
        self.step = 0
        self.go_step(0)
        return page

    def go_step(self, n: int):
        """Show step n (0-7) of the wizard; only its own buttons."""
        self.step = max(0, min(len(self.STEPS) - 1, n))
        short, title, hint, power, joint, servo = self.STEPS[self.step]
        self.step_pages.setCurrentIndex(self.step)
        self.step_number.setText(f"ΒΗΜΑ {self.step + 1} ΑΠΟ {len(self.STEPS)}")
        self.step_title.setText(title)
        self.step_hint.setText(hint)
        self.power.setText("Ρεύμα servo: ΑΝΑΜΜΕΝΟ · διακόπτης στο χέρι" if power else "Ρεύμα servo: σβηστό")
        self.power.setProperty("state", "on" if power else "ok")
        theme.repolish(self.power)
        for w in (self.joint_label, self.joint_choice):
            w.setVisible(joint)
        for w in (self.servo_label, self.servo_choice):
            w.setVisible(servo)
        for w in (self.motion_label, self.speed, self.duration_label, self.duration):
            w.setVisible(self.step in (2, 4))          # mapping and direction move a joint
        for i, button in enumerate(self.step_buttons):
            button.setProperty("step", "current" if i == self.step else ("done" if i < self.step else ""))
            theme.repolish(button)

    DETAIL_COLUMNS = range(5, 11)          # zero, direction, limits, deadband of the joints page

    def _show_details(self, shown: bool):
        for column in self.DETAIL_COLUMNS:
            self.joints.setColumnHidden(column, not shown)

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
        speed, duration = self.speed.value(), self.duration.value()
        self._move(lambda joint: self.wizard.jog_joint(joint, speed, duration), self.joint_choice.currentText())

    def _move(self, make_request, name):
        if not name or not self._ready():
            return
        if not self._confirm(f"Θα κινηθεί λίγο ({name}). Διακόπτης ρεύματος των servo στο χέρι, ελεύθερη διαδρομή;"):
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
            approved = sum(1 for _, ok in self.wizard.proposals if ok)
            self.apply_proposals_button.setText(f"Εφαρμογή ({approved})")

    def note(self, level: str, text: str):
        """A result the user must see: in the wizard's findings and the status line."""
        self.wizard.findings.append(procedure.Finding(level, text))
        self.status.setText(text)
        self._refresh_wizard()

    def apply_proposals(self):
        values, manual = self.wizard.approved()
        if manual:
            self.note(procedure.WARNING, "Θέλουν αλλαγή στο αρχείο με το χέρι (αφαίρεση): " + ", ".join(manual))
        if values:
            self.apply_requested.emit(CALIBRATION_NODE, values)
            self.note(procedure.INFO, f"Στάλθηκαν {len(values)} αλλαγές στο /arm_calibration…")
        elif not manual:
            if not self.wizard.proposals:
                self.note(procedure.WARNING, "Δεν υπάρχουν προτάσεις για εφαρμογή.")
            else:
                self.note(procedure.WARNING, "Δεν τσέκαρες καμία πρόταση: τσέκαρε στη στήλη «Έγκριση» όσες θέλεις.")
                self.go_step(3)

    def save_file(self):
        self.service_requested.emit(f"{CALIBRATION_NODE}/save")
        self.note(procedure.INFO, "Αποθήκευση στο arm.yaml…")

    def show_service_result(self, service, ok, message):
        if service.startswith(CALIBRATION_NODE):
            self.note(procedure.INFO if ok else procedure.ERROR, ("✓ " if ok else "") + message)

    def _refresh_wizard(self):
        self.mode.setText("Pi: αναγνώριση" + (" · κίνηση…" if self.wizard.busy else "") if self.wizard.active
                          else "Pi: κανονικός έλεγχος")
        self.mode.setProperty("state", "warn" if self.wizard.active else "off")
        theme.repolish(self.mode)
        approved = sum(1 for _, ok in self.wizard.proposals if ok)
        self.apply_proposals_button.setText(f"Εφαρμογή ({approved})")
        self.step_buttons[3].setText(f"4 · Έλεγχος ({len(self.wizard.proposals)})" if self.wizard.proposals
                                     else "4 · Έλεγχος")
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
            self.note(procedure.ERROR, "Απορρίφθηκαν (καμία δεν εφαρμόστηκε): " + "; ".join(rejected))
            return
        self.wizard.applied({name for name, ok, _ in results if ok})
        self.note(procedure.INFO, f"✓ Εφαρμόστηκαν {len(results)} αλλαγές (ισχύουν τώρα). Για να μείνουν: "
                                  "«Αποθήκευση στο αρχείο».")
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
