"""Calibration panel (ADR-0023): the one arm file in three pages, and the wizard on top of them.

    Servo        servos:         channel, neutral, deadband, description   + where it is used
    Αισθητήρες   encoders, force_sensors: where each is                    + live reading / status
    Αρθρώσεις    joints:         enabled, servos with sign, encoder, zero, direction, limits, deadband
                                                                           + live position / status
    Οδηγός       teleop on screen: live angles, arrows that move while held (or W/A/S/D), and the
                 bench procedures [6] [10] [9] run on the Pi, each ending in «Σωστό · αποθήκευση»

Values come from /arm_calibration (flat parameters of arm.yaml); «Εφαρμογή» sends the edited ones
all at once, and the node validates the whole file. «Αποθήκευση» is the Parameters panel's.
"""

import time

from .. import theme
from ..calibration import Bench, nest
from ..icons import motion_icon
from ..qt import QtCore, QtWidgets, Signal

CALIBRATION_NODE = "/arm_calibration"
KEEPALIVE_MS = 2000      # the Pi leaves identification mode after 5 s without a request
DRIVE_MS = 100           # an arrow held: a drive request this often (the Pi stops 0.3 s after the last)
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
        self.bench = Bench()
        self._groups, self._saving = [], False
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
                              "Θέση [°]", "Κατάσταση", "Κίνηση"])
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

    # ---------------------------------------------------------------- wizard page (teleop on screen)
    # (short name, title, what to do, servo power on)
    STEPS = (
        ("Σάρωση", "Τι υπάρχει στο I²C",
         "Ελέγχει τι απαντά σε κάθε κανάλι και αν κάθε encoder βλέπει μαγνήτη. Οι γωνίες πάνω ενημερώνονται "
         "συνέχεια.", False),
        ("Αναγνώριση", "Ποιος encoder, ποια φορά (teleop [6])",
         "Κάθε άρθρωση της ομάδας κινείται λίγο με «θετική» εντολή μέχρι να κινηθεί ένας encoder 3°, και "
         "γυρίζει πίσω. Μετά: «Σωστό · αποθήκευση», ή διόρθωσε.", True),
        ("Μηδέν", "Μηδέν και όρια (teleop [6])",
         "Φέρε την άρθρωση με τα βελάκια σε γνωστή γωνία (π.χ. κάθετα με αλφάδι), γράψε τη "
         "γωνία και τα όρια, «Μηδέν εδώ · αποθήκευση».", True),
        ("Νεκρή ζώνη servo", "Νεκρή ζώνη κάθε servo (teleop [10])",
         "Κάθε servo μόνο του, ο παλμός ανεβαίνει ανά 10 µs μέχρι να κινηθεί ένας encoder, και προς τις δύο "
         "φορές· τα άλλα φρενάρουν.", True),
        ("Νεκρή ζώνη άρθρωσης", "Ελάχιστη εντολή κάθε άρθρωσης (teleop [9])",
         "Η εντολή μεγαλώνει ανά 0.02 μέχρι η άρθρωση να κινηθεί 0.5° προς τη σωστή μεριά, + και −. "
         "Κάνε πρώτα τη νεκρή ζώνη servo.", True),
        ("Αρπάγη", "Όρια της αρπάγης",
         "Κλείσε τελείως → «Κλειστή», άνοιξε τελείως → «Ανοιχτή». Κλειστή = 0°.", True),
    )
    # six arrows, one per motion and direction; the joint of the group with that `motion` moves
    MOTIONS = (("yaw", "Yaw · στροφή", "A / D"), ("pitch", "Pitch · πάνω-κάτω", "W / S"),
               ("roll", "Roll · γύρω από τον άξονα", "Q / E"))
    KEYS = {QtCore.Qt.Key_D: ("yaw", 1), QtCore.Qt.Key_Right: ("yaw", 1), QtCore.Qt.Key_A: ("yaw", -1),
            QtCore.Qt.Key_Left: ("yaw", -1), QtCore.Qt.Key_W: ("pitch", 1), QtCore.Qt.Key_Up: ("pitch", 1),
            QtCore.Qt.Key_S: ("pitch", -1), QtCore.Qt.Key_Down: ("pitch", -1), QtCore.Qt.Key_E: ("roll", 1),
            QtCore.Qt.Key_Q: ("roll", -1)}

    def _button(self, text, slot, primary=False):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(slot)
        if primary:
            button.setProperty("primary", True)
        return button

    def _arrow(self, motion, direction):
        """A curved arrow that moves the group's joint with this motion while held (pressed → drive every
        100 ms, released → stop). Which way the joint really goes shows in the live angle."""
        button = QtWidgets.QPushButton(" −" if direction < 0 else " +")
        button.setObjectName("arrow")
        button.setIcon(motion_icon(motion, direction, theme.current().accent))
        button.setIconSize(QtCore.QSize(44, 44))
        button.setMinimumSize(88, 54)
        button.setAutoRepeat(False)
        button.pressed.connect(lambda: self.hold(motion, direction))
        button.released.connect(self.release)
        return button

    def _build_wizard(self):
        page = QtWidgets.QWidget()
        page.setFocusPolicy(QtCore.Qt.StrongFocus)
        v = QtWidgets.QVBoxLayout(page)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Ομάδα:"))
        self.group_choice = QtWidgets.QComboBox()
        self.group_choice.currentIndexChanged.connect(self._group_changed)
        top.addWidget(self.group_choice)
        top.addStretch(1)
        self.power = QtWidgets.QLabel()
        self.power.setProperty("pill", True)
        self.mode = QtWidgets.QLabel("Pi: κανονικός έλεγχος")
        self.mode.setProperty("pill", True)
        top.addWidget(self.power)
        top.addWidget(self.mode)
        v.addLayout(top)

        live_row = QtWidgets.QHBoxLayout()
        self.live_table = QtWidgets.QTableWidget(0, 5)
        self.live_table.setHorizontalHeaderLabels(["Άρθρωση", "Encoder", "Ένδειξη [°]", "Θέση [°]", "Μαγνήτης"])
        self.live_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.live_table.horizontalHeader().setStretchLastSection(True)
        self.live_table.verticalHeader().setVisible(False)
        self.live_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.live_table.setMaximumHeight(150)
        live_row.addWidget(self.live_table, 1)

        pad = QtWidgets.QGridLayout()
        self.motion_rows = {}
        for row, (motion, title, keys) in enumerate(self.MOTIONS):
            label = QtWidgets.QLabel()
            label.setToolTip(f"Πλήκτρα {keys}")
            minus, plus = self._arrow(motion, -1), self._arrow(motion, 1)
            pad.addWidget(label, row, 0)
            pad.addWidget(minus, row, 1)
            pad.addWidget(plus, row, 2)
            self.motion_rows[motion] = (title, keys, label, minus, plus)
        self.unassigned = QtWidgets.QLabel()
        self.unassigned.setWordWrap(True)
        self.unassigned.setProperty("muted", True)
        pad.addWidget(self.unassigned, 3, 0, 1, 3)
        self.speed = QtWidgets.QDoubleSpinBox()
        self.speed.setRange(0.05, 1.0)
        self.speed.setSingleStep(0.05)
        self.speed.setValue(0.4)
        self.speed.setPrefix("ταχύτητα ")
        pad.addWidget(self.speed, 4, 0, 1, 3)
        live_row.addLayout(pad)
        v.addLayout(live_row)

        steps = QtWidgets.QHBoxLayout()
        self.step_buttons = []
        for i, step in enumerate(self.STEPS):
            button = self._button(f"{i + 1} · {step[0]}", lambda _=False, n=i: self.go_step(n))
            self.step_buttons.append(button)
            steps.addWidget(button)
        steps.addStretch(1)
        v.addLayout(steps)
        self.step_title = QtWidgets.QLabel()
        self.step_title.setObjectName("step_title")
        self.step_hint = QtWidgets.QLabel()
        self.step_hint.setWordWrap(True)
        v.addWidget(self.step_title)
        v.addWidget(self.step_hint)

        self.step_pages = QtWidgets.QStackedWidget()
        self.save_button = self._button("Σωστό · αποθήκευση", self.save_changes, primary=True)
        self.save_button.setEnabled(False)
        self.zero_joint = QtWidgets.QComboBox()
        self.zero_angle, self.lower, self.upper = (QtWidgets.QDoubleSpinBox() for _ in range(3))
        for box in (self.zero_angle, self.lower, self.upper):
            box.setRange(-360.0, 360.0)
            box.setSuffix("°")
        self.flip_joint = QtWidgets.QComboBox()
        self.zero_joint.currentTextChanged.connect(self._zero_joint_changed)
        pages = (
            [self._button("Σάρωση I²C", self.do_scan, primary=True)],
            [self._button("Αυτόματη αναγνώριση", self.do_identify, primary=True),
             self._button("Ανάποδα ονόματα", lambda: self._act(self.bench.swap_names)),
             QtWidgets.QLabel("Αντίθετη φορά:"), self.flip_joint,
             self._button("Αντιστροφή", lambda: self._act(lambda: self.bench.flip(self.flip_joint.currentText())))],
            [QtWidgets.QLabel("Άρθρωση:"), self.zero_joint, QtWidgets.QLabel("γωνία τώρα:"), self.zero_angle,
             QtWidgets.QLabel("όρια:"), self.lower, self.upper,
             self._button("Μηδέν εδώ", self.do_zero, primary=True)],
            [self._button("Μέτρηση", lambda: self._routine(self.bench.servo_deadbands), primary=True)],
            [self._button("Μέτρηση", lambda: self._routine(self.bench.joint_deadbands), primary=True)],
            [self._button("Κλειστή", lambda: self._gripper(True)),
             self._button("Ανοιχτή", lambda: self._gripper(False))],
        )
        for widgets in pages:
            w = QtWidgets.QWidget()
            box = QtWidgets.QHBoxLayout(w)
            box.setContentsMargins(0, 0, 0, 0)
            for item in widgets:
                box.addWidget(item)
            box.addStretch(1)
            self.step_pages.addWidget(w)
        v.addWidget(self.step_pages)

        self.findings = QtWidgets.QListWidget()
        v.addWidget(self.findings, 1)
        foot = QtWidgets.QHBoxLayout()
        foot.addWidget(self._button("◀ Πίσω", lambda: self.go_step(self.step - 1)))
        stop = self._button("■ Διακοπή", self.do_stop)
        stop.setObjectName("stop_button")
        foot.addWidget(stop)
        foot.addStretch(1)
        foot.addWidget(self._button("Έξοδος από αναγνώριση", self.do_exit))
        foot.addWidget(self.save_button)
        foot.addWidget(self._button("Επόμενο ▶", lambda: self.go_step(self.step + 1)))
        v.addLayout(foot)

        self._held = None                               # (joint, +1 / −1) while an arrow is held
        self._motions = {}
        self._drive_timer = QtCore.QTimer(self)
        self._drive_timer.timeout.connect(self._send_drive)
        self._gripper_raw = {}
        self.step = 0
        self.go_step(0)
        return page

    def go_step(self, n: int):
        self.step = max(0, min(len(self.STEPS) - 1, n))
        short, title, hint, power = self.STEPS[self.step]
        self.step_pages.setCurrentIndex(self.step)
        self.step_title.setText(f"Βήμα {self.step + 1} από {len(self.STEPS)} · {title}")
        self.step_hint.setText(hint)
        self.power.setText("Ρεύμα servo: ΑΝΑΜΜΕΝΟ · διακόπτης στο χέρι" if power else "Ρεύμα servo: σβηστό")
        self.power.setProperty("state", "on" if power else "ok")
        theme.repolish(self.power)
        for i, button in enumerate(self.step_buttons):
            button.setProperty("step", "current" if i == self.step else ("done" if i < self.step else ""))
            theme.repolish(button)

    def _show_details(self, shown: bool):
        for column in self.DETAIL_COLUMNS:
            self.joints.setColumnHidden(column, not shown)

    DETAIL_COLUMNS = range(5, 11)          # zero, direction, limits, deadband of the joints page

    def _ask(self, text) -> bool:
        answer = QtWidgets.QMessageBox.question(self, "Κίνηση", text)
        return answer == QtWidgets.QMessageBox.Yes

    # ---------------------------------------------------------------- the group and its arrows
    @property
    def group(self):
        index = self.group_choice.currentIndex()
        return self._groups[index] if 0 <= index < len(self._groups) else None

    def _group_changed(self, _index=None):
        group = self.group
        joints = list(group.joints) if group else []
        self._motions = {}                           # {motion: joint of this group}
        for joint in joints:
            motion = self.data["joints"][joint].get("motion") if self.data else None
            if motion in self.motion_rows and motion not in self._motions:
                self._motions[motion] = joint
        for motion, (title, keys, label, minus, plus) in self.motion_rows.items():
            joint = self._motions.get(motion)
            label.setText(f"{title}\n{joint or '—'} · {keys}")
            minus.setEnabled(joint is not None)
            plus.setEnabled(joint is not None)
        loose = [j for j in joints if j not in self._motions.values()]
        self.unassigned.setText(f"{', '.join(loose)}: όρισε «Κίνηση» (yaw/pitch/roll) στη σελίδα «Αρθρώσεις»"
                                if loose else "")
        for combo in (self.zero_joint, self.flip_joint):
            combo.clear()
            combo.addItems(joints)
        self._refresh()

    def _zero_joint_changed(self, joint: str):
        """The limits shown are the chosen joint's (else «Μηδέν εδώ» would save another joint's)."""
        limits = (self.data or {}).get("joints", {}).get(joint, {}).get("limits_deg")
        if limits:
            self.lower.setValue(limits[0])
            self.upper.setValue(limits[1])

    def keyPressEvent(self, event):
        key = self.KEYS.get(event.key())
        if key and self.tabs.currentIndex() == 0 and not event.isAutoRepeat():
            self.hold(*key)
            return
        if event.key() == QtCore.Qt.Key_Space:
            self.do_stop()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if self.KEYS.get(event.key()) and not event.isAutoRepeat():
            self.release()
            return
        super().keyReleaseEvent(event)

    def hold(self, motion: str, direction: int):
        """The group's joint with this motion moves (+1 / −1) while held."""
        joint = self._motions.get(motion)
        if self.data is None or joint is None or self.bench.running:
            return
        self._held = (joint, direction)
        self._send_drive()
        self._drive_timer.start(DRIVE_MS)

    def release(self):
        self._held = None
        self._drive_timer.stop()              # the Pi stops by itself 0.3 s after the last drive

    def _send_drive(self):
        if self._held:
            joint, direction = self._held
            self.identify_requested.emit(self.bench.drive({joint: direction * self.speed.value()}))

    # ---------------------------------------------------------------- actions
    def _ready(self) -> bool:
        if self.data is None:
            self.status.setText("Δεν έχουν φορτωθεί τιμές από το /arm_calibration.")
        elif self.bench.running:
            self.status.setText("Τρέχει διαδικασία· περίμενε ή «Διακοπή».")
        else:
            return True
        return False

    def _send(self, request) -> None:
        if request is not None:
            self.identify_requested.emit(request)
        self._refresh()

    def do_scan(self):
        if self._ready():
            self._send(self.bench.scan())

    def do_exit(self):
        self._held = None
        self._drive_timer.stop()
        self._send(self.bench.exit())

    def do_stop(self):
        self._held = None
        self._drive_timer.stop()
        self._send(self.bench.stop())

    def do_identify(self):
        self._routine(self.bench.identify)

    def _routine(self, start):
        if not self._ready() or self.group is None:
            return
        if not self._confirm(f"Θα κινηθεί η ομάδα {' + '.join(self.group.joints)}. Διακόπτης ρεύματος των servo "
                             "στο χέρι, ελεύθερη διαδρομή;"):
            return
        self._send(start(self.group))

    def _act(self, action):
        action()
        self._refresh()

    def do_zero(self):
        joint = self.zero_joint.currentText()
        if joint and self.data is not None:
            self.bench.zero(joint, self.zero_angle.value(), self.lower.value(), self.upper.value())
            self._refresh()

    def _gripper(self, closed: bool):
        reading = self.bench.live.get(self.bench.channel_of("gripper")) if self.data else None
        if not reading or reading.get("raw_deg") is None:
            self.bench.log.append(("error", "αρπάγη: ο encoder δεν μετρά τώρα"))
        else:
            self._gripper_raw[closed] = reading["raw_deg"]
            self.bench.log.append(("info", f"αρπάγη {'κλειστή' if closed else 'ανοιχτή'}: {reading['raw_deg']}°"))
            if len(self._gripper_raw) == 2:
                self.bench.gripper(self._gripper_raw[True], self._gripper_raw[False])
                self._gripper_raw = {}
        self._refresh()

    def save_changes(self):
        """«Σωστό · αποθήκευση»: the values go to /arm_calibration together, then to arm.yaml."""
        if self.bench.changes:
            self._saving = True
            self.apply_requested.emit(CALIBRATION_NODE, dict(self.bench.changes))
            self.note("info", f"Στάλθηκαν {len(self.bench.changes)} τιμές…")

    def save_file(self):
        self.service_requested.emit(f"{CALIBRATION_NODE}/save")

    def show_service_result(self, service, ok, message):
        if service.startswith(CALIBRATION_NODE):
            self.note("info" if ok else "error", ("✓ " if ok else "") + message)

    def note(self, level: str, text: str):
        self.bench.log.append((level, text))
        self.status.setText(text)
        self._refresh()

    def _send_keepalive(self):
        request = self.bench.ping()
        if request is not None:
            self.identify_requested.emit(request)

    def on_identify_reply(self, reply: dict):
        for request in self.bench.handle(reply):
            self.identify_requested.emit(request)
        if reply.get("op") == "live":
            self._show_live()
            return
        self._refresh()

    def _show_live(self):
        if self.data is None:
            return
        rows = []
        for joint in self.data["joints"]:
            channel = self.bench.channel_of(joint)
            reading = self.bench.live.get(channel) if channel is not None else None
            if reading is None or reading.get("status") == "NO_RESPONSE":
                continue
            raw = reading.get("raw_deg")
            position = self.bench.position(joint)
            rows.append((joint, f"κανάλι {channel}", NONE if raw is None else f"{raw:.1f}",
                         NONE if position is None else f"{position:+.1f}", reading.get("status", NONE)))
        self.live_table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            for c, text in enumerate(row):
                item = self.live_table.item(r, c)
                if item is None:
                    self.live_table.setItem(r, c, QtWidgets.QTableWidgetItem(text))
                elif item.text() != text:
                    item.setText(text)
        for name, e in self.data["encoders"].items():
            reading = self.bench.live.get(e.get("channel"))
            if reading:
                raw = reading.get("raw_deg")
                self.encoders.set_live(name, "raw", NONE if raw is None else f"{raw:.2f}")
                self.encoders.set_live(name, "status", reading.get("status", NONE))

    def _refresh(self):
        running = self.bench.running
        self.mode.setText(("Pi: αναγνώριση" + (f" · {running}…" if running else "")) if self.bench.active
                          else "Pi: κανονικός έλεγχος")
        self.mode.setProperty("state", "warn" if self.bench.active else "off")
        theme.repolish(self.mode)
        self.save_button.setEnabled(bool(self.bench.changes) and not running)
        self.save_button.setText(f"Σωστό · αποθήκευση ({len(self.bench.changes)})" if self.bench.changes
                                 else "Σωστό · αποθήκευση")
        marks = {"info": "•", "warning": "⚠", "error": "✖"}
        if self.findings.count() > len(self.bench.log):
            self.findings.clear()
        for level, text in self.bench.log[self.findings.count():]:
            self.findings.addItem(f"{marks.get(level, '•')} {text}")
        self.findings.scrollToBottom()

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
        self.bench.data = self.data
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
                                ("live", NONE, "position"), ("live", NONE, "status"),
                                ("edit", f"{key}.motion", None, j.get("motion"))]))
        self.joints.set_rows(rows)
        current = self.group_choice.currentText()
        self._groups = self.bench.groups()
        self.group_choice.blockSignals(True)
        self.group_choice.clear()
        self.group_choice.addItems([" + ".join(g.joints) for g in self._groups])
        if current:
            self.group_choice.setCurrentText(current)
        self.group_choice.blockSignals(False)
        self._group_changed()
        self._zero_joint_changed(self.zero_joint.currentText())

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
        saving, self._saving = self._saving, False
        if rejected:
            self.note("error", "Απορρίφθηκαν (καμία δεν εφαρμόστηκε): " + "; ".join(rejected))
            return
        if saving:                                    # «Σωστό · αποθήκευση»: and to the file
            self.bench.saved()
            self.note("info", f"✓ Ισχύουν {len(results)} τιμές· αποθήκευση στο arm.yaml…")
            self.save_file()
        else:
            self.note("info", f"✓ Εφαρμόστηκαν {len(results)} αλλαγές (ισχύουν τώρα). Για να μείνουν: "
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
