"""Connection panel: where is the arm's Raspberry Pi, and does everything work (ADR-0021)?

Checks, from the cable up: direct cable → Pi found (arm_link UDP discovery) → link quality (arm_link ping) →
SSH without password → ROS 2 node of the Pi.
Network settings of the computer and the Pi are changed once, outside the app (shown here as commands).
"""

import threading
from pathlib import Path

from .. import link, theme
from ..qt import QtCore, QtWidgets, Signal
from ..tools import PiTarget

PI_NODE = "/arm_pi_driver"  # ADR-0021 (next step)
QUICK_PINGS = 10      # with "Εύρεση και έλεγχος"
FULL_PINGS = 100      # with "Έλεγχος σύνδεσης"
PING_TIMEOUT_S = 0.3  # per ping: well above the 0.4 ms measured on the cable, room for WiFi
SLOW_MS = 10.0        # max RTT above this (a 100 Hz control cycle is 10 ms) -> "ασταθής"
ROWS = [
    ("cable", "Καλώδιο Ethernet"),
    ("pi", "Raspberry Pi"),
    ("udp", "Σύνδεση (UDP)"),
    ("ssh", "SSH (Εργαλεία)"),
    ("ros", "ROS 2 στο Pi"),
]
SETUP_HELP = (
    "<b>Μία φορά</b> (οδηγός: docs/setup/raspberry_pi.md)<br>"
    "Στον υπολογιστή (τερματικό Ubuntu, όχι Docker): <code>software/launcher/pi_link.sh setup-direct</code><br>"
    "Στο Pi, για την εύρεση και το ping: <code>bash software/arm_link/systemd/install.sh</code> "
    "(υπηρεσία arm-link-echo, ξεκινά μόνη της)"
)


class ConnectionPanel(QtWidgets.QWidget):
    pi_changed = Signal(object)  # PiTarget
    _done = Signal(object, object)  # callback, result: back to the GUI thread

    def __init__(self, ssh_dir: Path, parent=None):
        super().__init__(parent)
        self._ssh_dir = Path(ssh_dir)
        self._pi = PiTarget()
        self._run_async = True  # tests run the checks inline
        self._done.connect(lambda callback, result: callback(result))
        layout = QtWidgets.QVBoxLayout(self)

        target = QtWidgets.QGroupBox("Raspberry Pi")
        form = QtWidgets.QGridLayout(target)
        self._host = QtWidgets.QLineEdit(self._pi.host)
        self._host.setPlaceholderText("κενό: το βρίσκει η «Εύρεση» · ή γράψε τη διεύθυνση")
        self._user = QtWidgets.QLineEdit()
        self._user.setPlaceholderText("χρήστης στο Pi")
        self._repo = QtWidgets.QLineEdit(self._pi.repo)
        self._repo.setToolTip("Φάκελος του repo στο Pi (σχετικά με το home ή απόλυτος)")
        for row, (label, w) in enumerate((("Host", self._host), ("Χρήστης", self._user), ("Repo", self._repo))):
            form.addWidget(QtWidgets.QLabel(label), row, 0)
            form.addWidget(w, row, 1)
            w.editingFinished.connect(self._on_edited)
        layout.addWidget(target)

        status = QtWidgets.QGroupBox("Κατάσταση")
        grid = QtWidgets.QGridLayout(status)
        self._pills, self._texts = {}, {}
        for row, (key, label) in enumerate(ROWS):
            pill = QtWidgets.QLabel()
            pill.setProperty("pill", True)
            text = QtWidgets.QLabel()
            text.setWordWrap(True)
            text.setProperty("muted", True)
            grid.addWidget(QtWidgets.QLabel(label), row, 0)
            grid.addWidget(pill, row, 1)
            grid.addWidget(text, row, 2)
            grid.setColumnStretch(2, 1)
            self._pills[key], self._texts[key] = pill, text
        layout.addWidget(status)

        buttons = QtWidgets.QHBoxLayout()
        self._find = QtWidgets.QPushButton("Εύρεση και έλεγχος")
        self._find.setProperty("primary", True)
        self._find.clicked.connect(self.check_all)
        self._measure = QtWidgets.QPushButton("Έλεγχος σύνδεσης")
        self._measure.setToolTip(f"{FULL_PINGS} ping UDP προς το Pi: απώλειες, χρόνος πήγαινε-έλα, jitter")
        self._measure.clicked.connect(self.measure)
        self._key = QtWidgets.QPushButton("Ρύθμιση κλειδιού SSH…")
        self._key.setToolTip("Μία φορά: ζητά τον κωδικό του χρήστη στο Pi και μετά δεν τον ξαναζητά")
        self._key.clicked.connect(self.setup_key)
        buttons.addWidget(self._find)
        buttons.addWidget(self._measure)
        buttons.addWidget(self._key)
        buttons.addStretch(1)
        layout.addLayout(buttons)

        help_ = QtWidgets.QLabel(SETUP_HELP)
        help_.setWordWrap(True)
        help_.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        help_.setProperty("muted", True)
        layout.addWidget(help_)
        layout.addStretch(1)

        for key, _ in ROWS:
            self._set(key, "off", "—", "δεν ελέγχθηκε")
        self.set_nodes([])

    # ------------------------------------------------------------------ inputs
    def set_pi(self, pi: PiTarget):
        self._pi = pi
        self._host.setText(pi.host)
        self._user.setText(pi.user)
        self._repo.setText(pi.repo)

    @property
    def pi(self) -> PiTarget:
        return self._pi

    def set_nodes(self, nodes):
        if PI_NODE in nodes:
            self._set("ros", "ok", "τρέχει", PI_NODE)
        else:
            self._set("ros", "off", "όχι", f"δεν βλέπω το {PI_NODE} (το node του Pi είναι το επόμενο βήμα)")

    # ------------------------------------------------------------------ actions
    def check_all(self):
        self._find.setEnabled(False)
        self._set("pi", "off", "…", "αναζήτηση")
        host, user = self._pi.host, self._pi.user

        def work():
            cable = link.has_local_address(link.PC_DIRECT)
            reachable = link.port_open(host) if host else False
            found, how = (host, "όπως ορίστηκε") if reachable else link.find_pi()
            quality = link.measure_link(found, QUICK_PINGS, PING_TIMEOUT_S) if found else (None, "")
            ssh = link.check_ssh(found, user, self._ssh_dir) if found and user else None
            return cable, host, found, how, quality, ssh

        self._run(work, self._show_checks)

    def measure(self):
        host = self._pi.host
        if not host:
            self._set("udp", "warn", "λείπει", "πάτα πρώτα «Εύρεση και έλεγχος» ή γράψε τον host")
            return
        self._measure.setEnabled(False)
        self._set("udp", "off", "…", f"{FULL_PINGS} ping προς {host}")
        self._run(lambda: link.measure_link(host, FULL_PINGS, PING_TIMEOUT_S), self._show_measure)

    def setup_key(self):
        if not (self._pi.host and self._pi.user):
            self._set("ssh", "warn", "λείπει", "συμπλήρωσε host και χρήστη")
            return
        password = self.ask_password()
        if password is None:
            return
        self._key.setEnabled(False)
        self._set("ssh", "off", "…", "αντιγραφή κλειδιού στο Pi")
        host, user = self._pi.host, self._pi.user
        self._run(lambda: link.install_key(host, user, password, self._ssh_dir), self._show_key)

    def ask_password(self):
        text, ok = QtWidgets.QInputDialog.getText(
            self, "Κλειδί SSH", f"Κωδικός του {self._pi.user}@{self._pi.host} (μία φορά, δεν αποθηκεύεται):",
            QtWidgets.QLineEdit.Password)
        return text if ok else None

    # ------------------------------------------------------------------ internals
    def _run(self, work, callback):
        if not self._run_async:
            callback(work())
            return
        threading.Thread(target=lambda: self._done.emit(callback, work()), daemon=True).start()

    def _show_checks(self, result):
        cable, tried, found, how, quality, ssh = result
        self._find.setEnabled(True)
        if cable:
            self._set("cable", "ok", "ενεργό", f"αυτός ο υπολογιστής είναι {link.PC_DIRECT}")
        else:
            self._set("cable", "off", "όχι", "καλώδιο αποσυνδεδεμένο ή δεν έγινε το pi_link.sh setup-direct")
        if not found:
            where = f"{tried}, " if tried else ""
            detail = f" ({how})" if how else ""
            self._set("pi", "warn", "δεν βρέθηκε",
                      f"δοκιμάστηκαν: {where}εύρεση UDP σε όλα τα δίκτυα, {link.PI_DIRECT}, "
                      f"{link.PI_NAME}.local{detail}")
            self._set("udp", "off", "—", "χωρίς Pi")
            self._set("ssh", "off", "—", "χωρίς Pi")
            return
        self._set("pi", "ok", found, how)
        self._show_quality(quality)
        if found != self._pi.host:
            self._host.setText(found)
            self._on_edited()
        if ssh is None:
            self._set("ssh", "warn", "λείπει", "συμπλήρωσε τον χρήστη του Pi")
        else:
            ok, message = ssh
            self._set("ssh", "ok" if ok else "warn", "εντάξει" if ok else "όχι", message)

    def _show_measure(self, result):
        self._measure.setEnabled(True)
        self._show_quality(result)

    def _show_quality(self, result):
        stats, why = result
        if stats is None:
            self._set("udp", "warn", "όχι", why)
            return
        received = len(stats.rtts_ns)
        if received == 0:
            self._set("udp", "warn", "δεν απαντά",
                      f"0/{stats.sent_count} απαντήσεις: τρέχει η υπηρεσία arm-link-echo στο Pi;")
            return
        text = (f"{stats.loss_percent:.0f} % απώλειες ({received}/{stats.sent_count}) · "
                f"RTT {stats.avg_ms:.2f} ms (max {stats.max_ms:.2f}) · jitter {stats.jitter_ms:.2f} ms")
        good = stats.loss_percent == 0 and stats.max_ms < SLOW_MS
        self._set("udp", "ok" if good else "warn", "καλή" if good else "ασταθής", text)

    def _show_key(self, result):
        ok, message = result
        self._key.setEnabled(True)
        self._set("ssh", "ok" if ok else "warn", "εντάξει" if ok else "όχι", message)

    def _on_edited(self):
        self._pi = PiTarget(self._host.text().strip(), self._user.text().strip(), self._repo.text().strip())
        self.pi_changed.emit(self._pi)

    def _set(self, key, state, pill_text, text):
        pill = self._pills[key]
        pill.setText(pill_text)
        if pill.property("state") != state:
            pill.setProperty("state", state)
            theme.repolish(pill)
        self._texts[key].setText(text)
