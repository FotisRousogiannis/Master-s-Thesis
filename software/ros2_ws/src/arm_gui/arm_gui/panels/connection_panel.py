"""Connection panel: where is the arm's Raspberry Pi, and does everything work (ADR-0021)?

Checks, from the cable up: direct cable → Pi reachable → SSH without password → ROS 2 node of the Pi.
Network settings of the computer and the Pi are changed once, outside the app (shown here as commands).
"""

import threading
from pathlib import Path

from .. import link, theme
from ..qt import QtCore, QtWidgets, Signal
from ..tools import PiTarget

PI_NODE = "/arm_pi_driver"  # ADR-0021 (next step)
ROWS = [
    ("cable", "Καλώδιο Ethernet"),
    ("pi", "Raspberry Pi"),
    ("ssh", "SSH (Εργαλεία)"),
    ("ros", "ROS 2 στο Pi"),
]
SETUP_HELP = (
    "<b>Μία φορά</b> (οδηγός: docs/setup/raspberry_pi.md)<br>"
    "Στο Pi: <code>sudo software/pi/setup_network.sh base</code><br>"
    "Στον υπολογιστή (τερματικό Ubuntu, όχι Docker): <code>software/launcher/pi_link.sh setup-direct</code><br>"
    "Νέο WiFi στο Pi (καλύτερα από το καλώδιο): <code>sudo software/pi/setup_network.sh add-wifi \"Όνομα\"</code>"
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
        self._host.setPlaceholderText("10.42.0.2 (καλώδιο) ή διεύθυνση στο WiFi")
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
        self._key = QtWidgets.QPushButton("Ρύθμιση κλειδιού SSH…")
        self._key.setToolTip("Μία φορά: ζητά τον κωδικό του χρήστη στο Pi και μετά δεν τον ξαναζητά")
        self._key.clicked.connect(self.setup_key)
        buttons.addWidget(self._find)
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
            ssh = link.check_ssh(found, user, self._ssh_dir) if found and user else None
            return cable, found, how, ssh

        self._run(work, self._show_checks)

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
        cable, found, how, ssh = result
        self._find.setEnabled(True)
        if cable:
            self._set("cable", "ok", "ενεργό", f"αυτός ο υπολογιστής είναι {link.PC_DIRECT}")
        else:
            self._set("cable", "off", "όχι", "καλώδιο αποσυνδεδεμένο ή δεν έγινε το pi_link.sh setup-direct")
        if not found:
            self._set("pi", "warn", "δεν βρέθηκε", f"ούτε στο {link.PI_DIRECT} ούτε ως {link.PI_NAME}.local")
            self._set("ssh", "off", "—", "χωρίς Pi")
            return
        self._set("pi", "ok", found, how)
        if found != self._pi.host:
            self._host.setText(found)
            self._on_edited()
        if ssh is None:
            self._set("ssh", "warn", "λείπει", "συμπλήρωσε τον χρήστη του Pi")
        else:
            ok, message = ssh
            self._set("ssh", "ok" if ok else "warn", "εντάξει" if ok else "όχι", message)

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
