"""System panel: choose the hardware, start/stop the stack, see what is running."""

import glob

from ..qt import QtWidgets, Signal

HARDWARE = [
    # (key, label, available)
    ("mock", "Προσομοίωση χωρίς φυσική (mock)", True),
    ("gazebo", "Προσομοίωση Gazebo", True),
    ("real", "Πραγματικός βραχίονας (επόμενη φάση)", False),
]
CONNECTIONS = [("wifi", "WiFi (micro-ROS / UDP)"), ("usb", "USB (micro-ROS / σειριακό)")]
DEFAULT_AGENT_PORT = 8888


def serial_devices() -> list[str]:
    """USB serial ports the ESP32-S3 may appear as (native USB: ttyACM; USB-UART bridge: ttyUSB)."""
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


class SystemPanel(QtWidgets.QWidget):
    start_requested = Signal(str, bool)  # hardware, open RViz
    stop_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)

        hw_box = QtWidgets.QGroupBox("Υλικό")
        hw_layout = QtWidgets.QVBoxLayout(hw_box)
        self._hardware = QtWidgets.QComboBox()
        for key, label, available in HARDWARE:
            self._hardware.addItem(label, key)
            self._hardware.model().item(self._hardware.count() - 1).setEnabled(available)
        self._hardware.currentIndexChanged.connect(self._update_connection_enabled)
        self._rviz = QtWidgets.QCheckBox("Άνοιγμα RViz")
        self._rviz.setChecked(True)
        hw_layout.addWidget(self._hardware)
        hw_layout.addWidget(self._rviz)
        layout.addWidget(hw_box)

        # Connection to the real arm (ADR-0016): both are micro-ROS transports; used once hardware:=real exists.
        self._conn_box = QtWidgets.QGroupBox("Σύνδεση με τον βραχίονα")
        conn = QtWidgets.QFormLayout(self._conn_box)
        self._connection = QtWidgets.QComboBox()
        for key, label in CONNECTIONS:
            self._connection.addItem(label, key)
        self._port = QtWidgets.QSpinBox()
        self._port.setRange(1024, 65535)
        self._port.setValue(DEFAULT_AGENT_PORT)
        self._device = QtWidgets.QComboBox()
        refresh = QtWidgets.QPushButton("Ανανέωση")
        refresh.clicked.connect(self._refresh_devices)
        device_row = QtWidgets.QHBoxLayout()
        device_row.addWidget(self._device, 1)
        device_row.addWidget(refresh)
        conn.addRow("Τρόπος", self._connection)
        conn.addRow("Θύρα UDP", self._port)
        conn.addRow("Συσκευή USB", device_row)
        layout.addWidget(self._conn_box)

        buttons = QtWidgets.QHBoxLayout()
        self._start = QtWidgets.QPushButton("▶ Εκκίνηση")
        self._stop = QtWidgets.QPushButton("■ Τερματισμός")
        self._start.clicked.connect(lambda: self.start_requested.emit(self.hardware, self._rviz.isChecked()))
        self._stop.clicked.connect(self.stop_requested.emit)
        buttons.addWidget(self._start)
        buttons.addWidget(self._stop)
        layout.addLayout(buttons)
        self._status = QtWidgets.QLabel()
        layout.addWidget(self._status)

        self._controllers = QtWidgets.QTableWidget(0, 2)
        self._controllers.setHorizontalHeaderLabels(["Controller", "Κατάσταση"])
        self._controllers.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self._controllers.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self._controllers.verticalHeader().setVisible(False)
        self._controllers.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        layout.addWidget(QtWidgets.QLabel("Controllers"))
        layout.addWidget(self._controllers, 1)

        self._nodes = QtWidgets.QListWidget()
        layout.addWidget(QtWidgets.QLabel("Nodes"))
        layout.addWidget(self._nodes, 1)

        self._refresh_devices()
        self._update_connection_enabled()
        self.set_running(False)

    @property
    def hardware(self) -> str:
        return self._hardware.currentData()

    def set_running(self, running: bool):
        self._start.setEnabled(not running)
        self._stop.setEnabled(running)
        self._hardware.setEnabled(not running)
        self._rviz.setEnabled(not running)
        self._status.setText("Το σύστημα τρέχει" if running else "Το σύστημα είναι σταματημένο")

    def set_controllers(self, controllers):
        if controllers is None:
            controllers = [("controller_manager", "μη διαθέσιμος")]
        self._controllers.setRowCount(len(controllers))
        for row, (name, state) in enumerate(controllers):
            self._controllers.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
            self._controllers.setItem(row, 1, QtWidgets.QTableWidgetItem(state))

    def set_nodes(self, nodes):
        if [self._nodes.item(i).text() for i in range(self._nodes.count())] != list(nodes):
            self._nodes.clear()
            self._nodes.addItems(nodes)

    def _refresh_devices(self):
        self._device.clear()
        self._device.addItems(serial_devices() or ["(καμία συσκευή)"])

    def _update_connection_enabled(self):
        is_real = self.hardware == "real"
        self._conn_box.setEnabled(is_real)
        self._conn_box.setToolTip("" if is_real else "Μόνο για τον πραγματικό βραχίονα")
