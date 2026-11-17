"""Entry point: `ros2 run arm_gui arm_gui` (the desktop icon runs this inside the container)."""

import signal
import sys
import traceback

from . import APP_NAME
from .qt import QtCore, QtWidgets


def report_error(window, kind, value, tb):
    """An error in a Qt slot would otherwise abort the whole application (PyQt5 without a hook):
    it goes to the Log and the Messages instead, and the application keeps running."""
    text = "".join(traceback.format_exception(kind, value, tb))
    sys.stderr.write(text)
    try:
        window.log.append("σφάλμα", text)
        window.messages.add("error", f"Σφάλμα εφαρμογής: {kind.__name__}: {value} (λεπτομέρειες στο Log)")
    except Exception:  # the window itself is broken: at least stderr has it
        pass


def main():
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)

    try:
        from .ros_bridge import RosBridge

        bridge = RosBridge()
    except Exception as exc:  # ROS not sourced / not available
        QtWidgets.QMessageBox.critical(None, APP_NAME, f"Δεν ήταν δυνατή η σύνδεση με το ROS 2:\n{exc}")
        return 1

    from .main_window import MainWindow

    window = MainWindow(bridge)
    window.show()
    sys.excepthook = lambda *exc: report_error(window, *exc)
    app.aboutToQuit.connect(bridge.shutdown)

    # Ctrl+C in a terminal closes the window normally (layout saved, system stopped).
    signal.signal(signal.SIGINT, lambda *_: window.close())
    keep_python_responsive = QtCore.QTimer()
    keep_python_responsive.timeout.connect(lambda: None)
    keep_python_responsive.start(200)

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
