"""Entry point: `ros2 run arm_gui arm_gui` (the desktop icon runs this inside the container)."""

import signal
import sys

from . import APP_NAME
from .qt import QtCore, QtWidgets


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
    app.aboutToQuit.connect(bridge.shutdown)

    # Ctrl+C in a terminal closes the window normally (layout saved, system stopped).
    signal.signal(signal.SIGINT, lambda *_: window.close())
    keep_python_responsive = QtCore.QTimer()
    keep_python_responsive.timeout.connect(lambda: None)
    keep_python_responsive.start(200)

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
