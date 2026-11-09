"""Single import point for Qt.

Inside ROS the binding comes from python_qt_binding (PyQt5 in Jazzy, the same Qt as RViz/rqt);
outside ROS (tests, CI) plain PyQt5 is used. Code imports Qt only from here.
"""

try:
    from python_qt_binding import QtCore, QtGui, QtWidgets  # noqa: F401
except ImportError:  # no ROS: tests / development outside the container
    from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: F401

    QtCore.Signal = QtCore.pyqtSignal
    QtCore.Slot = QtCore.pyqtSlot

Signal = QtCore.Signal
Qt = QtCore.Qt
