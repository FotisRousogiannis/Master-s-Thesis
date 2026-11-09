"""Log panel: output of the launched programs and messages of the application."""

import time

from ..qt import QtGui, QtWidgets

MAX_LINES = 5000


class LogPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        self._text = QtWidgets.QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setMaximumBlockCount(MAX_LINES)
        self._text.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        clear = QtWidgets.QPushButton("Καθαρισμός")
        clear.clicked.connect(self._text.clear)
        layout.addWidget(self._text)
        layout.addWidget(clear)

    def append(self, source: str, line: str):
        self._text.appendPlainText(f"{time.strftime('%H:%M:%S')} [{source}] {line}")
