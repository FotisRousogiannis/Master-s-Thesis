"""Starts and stops the external programs (ros2 launch ...) that the application controls."""

import os
import signal

from .qt import QtCore, Signal

STOP_TIMEOUT_MS = 15000


class ProcessManager(QtCore.QObject):
    output = Signal(str, str)  # name, line
    running_changed = Signal(str, bool)  # name, running

    def __init__(self, parent=None):
        super().__init__(parent)
        self._procs = {}

    def is_running(self, name: str) -> bool:
        proc = self._procs.get(name)
        return proc is not None and proc.state() != QtCore.QProcess.NotRunning

    def any_running(self) -> bool:
        return any(self.is_running(n) for n in self._procs)

    def start(self, name: str, program: str, args: list[str]):
        if self.is_running(name):
            return
        proc = QtCore.QProcess(self)
        proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(lambda: self._read(name, proc))
        proc.finished.connect(lambda code, _status: self._finished(name, code))
        proc.errorOccurred.connect(lambda err: self.output.emit(name, f"σφάλμα εκκίνησης: {err}"))
        self._procs[name] = proc
        self.output.emit(name, "$ " + " ".join([program, *args]))
        proc.start(program, args)
        self.running_changed.emit(name, True)

    def stop(self, name: str):
        """Ctrl+C (SIGINT) like in a terminal, so ros2 launch shuts its nodes down cleanly; kill after a timeout."""
        proc = self._procs.get(name)
        if proc is None or proc.state() == QtCore.QProcess.NotRunning:
            return
        os.kill(int(proc.processId()), signal.SIGINT)
        QtCore.QTimer.singleShot(STOP_TIMEOUT_MS, lambda: proc.kill() if proc.state() != QtCore.QProcess.NotRunning else None)

    def stop_all(self, wait_ms: int = 0):
        for name in list(self._procs):
            self.stop(name)
        if wait_ms:
            for proc in self._procs.values():
                if not proc.waitForFinished(wait_ms):
                    proc.kill()

    def _read(self, name, proc):
        text = bytes(proc.readAllStandardOutput()).decode(errors="replace")
        for line in text.splitlines():
            self.output.emit(name, line)

    def _finished(self, name, code):
        self.output.emit(name, f"τερμάτισε (κωδικός {code})")
        self.running_changed.emit(name, False)
