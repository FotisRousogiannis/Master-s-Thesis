"""MoveIt panel: named poses of the SRDF and a target point for the tool, planned with collision
checking by move_group (docs/decisions/0025). The Joints panel moves joints directly, without MoveIt."""

from .. import moveit_goals
from ..qt import QtWidgets, Signal


class MoveItPanel(QtWidgets.QGroupBox):
    move_requested = Signal(object)  # a goal of moveit_goals (dict)

    def __init__(self, parent=None):
        super().__init__("MoveIt · σχεδιασμός χωρίς συγκρούσεις", parent)
        layout = QtWidgets.QVBoxLayout(self)

        self._poses_row = QtWidgets.QHBoxLayout()
        self._poses_row.addWidget(QtWidgets.QLabel("Θέσεις:"))
        self._no_poses = QtWidgets.QLabel("περιμένει το move_group…")
        self._no_poses.setProperty("muted", True)
        self._poses_row.addWidget(self._no_poses)
        self._poses_row.addStretch(1)
        layout.addLayout(self._poses_row)
        self._pose_buttons = {}
        self.poses = {}

        target = QtWidgets.QHBoxLayout()
        target.addWidget(QtWidgets.QLabel("Εργαλείο στο σημείο (m):"))
        self.xyz = []
        for axis, value in zip("xyz", (0.3, 0.0, 0.5)):
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(-1.5, 1.5)
            box.setDecimals(3)
            box.setSingleStep(0.01)
            box.setValue(value)
            box.setPrefix(f"{axis} ")
            self.xyz.append(box)
            target.addWidget(box)
        self.move_tool = QtWidgets.QPushButton("Μετακίνηση εργαλείου")
        self.move_tool.clicked.connect(self._move_tool)
        target.addWidget(self.move_tool)
        target.addStretch(1)
        layout.addLayout(target)

        speed = QtWidgets.QHBoxLayout()
        speed.addWidget(QtWidgets.QLabel("Ταχύτητα:"))
        self.speed = QtWidgets.QSpinBox()
        self.speed.setRange(1, 100)
        self.speed.setValue(30)
        self.speed.setSuffix(" %")
        speed.addWidget(self.speed)
        note = QtWidgets.QLabel("του ορίου του joint_limits.yaml· STOP (Esc) ακυρώνει.")
        note.setProperty("muted", True)
        speed.addWidget(note)
        speed.addStretch(1)
        layout.addLayout(speed)

        self.status = QtWidgets.QLabel("")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

    def set_semantic(self, srdf: str):
        """The SRDF that move_group publishes: one button per named pose."""
        try:
            poses = moveit_goals.named_poses(srdf)
        except Exception as exc:                       # a broken SRDF must not take the window down
            self.status.setText(f"SRDF: {exc}")
            return
        for button in self._pose_buttons.values():
            button.deleteLater()
        self._pose_buttons, self.poses = {}, poses
        self._no_poses.setVisible(not poses)
        for name in poses:
            button = QtWidgets.QPushButton(name)
            button.clicked.connect(lambda _=False, n=name: self.move_to_pose(n))
            self._poses_row.insertWidget(self._poses_row.count() - 1, button)
            self._pose_buttons[name] = button

    def move_to_pose(self, name: str):
        self.move_requested.emit(moveit_goals.joints_goal(self.poses[name], self.speed.value() / 100))

    def _move_tool(self):
        x, y, z = (box.value() for box in self.xyz)
        self.move_requested.emit(moveit_goals.position_goal(x, y, z, self.speed.value() / 100))

    def set_status(self, text: str):
        self.status.setText(text)
