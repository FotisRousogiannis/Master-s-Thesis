"""Window layouts for the three windows: the application, Gazebo and RViz (ADR-0016).

The geometry logic is pure (unit-tested). Moving the Gazebo/RViz windows uses `wmctrl`;
this is best effort: if the window manager refuses, the windows stay where they are.
"""

import re
import shutil
import subprocess
from dataclasses import asdict, dataclass

APP, GAZEBO, RVIZ = "app", "gazebo", "rviz"
EXTERNAL_WINDOWS = {GAZEBO: re.compile(r"Gazebo"), RVIZ: re.compile(r"RViz")}

PRESETS = {
    "laptop": "Laptop (μία οθόνη)",
    "two_screens": "2 οθόνες",
    "no_gazebo": "Χωρίς Gazebo",
}


@dataclass(frozen=True)
class Rect:
    x: int
    y: int
    w: int
    h: int

    @property
    def center(self):
        return self.x + self.w // 2, self.y + self.h // 2

    def contains(self, point) -> bool:
        px, py = point
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        return cls(int(data["x"]), int(data["y"]), int(data["w"]), int(data["h"]))


def _split_h(r: Rect, fraction: float):
    left = int(r.w * fraction)
    return Rect(r.x, r.y, left, r.h), Rect(r.x + left, r.y, r.w - left, r.h)


def _split_v(r: Rect, fraction: float):
    top = int(r.h * fraction)
    return Rect(r.x, r.y, r.w, top), Rect(r.x, r.y + top, r.w, r.h - top)


def preset_layout(name: str, screens: list[Rect]) -> dict[str, Rect]:
    """Target rectangles for a preset. screens[0] is the primary screen."""
    if not screens:
        raise ValueError("no screens")
    primary = screens[0]
    if name == "two_screens" and len(screens) >= 2:
        gazebo, rviz = _split_h(screens[1], 0.5)
        return {APP: primary, GAZEBO: gazebo, RVIZ: rviz}
    if name == "no_gazebo":
        app, rviz = _split_h(primary, 0.4)
        return {APP: app, RVIZ: rviz}
    # "laptop", and "two_screens" when only one screen is connected.
    top, app = _split_v(primary, 0.6)
    gazebo, rviz = _split_h(top, 0.5)
    return {APP: app, GAZEBO: gazebo, RVIZ: rviz}


def fit_to_screens(rect: Rect, screens: list[Rect]) -> Rect:
    """Keep a saved rectangle visible: if its centre is on no screen (e.g. the second monitor was
    unplugged), move it to the primary screen; never let it be larger than its screen."""
    screen = next((s for s in screens if s.contains(rect.center)), None)
    if screen is None:
        screen = screens[0]
        rect = Rect(screen.x, screen.y, rect.w, rect.h)
    w, h = min(rect.w, screen.w), min(rect.h, screen.h)
    x = min(max(rect.x, screen.x), screen.x + screen.w - w)
    y = min(max(rect.y, screen.y), screen.y + screen.h - h)
    return Rect(x, y, w, h)


def parse_wmctrl(output: str) -> dict[str, tuple[str, Rect]]:
    """`wmctrl -l -G` lines -> {window key: (window id, geometry)} for Gazebo and RViz."""
    found = {}
    for line in output.splitlines():
        parts = line.split(None, 7)
        if len(parts) < 8:
            continue
        win_id, _desktop, x, y, w, h, _host, title = parts
        for key, pattern in EXTERNAL_WINDOWS.items():
            if key not in found and pattern.search(title):
                found[key] = (win_id, Rect(int(x), int(y), int(w), int(h)))
    return found


class WindowPlacer:
    """Finds and moves the Gazebo and RViz windows through wmctrl."""

    def __init__(self, run=subprocess.run):
        self._run = run

    @property
    def available(self) -> bool:
        return shutil.which("wmctrl") is not None

    def windows(self) -> dict[str, tuple[str, Rect]]:
        if not self.available:
            return {}
        result = self._run(["wmctrl", "-l", "-G"], capture_output=True, text=True, check=False)
        return parse_wmctrl(result.stdout) if result.returncode == 0 else {}

    def move(self, win_id: str, rect: Rect) -> bool:
        if not self.available:
            return False
        self._run(["wmctrl", "-i", "-r", win_id, "-b", "remove,maximized_vert,maximized_horz"], check=False)
        result = self._run(
            ["wmctrl", "-i", "-r", win_id, "-e", f"0,{rect.x},{rect.y},{rect.w},{rect.h}"], check=False
        )
        return result.returncode == 0
