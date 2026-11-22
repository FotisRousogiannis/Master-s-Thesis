"""Icons drawn in code (no image files, no Qt SVG plugin): curved arrows for moving a joint.

One per motion and direction, as the joint is seen from the front:

    yaw    turning about the vertical axis   flat ellipse, head right (+) or left (−)
    pitch  up and down                       tall ellipse, head up (+) or down (−)
    roll   turning about the arm's own axis  circle, counter-clockwise (+) or clockwise (−)
"""

import math

from .qt import QtCore, QtGui

# (width, height of the ellipse as part of the icon), (start, sweep) in Qt degrees (counter-clockwise +)
SHAPES = {
    ("yaw", 1): ((0.80, 0.42), (20.0, 280.0)), ("yaw", -1): ((0.80, 0.42), (160.0, -280.0)),
    ("pitch", 1): ((0.42, 0.80), (100.0, 270.0)), ("pitch", -1): ((0.42, 0.80), (260.0, -270.0)),
    ("roll", 1): ((0.62, 0.62), (60.0, 270.0)), ("roll", -1): ((0.62, 0.62), (120.0, -270.0)),
}


def _mix(c1: QtGui.QColor, c2: QtGui.QColor, t: float) -> QtGui.QColor:
    return QtGui.QColor(*(round(a + (b - a) * t) for a, b in zip(c1.getRgb()[:3], c2.getRgb()[:3])))


def _depth(motion: str, angle: float) -> float:
    """0 (far, behind the axis) .. 1 (near, in front): how the ribbon is seen in 3D.

    yaw: a flat ellipse seen from slightly above, its lower half is in front; pitch: a tall ellipse
    seen from slightly to the side, its right half is in front; roll: a circle facing us, all in front.
    """
    if motion == "yaw":
        return 0.5 - 0.5 * math.sin(angle)
    if motion == "pitch":
        return 0.5 + 0.5 * math.cos(angle)
    return 1.0


def motion_icon(motion: str, direction: int, color: str, size: int = 64) -> QtGui.QIcon:
    """A curved 3D-looking arrow for this motion (yaw, pitch, roll) and direction (+1 / −1).

    The ribbon is thinner and darker where it passes behind the axis, thicker and lighter in front;
    the head is always in front.
    """
    (w, h), (start, sweep) = SHAPES[(motion, 1 if direction > 0 else -1)]
    pixmap = QtGui.QPixmap(size, size)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    light = QtGui.QColor(color)
    dark = _mix(light, QtGui.QColor("#000000"), 0.6)
    rect = QtCore.QRectF(size * (1 - w) / 2, size * (1 - h) / 2, size * w, size * h)
    a, b, centre = rect.width() / 2, rect.height() / 2, rect.center()

    def point(deg):
        r = math.radians(deg)
        return QtCore.QPointF(centre.x() + a * math.cos(r), centre.y() - b * math.sin(r))

    steps = 48
    segments = []
    for i in range(steps):                      # short pieces, the far ones first (painter's algorithm)
        d0, d1 = start + sweep * i / steps, start + sweep * (i + 1) / steps
        depth = _depth(motion, math.radians((d0 + d1) / 2))
        segments.append((depth, d0, d1))
    for depth, d0, d1 in sorted(segments):
        pen = QtGui.QPen(_mix(dark, light, depth), size * (0.05 + 0.06 * depth), QtCore.Qt.SolidLine,
                         QtCore.Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(point(d0), point(d1))

    end = math.radians(start + sweep)
    tip = point(start + sweep)
    tx, ty = -a * math.sin(end), -b * math.cos(end)          # travel for a growing angle (screen y down)
    if sweep < 0:
        tx, ty = -tx, -ty
    norm = math.hypot(tx, ty) or 1.0
    tx, ty = tx / norm, ty / norm
    nx, ny = -ty, tx
    head = size * (0.18 + 0.08 * _depth(motion, end))
    length, back, half = head * 1.0, head * 0.25, head * 0.42     # long and narrow: clearly pointing
    points = [QtCore.QPointF(tip.x() + tx * length, tip.y() + ty * length),
              QtCore.QPointF(tip.x() - tx * back + nx * half, tip.y() - ty * back + ny * half),
              QtCore.QPointF(tip.x() - tx * back - nx * half, tip.y() - ty * back - ny * half)]
    shade = QtGui.QLinearGradient(points[1], points[2])          # one side lit, the other in shadow
    shade.setColorAt(0.0, light)
    shade.setColorAt(1.0, _mix(dark, light, 0.5))
    painter.setPen(QtGui.QPen(dark, size * 0.015))
    painter.setBrush(QtGui.QBrush(shade))
    painter.drawPolygon(QtGui.QPolygonF(points))
    painter.end()
    return QtGui.QIcon(pixmap)
