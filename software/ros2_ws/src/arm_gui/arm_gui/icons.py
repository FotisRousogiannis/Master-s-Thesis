"""Icons drawn in code (no image files, no Qt SVG plugin): curved arrows for rotating a joint."""

import math

from .qt import QtCore, QtGui


def rotate_icon(clockwise: bool, color: str, size: int = 64) -> QtGui.QIcon:
    """A circular arrow of about 270°, its head pointing clockwise or counter-clockwise."""
    pixmap = QtGui.QPixmap(size, size)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    pen = QtGui.QPen(QtGui.QColor(color), size * 0.09, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
    painter.setPen(pen)
    margin = size * 0.2
    rect = QtCore.QRectF(margin, margin, size - 2 * margin, size - 2 * margin)
    start, sweep = (120.0, -270.0) if clockwise else (60.0, 270.0)       # Qt: degrees, counter-clockwise +
    path = QtGui.QPainterPath()
    path.arcMoveTo(rect, start)
    path.arcTo(rect, start, sweep)
    painter.drawPath(path)

    end = math.radians(start + sweep)
    centre, radius = rect.center(), rect.width() / 2
    tip_x, tip_y = centre.x() + radius * math.cos(end), centre.y() - radius * math.sin(end)
    # direction of travel at the end of the arc (screen y grows downwards)
    tx, ty = (math.sin(end), math.cos(end)) if clockwise else (-math.sin(end), -math.cos(end))
    nx, ny = -ty, tx
    head = size * 0.22
    points = [QtCore.QPointF(tip_x + tx * head * 0.6, tip_y + ty * head * 0.6),
              QtCore.QPointF(tip_x - tx * head * 0.4 + nx * head * 0.55, tip_y - ty * head * 0.4 + ny * head * 0.55),
              QtCore.QPointF(tip_x - tx * head * 0.4 - nx * head * 0.55, tip_y - ty * head * 0.4 - ny * head * 0.55)]
    painter.setPen(QtCore.Qt.NoPen)
    painter.setBrush(QtGui.QColor(color))
    painter.drawPolygon(QtGui.QPolygonF(points))
    painter.end()
    return QtGui.QIcon(pixmap)
