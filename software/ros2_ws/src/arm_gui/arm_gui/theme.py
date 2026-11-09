"""Dark and light themes for Arm Studio (Fusion style + palette + stylesheet).

Colours come from the application icon: navy surfaces, servo orange as the accent, sky blue
for selection. Semantic colours (ok / warning / error) are separate from the accent.
"""

from dataclasses import dataclass

from .qt import QtGui, QtWidgets


@dataclass(frozen=True)
class Theme:
    name: str
    bg: str  # window background
    surface: str  # panels, inputs
    raised: str  # buttons, headers
    border: str
    fg: str
    muted: str
    accent: str  # orange
    accent_fg: str
    select: str  # sky
    ok: str
    warn: str
    error: str
    stop: str
    stop_pressed: str


DARK = Theme(
    name="dark",
    bg="#141a24", surface="#1b2230", raised="#253043", border="#2f3b51",
    fg="#e6eaf1", muted="#8e99ad", accent="#f08a24", accent_fg="#1b1205", select="#3d8fc6",
    ok="#4cb860", warn="#e7a33a", error="#ef5b5b", stop="#d32f2f", stop_pressed="#9a1f1f",
)
LIGHT = Theme(
    name="light",
    bg="#eef1f6", surface="#ffffff", raised="#e3e8f0", border="#c9d1de",
    fg="#18202e", muted="#5c6779", accent="#d9730d", accent_fg="#ffffff", select="#2f7fb8",
    ok="#2e8540", warn="#a86200", error="#c62828", stop="#d32f2f", stop_pressed="#9a1f1f",
)
THEMES = {"dark": DARK, "light": LIGHT}

_current = DARK


def current() -> Theme:
    return _current


def _palette(t: Theme) -> QtGui.QPalette:
    p = QtGui.QPalette()
    c = QtGui.QColor
    roles = {
        QtGui.QPalette.Window: t.bg,
        QtGui.QPalette.WindowText: t.fg,
        QtGui.QPalette.Base: t.surface,
        QtGui.QPalette.AlternateBase: t.raised,
        QtGui.QPalette.ToolTipBase: t.raised,
        QtGui.QPalette.ToolTipText: t.fg,
        QtGui.QPalette.Text: t.fg,
        QtGui.QPalette.Button: t.raised,
        QtGui.QPalette.ButtonText: t.fg,
        QtGui.QPalette.BrightText: t.error,
        QtGui.QPalette.Highlight: t.select,
        QtGui.QPalette.HighlightedText: "#ffffff",
        QtGui.QPalette.Link: t.select,
        QtGui.QPalette.PlaceholderText: t.muted,
    }
    for role, colour in roles.items():
        p.setColor(role, c(colour))
    for role in (QtGui.QPalette.WindowText, QtGui.QPalette.Text, QtGui.QPalette.ButtonText):
        p.setColor(QtGui.QPalette.Disabled, role, c(t.muted))
    return p


def stylesheet(t: Theme) -> str:
    return f"""
QMainWindow, QDialog {{ background: {t.bg}; }}
QToolTip {{ background: {t.raised}; color: {t.fg}; border: 1px solid {t.border}; padding: 4px; }}
QMenuBar {{ background: {t.bg}; }}
QMenuBar::item:selected, QMenu::item:selected {{ background: {t.select}; color: #ffffff; }}
QMenu {{ background: {t.surface}; border: 1px solid {t.border}; padding: 4px; }}
QMenu::item {{ padding: 5px 22px; border-radius: 4px; }}

QToolBar#toolbar_main {{ background: {t.surface}; border: none; border-bottom: 1px solid {t.border}; padding: 6px 10px; spacing: 12px; }}
QLabel#app_title {{ font-size: 15px; font-weight: 600; color: {t.fg}; }}
QLabel#app_subtitle {{ color: {t.muted}; }}
QLabel[pill="true"] {{ border-radius: 10px; padding: 2px 10px; font-weight: 600; }}
QLabel[pill="true"][state="ok"] {{ background: {t.ok}; color: #ffffff; }}
QLabel[pill="true"][state="warn"] {{ background: {t.warn}; color: #1b1205; }}
QLabel[pill="true"][state="off"] {{ background: {t.raised}; color: {t.muted}; }}
QLabel[muted="true"] {{ color: {t.muted}; }}

QDockWidget {{ color: {t.fg}; font-weight: 600; titlebar-close-icon: none; }}
QDockWidget::title {{ background: {t.raised}; padding: 6px 10px; border: 1px solid {t.border}; border-bottom: 2px solid {t.accent}; }}
QDockWidget > QWidget {{ background: {t.bg}; }}
QTabBar::tab {{ background: {t.raised}; color: {t.muted}; padding: 6px 14px; border: 1px solid {t.border}; border-bottom: none;
               border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px; }}
QTabBar::tab:selected {{ background: {t.surface}; color: {t.fg}; border-top: 2px solid {t.accent}; }}

QGroupBox {{ border: 1px solid {t.border}; border-radius: 8px; margin-top: 14px; padding: 12px 10px 10px 10px; background: {t.surface}; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {t.muted}; font-weight: 600; }}
QGroupBox:disabled {{ color: {t.muted}; }}

QPushButton {{ background: {t.raised}; border: 1px solid {t.border}; border-radius: 6px; padding: 6px 14px; }}
QPushButton:hover {{ border-color: {t.select}; }}
QPushButton:disabled {{ color: {t.muted}; background: {t.bg}; }}
QPushButton:checked {{ border: 1px solid {t.select}; color: {t.select}; font-weight: 600; }}
QPushButton[primary="true"] {{ background: {t.accent}; color: {t.accent_fg}; border: none; font-weight: 600; }}
QPushButton[primary="true"]:disabled {{ background: {t.raised}; color: {t.muted}; }}
QPushButton#stop_button {{ background: {t.stop}; color: #ffffff; border: none; font-weight: 700; font-size: 14px;
                           padding: 8px 28px; border-radius: 8px; }}
QPushButton#stop_button:pressed {{ background: {t.stop_pressed}; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background: {t.surface}; border: 1px solid {t.border}; border-radius: 5px;
                                                  padding: 4px 6px; selection-background-color: {t.select}; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {t.select}; }}
QSpinBox[modified="true"], QDoubleSpinBox[modified="true"], QLineEdit[modified="true"] {{ border: 2px solid {t.accent}; }}

QTableWidget, QTreeWidget, QListWidget, QPlainTextEdit {{ background: {t.surface}; border: 1px solid {t.border};
                                                         border-radius: 6px; gridline-color: {t.border}; }}
QHeaderView::section {{ background: {t.raised}; color: {t.muted}; border: none; border-right: 1px solid {t.border};
                       padding: 5px 8px; font-weight: 600; }}

QSlider::groove:horizontal {{ height: 6px; background: {t.raised}; border-radius: 3px; }}
QSlider::sub-page:horizontal {{ background: {t.accent}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: {t.fg}; width: 16px; height: 16px; margin: -6px 0; border-radius: 8px; }}
QSlider::handle:horizontal:hover {{ background: {t.select}; }}
QSlider:disabled::sub-page:horizontal {{ background: {t.muted}; }}

QScrollBar:vertical {{ background: transparent; width: 10px; }}
QScrollBar::handle:vertical {{ background: {t.border}; border-radius: 5px; min-height: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
"""


def apply(name: str):
    global _current
    _current = THEMES.get(name, DARK)
    app = QtWidgets.QApplication.instance()
    app.setStyle("Fusion")
    app.setPalette(_palette(_current))
    app.setStyleSheet(stylesheet(_current))


def repolish(widget):
    """Re-apply the stylesheet after a dynamic property (state, modified) changed."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)
