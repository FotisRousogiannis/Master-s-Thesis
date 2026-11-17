"""Τα tests τρέχουν και χωρίς colcon build: το arm_calibration από τον πηγαίο κώδικά του."""

from pathlib import Path
import sys

try:
    import arm_calibration  # noqa: F401  (εγκατεστημένο με colcon: τίποτα να γίνει)
except ImportError:
    SOURCE = Path(__file__).resolve().parents[3] / 'ros2_ws' / 'src' / 'arm_calibration'
    sys.path.insert(0, str(SOURCE))
