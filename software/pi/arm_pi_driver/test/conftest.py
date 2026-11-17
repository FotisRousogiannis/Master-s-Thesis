"""Τα tests τρέχουν και χωρίς colcon build: το arm_calibration από τον πηγαίο κώδικά του."""

from pathlib import Path
import sys

import pytest

try:
    import arm_calibration  # noqa: F401  (εγκατεστημένο με colcon: τίποτα να γίνει)
except ImportError:
    SOURCE = Path(__file__).resolve().parents[3] / 'ros2_ws' / 'src' / 'arm_calibration'
    sys.path.insert(0, str(SOURCE))

PACKAGE = Path(__file__).resolve().parents[1]
ARM_FILE = PACKAGE.parents[1] / 'ros2_ws' / 'src' / 'arm_calibration' / 'config' / 'arm.yaml'


def arm_data() -> dict:
    """Τα δεδομένα του arm.yaml του repo (καινούργιο αντίγραφο κάθε φορά)."""
    from arm_pi_driver import arm_file
    return arm_file.load(ARM_FILE)


def bench_config() -> dict:
    """Η ρύθμιση του driver από το arm.yaml, όπως τη φτιάχνει το node."""
    from arm_pi_driver import arm_file
    config, _ = arm_file.driver_config(arm_data())
    return config


@pytest.fixture
def data():
    """Τα δεδομένα του arm.yaml (τα tests μπορούν να τα αλλάζουν)."""
    return arm_data()


@pytest.fixture
def config():
    """Ένα καινούργιο αντίγραφο κάθε φορά (τα tests μπορούν να το αλλάζουν)."""
    return bench_config()
