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
CALIBRATION = (PACKAGE.parents[1] / 'ros2_ws' / 'src' / 'arm_calibration' / 'config'
               / 'calibration.yaml')


def bench_config() -> dict:
    """Το base.yaml με τη βαθμονόμηση του calibration.yaml, όπως το φτιάχνει το node."""
    import yaml

    from arm_pi_driver.calibration import apply_calibration, load_calibration
    config = yaml.safe_load((PACKAGE / 'config' / 'base.yaml').read_text(encoding='utf-8'))
    return apply_calibration(config, load_calibration(CALIBRATION))


@pytest.fixture
def config():
    """Ένα καινούργιο αντίγραφο κάθε φορά (τα tests μπορούν να το αλλάζουν)."""
    return bench_config()
