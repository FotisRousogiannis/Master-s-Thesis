"""arm_link (software/arm_link) is found through PYTHONPATH in the container (software/docker/compose.yaml);
the tests add it here so they also run outside the container."""

import sys
from pathlib import Path

ARM_LINK = Path(__file__).resolve().parents[4] / "arm_link"
if ARM_LINK.is_dir() and str(ARM_LINK) not in sys.path:
    sys.path.insert(0, str(ARM_LINK))
