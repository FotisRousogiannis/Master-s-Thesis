"""Ένα πρόγραμμα τη φορά στο I²C του Pi.

Οι encoders είναι πίσω από τον πολυπλέκτη TCA9548A, που έχει ΕΝΑ ανοιχτό κανάλι: αν δύο προγράμματα
(ο driver και το teleop.py, ή το wrist_bench.py) διαβάζουν μαζί, το ένα αλλάζει κανάλι ανάμεσα στο
«άνοιξε το κανάλι» και το «διάβασε» του άλλου, και βγαίνουν λάθος γωνίες ή «δεν απαντά» (πάγκος,
2026-11-22). Όλα κρατούν το ίδιο αρχείο-κλειδαριά (flock) όσο μιλούν στο I²C:

    <repo>/.local/i2c.lock       (στο container του Pi: /root/master-s-thesis/.local/i2c.lock)

Το ίδιο σύστημα κλειδώματος (flock) δουλεύει ανάμεσα στο container και στο Pi, αφού το αρχείο είναι
στον ίδιο φάκελο (volume). Η κλειδαριά φεύγει μόνη της όταν τελειώσει το πρόγραμμα.
"""

import fcntl
import os
from pathlib import Path


class I2CBusy(Exception):
    """Το I²C το κρατά ήδη άλλο πρόγραμμα."""


def hold(path) -> int:
    """Κρατά την κλειδαριά (fd) ως το τέλος του προγράμματος· I2CBusy αν την έχει άλλος."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        os.chmod(path, 0o666)       # και ο χρήστης του Pi, όχι μόνο ο root του container
    except PermissionError:
        pass
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        raise I2CBusy(f'το I²C το χρησιμοποιεί άλλο πρόγραμμα ({path})') from None
    return fd


def release(fd: int) -> None:
    """Αφήνει την κλειδαριά (αλλιώς φεύγει με το τέλος του προγράμματος)."""
    fcntl.flock(fd, fcntl.LOCK_UN)
    os.close(fd)
