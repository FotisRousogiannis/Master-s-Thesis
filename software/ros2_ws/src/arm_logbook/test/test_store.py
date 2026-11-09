import sqlite3

import pytest

from arm_logbook.store import Logbook, StillDetector


@pytest.fixture
def book(tmp_path):
    b = Logbook(tmp_path / "data" / "arm_logbook.db")
    yield b
    b.close()


def test_last_position_survives_reopening(tmp_path, book):
    book.save_positions({"joint_1": 3.15, "joint_2": 0.5}, time=100.0)
    book.save_positions({"joint_1": 3.20}, time=101.0)  # only joint_1 moved
    book.close()
    again = Logbook(book.path)
    assert again.last_positions() == {"joint_1": {"position": 3.20, "time": 101.0},
                                      "joint_2": {"position": 0.5, "time": 100.0}}
    again.close()


def test_events_from_ros_fill_the_history(book):
    book.handle_event({"kind": "calibration", "joint": "joint_3", "zero_deg": 130.0, "reference_deg": 90.0,
                       "invert": False}, time=1.0)
    book.handle_event({"kind": "recording_start", "path": "/r/a", "note": "PID joint_2", "topics": ["/joint_states"]}, 2.0)
    book.handle_event({"kind": "recording_stop", "path": "/r/a"}, 5.0)
    book.handle_event({"kind": "fault", "source": "encoder/joint_1", "message": "άλμα 50°", "raw_deg": 12.0}, 6.0)
    assert book.calibrations("joint_3") == [
        {"time": 1.0, "joint": "joint_3", "zero_deg": 130.0, "reference_deg": 90.0, "invert": 0}]
    assert book.recordings() == [
        {"start": 2.0, "stop": 5.0, "path": "/r/a", "note": "PID joint_2", "topics": ["/joint_states"]}]
    assert book.faults() == [{"time": 6.0, "source": "encoder/joint_1", "message": "άλμα 50°"}]
    with pytest.raises(ValueError):
        book.handle_event({"kind": "nonsense"}, 7.0)


def test_a_newer_schema_is_not_touched(tmp_path):
    path = tmp_path / "x.db"
    db = sqlite3.connect(path)
    db.execute("PRAGMA user_version = 99")
    db.close()
    with pytest.raises(RuntimeError):
        Logbook(path)


def test_position_is_saved_once_when_the_arm_stops():
    d = StillDetector(still_s=0.5)
    moving = [d.update({"j": 0.1 * i}, now=0.01 * i) for i in range(10)]
    assert moving == [None] * 10
    assert d.update({"j": 0.9}, now=0.3) is None   # still for 0.21 s only
    assert d.update({"j": 0.9}, now=0.6) == {"j": 0.9}
    assert d.update({"j": 0.9}, now=5.0) is None   # already saved
    d.update({"j": 1.5}, now=6.0)
    assert d.update({"j": 1.5}, now=6.6) == {"j": 1.5}
