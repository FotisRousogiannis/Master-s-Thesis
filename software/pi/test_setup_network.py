"""setup_network.sh with a fake netplan: a change stays only if it is confirmed (no Pi needed)."""

import os
import subprocess
import threading
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent / "setup_network.sh"


@pytest.fixture
def env(tmp_path):
    bin_dir, netplan = tmp_path / "bin", tmp_path / "netplan"
    bin_dir.mkdir()
    netplan.mkdir()
    fake = bin_dir / "netplan"
    fake.write_text(f'#!/bin/sh\necho "$@" >> {tmp_path}/netplan.log\n')
    fake.chmod(0o755)
    return dict(os.environ, PATH=f"{bin_dir}:{os.environ['PATH']}", ARM_NETPLAN_DIR=str(netplan),
                ARM_CONFIRM_TIMEOUT="3"), netplan, tmp_path


def _add_wifi(env, tmp_path, ssid, password, confirm_after=None):
    pw = tmp_path / "pw"
    pw.write_text(password)
    confirm = tmp_path / "ok"
    if confirm_after is not None:
        threading.Timer(confirm_after, confirm.touch).start()
    return subprocess.run(["bash", str(SCRIPT), "--confirm-file", str(confirm), "--password-file", str(pw),
                           "add-wifi", ssid], env=env, capture_output=True, text=True, timeout=20), pw


def test_confirmed_wifi_is_kept_and_the_password_file_removed(env):
    env, netplan, tmp = env
    r, pw = _add_wifi(env, tmp, "Lab WiFi", "s3cret", confirm_after=1)
    assert r.returncode == 0 and "OK: confirmed" in r.stdout
    files = list(netplan.glob("61-arm-wifi-*.yaml"))
    assert len(files) == 1 and '"Lab WiFi"' in files[0].read_text() and "s3cret" in files[0].read_text()
    assert oct(files[0].stat().st_mode & 0o777) == "0o600" and not pw.exists()
    assert (tmp / "netplan.log").read_text().splitlines() == ["generate", "apply"]


def test_unconfirmed_change_is_rolled_back(env):
    env, netplan, tmp = env
    _add_wifi(env, tmp, "Home", "first", confirm_after=1)  # an earlier, confirmed network
    r, _ = _add_wifi(env, tmp, "Other", "second")           # nobody confirms
    assert r.returncode == 1 and "ROLLED BACK" in r.stdout
    texts = [f.read_text() for f in netplan.glob("61-arm-wifi-*.yaml")]
    assert len(texts) == 1 and '"Home"' in texts[0]  # back to the previous state


def test_quotes_in_the_password_are_refused(env):
    env, netplan, tmp = env
    r, _ = _add_wifi(env, tmp, "Lab", 'pa"ss')
    assert r.returncode == 1 and not list(netplan.glob("61-arm-*.yaml"))
