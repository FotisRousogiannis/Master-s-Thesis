#!/usr/bin/env python3
"""Arm bench teleop with a menu: I2C scan, magnet readings, channel selection, differential drive.

Port of Robotic-Arm software/tools/servo_diff_keyboard.py (commit 140b76d), which drove the base on
the bench, onto this repo's drivers (wrist_bench.py): the PCA9685 and the AS5600 are reached through
the TCA9548A, so nothing has to be selected by hand, and no Adafruit libraries or venv are needed.

    python3 teleop.py                       # menu
    python3 teleop.py --a 0 --b 1 --enc 2,4 # start with these channels

Closed loop (control.py): [6] calibration finds which encoder measures each axis (pitch = W/S,
roll = A/D) and its direction with a short automatic move, then the zero at a known angle;
[7] moves both axes to target angles with one PID per axis; [8] PID settings. The calibration
is saved to base_calibration.json (commit it: it is the bench's record).

Kept from the old tool: speed as a fraction (-1..1) mapped to the pulse like its speed_to_pulse,
start magnitude 0.4, +/- step 0.1, W/S pitch (motors together), A/D roll (motors opposed), motor B
mirrored by default. Changed: the first key press is honoured for 0.6 s (the terminal starts its
auto-repeat after ~0.5 s; the old 0.25 s made every press start-stop-start), pulses stay within the
bench safety limit (max_offset_us), and the outputs are switched off - not left at neutral - on exit.
"""

import argparse
import csv
import json
import os
import select
import sys
import time
from pathlib import Path

import control as ctl
import wrist_bench as wb

PCA_CHANNELS = range(16)
MUX_CHANNELS = range(8)


def speed_to_pulse(speed, neutral_us, min_us, max_us, max_offset_us):
    """Old tool's mapping (-1..1 -> min..neutral..max), then the bench safety limit."""
    speed = max(-1.0, min(1.0, speed))
    pulse = neutral_us + speed * ((max_us - neutral_us) if speed >= 0 else (neutral_us - min_us))
    return max(neutral_us - max_offset_us, min(neutral_us + max_offset_us, pulse))


def parse_list(text, allowed):
    """'2,4' -> [2, 4]; empty -> []; raises ValueError outside `allowed`."""
    values = [int(x) for x in text.replace(" ", "").split(",") if x != ""]
    bad = [v for v in values if v not in allowed]
    if bad:
        raise ValueError(f"εκτός ορίων: {bad}")
    return values


class TeleopController(wb.JogController):
    """W/S pitch, A/D roll, +/- speed fraction; hold-to-run (first press 0.6 s, repeats 0.25 s)."""

    MOTIONS = {"w": (1, 0), "s": (-1, 0), "d": (0, 1), "a": (0, -1)}

    def __init__(self, magnitude=0.4, step=0.1, first_hold_s=0.6, repeat_hold_s=0.25):
        super().__init__(max_offset_us=1, first_hold_s=first_hold_s, repeat_hold_s=repeat_hold_s)
        self.magnitude, self.step = magnitude, step

    def on_key(self, key, now):
        if key in "+=":
            self.magnitude = round(min(1.0, self.magnitude + self.step), 3)
        elif key in "-_":
            self.magnitude = round(max(self.step, self.magnitude - self.step), 3)
        else:
            super().on_key(key, now)

    def command(self, now):
        """(pitch, roll) as fractions of full speed; (0, 0) = stop."""
        direction = self.active(now)
        if direction is None:
            return 0.0, 0.0
        return direction[0] * self.magnitude, direction[1] * self.magnitude


class Teleop:
    def __init__(self, bench, a=0, b=1, invert_b=True, encoders=(), out=print, calibration=ctl.DEFAULT_CALIBRATION):
        self.bench, self.out = bench, out
        self.a, self.b, self.invert_b = a, b, invert_b
        self.encoder_channels = list(encoders)
        self.found = None  # last scan
        self.calibration_path = Path(calibration)
        self.axes, self.pid = ctl.load_calibration(self.calibration_path)

    # -- menu actions -----------------------------------------------------------------------
    def scan(self):
        self.found = self.bench.scan()
        self.out("Σάρωση I²C (TCA9548A 0x70):")
        for where, devices in self.found.items():
            names = [{0x40: "PCA9685", 0x36: "AS5600", 0x48: "ADS1115"}.get(a, "?") for a in devices]
            self.out(f"  {where:5s}: " + (", ".join(f"0x{a:02X} {n}" for a, n in zip(devices, names)) or "-"))
        return self.found

    def encoder_channels_found(self):
        found = self.found if self.found is not None else self.bench.scan()
        return [int(k[3:]) for k, devs in found.items() if k.startswith("mux") and 0x36 in devs]

    def read_magnets(self, channels=None, samples=1, period=0.5):
        channels = channels if channels is not None else self.encoder_channels_found()
        if not channels:
            self.out("Δεν βρέθηκε AS5600 (0x36) σε κανένα κανάλι.")
            return {}
        result = {}
        for i in range(samples):
            for ch in channels:
                try:
                    angle, raw, status, agc = wb.AS5600(self.bench.bus, self.bench.mux, ch).read()
                    result[ch] = (angle, status, agc)
                    self.out(f"  mux{ch}: {angle:7.2f}°  raw {raw:4d}  μαγνήτης {status:10s}  AGC {agc}")
                except OSError:
                    result[ch] = None
                    self.out(f"  mux{ch}: δεν απαντά")
            if i < samples - 1:
                self.out("")
                time.sleep(period)
        self.bench.mux.select(None)
        return result

    def choose(self, ask=input):
        """Ask for the channels; Enter keeps the current value."""
        def field(prompt, current, allowed, single=True):
            while True:
                text = ask(f"{prompt} [{current}]: ").strip()
                if not text:
                    return current
                try:
                    values = parse_list(text, allowed)
                    if single and len(values) != 1:
                        raise ValueError("ένα κανάλι")
                    return values[0] if single else values
                except ValueError as exc:
                    self.out(f"  ✖ {exc}")

        while True:
            a = field("Κανάλι PCA9685 για το μοτέρ A (0-15)", self.a, PCA_CHANNELS)
            b = field("Κανάλι PCA9685 για το μοτέρ B (0-15)", self.b, PCA_CHANNELS)
            if a != b:
                break
            self.out("  ✖ τα A και B πρέπει να είναι διαφορετικά κανάλια")
        inv = ask(f"Ανεστραμμένο (κατοπτρικό) μοτέρ B; [{'Y/n' if self.invert_b else 'y/N'}]: ").strip().lower()
        invert_b = self.invert_b if not inv else inv in ("y", "yes", "ν", "ναι")
        found = self.encoder_channels_found()
        default = ",".join(map(str, self.encoder_channels or found))
        self.out(f"  encoders που βρέθηκαν στη σάρωση: {found or 'κανένας'}")
        enc = field("Encoders (κανάλια TCA, με κόμμα· κενό για κανέναν)", default, MUX_CHANNELS, single=False)
        self.a, self.b, self.invert_b = a, b, invert_b
        self.encoder_channels = enc if isinstance(enc, list) else parse_list(enc, MUX_CHANNELS)
        self.out(f"Επιλογή: A=ch{self.a} B=ch{self.b} ανεστραμμένο B={'ναι' if self.invert_b else 'όχι'} "
                 f"encoders={self.encoder_channels or 'κανένας'}")

    def outputs_off(self):
        self.bench.stop()
        self.out("Όλες οι έξοδοι off.")

    # -- differential drive -------------------------------------------------------------------
    def pulses(self, pitch, roll):
        """{channel: pulse_us or None (off)} for speed fractions pitch/roll."""
        pca, s = self.bench.cfg["pca"], self.bench.safety
        neutral = pca.get("default_neutral_us", 1600)
        sa = max(-1.0, min(1.0, pitch + roll))
        sb = max(-1.0, min(1.0, pitch - roll)) * (-1 if self.invert_b else 1)
        if sa == 0 and sb == 0:
            return {self.a: None, self.b: None}          # both stopped: outputs off
        # A motor at 0 while the other runs gets the neutral pulse, so its own controller brakes it.
        # Without a pulse it is free, and the running motor just turns it backwards through the
        # differential instead of moving the joint (seen on the base, 2026-11-14: roll stuck at 18.8 deg).
        to_pulse = lambda sp: speed_to_pulse(  # noqa: E731
            sp, neutral, pca.get("pulse_min_us", 500), pca.get("pulse_max_us", 2500), s["max_offset_us"])
        return {self.a: to_pulse(sa), self.b: to_pulse(sb)}

    def drive(self, rate=50, csv_path=None):
        import termios
        import tty

        bench, ctrl = self.bench, TeleopController()
        limit = bench.safety.get("jog_max_travel_deg", 90)
        encoders = {ch: wb.AS5600(bench.bus, bench.mux, ch) for ch in self.encoder_channels}
        calibrated = (set(self.axes) == set(ctl.AXES)
                      and all(c.mux_channel in encoders for c in self.axes.values()))
        trackers = {ax: c.tracker() for ax, c in self.axes.items()} if calibrated else {}
        start, limited = {}, []
        for ch, e in encoders.items():
            try:
                angle, _raw, status, _agc = e.read()
            except OSError:
                bench.mux.select(None)
                self.out(f"✖ ο encoder στο mux{ch} δεν απαντά: [1] σάρωση, ή [3] χωρίς αυτόν")
                return
            start[ch] = angle
            if status == "OK":
                limited.append(ch)
            else:
                self.out(f"  σημείωση: mux{ch} μαγνήτης {status} - φαίνεται, αλλά δεν χρησιμοποιείται για όριο")
        self.out(f"Διαφορικό A=ch{self.a} B=ch{self.b} (B {'ανεστραμμένο' if self.invert_b else 'κανονικό'})\n"
                 "  κράτα W / S : pitch (μοτέρ μαζί)     κράτα A / D : roll (μοτέρ αντίθετα)\n"
                 "  + / - : ταχύτητα     SPACE : στάση     Q : πίσω στο μενού\n"
                 + (f"  σταματά αν {', '.join(f'mux{c}' for c in limited)} κινηθεί πάνω από {limit}°\n" if limited
                    else "  ΧΩΡΙΣ ΟΡΙΟ ΔΙΑΔΡΟΜΗΣ: η κίνηση σταματά μόνο όταν αφήνεις το πλήκτρο\n"))
        f = open(csv_path, "w", newline="") if csv_path else None
        writer = csv.writer(f) if f else None
        if writer:
            writer.writerow(["t_s", "state", "pitch", "roll", f"ch{self.a}_us", f"ch{self.b}_us"]
                            + [f"mux{c}_deg" for c in encoders] + [f"mux{c}_travel_deg" for c in encoders])
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        applied, t0 = None, time.monotonic()
        try:
            tty.setcbreak(fd)
            while not ctrl.quit:
                now = time.monotonic()
                while select.select([fd], [], [], 0)[0]:
                    data = os.read(fd, 64)
                    if not data:
                        break
                    for key in data.decode("utf-8", errors="ignore"):
                        ctrl.on_key(key, now)
                pitch, roll = ctrl.command(now)
                pulses = self.pulses(pitch, roll)
                if pulses != applied:
                    for ch, pulse in pulses.items():
                        if pulse is None:
                            bench.pca.channel_off(ch)
                        else:
                            bench.pca.set_pulse(ch, pulse)
                    applied = pulses
                readings = {}
                for ch, e in encoders.items():
                    try:
                        readings[ch] = e.read()
                    except OSError:
                        self.out(f"\nΣΤΟΠ: ο encoder στο mux{ch} σταμάτησε να απαντά")
                        return
                travel = {ch: wb.wrap_delta(readings[ch][0], start[ch]) for ch in encoders}
                state = ("STOP" if not (pitch or roll) else "PITCH+" if pitch > 0 else "PITCH-" if pitch < 0
                         else "ROLL+" if roll > 0 else "ROLL-")
                if writer:
                    writer.writerow([f"{now - t0:.4f}", state, pitch, roll]
                                    + [f"{p:.0f}" if p else "off" for p in pulses.values()]
                                    + [f"{readings[c][0]:.3f}" for c in encoders]
                                    + [f"{travel[c]:.3f}" for c in encoders])
                joints = ""
                if trackers:
                    try:
                        joints = "  ".join(
                            f"{self.axes[ax].name(ax)} {t.update(readings[self.axes[ax].mux_channel][0]):+7.2f}°"
                            for ax, t in trackers.items()) + "  | "
                    except ctl.EncoderJump:
                        self.out("\nΣΤΟΠ: άλμα encoder")
                        return
                line = (f"\r[{state:6s}] ταχύτητα {ctrl.magnitude:.1f}  " + joints
                        + "  ".join(f"ch{c}={p:.0f}" if p else f"ch{c}=off" for c, p in pulses.items()) + "  "
                        + "  ".join(f"mux{c} {readings[c][0]:6.1f}° ({travel[c]:+6.1f}) {readings[c][2]}"
                                    for c in encoders))
                sys.stdout.write(line + "   ")
                sys.stdout.flush()
                over = [c for c in limited if abs(travel[c]) > limit]
                if over:
                    self.out(f"\nΣΤΟΠ: όριο {limit}° στο {', '.join(f'mux{c}' for c in over)}")
                    break
                time.sleep(1.0 / rate)
        finally:
            bench.stop()
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            if f:
                f.close()
                self.out(f"\nκαταγραφή: {csv_path}")
            self.out("\nέξοδοι off")

    # -- closed loop ----------------------------------------------------------------------------
    def raw_angles(self, channels):
        """{mux channel: (raw degrees, magnet status)}; raises OSError if one does not answer."""
        out = {}
        for ch in channels:
            angle, _raw, status, _agc = wb.AS5600(self.bench.bus, self.bench.mux, ch).read()
            out[ch] = (angle, status)
        return out

    def pulse_axis(self, axis, speed, duration, sleep=time.sleep):
        """Short bounded move on one axis (calibration): speed fraction for `duration` s, then off."""
        pitch, roll = (speed, 0.0) if axis == "pitch" else (0.0, speed)
        try:
            for ch, pulse in self.pulses(pitch, roll).items():
                self.bench.pca.set_pulse(ch, pulse)
            sleep(min(duration, self.bench.safety["max_duration_s"]))
        finally:
            self.bench.stop()

    def save(self):
        ctl.save_calibration(self.calibration_path, self.axes, self.pid,
                             {"a": self.a, "b": self.b, "invert_b": self.invert_b})
        self.out(f"Αποθηκεύτηκε: {self.calibration_path}")

    def identify(self, channels, ask=input, sleep=time.sleep, speed=0.4, step_s=0.5, max_s=3.0, enough_deg=3.0):
        """Move each axis forward in short steps until an encoder has moved `enough_deg` (at most `max_s`),
        see which encoder changed and which way, move back as long, and ask to confirm.
        Returns {axis: (mux channel, sign)} or None."""
        mapping = {}
        for axis in ctl.AXES:
            before = {ch: a for ch, (a, _s) in self.raw_angles(channels).items()}
            moved, total = {ch: 0.0 for ch in channels}, 0.0
            while total < max_s and max(abs(m) for m in moved.values()) < enough_deg:
                self.pulse_axis(axis, speed, step_s, sleep)
                total += step_s
                sleep(0.3)
                after = {ch: a for ch, (a, _s) in self.raw_angles(channels).items()}
                moved = {ch: (after[ch] - before[ch] + 180) % 360 - 180 for ch in channels}
            main = max(moved, key=lambda ch: abs(moved[ch]))         # back, watching that encoder
            spent = 0.0
            while spent < 2 * total + 1.0:
                now = (self.raw_angles([main])[main][0] - before[main] + 180) % 360 - 180
                if abs(now) <= 1.0 or now * moved[main] < 0:          # there, or just past it
                    break
                self.pulse_axis(axis, -speed, 0.1, sleep)
                spent += 0.1
            self.bench.mux.select(None)
            moves = "  ".join(f"mux{ch} {moved[ch]:+.1f}°" for ch in channels)
            self.out(f"  {axis} ({'W' if axis == 'pitch' else 'D'}, {total:.1f} s): {moves}")
            try:
                mapping[axis] = ctl.detect_axis(before, after)
            except ValueError as exc:
                self.out(f"✖ {axis}: {exc}")
                return None
        if mapping["pitch"][0] == mapping["roll"][0]:
            self.out(f"✖ Και οι δύο κινήσεις άλλαξαν μόνο τον mux{mapping['pitch'][0]}: έλεγξε τον άλλον encoder.")
            return None
        self.out("Κατάλαβα:")
        for axis, (ch, sign) in mapping.items():
            keys = "W/S" if axis == "pitch" else "A/D"
            self.out(f"  {axis} ({keys}) → encoder mux{ch}, η γωνία {'ανεβαίνει' if sign > 0 else 'κατεβαίνει'} "
                     f"με {'W' if axis == 'pitch' else 'D'}")
        if ask("Σωστό; [Y/n]: ").strip().lower() in ("n", "no", "ο", "όχι"):
            self.out("Ακυρώθηκε· τίποτα δεν άλλαξε.")
            return None
        return mapping

    def calibrate(self, ask=input, sleep=time.sleep, jog=None):
        """Per axis: find its encoder and direction (short automatic move), then its zero at a known angle."""
        jog = jog or self.drive
        channels = self.encoder_channels
        if len(channels) < 2:
            self.out("✖ Διάλεξε πρώτα (επιλογή 3) τους δύο encoders του ζεύγους.")
            return
        try:
            status = self.raw_angles(channels)
        except OSError:
            self.bench.mux.select(None)
            self.out("✖ Κάποιος encoder δεν απαντά: σάρωση (1).")
            return
        missing = [ch for ch, (_a, st) in status.items() if st == "NO_MAGNET"]
        if missing:
            self.out(f"✖ Δεν βλέπει μαγνήτη ο {', '.join(f'mux{c}' for c in missing)}: πρώτα αυτό (επιλογή 2).")
            return
        weak = [f"mux{ch} {st}" for ch, (_a, st) in status.items() if st != "OK"]
        if weak and ask(f"⚠ Μαγνήτης όχι OK ({', '.join(weak)}): η γωνία μπορεί να έχει θόρυβο ή μικρότερη ακρίβεια. "
                        "Συνέχεια για δοκιμή; [y/N]: ").strip().lower() not in ("y", "yes", "ν", "ναι"):
            return
        mapping = {ax: (c.mux_channel, c.sign) for ax, c in self.axes.items()}
        if ask("Αυτόματη αναγνώριση: κινώ λίγο τον pitch (W/S) και τον roll (A/D) και βλέπω ποιος encoder "
               "αλλάζει. Συνέχεια; [Y/n]: ").strip().lower() not in ("n", "no", "ο", "όχι"):
            found = self.identify(channels, ask, sleep)
            if found is None:
                return
            mapping = found
        elif set(mapping) != set(ctl.AXES):
            self.out("✖ Χωρίς αναγνώριση δεν ξέρω ποιος encoder μετρά κάθε κίνηση.")
            return
        for axis in ctl.AXES:
            keys = "W/S" if axis == "pitch" else "A/D"
            channel, sign = mapping[axis]
            current = self.axes.get(axis)
            self.out(f"\n--- μηδέν του {axis} (mux{channel}) ---")
            self.out(f"Φέρε τον {axis} σε γνωστή γωνία με {keys} (π.χ. κάθετα με αλφάδι) και πάτα Q.")
            jog()
            angle = float(ask("Γωνία αυτής της θέσης σε μοίρες [0]: ").strip() or 0)
            lower = float(ask(f"Κάτω όριο σε μοίρες [{current.lower_deg if current else -90}]: ").strip()
                          or (current.lower_deg if current else -90))
            upper = float(ask(f"Πάνω όριο σε μοίρες [{current.upper_deg if current else 90}]: ").strip()
                          or (current.upper_deg if current else 90))
            raw = self.raw_angles([channel])[channel][0]
            self.bench.mux.select(None)
            keep = dict(label=current.label, deadband_pos=current.deadband_pos, deadband_neg=current.deadband_neg) \
                if current and current.mux_channel == channel and current.sign == sign else {}
            self.axes[axis] = ctl.AxisCalibration(channel, sign, ctl.zero_for(raw, sign, angle), lower, upper, **keep)
            self.out(f"  {axis}: μηδέν {self.axes[axis].zero_deg}° (ένδειξη {raw:.2f}° = {angle}°), "
                     f"όρια [{lower}, {upper}]")
        self.save()

    def move_to(self, targets, rate=50, csv_path=None, clock=time.monotonic, sleep=time.sleep, keys=None):
        """Closed loop to {axis: degrees}; stops when there, on SPACE/Q, timeout, limit or encoder loss."""
        try:
            pair = ctl.PairController(self.axes, self.pid)
        except ValueError as exc:
            self.out(f"✖ {exc}")
            return False
        channels = sorted({c.mux_channel for c in self.axes.values()})
        f = open(csv_path, "w", newline="") if csv_path else None
        writer = csv.writer(f) if f else None
        if writer:
            writer.writerow(["t_s"] + [f"{ax}_{k}" for ax in ctl.AXES for k in ("target", "deg", "cmd")]
                            + [f"ch{self.a}_us", f"ch{self.b}_us"])
        keys = keys or _Keys()
        t0 = last = clock()
        settled_since, ok = None, False
        try:
            with keys:
                raw = {ch: a for ch, (a, _s) in self.raw_angles(channels).items()}
                angles = pair.angles(raw)
                set_to = pair.set_targets({ax: targets.get(ax, angles[ax]) for ax in ctl.AXES})
                self.out(f"Στόχος: " + "  ".join(f"{ax} {v:.1f}°" for ax, v in set_to.items())
                         + "   (SPACE ή Q: στάση)")
                while True:
                    now = clock()
                    if keys.stop_pressed():
                        self.out("\nΣτάση από τον χρήστη.")
                        break
                    if now - t0 > self.pid["timeout_s"]:
                        self.out(f"\nΣΤΟΠ: δεν έφτασε σε {self.pid['timeout_s']} s.")
                        break
                    raw = {ch: a for ch, (a, _s) in self.raw_angles(channels).items()}
                    angles = pair.angles(raw)
                    cmd, errors = pair.step(angles, max(now - last, 1e-3))
                    last = now
                    pulses = self.pulses(cmd["pitch"], cmd["roll"])
                    for ch, pulse in pulses.items():
                        if pulse is None:
                            self.bench.pca.channel_off(ch)
                        else:
                            self.bench.pca.set_pulse(ch, pulse)
                    if writer:
                        writer.writerow([f"{now - t0:.4f}"]
                                        + [v for ax in ctl.AXES for v in (f"{pair.targets[ax]:.2f}",
                                           f"{angles[ax]:.3f}", f"{cmd[ax]:.3f}")]
                                        + [f"{p:.0f}" if p else "off" for p in pulses.values()])
                    sys.stdout.write("\r" + "  ".join(f"{self.axes[ax].name(ax)} {angles[ax]:7.2f}° (σφάλμα "
                                                        f"{errors[ax]:+6.2f}, εντολή {cmd[ax]:+.2f})" for ax in ctl.AXES)
                                     + "   ")
                    sys.stdout.flush()
                    if pair.there(errors):
                        settled_since = settled_since if settled_since is not None else now
                        if now - settled_since >= self.pid["settle_s"]:
                            self.out("\n✔ Έφτασε.")
                            ok = True
                            break
                    else:
                        settled_since = None
                    sleep(1.0 / rate)
        except OSError:
            self.out("\nΣΤΟΠ: κάποιος encoder σταμάτησε να απαντά.")
        except ctl.EncoderJump as exc:
            self.out(f"\nΣΤΟΠ: σφάλμα encoder ({exc}).")
        except ctl.LimitError as exc:
            self.out(f"\nΣΤΟΠ: {exc}")
        finally:
            self.bench.stop()
            if f:
                f.close()
                self.out(f"καταγραφή: {csv_path}")
        return ok

    def measure_deadband(self, ask=input, sleep=time.sleep, start=0.05, step=0.02, top=0.6, hold_s=0.6,
                         min_move_deg=0.5):
        """For each axis and direction: short moves with a growing command until the axis encoder shows
        movement. The smallest command that moves is saved per axis and direction and used by the PID."""
        if any(ax not in self.axes for ax in ctl.AXES):
            self.out("✖ Πρώτα calibration (επιλογή 6).")
            return None
        if ask("Μέτρηση νεκρής ζώνης: κάθε άξονας κινείται λίγο προς τις δύο φορές. Ελεύθερος ο βραχίονας; [y/N]: "
               ).strip().lower() not in ("y", "yes", "ν", "ναι"):
            return None
        pair = ctl.PairController(self.axes, self.pid)
        channels = sorted({c.mux_channel for c in self.axes.values()})
        results = {}
        try:
            for ax in ctl.AXES:
                c = self.axes[ax]
                for direction in (1, -1):
                    steps, command, found = [], start, None
                    while command <= top + 1e-9:
                        before = pair.angles({ch: a for ch, (a, _s) in self.raw_angles(channels).items()})[ax]
                        self.pulse_axis(ax, direction * command, hold_s, sleep)
                        sleep(0.2)
                        now = pair.angles({ch: a for ch, (a, _s) in self.raw_angles(channels).items()})[ax]
                        steps.append((direction * command, now - before))
                        if not c.lower_deg - 10 <= now <= c.upper_deg + 10:
                            self.out(f"✖ {c.name(ax)} στις {now:.1f}°, κοντά στα όρια: σταματώ.")
                            return None
                        found = ctl.first_moving(steps, min_move_deg)
                        if found is not None:
                            break
                        command = round(command + step, 3)
                    results[(ax, direction)] = found
                    self.out(f"  {c.name(ax):24s} {'+' if direction > 0 else '-'}: "
                             + (f"κινείται από {found:.2f}" if found is not None else f"δεν κινήθηκε ως {top}"))
        except (OSError, ctl.EncoderJump) as exc:
            self.out(f"✖ encoder: {exc}")
            return None
        finally:
            self.bench.stop()
        for ax in ctl.AXES:
            self.axes[ax].deadband_pos = results.get((ax, 1))
            self.axes[ax].deadband_neg = results.get((ax, -1))
        self.save()
        return results

    def ask_move(self, ask=input):
        if any(ax not in self.axes for ax in ctl.AXES):
            self.out("✖ Πρώτα calibration (επιλογή 6).")
            return
        targets = {}
        for ax in ctl.AXES:
            c = self.axes[ax]
            text = ask(f"Στόχος {c.name(ax)} σε μοίρες [{c.lower_deg} … {c.upper_deg}], Enter = μένει όπου είναι: "
                       ).strip()
            if text:
                targets[ax] = float(text)
        path = ask("Αρχείο CSV για καταγραφή (κενό = χωρίς): ").strip()
        self.move_to(targets, csv_path=path or None)

    def edit_pid(self, ask=input):
        for key, value in self.pid.items():
            text = ask(f"{key} [{value}]: ").strip()
            if text:
                self.pid[key] = float(text)
        self.save()

    # -- menu -----------------------------------------------------------------------------------
    def menu(self, ask=input):
        while True:
            self.out(f"\n=== Πάγκος βραχίονα ===  A=ch{self.a} B=ch{self.b} "
                     f"B {'ανεστρ.' if self.invert_b else 'κανον.'}  encoders={self.encoder_channels or '-'}\n"
                     "  [1] Σάρωση I²C\n  [2] Μαγνήτες encoders (5 αναγνώσεις)\n  [3] Επιλογή καναλιών\n"
                     "  [4] Κίνηση διαφορικού\n  [5] Έξοδοι off\n"
                     f"  [6] Calibration (κλειστός βρόχος)   {'✔ ' + ', '.join(self.axes) if self.axes else '✖ δεν έχει γίνει'}\n"
                     "  [7] Μετακίνηση σε γωνία (PID)\n  [8] Ρυθμίσεις PID\n"
                     "  [9] Νεκρή ζώνη (αυτόματη μέτρηση)\n  [q] Έξοδος")
            choice = ask("> ").strip().lower()
            if choice == "1":
                self.scan()
            elif choice == "2":
                self.read_magnets(samples=5)
            elif choice == "3":
                self.choose(ask)
            elif choice == "4":
                path = ask("Αρχείο CSV για καταγραφή (κενό = χωρίς): ").strip()
                self.drive(csv_path=path or None)
            elif choice == "5":
                self.outputs_off()
            elif choice == "6":
                self.calibrate(ask)
            elif choice == "7":
                self.ask_move(ask)
            elif choice == "8":
                self.edit_pid(ask)
            elif choice == "9":
                self.measure_deadband(ask)
            elif choice in ("q", ";"):
                return


class _Keys:
    """Non-blocking SPACE/Q check during a closed-loop move (terminal in cbreak mode)."""

    def __enter__(self):
        import termios
        import tty

        self.fd = sys.stdin.fileno()
        self.termios = termios
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.old)
        return False

    def stop_pressed(self):
        while select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 64).decode("utf-8", errors="ignore").lower()
            if not data:
                return False
            if any(k in data for k in " q;"):
                return True
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description="Πάγκος βραχίονα: σάρωση, μαγνήτες, κανάλια, διαφορικό")
    ap.add_argument("--config", default=str(wb.DEFAULT_CONFIG))
    ap.add_argument("--a", type=int, default=0, help="PCA9685 channel of motor A")
    ap.add_argument("--b", type=int, default=1, help="PCA9685 channel of motor B")
    ap.add_argument("--no-invert-b", action="store_true", help="motor B is not mirrored")
    ap.add_argument("--enc", default="", help="TCA9548A channels of the encoders to show, e.g. 2,4")
    ap.add_argument("--calibration", default=str(ctl.DEFAULT_CALIBRATION), help="closed-loop calibration file")
    args = ap.parse_args(argv)
    config = json.loads(Path(args.config).read_text())
    from smbus2 import SMBus  # imported here so the tests run without hardware

    with SMBus(config["i2c_bus"]) as bus:
        bench = wb.Bench(config, bus)
        tele = Teleop(bench, args.a, args.b, not args.no_invert_b, parse_list(args.enc, MUX_CHANNELS),
                      calibration=args.calibration)
        saved = json.loads(Path(args.calibration).read_text()).get("motors") if Path(args.calibration).exists() else None
        if saved and argv is None and len(sys.argv) == 1:   # no channels given: use the calibrated ones
            tele.a, tele.b, tele.invert_b = saved["a"], saved["b"], saved["invert_b"]
            tele.encoder_channels = sorted({c.mux_channel for c in tele.axes.values()})
        try:
            tele.menu()
        except (KeyboardInterrupt, EOFError):
            print()
        finally:
            bench.stop()
            print("έξοδοι off")
    return 0


if __name__ == "__main__":
    sys.exit(main())
