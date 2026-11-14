#!/usr/bin/env python3
"""Arm bench teleop with a menu: I2C scan, magnet readings, channel selection, differential drive.

Port of Robotic-Arm software/tools/servo_diff_keyboard.py (commit 140b76d), which drove the base on
the bench, onto this repo's drivers (wrist_bench.py): the PCA9685 and the AS5600 are reached through
the TCA9548A, so nothing has to be selected by hand, and no Adafruit libraries or venv are needed.

    python3 teleop.py                       # menu
    python3 teleop.py --a 0 --b 1 --enc 2,4 # start with these channels

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
    def __init__(self, bench, a=0, b=1, invert_b=True, encoders=(), out=print):
        self.bench, self.out = bench, out
        self.a, self.b, self.invert_b = a, b, invert_b
        self.encoder_channels = list(encoders)
        self.found = None  # last scan

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
        to_pulse = lambda sp: None if sp == 0 else speed_to_pulse(  # noqa: E731
            sp, neutral, pca.get("pulse_min_us", 500), pca.get("pulse_max_us", 2500), s["max_offset_us"])
        return {self.a: to_pulse(sa), self.b: to_pulse(sb)}

    def drive(self, rate=50, csv_path=None):
        import termios
        import tty

        bench, ctrl = self.bench, TeleopController()
        limit = bench.safety.get("jog_max_travel_deg", 90)
        encoders = {ch: wb.AS5600(bench.bus, bench.mux, ch) for ch in self.encoder_channels}
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
                line = (f"\r[{state:6s}] ταχύτητα {ctrl.magnitude:.1f}  "
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

    # -- menu -----------------------------------------------------------------------------------
    def menu(self, ask=input):
        while True:
            self.out(f"\n=== Πάγκος βραχίονα ===  A=ch{self.a} B=ch{self.b} "
                     f"B {'ανεστρ.' if self.invert_b else 'κανον.'}  encoders={self.encoder_channels or '-'}\n"
                     "  [1] Σάρωση I²C\n  [2] Μαγνήτες encoders (5 αναγνώσεις)\n  [3] Επιλογή καναλιών\n"
                     "  [4] Κίνηση διαφορικού\n  [5] Έξοδοι off\n  [q] Έξοδος")
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
            elif choice in ("q", ";"):
                return


def main(argv=None):
    ap = argparse.ArgumentParser(description="Πάγκος βραχίονα: σάρωση, μαγνήτες, κανάλια, διαφορικό")
    ap.add_argument("--config", default=str(wb.DEFAULT_CONFIG))
    ap.add_argument("--a", type=int, default=0, help="PCA9685 channel of motor A")
    ap.add_argument("--b", type=int, default=1, help="PCA9685 channel of motor B")
    ap.add_argument("--no-invert-b", action="store_true", help="motor B is not mirrored")
    ap.add_argument("--enc", default="", help="TCA9548A channels of the encoders to show, e.g. 2,4")
    args = ap.parse_args(argv)
    config = json.loads(Path(args.config).read_text())
    from smbus2 import SMBus  # imported here so the tests run without hardware

    with SMBus(config["i2c_bus"]) as bus:
        bench = wb.Bench(config, bus)
        tele = Teleop(bench, args.a, args.b, not args.no_invert_b, parse_list(args.enc, MUX_CHANNELS))
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
