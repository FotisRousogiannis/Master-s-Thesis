#!/usr/bin/env python3
"""Bench test tool for the wrist: Raspberry Pi -> TCA9548A -> AS5600, PCA9685 -> servos.

Temporary test rig (ADR-0012): no ROS, no ESP32. Every motion command is bounded
in pulse offset, duration and encoder travel, and always ends with the outputs off.

    python3 wrist_bench.py scan
    python3 wrist_bench.py enc [--rate 50] [--duration 10] [--csv out.csv]
    python3 wrist_bench.py pwm SERVO OFFSET_US [--duration 1.0] [--csv out.csv]
    python3 wrist_bench.py diff PITCH_US ROLL_US [--duration 1.0] [--csv out.csv]
    python3 wrist_bench.py deadband SERVO ENCODER [--span 60] [--step 5] [--dwell 1.0]
    python3 wrist_bench.py probe [--channels 0-15] [--offset US] [--duration 1.0]
    python3 wrist_bench.py jog (SERVO | --channel N)    keyboard: hold W/S to turn, A/D speed
    python3 wrist_bench.py diffjog PAIR [--no-encoders]  differential pair (base, wrist): hold W/S pitch,
                                                         A/D roll, +/- speed
    python3 wrist_bench.py stop

OFFSET_US is relative to the servo's neutral pulse (continuous-rotation servos:
the offset sets speed and direction, not position).
"""

import argparse
import csv
import json
import os
import select
import signal
import sys
import time
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).with_name("wrist_bench.json")

# AS5600 registers
AS5600_ADDR = 0x36
AS5600_STATUS = 0x0B
AS5600_RAW_ANGLE = 0x0C
AS5600_AGC = 0x1A
AS5600_MD, AS5600_ML, AS5600_MH = 0x20, 0x10, 0x08

# PCA9685 registers
PCA_MODE1, PCA_MODE2, PCA_PRESCALE, PCA_LED0 = 0x00, 0x01, 0xFE, 0x06
PCA_SLEEP, PCA_AI, PCA_RESTART, PCA_OUTDRV = 0x10, 0x20, 0x80, 0x04
PCA_ALL_OFF = 0xFA


class Mux:
    """TCA9548A: one byte selects the active downstream channel(s)."""

    def __init__(self, bus, address):
        self.bus, self.address, self.current = bus, address, None

    def select(self, channel):
        if channel == self.current:
            return
        self.bus.write_byte(self.address, 0 if channel is None else 1 << channel)
        self.current = channel


class PCA9685:
    def __init__(self, bus, mux, address, mux_channel, freq_hz, osc_hz, pulse_min_us=500, pulse_max_us=2500):
        self.bus, self.mux, self.address, self.mux_channel = bus, mux, address, mux_channel
        self.freq_hz = freq_hz
        self.pulse_min_us, self.pulse_max_us = pulse_min_us, pulse_max_us
        self.mux.select(mux_channel)
        prescale = round(osc_hz / (4096 * freq_hz)) - 1
        self.bus.write_byte_data(address, PCA_MODE1, PCA_SLEEP)
        self.bus.write_byte_data(address, PCA_PRESCALE, prescale)
        self.bus.write_byte_data(address, PCA_MODE2, PCA_OUTDRV)
        self.bus.write_byte_data(address, PCA_MODE1, PCA_AI)
        time.sleep(0.001)
        self.bus.write_byte_data(address, PCA_MODE1, PCA_AI | PCA_RESTART)

    def us_to_ticks(self, pulse_us):
        return round(pulse_us * self.freq_hz * 4096 / 1e6)

    def set_pulse(self, channel, pulse_us):
        pulse_us = max(self.pulse_min_us, min(self.pulse_max_us, pulse_us))  # servo spec, never exceeded
        off = self.us_to_ticks(pulse_us)
        self.mux.select(self.mux_channel)
        self.bus.write_i2c_block_data(self.address, PCA_LED0 + 4 * channel, [0, 0, off & 0xFF, off >> 8])

    def channel_off(self, channel):
        self.mux.select(self.mux_channel)
        self.bus.write_i2c_block_data(self.address, PCA_LED0 + 4 * channel, [0, 0, 0, 0x10])

    def all_off(self):
        self.mux.select(self.mux_channel)
        self.bus.write_i2c_block_data(self.address, PCA_ALL_OFF, [0, 0, 0, 0x10])


class AS5600:
    def __init__(self, bus, mux, mux_channel, sign=1, zero_deg=0.0):
        self.bus, self.mux, self.mux_channel = bus, mux, mux_channel
        self.sign, self.zero_deg = sign, zero_deg

    def read(self):
        """Return (angle_deg, raw, status_text, agc)."""
        self.mux.select(self.mux_channel)
        hi, lo = self.bus.read_i2c_block_data(AS5600_ADDR, AS5600_RAW_ANGLE, 2)
        raw = ((hi & 0x0F) << 8) | lo
        status = self.bus.read_byte_data(AS5600_ADDR, AS5600_STATUS)
        agc = self.bus.read_byte_data(AS5600_ADDR, AS5600_AGC)
        angle = (self.sign * (raw * 360.0 / 4096.0 - self.zero_deg)) % 360.0
        return angle, raw, magnet_status(status), agc


def magnet_status(status):
    if not status & AS5600_MD:
        return "NO_MAGNET"
    if status & AS5600_ML:
        return "TOO_WEAK"
    if status & AS5600_MH:
        return "TOO_STRONG"
    return "OK"


def wrap_delta(a, b):
    """Shortest signed difference a - b in degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


class Bench:
    def __init__(self, config, bus):
        self.cfg = config
        self.bus = bus
        self.mux = Mux(bus, config["tca_address"])
        self.safety = config["safety"]
        self._pca = None
        self.encoders = {
            name: AS5600(bus, self.mux, e["mux_channel"], e.get("sign", 1), e.get("zero_deg", 0.0))
            for name, e in config["encoders"].items()
        }

    @property
    def pca(self):
        if self._pca is None:
            p = self.cfg["pca"]
            self._pca = PCA9685(self.bus, self.mux, p["address"], p.get("mux_channel"), p["freq_hz"], p["osc_hz"],
                                p.get("pulse_min_us", 500), p.get("pulse_max_us", 2500))
        return self._pca

    # -- safety ------------------------------------------------------------
    def clamp_offset(self, offset_us):
        limit = self.safety["max_offset_us"]
        if abs(offset_us) > limit:
            print(f"offset {offset_us} us clamped to +/-{limit} us", file=sys.stderr)
        return max(-limit, min(limit, offset_us))

    def clamp_duration(self, duration):
        return min(duration, self.safety["max_duration_s"])

    def stop(self):
        try:
            self.pca.all_off()
        finally:
            self.mux.select(None)

    # -- commands ----------------------------------------------------------
    def scan(self):
        found = {}
        self.mux.select(None)
        root = self._probe(exclude={self.cfg["tca_address"]})
        found["root"] = root
        for ch in range(8):
            self.mux.select(ch)
            devices = [a for a in self._probe(exclude={self.cfg["tca_address"]}) if a not in root]
            if devices:
                found[f"mux{ch}"] = devices
        self.mux.select(None)
        return found

    def _probe(self, exclude):
        devices = []
        for addr in range(0x03, 0x78):
            if addr in exclude:
                continue
            try:
                self.bus.read_byte(addr)
                devices.append(addr)
            except OSError:
                pass
        return devices

    def read_encoders(self):
        return {name: enc.read() for name, enc in self.encoders.items()}

    def run_motion(self, pulses, duration, rate, writer=None, label=""):
        """Apply {servo: offset_us} for `duration` s, sampling encoders; abort on travel limit."""
        servos = self.cfg["servos"]
        channel_pulses = {servos[n]["channel"]: servos[n]["neutral_us"] + self.clamp_offset(o)
                          for n, o in pulses.items()}
        return self.run_channels(channel_pulses, duration, rate, writer, label, pulses)

    def run_channels(self, channel_pulses, duration, rate, writer=None, label="", logged=None):
        """Apply {pca_channel: pulse_us} for `duration` s, sampling encoders; abort on travel limit."""
        duration = self.clamp_duration(duration)
        pulses = logged if logged is not None else channel_pulses
        start = {n: r[0] for n, r in self.read_encoders().items()}
        t0 = time.monotonic()
        try:
            for channel, pulse_us in channel_pulses.items():
                self.pca.set_pulse(channel, pulse_us)
            while (t := time.monotonic() - t0) < duration:
                readings = self.read_encoders()
                travel = {n: wrap_delta(r[0], start[n]) for n, r in readings.items()}
                if writer:
                    writer.writerow([f"{t:.4f}", label, json.dumps(pulses)]
                                    + [f"{readings[n][0]:.3f}" for n in self.encoders]
                                    + [f"{travel[n]:.3f}" for n in self.encoders])
                over = [n for n, d in travel.items() if abs(d) > self.safety["max_travel_deg"]]
                if over:
                    print(f"ABORT: travel limit exceeded on {over}", file=sys.stderr)
                    break
                time.sleep(1.0 / rate)
        finally:
            self.stop()
        return {n: wrap_delta(r[0], start[n]) for n, r in self.read_encoders().items()}

    def probe(self, channels, offset_us, duration, pause=0.5):
        """Pulse each PCA channel briefly and report the travel of every encoder."""
        neutral = self.cfg["pca"].get("default_neutral_us", 1600)
        offset_us = self.clamp_offset(offset_us)
        results = {}
        for ch in channels:
            print(f"  channel {ch:2d} ...", flush=True)
            results[ch] = self.run_channels({ch: neutral + offset_us}, duration, 50, label=f"probe ch{ch}")
            time.sleep(pause)
        return results

    def differential(self, pitch_us, roll_us, pair=None):
        """Offsets of the two motors: A = p + r, B = b_sign * (p - r).
        pair: a name in config["differentials"] (base, wrist); None = config["differential"]."""
        d = self.cfg["differentials"][pair] if pair else self.cfg["differential"]
        a = self.clamp_offset(pitch_us + roll_us)
        b = self.clamp_offset(d.get("b_sign", 1) * (pitch_us - roll_us))
        return {d["motor_a"]: a, d["motor_b"]: b}

    def pair_encoders(self, pair):
        """The encoders listed under a differential pair (its own mux channels)."""
        return {name: AS5600(self.bus, self.mux, e["mux_channel"], e.get("sign", 1), e.get("zero_deg", 0.0))
                for name, e in self.cfg["differentials"][pair].get("encoders", {}).items()}


class JogController:
    """Hold-to-run keyboard logic, independent of the terminal (testable).

    Terminals report key presses, not releases; a held key arrives as an
    auto-repeat stream after an initial delay (~0.5 s). Motion continues only
    while W/S keep arriving: the first press is honoured for `first_hold_s`
    (covers the auto-repeat delay), later ones for `repeat_hold_s`.
    """

    def __init__(self, max_offset_us, start_offset_us=30, step_us=5, first_hold_s=0.6, repeat_hold_s=0.15):
        self.max_offset_us = max_offset_us
        self.offset_us = min(start_offset_us, max_offset_us)
        self.step_us = step_us
        self.first_hold_s, self.repeat_hold_s = first_hold_s, repeat_hold_s
        self.direction = 0
        self.last_press = None
        self.repeats = 0
        self.quit = False

    # Greek keyboard layout: the same physical keys send these characters.
    GREEK_KEYS = {"ς": "w", "Σ": "s", "σ": "s", "α": "a", "Α": "a", "δ": "d", "Δ": "d", ";": "q", "·": "q"}
    MOTIONS = {"w": 1, "s": -1}   # key -> direction (held keys)
    FASTER, SLOWER = "d", "a"

    def on_key(self, key, now):
        key = self.GREEK_KEYS.get(key, key).lower()
        if key in self.MOTIONS:
            direction = self.MOTIONS[key]
            if direction == self.direction and self.last_press is not None \
                    and now - self.last_press < self.first_hold_s:
                self.repeats += 1
            else:
                self.repeats = 0
            self.direction, self.last_press = direction, now
        elif key in self.FASTER:
            self.offset_us = min(self.offset_us + self.step_us, self.max_offset_us)
        elif key in self.SLOWER:
            self.offset_us = max(self.offset_us - self.step_us, self.step_us)
        elif key == " ":
            self.direction, self.last_press = 0, None
        elif key == "q":
            self.quit = True

    def active(self, now):
        """The held direction, or None once the key stopped arriving."""
        if self.direction == 0 or self.last_press is None:
            return None
        hold = self.first_hold_s if self.repeats == 0 else self.repeat_hold_s
        if now - self.last_press > hold:
            self.direction, self.last_press = 0, None
            return None
        return self.direction

    def command(self, now):
        """Signed offset to apply now, or 0 to stop."""
        direction = self.active(now)
        return 0 if direction is None else direction * self.offset_us


class DiffJogController(JogController):
    """Keyboard logic for a differential pair: W/S pitch (motors together), A/D roll (opposed),
    +/- speed. Same hold-to-run behaviour as JogController."""

    MOTIONS = {"w": (1, 0), "s": (-1, 0), "d": (0, 1), "a": (0, -1)}   # (pitch, roll)
    FASTER, SLOWER = "+=", "-_"

    def command(self, now):
        """(pitch_us, roll_us) to apply now; (0, 0) = stop."""
        direction = self.active(now)
        if direction is None:
            return 0, 0
        return direction[0] * self.offset_us, direction[1] * self.offset_us


def jog(bench, channel, neutral_us, label, rate=50, use_encoders=True):
    import termios
    import tty

    ctrl = JogController(bench.safety["max_offset_us"], bench.safety.get("jog_start_offset_us", 30),
                         bench.safety.get("jog_step_us", 5))
    limit = bench.safety.get("jog_max_travel_deg", 90)
    names = list(bench.encoders) if use_encoders else []
    start = {n: r[0] for n, r in bench.read_encoders().items()} if use_encoders else {}
    print(f"jog {label} (PCA channel {channel}, neutral {neutral_us} us)\n"
          "  hold W / S : turn + / -     A / D : slower / faster     SPACE : stop     Q : quit\n"
          + (f"  stops if any encoder travels more than {limit} deg from the start\n" if use_encoders
             else "  NO ENCODERS: no travel limit - motion stops only when you release W / S\n"))
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    applied = None
    try:
        tty.setcbreak(fd)
        while not ctrl.quit:
            now = time.monotonic()
            # read raw bytes from the terminal: no Python buffering, multi-byte (Greek) keys decoded whole
            while select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 64)
                if not data:
                    break
                for key in data.decode("utf-8", errors="ignore"):
                    ctrl.on_key(key, now)
            offset = ctrl.command(now)
            if offset != applied:
                if offset:
                    bench.pca.set_pulse(channel, neutral_us + offset)
                else:
                    bench.pca.channel_off(channel)
                applied = offset
            readings = bench.read_encoders() if use_encoders else {}
            travel = {n: wrap_delta(readings[n][0], start[n]) for n in names}
            state = "STOP" if not offset else ("+ W " if offset > 0 else "- S ")
            line = (f"\r[{state}] speed {ctrl.offset_us:3d} us (pulse {neutral_us + (offset or ctrl.offset_us)} us)  "
                    + "  ".join(f"{n} {readings[n][0]:6.1f}° ({travel[n]:+6.1f}) {readings[n][2]}" for n in names))
            sys.stdout.write(line + "   ")
            sys.stdout.flush()
            over = [n for n in names if abs(travel[n]) > limit]
            if over:
                print(f"\nABORT: travel limit {limit} deg exceeded on {over}")
                break
            time.sleep(1.0 / rate)
    finally:
        bench.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\noutputs off")


def diffjog(bench, pair, rate=50, use_encoders=True):
    """Keyboard teleop of a differential pair (both motors together, ADR-0021 bench)."""
    import termios
    import tty

    servos = bench.cfg["servos"]
    d = bench.cfg["differentials"][pair]
    ch = {d["motor_a"]: servos[d["motor_a"]]["channel"], d["motor_b"]: servos[d["motor_b"]]["channel"]}
    neutral = {m: servos[m]["neutral_us"] for m in ch}
    s = bench.safety
    ctrl = DiffJogController(s["max_offset_us"], s.get("diffjog_start_offset_us", 150), s.get("diffjog_step_us", 30))
    limit = s.get("jog_max_travel_deg", 90)
    encoders = bench.pair_encoders(pair) if use_encoders else {}
    start = {}
    for n, e in encoders.items():
        try:
            start[n] = e.read()[0]
        except OSError:
            bench.mux.select(None)
            raise SystemExit(f"encoder {n} (TCA9548A channel {e.mux_channel}) does not answer: check its wiring "
                             f"and run `scan`; or run without the travel limit: diffjog {pair} --no-encoders")
    print(f"diffjog {pair}: A={d['motor_a']} (ch{ch[d['motor_a']]})  B={d['motor_b']} (ch{ch[d['motor_b']]})  "
          f"b_sign={d.get('b_sign', 1)}\n"
          "  hold W / S : pitch + / - (motors together)     hold A / D : roll - / + (motors opposed)\n"
          "  + / - : faster / slower     SPACE : stop     Q : quit\n"
          + (f"  stops if an encoder travels more than {limit} deg from the start\n" if encoders
             else "  NO ENCODERS: no travel limit - motion stops only when you release the key\n"))
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    applied = None
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
            offsets = bench.differential(pitch, roll, pair)
            if offsets != applied:
                for motor, offset in offsets.items():
                    if offset:
                        bench.pca.set_pulse(ch[motor], neutral[motor] + offset)
                    else:
                        bench.pca.channel_off(ch[motor])
                applied = offsets
            readings = {n: e.read() for n, e in encoders.items()}
            travel = {n: wrap_delta(readings[n][0], start[n]) for n in encoders}
            state = ("STOP" if not (pitch or roll) else "PITCH+" if pitch > 0 else "PITCH-" if pitch < 0
                     else "ROLL+" if roll > 0 else "ROLL-")
            line = (f"\r[{state:6s}] speed {ctrl.offset_us:3d} us  "
                    + "  ".join(f"{m} {o:+4d}" for m, o in offsets.items()) + "  "
                    + "  ".join(f"{n} {readings[n][0]:6.1f}° ({travel[n]:+6.1f}) {readings[n][2]}" for n in encoders))
            sys.stdout.write(line + "   ")
            sys.stdout.flush()
            over = [n for n in encoders if abs(travel[n]) > limit]
            if over:
                print(f"\nABORT: travel limit {limit} deg exceeded on {over}")
                break
            time.sleep(1.0 / rate)
    finally:
        bench.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\noutputs off")


def open_csv(path, encoder_names):
    if not path:
        return None, None
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_s", "label", "command_us"] + [f"{n}_deg" for n in encoder_names]
               + [f"{n}_travel_deg" for n in encoder_names])
    return f, w


def parse_channels(text):
    channels = []
    for part in text.split(","):
        if "-" in part:
            lo, hi = part.split("-")
            channels.extend(range(int(lo), int(hi) + 1))
        else:
            channels.append(int(part))
    if any(not 0 <= c <= 15 for c in channels):
        raise ValueError("PCA9685 channels are 0-15")
    return channels


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    sub.add_parser("stop")
    p = sub.add_parser("enc")
    p.add_argument("--rate", type=float, default=20)
    p.add_argument("--duration", type=float, default=0, help="0 = until Ctrl+C")
    p.add_argument("--csv")
    p = sub.add_parser("pwm")
    p.add_argument("servo")
    p.add_argument("offset_us", type=int)
    p.add_argument("--duration", type=float, default=1.0)
    p.add_argument("--rate", type=float, default=50)
    p.add_argument("--csv")
    p = sub.add_parser("diff")
    p.add_argument("pitch_us", type=int)
    p.add_argument("roll_us", type=int)
    p.add_argument("--duration", type=float, default=1.0)
    p.add_argument("--rate", type=float, default=50)
    p.add_argument("--csv")
    p = sub.add_parser("deadband")
    p.add_argument("servo")
    p.add_argument("encoder")
    p.add_argument("--span", type=int, default=60)
    p.add_argument("--step", type=int, default=5)
    p.add_argument("--dwell", type=float, default=1.0)
    p.add_argument("--csv")
    p = sub.add_parser("probe")
    p.add_argument("--channels", default="0-15", help="e.g. 0-15 or 8,9,10,11")
    p.add_argument("--offset", type=int, help="default: safety.jog_start_offset_us")
    p.add_argument("--duration", type=float, default=1.0)
    p = sub.add_parser("jog")
    p.add_argument("servo", nargs="?", help="servo name from the config")
    p.add_argument("--channel", type=int, help="raw PCA9685 channel (uses default_neutral_us)")
    p.add_argument("--no-encoders", action="store_true", help="do not read encoders (no travel limit)")
    p = sub.add_parser("diffjog")
    p.add_argument("pair", help="differential pair from the config: base or wrist")
    p.add_argument("--no-encoders", action="store_true", help="do not read the pair's encoders (no travel limit)")
    args = ap.parse_args(argv)

    config = json.loads(Path(args.config).read_text())
    from smbus2 import SMBus  # imported here so the tests run without hardware

    with SMBus(config["i2c_bus"]) as bus:
        bench = Bench(config, bus)
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))
        try:
            return dispatch(bench, args)
        except KeyboardInterrupt:
            print("\ninterrupted", file=sys.stderr)
        finally:
            if args.cmd in ("pwm", "diff", "deadband", "probe", "jog", "diffjog", "stop"):
                bench.stop()
    return 0


def dispatch(bench, args):
    names = list(bench.encoders)
    if args.cmd == "scan":
        for where, devices in bench.scan().items():
            print(f"{where:6s}: " + (" ".join(f"0x{a:02X}" for a in devices) or "-"))
        return 0
    if args.cmd == "stop":
        print("all outputs off")
        return 0
    if args.cmd == "jog":
        if args.channel is not None:
            channel, neutral, label = args.channel, bench.cfg["pca"].get("default_neutral_us", 1600), f"ch{args.channel}"
        elif args.servo:
            servo = bench.cfg["servos"][args.servo]
            channel, neutral, label = servo["channel"], servo["neutral_us"], args.servo
        else:
            raise SystemExit("give a servo name or --channel N")
        jog(bench, channel, neutral, label, use_encoders=not args.no_encoders)
        return 0
    if args.cmd == "diffjog":
        if args.pair not in bench.cfg.get("differentials", {}):
            raise SystemExit(f"unknown pair {args.pair!r}; config has: {', '.join(bench.cfg.get('differentials', {}))}")
        diffjog(bench, args.pair, use_encoders=not args.no_encoders)
        return 0
    if args.cmd == "probe":
        channels = parse_channels(args.channels)
        offset = args.offset if args.offset is not None else bench.safety.get("jog_start_offset_us", 300)
        print(f"probing channels {channels}: +{offset} us for {args.duration} s each - watch the arm")
        results = bench.probe(channels, offset, args.duration)
        print(f"{'ch':>3}  " + "  ".join(f"{n:>10}" for n in names))
        for ch, travel in results.items():
            moved = [n for n, d in travel.items() if abs(d) > 0.5]
            print(f"{ch:3d}  " + "  ".join(f"{travel[n]:+9.2f}°" for n in names)
                  + ("   <- " + ", ".join(moved) if moved else ""))
        return 0
    f, writer = open_csv(args.csv, names)
    try:
        if args.cmd == "enc":
            t0 = time.monotonic()
            while not args.duration or time.monotonic() - t0 < args.duration:
                r = bench.read_encoders()
                t = time.monotonic() - t0
                print(f"{t:7.2f}s  " + "  ".join(f"{n}={r[n][0]:7.2f}° ({r[n][2]}, agc {r[n][3]})" for n in names))
                if writer:
                    writer.writerow([f"{t:.4f}", "enc", ""] + [f"{r[n][0]:.3f}" for n in names] + [""] * len(names))
                time.sleep(1.0 / args.rate)
        elif args.cmd == "pwm":
            offset = bench.clamp_offset(args.offset_us)
            travel = bench.run_motion({args.servo: offset}, args.duration, args.rate, writer, "pwm")
            print("travel [deg]: " + "  ".join(f"{n}={d:+.2f}" for n, d in travel.items()))
        elif args.cmd == "diff":
            pulses = bench.differential(args.pitch_us, args.roll_us)
            travel = bench.run_motion(pulses, args.duration, args.rate, writer, "diff")
            print(f"pulses {pulses}\ntravel [deg]: " + "  ".join(f"{n}={d:+.2f}" for n, d in travel.items()))
        elif args.cmd == "deadband":
            print(f"{'offset_us':>9}  {'speed_deg_s':>11}")
            for offset in range(-args.span, args.span + 1, args.step):
                offset = bench.clamp_offset(offset)
                travel = bench.run_motion({args.servo: offset}, args.dwell, 50, writer, f"deadband {offset}")
                print(f"{offset:9d}  {travel[args.encoder] / args.dwell:11.2f}")
                time.sleep(0.3)
    finally:
        if f:
            f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
