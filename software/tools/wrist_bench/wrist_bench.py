#!/usr/bin/env python3
"""Bench test tool for the wrist: Raspberry Pi -> TCA9548A -> AS5600, PCA9685 -> servos.

Temporary test rig (ADR-0012): no ROS, no ESP32. Every motion command is bounded
in pulse offset, duration and encoder travel, and always ends with the outputs off.

    python3 wrist_bench.py scan
    python3 wrist_bench.py enc [--rate 50] [--duration 10] [--csv out.csv]
    python3 wrist_bench.py pwm SERVO OFFSET_US [--duration 1.0] [--csv out.csv]
    python3 wrist_bench.py diff PITCH_US ROLL_US [--duration 1.0] [--csv out.csv]
    python3 wrist_bench.py deadband SERVO ENCODER [--span 60] [--step 5] [--dwell 1.0]
    python3 wrist_bench.py stop

OFFSET_US is relative to the servo's neutral pulse (continuous-rotation servos:
the offset sets speed and direction, not position).
"""

import argparse
import csv
import json
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
    def __init__(self, bus, mux, address, mux_channel, freq_hz, osc_hz):
        self.bus, self.mux, self.address, self.mux_channel = bus, mux, address, mux_channel
        self.freq_hz = freq_hz
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
            self._pca = PCA9685(self.bus, self.mux, p["address"], p.get("mux_channel"), p["freq_hz"], p["osc_hz"])
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
        duration = self.clamp_duration(duration)
        start = {n: r[0] for n, r in self.read_encoders().items()}
        servos = self.cfg["servos"]
        t0 = time.monotonic()
        try:
            for name, offset in pulses.items():
                self.pca.set_pulse(servos[name]["channel"], servos[name]["neutral_us"] + offset)
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

    def differential(self, pitch_us, roll_us):
        d = self.cfg["differential"]
        a = self.clamp_offset(pitch_us + roll_us)
        b = self.clamp_offset(d.get("b_sign", 1) * (pitch_us - roll_us))
        return {d["motor_a"]: a, d["motor_b"]: b}


def open_csv(path, encoder_names):
    if not path:
        return None, None
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_s", "label", "command_us"] + [f"{n}_deg" for n in encoder_names]
               + [f"{n}_travel_deg" for n in encoder_names])
    return f, w


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
            if args.cmd in ("pwm", "diff", "deadband", "stop"):
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
