"""Calibration file <-> flat ROS 2 parameters, with validation. No ROS dependency (unit-tested).

The YAML is nested (spec / servos.<name> / encoders.<joint> / safety); ROS parameters are its flattened,
dot-separated keys, e.g. ``servos.wrist_diff_a.neutral_us``.
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

# Absolute limits that no edit may exceed: servo datasheet (QY3240MG, 500-2500 us) and PCA9685 (16 channels).
HARD_PULSE_MIN_US = 500
HARD_PULSE_MAX_US = 2500
PCA_CHANNELS = 16

HEADER = """\
# Βαθμονόμηση servo, μηδενικά encoders και όρια ασφαλείας του βραχίονα (ADR-0017, ADR-0019).
# Μοναδική πηγή: αλλάζει από το Arm Studio (πάνελ «Παράμετροι» → «Αποθήκευση στο αρχείο») ή με το χέρι.
# Τιμές από software/tools/wrist_bench/wrist_bench.json (μετρήσεις πάγκου καρπού, 2026-11).
"""


@dataclass(frozen=True)
class Field:
    """How one leaf of the file is exposed: type, allowed range and a description for the GUI."""

    type: type
    description: str
    min: float | None = None
    max: float | None = None
    step: float | None = None


FIXED_FIELDS = {
    "spec.pwm_freq_hz": Field(int, "Συχνότητα PWM του PCA9685 [Hz]", 40, 400, 1),
    "spec.pulse_min_us": Field(int, "Ελάχιστος παλμός servo [µs]", HARD_PULSE_MIN_US, HARD_PULSE_MAX_US, 10),
    "spec.pulse_max_us": Field(int, "Μέγιστος παλμός servo [µs]", HARD_PULSE_MIN_US, HARD_PULSE_MAX_US, 10),
    "safety.max_offset_us": Field(int, "Μέγιστη απόκλιση από το νεκρό σημείο [µs] (±900 = πλήρης ταχύτητα)", 0, 1000, 10),
    "safety.max_duration_s": Field(float, "Μέγιστη διάρκεια συνεχούς κίνησης [s]", 0.1, 30.0, 0.1),
    "safety.max_travel_deg": Field(float, "Διακοπή αν ένας encoder μετακινηθεί περισσότερο [°]", 1.0, 360.0, 1.0),
}

SERVO_FIELDS = {
    "channel": Field(int, "Κανάλι PCA9685", 0, PCA_CHANNELS - 1, 1),
    "neutral_us": Field(int, "Νεκρό σημείο (στάση) [µs]", HARD_PULSE_MIN_US, HARD_PULSE_MAX_US, 5),
    "invert": Field(bool, "Αντίστροφη φορά"),
    "note": Field(str, "Σημείωση"),
}

# One per joint: AS5600 on the joint output axis, 1:1 (ADR-0019).
ENCODER_FIELDS = {
    "zero_deg": Field(float, "Ένδειξη του encoder όταν η άρθρωση είναι στο 0 [°]", 0.0, 360.0, 0.01),
    "invert": Field(bool, "Ο encoder μετρά αντίθετα από τη θετική φορά της άρθρωσης"),
    "note": Field(str, "Σημείωση"),
}
GROUPS = {"servos": SERVO_FIELDS, "encoders": ENCODER_FIELDS}


def field_for(name: str) -> Field:
    if name in FIXED_FIELDS:
        return FIXED_FIELDS[name]
    parts = name.split(".")
    if len(parts) == 3 and parts[0] in GROUPS and parts[2] in GROUPS[parts[0]]:
        return GROUPS[parts[0]][parts[2]]
    raise KeyError(f"unknown calibration parameter: {name}")


def flatten(tree: dict, prefix: str = "") -> dict:
    flat = {}
    for key, value in tree.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(flatten(value, f"{name}."))
        else:
            flat[name] = value
    return flat


def unflatten(flat: dict) -> dict:
    tree: dict = {}
    for name, value in flat.items():
        node = tree
        *parents, leaf = name.split(".")
        for p in parents:
            node = node.setdefault(p, {})
        node[leaf] = value
    return tree


def coerce(name: str, value):
    """Value converted to the field's type (YAML may give 3 for 3.0); raises ValueError if impossible."""
    kind = field_for(name).type
    if kind is bool:
        if not isinstance(value, bool):
            raise ValueError(f"{name}: expected true/false")
        return value
    if kind is int:
        if isinstance(value, bool) or not float(value).is_integer():
            raise ValueError(f"{name}: expected an integer")
        return int(value)
    if kind is float:
        if isinstance(value, bool):
            raise ValueError(f"{name}: expected a number")
        return float(value)
    return str(value)


def names_in(flat: dict, group: str) -> list[str]:
    return sorted({name.split(".")[1] for name in flat if name.startswith(f"{group}.")})


def servo_names(flat: dict) -> list[str]:
    return names_in(flat, "servos")


def encoder_names(flat: dict) -> list[str]:
    return names_in(flat, "encoders")


def validate(flat: dict) -> list[str]:
    """All problems of a complete calibration; empty list = valid.

    neutral ± max_offset may exceed the pulse limits: every command is clamped to
    [pulse_min_us, pulse_max_us] when it is sent (as in wrist_bench), so it never reaches the servo."""
    errors = []
    for name in FIXED_FIELDS:
        if name not in flat:
            errors.append(f"λείπει το {name}")
    for group, fields in GROUPS.items():
        for item in names_in(flat, group):
            for leaf in fields:
                if f"{group}.{item}.{leaf}" not in flat:
                    errors.append(f"λείπει το {group}.{item}.{leaf}")
    if errors:
        return errors

    for name, value in flat.items():
        try:
            f = field_for(name)
        except KeyError as exc:
            errors.append(str(exc))
            continue
        if name.endswith(".zero_deg") and not 0.0 <= value < 360.0:
            errors.append(f"{name} = {value}: εκτός ορίων [0, 360)")
        elif f.min is not None and not f.min <= value <= f.max:
            errors.append(f"{name} = {value}: εκτός ορίων [{f.min}, {f.max}]")

    lo, hi = flat["spec.pulse_min_us"], flat["spec.pulse_max_us"]
    if lo >= hi:
        errors.append(f"spec.pulse_min_us ({lo}) πρέπει να είναι μικρότερο από spec.pulse_max_us ({hi})")
    channels = {}
    for servo in servo_names(flat):
        neutral = flat[f"servos.{servo}.neutral_us"]
        if not lo < neutral < hi:
            errors.append(f"servos.{servo}.neutral_us ({neutral}) πρέπει να είναι μέσα στο ({lo}, {hi})")
        channel = flat[f"servos.{servo}.channel"]
        if channel in channels:
            errors.append(f"το κανάλι {channel} χρησιμοποιείται από {channels[channel]} και {servo}")
        channels[channel] = servo
    return errors


def load(path: Path) -> dict:
    flat = flatten(yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {})
    flat = {name: coerce(name, value) for name, value in flat.items()}
    errors = validate(flat)
    if errors:
        raise ValueError(f"{path}: " + "; ".join(errors))
    return flat


def dump(flat: dict) -> str:
    tree = unflatten(flat)
    ordered = {
        "spec": tree.get("spec", {}),
        "servos": {name: tree["servos"][name] for name in sorted(tree.get("servos", {}))},
        "encoders": {name: tree["encoders"][name] for name in sorted(tree.get("encoders", {}))},
        "safety": tree.get("safety", {}),
    }
    return HEADER + yaml.safe_dump(ordered, allow_unicode=True, sort_keys=False, default_flow_style=False)


def save(path: Path, flat: dict):
    errors = validate(flat)
    if errors:
        raise ValueError("; ".join(errors))
    path = Path(path).resolve()  # with colcon --symlink-install this is the file in the repo
    tmp = path.with_suffix(".tmp")
    tmp.write_text(dump(flat), encoding="utf-8")
    tmp.replace(path)
