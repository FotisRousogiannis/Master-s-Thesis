"""The one arm file of ADR-0023: load, validate, save and the structure it implies. No ROS dependency.

Lists (``servos``, ``encoders``, ``force_sensors``) hold each part once; ``joints`` refer to them by name:
a joint names its servos with a sign (+1 / -1) and its encoder. Joints that share servos form one group,
whose kind follows from the shape: one joint and one servo = single, one joint and more servos = mirrored,
more joints = differential.
"""

from array import array
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path

import yaml

FILE = Path(__file__).resolve().parents[1] / "config" / "arm.yaml"
SECTIONS = ("servos", "encoders", "force_sensors", "joints", "controller")

# Absolute limits: servo datasheet (QY3240MG, 500-2500 us), PCA9685 (16 outputs), TCA9548A (8 channels),
# ADS1115 (4 inputs).
HARD_PULSE_MIN_US = 500
HARD_PULSE_MAX_US = 2500
PCA_CHANNELS = 16
TCA_CHANNELS = 8
ADC_INPUTS = 4
LAWS = ("pid", "bang_bang")

SINGLE = "single"
MIRRORED = "mirrored"
DIFFERENTIAL = "differential"


class ArmConfigError(ValueError):
    """The file breaks the rules; ``errors`` lists every problem, in Greek, for the GUI."""

    def __init__(self, errors: list[str]):
        super().__init__("\n".join(errors))
        self.errors = errors


@dataclass(frozen=True)
class Group:
    """Joints moved by the same servos: they are controlled together."""

    joints: tuple[str, ...]
    servos: tuple[str, ...]
    kind: str
    enabled: bool


def load(path: Path = FILE) -> dict:
    """The file as a dict; raises ArmConfigError if it breaks the rules."""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    errors = validate(data)
    if errors:
        raise ArmConfigError(errors)
    return data


def save(data: dict, path: Path = FILE) -> None:
    """Write the file (validated first), keeping its header comment."""
    errors = validate(data)
    if errors:
        raise ArmConfigError(errors)
    path = Path(path)
    header = _header(path)
    body = yaml.dump(data, Dumper=_Dumper, sort_keys=False, allow_unicode=True, width=120)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(header + body, encoding="utf-8")
    tmp.replace(path)  # atomic: a crash never leaves half a file


def groups(data: dict) -> list[Group]:
    """Joints linked by shared servos, in file order, with their inferred kind."""
    joints = data["joints"]
    seen: set[str] = set()
    result = []
    for start in joints:
        if start in seen:
            continue
        members, servos, todo = [], [], [start]
        while todo:
            name = todo.pop()
            if name in members:
                continue
            members.append(name)
            for servo in joints[name]["servos"]:
                if servo not in servos:
                    servos.append(servo)
                todo += [j for j in joints if j not in members and servo in joints[j]["servos"]]
        members.sort(key=list(joints).index)
        seen.update(members)
        kind = DIFFERENTIAL if len(members) > 1 else MIRRORED if len(servos) > 1 else SINGLE
        enabled = all(joints[j].get("enabled") for j in members)
        result.append(Group(tuple(members), tuple(servos), kind, enabled))
    return result


def servo_signs(data: dict, group: Group) -> dict[str, dict[str, int]]:
    """Per servo, its sign in each joint of the group (0 if it does not move that joint)."""
    joints = data["joints"]
    return {s: {j: int(joints[j]["servos"].get(s, 0)) for j in group.joints} for s in group.servos}


# ---------------------------------------------------------------------------------------------- parameters
# The file as flat ROS 2 parameters ("joints.joint_1.zero_deg"), shared by the PC node (/arm_calibration)
# and the Pi driver, so that both compare the same values. null = not known yet: no parameter.

FLOAT_LEAVES = {"zero_deg", "kp", "ki", "kd", "speed", "max_v", "travel_margin", "force_margin",
                "limits_deg", "deadband", "full_scale_v", "safety_v"}
# Leaves that may be added while running (a null in the file has no parameter until it gets a value).
NEW_LEAVES = {"channel", "adc_channel", "max_v", "deadband_us", "deadband", "encoder"}
DESCRIPTIONS = {
    "channel": "Κανάλι (PCA9685 για servo, TCA9548A για encoder)",
    "neutral_us": "Νεκρό σημείο (στάση) [µs]",
    "deadband_us": "Νεκρή ζώνη servo [κάτω, πάνω] [µs]",
    "description": "Περιγραφή",
    "adc_channel": "Κανάλι TCA9548A του ADS1115",
    "input": "Είσοδος ADS1115 (0–3)",
    "max_v": "Μέγιστη πίεση: τάση ZD10 [V]",
    "enabled": "Ενεργή άρθρωση",
    "encoder": "Encoder της άρθρωσης",
    "zero_deg": "Ένδειξη του encoder όταν η άρθρωση είναι στο 0 [°]",
    "invert": "Αντίθετη φορά από τον encoder",
    "limits_deg": "Όρια [κάτω, πάνω] [°]",
    "deadband": "Ελάχιστη εντολή που κινεί την άρθρωση [θετικά, αρνητικά]",
    "travel_margin": "Μέρος της διαδρομής που χρησιμοποιείται (0.9 = 90 %)",
    "force_margin": "Στάση κλεισίματος σε αυτό το μέρος της μέγιστης πίεσης",
}


def _flatten(tree: dict, prefix: str = "") -> dict:
    flat = {}
    for key, value in tree.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(_flatten(value, f"{name}."))
        else:
            flat[name] = value
    return flat


def coerce(name: str, value):
    """A value in the type its parameter has: floats where a float is meant (YAML may write 0 for 0.0)."""
    sequence = isinstance(value, (list, tuple, array))     # rclpy gives arrays as array.array
    if name.rsplit(".", 1)[-1] not in FLOAT_LEAVES or value is None or isinstance(value, bool):
        return list(value) if sequence else value
    if sequence:
        return [float(v) for v in value]
    return float(value)


def parameters(data: dict) -> dict:
    """{flat name: value} for every known value (nulls left out)."""
    return {name: coerce(name, v) for name, v in _flatten(data).items() if v is not None}


def get_value(data: dict, name: str):
    node = data
    for part in name.split("."):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(name)
        node = node[part]
    return node


def set_value(data: dict, name: str, value) -> None:
    """Set one flat parameter in the nested data; KeyError for a name the file cannot have."""
    *parents, leaf = name.split(".")
    node = data
    for part in parents:
        if not isinstance(node, dict) or not isinstance(node.get(part), dict):
            raise KeyError(f"άγνωστη παράμετρος: {name}")
        node = node[part]
    known = leaf in node or leaf in NEW_LEAVES or (len(parents) == 3 and parents[2] == "servos")
    if not known:
        raise KeyError(f"άγνωστη παράμετρος: {name}")
    node[leaf] = coerce(name, value)


# ---------------------------------------------------------------------------------------------- validation


def validate(data: dict) -> list[str]:
    """Every rule the file breaks (empty = valid)."""
    if not isinstance(data, dict):
        return ["το αρχείο δεν είναι λεξικό YAML"]
    errors = [f"λείπει το τμήμα «{s}»" for s in SECTIONS if not isinstance(data.get(s), dict)]
    if errors:
        return errors
    errors += _check_controller(data["controller"])
    pulse = (data["controller"].get("pca") or {})
    lo = pulse.get("pulse_min_us", HARD_PULSE_MIN_US)
    hi = pulse.get("pulse_max_us", HARD_PULSE_MAX_US)
    errors += _check_servos(data["servos"], lo, hi)
    errors += _check_encoders(data["encoders"])
    errors += _check_force_sensors(data["force_sensors"])
    errors += _check_joints(data)
    if not errors:
        errors += _check_groups(data)
    return errors


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _channel(where: str, value, count: int) -> list[str]:
    if value is None or (_is_int(value) and 0 <= value < count):
        return []
    return [f"{where}: κανάλι {value!r} (επιτρέπεται 0–{count - 1} ή null)"]


def _pair(where: str, value, low: float, high: float) -> list[str]:
    if (isinstance(value, list) and len(value) == 2 and all(_is_num(v) and low <= v <= high for v in value)):
        return []
    return [f"{where}: θέλει [α, β] με τιμές {low}–{high}"]


def _duplicates(where: str, owners: dict) -> list[str]:
    by_value: dict = {}
    for name, value in owners.items():
        if value is not None:
            by_value.setdefault(value, []).append(name)
    return [f"{where} {value}: το έχουν {', '.join(names)}" for value, names in by_value.items() if len(names) > 1]


def _check_controller(c: dict) -> list[str]:
    errors = []
    pca = c.get("pca")
    if not isinstance(pca, dict):
        return ["controller: λείπει το «pca»"]
    lo, hi = pca.get("pulse_min_us"), pca.get("pulse_max_us")
    if not (_is_int(lo) and _is_int(hi) and HARD_PULSE_MIN_US <= lo < hi <= HARD_PULSE_MAX_US):
        errors.append(f"controller.pca: παλμοί {lo}–{hi} έξω από {HARD_PULSE_MIN_US}–{HARD_PULSE_MAX_US} µs")
    errors += _channel("controller.pca.mux_channel", pca.get("mux_channel"), TCA_CHANNELS)
    if not isinstance(c.get("control"), dict):
        errors.append("controller: λείπει το «control»")
    return errors


def _check_servos(servos: dict, lo: int, hi: int) -> list[str]:
    errors = []
    for name, s in servos.items():
        where = f"servos.{name}"
        if not isinstance(s, dict):
            errors.append(f"{where}: δεν είναι λεξικό")
            continue
        errors += _channel(where, s.get("channel"), PCA_CHANNELS)
        neutral = s.get("neutral_us")
        if not (_is_int(neutral) and lo <= neutral <= hi):
            errors.append(f"{where}: νεκρό σημείο {neutral!r} έξω από {lo}–{hi} µs")
            continue
        if "deadband_us" in s:
            band = s["deadband_us"]
            bad = _pair(f"{where}.deadband_us", band, lo, hi)
            errors += bad
            if not bad and not band[0] <= neutral <= band[1]:
                errors.append(f"{where}: το νεκρό σημείο {neutral} είναι έξω από τη νεκρή ζώνη {band}")
    errors += _duplicates("κανάλι PCA", {n: s.get("channel") for n, s in servos.items() if isinstance(s, dict)})
    return errors


def _check_encoders(encoders: dict) -> list[str]:
    errors = []
    for name, e in encoders.items():
        if not isinstance(e, dict):
            errors.append(f"encoders.{name}: δεν είναι λεξικό")
            continue
        errors += _channel(f"encoders.{name}", e.get("channel"), TCA_CHANNELS)
    errors += _duplicates("κανάλι encoder", {n: e.get("channel") for n, e in encoders.items()
                                             if isinstance(e, dict)})
    return errors


def _check_force_sensors(sensors: dict) -> list[str]:
    errors = []
    for name, f in sensors.items():
        where = f"force_sensors.{name}"
        if not isinstance(f, dict):
            errors.append(f"{where}: δεν είναι λεξικό")
            continue
        errors += _channel(where, f.get("adc_channel"), TCA_CHANNELS)
        if not (_is_int(f.get("input")) and 0 <= f["input"] < ADC_INPUTS):
            errors.append(f"{where}: είσοδος {f.get('input')!r} (επιτρέπεται 0–{ADC_INPUTS - 1})")
        if "max_v" in f and not (_is_num(f["max_v"]) and f["max_v"] > 0):
            errors.append(f"{where}: max_v πρέπει να είναι θετικός αριθμός")
    places = {n: (f.get("adc_channel"), f.get("input")) for n, f in sensors.items()
              if isinstance(f, dict) and f.get("adc_channel") is not None}
    errors += _duplicates("ADS1115 (κανάλι, είσοδος)", places)
    return errors


def _check_joints(data: dict) -> list[str]:
    errors = []
    servos, encoders, sensors = data["servos"], data["encoders"], data["force_sensors"]
    for name, j in data["joints"].items():
        w = f"joints.{name}"
        if not isinstance(j, dict):
            errors.append(f"{w}: δεν είναι λεξικό")
            continue
        if not isinstance(j.get("enabled"), bool):
            errors.append(f"{w}: enabled πρέπει να είναι true/false")
        signs = j.get("servos")
        if not isinstance(signs, dict) or not signs:
            errors.append(f"{w}: χρειάζεται τουλάχιστον ένα servo με πρόσημο")
            signs = {}
        for servo, sign in signs.items():
            if servo not in servos:
                errors.append(f"{w}: άγνωστο servo «{servo}»")
            if sign not in (1, -1) or isinstance(sign, bool):
                errors.append(f"{w}.servos.{servo}: πρόσημο {sign!r} (επιτρέπεται +1 ή -1)")
        encoder = j.get("encoder")
        if encoder is not None and encoder not in encoders:
            errors.append(f"{w}: άγνωστος encoder «{encoder}»")
        zero = j.get("zero_deg")
        if not (_is_num(zero) and 0 <= zero < 360):
            errors.append(f"{w}: zero_deg {zero!r} (επιτρέπεται 0–360)")
        if not isinstance(j.get("invert"), bool):
            errors.append(f"{w}: invert πρέπει να είναι true/false")
        limits = j.get("limits_deg")
        errors += _pair(f"{w}.limits_deg", limits, -360, 360)
        if isinstance(limits, list) and len(limits) == 2 and _is_num(limits[0]) and _is_num(limits[1]) \
                and limits[0] > limits[1]:
            errors.append(f"{w}: κάτω όριο {limits[0]} > πάνω όριο {limits[1]}")
        if "deadband" in j:
            errors += _pair(f"{w}.deadband", j["deadband"], 0, 1)
        law = j.get("law")
        if not (isinstance(law, dict) and law.get("type") in LAWS):
            errors.append(f"{w}: law.type πρέπει να είναι ένα από {', '.join(LAWS)}")
        for sensor in j.get("force_sensors", []):
            if sensor not in sensors:
                errors.append(f"{w}: άγνωστος αισθητήρας πίεσης «{sensor}»")
        for key in ("travel_margin", "force_margin"):
            if key in j and not (_is_num(j[key]) and 0 < j[key] <= 1):
                errors.append(f"{w}: {key} πρέπει να είναι στο (0, 1]")
        if j.get("enabled") is True:
            errors += _check_enabled(w, j, data)
    used = {n: j.get("encoder") for n, j in data["joints"].items() if isinstance(j, dict)}
    errors += _duplicates("encoder", used)
    return errors


def _check_enabled(where: str, joint: dict, data: dict) -> list[str]:
    """An enabled joint must be fully wired: every part it uses has a known channel."""
    errors = []
    for servo in joint.get("servos") or {}:
        if data["servos"].get(servo, {}).get("channel") is None:
            errors.append(f"{where}: ενεργή, αλλά το {servo} δεν έχει κανάλι")
    encoder = joint.get("encoder")
    if encoder is None or data["encoders"].get(encoder, {}).get("channel") is None:
        errors.append(f"{where}: ενεργή, αλλά δεν έχει encoder με κανάλι")
    for sensor in joint.get("force_sensors", []):
        if data["force_sensors"].get(sensor, {}).get("adc_channel") is None:
            errors.append(f"{where}: ενεργή, αλλά ο {sensor} δεν έχει κανάλι")
    return errors


def _check_groups(data: dict) -> list[str]:
    errors = []
    joints = data["joints"]
    for group in groups(data):
        names = ", ".join(group.joints)
        states = {joints[j]["enabled"] for j in group.joints}
        if len(states) > 1:
            errors.append(f"{names}: μοιράζονται servo, άρα ενεργές ή ανενεργές όλες μαζί")
        if group.kind == DIFFERENTIAL:
            signs = servo_signs(data, group)
            rows = [[signs[s][j] for s in group.servos] for j in group.joints]
            if _rank(rows) < len(rows):
                errors.append(f"{names}: με αυτά τα πρόσημα οι αρθρώσεις δεν κινούνται ανεξάρτητα "
                              "(π.χ. ίδια πρόσημα και στις δύο)")
    return errors


def _rank(rows: list[list[int]]) -> int:
    m = [[Fraction(v) for v in row] for row in rows]
    rank = 0
    for col in range(len(m[0]) if m else 0):
        pivot = next((r for r in range(rank, len(m)) if m[r][col] != 0), None)
        if pivot is None:
            continue
        m[rank], m[pivot] = m[pivot], m[rank]
        for r in range(len(m)):
            if r != rank and m[r][col] != 0:
                factor = m[r][col] / m[rank][col]
                m[r] = [a - factor * b for a, b in zip(m[r], m[rank])]
        rank += 1
    return rank


# ---------------------------------------------------------------------------------------------- writing


class _Dumper(yaml.SafeDumper):
    """Short dicts of plain values and all lists on one line, like the hand-written file."""


def _represent_dict(dumper, value):
    flow = len(value) <= 6 and all(not isinstance(v, (dict, list)) for v in value.values())
    return dumper.represent_mapping("tag:yaml.org,2002:map", value, flow_style=flow)


def _represent_list(dumper, value):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", value, flow_style=True)


_Dumper.add_representer(dict, _represent_dict)
_Dumper.add_representer(list, _represent_list)


def _header(path: Path) -> str:
    if not path.exists():
        return ""
    lines = []
    for line in path.read_text(encoding="utf-8").splitlines(keepends=True):
        if not line.startswith("#"):
            break
        lines.append(line)
    return "".join(lines)
