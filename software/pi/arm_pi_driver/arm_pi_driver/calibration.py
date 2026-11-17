"""Η βαθμονόμηση από το ΚΟΙΝΟ αρχείο calibration.yaml (arm_calibration, ADR-0017/0019), χωρίς ROS.

Ό,τι μετριέται έρχεται από εκεί· το base.yaml του driver κρατά μόνο καλωδίωση και σχεδιασμό:

    encoders.<άρθρωση>.zero_deg / invert / deadband_pos / deadband_neg → η άρθρωση
    servos.<servo>.channel / neutral_us / invert / deadband_down_us/up_us → μοτέρ `servo: <servo>`
    gripper.open_deg → όρια αρπάγης (0° = κλειστή) · gripper.contact_v → όριο επαφής

Στην εκκίνηση από το αρχείο· ενώ τρέχει, από τις παραμέτρους του node /arm_calibration (node.py):
κάθε αλλαγή γίνεται μια `Change`, που την εφαρμόζει ο Driver (Driver.apply_change).
"""

from dataclasses import dataclass
from pathlib import Path

GRIPPER = 'gripper'
JOINT, SERVO, GRIPPER_GROUP = 'joint', 'servo', 'gripper'


@dataclass(frozen=True)
class Change:
    """Μία αλλαγή βαθμονόμησης: σε άρθρωση, σε servo, ή στην αρπάγη."""

    kind: str           # JOINT, SERVO ή GRIPPER_GROUP
    name: str           # άρθρωση ή servo
    fields: dict        # τα πεδία του config που αλλάζουν


def load_calibration(path) -> dict:
    """Το calibration.yaml ως «επίπεδο» λεξικό (encoders.joint_1.zero_deg: …).

    Ελέγχεται από το arm_calibration, με τους ίδιους κανόνες με το node του PC.
    """
    from arm_calibration import calibration    # το πακέτο του PC, χωρίς ROS
    return calibration.load(Path(path))


def gripper_limits(open_deg: float) -> dict:
    """Η αρπάγη κινείται μόνο από το 0° (κλειστή) ως το open_deg (0 = δεν ανοίγει καθόλου)."""
    return {'lower_deg': min(0.0, open_deg), 'upper_deg': max(0.0, open_deg)}


def joint_fields(flat: dict, joint: str) -> dict:
    """Τα πεδία μιας άρθρωσης από το επίπεδο λεξικό (όσα υπάρχουν)."""
    key, fields = f'encoders.{joint}', {}
    if f'{key}.zero_deg' in flat:
        fields['zero_deg'] = float(flat[f'{key}.zero_deg'])
    if f'{key}.invert' in flat:
        fields['sign'] = -1 if flat[f'{key}.invert'] else 1
    for leaf in ('deadband_pos', 'deadband_neg'):
        if f'{key}.{leaf}' in flat:
            fields[leaf] = float(flat[f'{key}.{leaf}'])
    if joint == GRIPPER and 'gripper.open_deg' in flat:
        fields.update(gripper_limits(float(flat['gripper.open_deg'])))
    return fields


def servo_fields(flat: dict, servo: str) -> dict:
    """Τα πεδία ενός μοτέρ από το servos.<servo> (όσα υπάρχουν)."""
    key, fields = f'servos.{servo}', {}
    if f'{key}.channel' in flat:
        fields['channel'] = int(flat[f'{key}.channel'])
    if f'{key}.neutral_us' in flat:
        fields['neutral_us'] = float(flat[f'{key}.neutral_us'])
    if f'{key}.invert' in flat:
        fields['mirrored'] = bool(flat[f'{key}.invert'])
    down, up = flat.get(f'{key}.deadband_down_us'), flat.get(f'{key}.deadband_up_us')
    if down is not None and up is not None:
        fields['deadband_edges_us'] = [float(down), float(up)]
    return fields


def apply_calibration(config: dict, flat: dict) -> dict:
    """Γράφει όσα μετριούνται (από το calibration.yaml) στο config του driver.

    Σκάει με ValueError αν μια άρθρωση δεν έχει μηδέν/φορά ή ένα μοτέρ δεν έχει servo με κανάλι
    και νεκρό σημείο: χωρίς αυτά ο driver δεν πρέπει να ξεκινήσει.
    """
    for group in config['groups'].values():
        for name, joint in group['joints'].items():
            fields = joint_fields(flat, name)
            if not {'zero_deg', 'sign'} <= set(fields):
                raise ValueError(f'{name}: λείπει από το calibration.yaml '
                                 f'(encoders.{name}.zero_deg / invert)')
            joint.update(fields)
        for motor in group['motors'].values():
            fields = servo_fields(flat, motor['servo'])
            if not {'channel', 'neutral_us'} <= set(fields):
                raise ValueError(f"servo {motor['servo']}: λείπει από το calibration.yaml "
                                 f"(servos.{motor['servo']}.channel / neutral_us)")
            motor.setdefault('mirrored', False)
            motor.update(fields)
        if 'gripper.contact_v' in flat and 'force_sensors' in group:
            group['contact'] = float(flat['gripper.contact_v'])
    return config


def parameter_change(name: str, value) -> Change | None:
    """Μια αλλαγή παραμέτρου του /arm_calibration → Change, ή None αν δεν μας αφορά."""
    parts = name.split('.')
    if len(parts) == 3 and parts[0] == 'encoders':
        fields = joint_fields({name: value}, parts[1])
        return Change(JOINT, parts[1], fields) if fields else None
    if len(parts) == 3 and parts[0] == 'servos':
        if parts[2] in ('deadband_down_us', 'deadband_up_us'):
            return Change(SERVO, parts[1], {parts[2]: float(value)})   # η άλλη άκρη από τον Driver
        fields = servo_fields({name: value}, parts[1])
        return Change(SERVO, parts[1], fields) if fields else None
    if name == 'gripper.open_deg':
        return Change(JOINT, GRIPPER, gripper_limits(float(value)))
    if name == 'gripper.contact_v':
        return Change(GRIPPER_GROUP, GRIPPER, {'contact': float(value)})
    return None
