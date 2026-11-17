"""Η βαθμονόμηση από το calibration.yaml του arm_calibration (μία πηγή, ADR-0017/0019), χωρίς ROS.

Από εκεί έρχονται: το μηδέν και η φορά κάθε encoder (`encoders.<άρθρωση>.zero_deg / invert`) και
το άνοιγμα της αρπάγης (`gripper.open_deg`, 0° = κλειστή). Το config του driver (base.yaml) κρατά
μόνο την καλωδίωση, τα όρια κίνησης και τον έλεγχο.

Οι αλλαγές από το Arm Studio («Μηδέν = τρέχουσα θέση») φτάνουν ζωντανά ως αλλαγές παραμέτρων του
node /arm_calibration (node.py → Driver.update_joint).
"""

from pathlib import Path

GRIPPER = 'gripper'


def load_calibration(path) -> dict:
    """Το calibration.yaml ως «επίπεδο» λεξικό (encoders.joint_1.zero_deg: …).

    Ελέγχεται από το arm_calibration, με τους ίδιους κανόνες με το node του PC.
    """
    from arm_calibration import calibration    # το πακέτο του PC, χωρίς ROS
    return calibration.load(Path(path))


def calibration_fields(flat: dict, joint: str) -> dict:
    """Τα πεδία μιας άρθρωσης από το «επίπεδο» λεξικό του arm_calibration (encoders.<j>.…)."""
    fields = {}
    key = f'encoders.{joint}'
    if f'{key}.zero_deg' in flat:
        fields['zero_deg'] = float(flat[f'{key}.zero_deg'])
    if f'{key}.invert' in flat:
        fields['sign'] = -1 if flat[f'{key}.invert'] else 1
    if joint == GRIPPER and 'gripper.open_deg' in flat:
        fields.update(gripper_limits(float(flat['gripper.open_deg'])))
    return fields


def gripper_limits(open_deg: float) -> dict:
    """Η αρπάγη κινείται μόνο από το 0° (κλειστή) ως το open_deg (0 = δεν ανοίγει καθόλου)."""
    return {'lower_deg': min(0.0, open_deg), 'upper_deg': max(0.0, open_deg)}


def apply_calibration(config: dict, flat: dict) -> dict:
    """Γράφει μηδέν/φορά (και όρια αρπάγης) από το calibration.yaml στις αρθρώσεις του config.

    Σκάει με ValueError αν μια άρθρωση δεν έχει μηδέν και φορά: χωρίς αυτά η θέση της είναι
    άγνωστη, και ο driver δεν πρέπει να ξεκινήσει.
    """
    for group in config['groups'].values():
        for name, joint in group['joints'].items():
            fields = calibration_fields(flat, name)
            missing = {'zero_deg', 'sign'} - set(fields)
            if missing:
                raise ValueError(f'{name}: λείπει από το calibration.yaml '
                                 f'(encoders.{name}.zero_deg / invert)')
            joint.update(fields)
    return config


def parameter_change(name: str, value) -> tuple[str, dict] | None:
    """Μια αλλαγή παραμέτρου του /arm_calibration → (άρθρωση, πεδία) ή None αν δεν μας αφορά."""
    parts = name.split('.')
    if len(parts) == 3 and parts[0] == 'encoders':
        if parts[2] == 'zero_deg':
            return parts[1], {'zero_deg': float(value)}
        if parts[2] == 'invert':
            return parts[1], {'sign': -1 if value else 1}
    if name == 'gripper.open_deg':
        return GRIPPER, gripper_limits(float(value))
    return None
