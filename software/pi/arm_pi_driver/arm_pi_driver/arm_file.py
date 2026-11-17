"""Το ΕΝΑ αρχείο του βραχίονα (arm.yaml, ADR-0023) → η ρύθμιση του driver, χωρίς ROS.

Το αρχείο (arm_calibration/config/arm.yaml) έχει λίστες servo / encoders / αισθητήρων πίεσης και
αρθρώσεις που τις αναφέρουν. Ο driver δουλεύει με ΟΜΑΔΕΣ (control/): αρθρώσεις που μοιράζονται
servo ελέγχονται μαζί. Εδώ γίνεται η «μετάφραση»:

    άρθρωση.servos {servo: ±1}   → μοτέρ.mix {άρθρωση: ±1}  (το πρόσημο ΕΙΝΑΙ η μίξη)
    άρθρωση.encoder → κανάλι     → άρθρωση.mux_channel
    άρθρωση.invert               → άρθρωση.sign (−1 / +1)
    άρθρωση.limits_deg           → lower_deg / upper_deg  (αρπάγη: το μεσαίο travel_margin)
    άρθρωση.deadband [+, −]      → deadband_pos / deadband_neg
    servo.deadband_us [κ, π]     → μοτέρ.deadband_edges_us
    αισθητήρας.max_v × force_margin → όριο επαφής ανά αισθητήρα (αρπάγη)

Ενώ τρέχει, κάθε αλλαγή παραμέτρου του /arm_calibration (PC) γράφεται στα δεδομένα του αρχείου και
γίνεται `Change` για τον Driver (Driver.apply_change).
"""

from dataclasses import dataclass
from pathlib import Path

JOINT, SERVO, GRIPPER_GROUP, RESTART = 'joint', 'servo', 'gripper', 'restart'
LIVE_JOINT = {'zero_deg', 'invert', 'limits_deg', 'deadband', 'travel_margin'}
LIVE_SERVO = {'neutral_us', 'deadband_us'}
IGNORED = {'description'}


@dataclass(frozen=True)
class Change:
    """Μία αλλαγή: σε άρθρωση, σε servo, στα όρια επαφής, ή κάτι που θέλει επανεκκίνηση."""

    kind: str           # JOINT, SERVO, GRIPPER_GROUP ή RESTART
    name: str           # άρθρωση, servo, αρπάγη, ή η παράμετρος (RESTART)
    fields: dict        # τα πεδία της ρύθμισης του driver που αλλάζουν


def load(path) -> dict:
    """Το arm.yaml, ελεγμένο με τους ίδιους κανόνες με το Arm Studio (arm_calibration)."""
    from arm_calibration import arm_config      # το πακέτο του PC, χωρίς ROS
    return arm_config.load(Path(path))


def joint_fields(data: dict, name: str) -> dict:
    """Όσα της άρθρωσης αλλάζουν ζωντανά: μηδέν, φορά, όρια, νεκρή ζώνη."""
    joint = data['joints'][name]
    lower, upper = joint['limits_deg']
    if 'travel_margin' in joint:                # αρπάγη: στάση λίγο πριν από κάθε άκρο
        gap = (1.0 - joint['travel_margin']) / 2 * (upper - lower)
        lower, upper = lower + gap, upper - gap
    fields = {'zero_deg': float(joint['zero_deg']), 'sign': -1 if joint['invert'] else 1,
              'lower_deg': float(lower), 'upper_deg': float(upper)}
    if 'deadband' in joint:
        fields['deadband_pos'], fields['deadband_neg'] = (float(v) for v in joint['deadband'])
    return fields


def servo_fields(data: dict, name: str) -> dict:
    """Όσα του servo αλλάζουν ζωντανά: νεκρό σημείο και νεκρή ζώνη."""
    servo = data['servos'][name]
    fields = {'neutral_us': float(servo['neutral_us'])}
    if 'deadband_us' in servo:
        fields['deadband_edges_us'] = [float(v) for v in servo['deadband_us']]
    return fields


def contact_levels(data: dict, name: str) -> dict:
    """{αισθητήρας: τάση στάσης του κλεισίματος} = max_v × force_margin· όσοι έχουν max_v."""
    joint = data['joints'][name]
    margin = joint.get('force_margin', 1.0)
    sensors = data['force_sensors']
    return {s: sensors[s]['max_v'] * margin for s in joint.get('force_sensors', [])
            if sensors[s].get('max_v') is not None}


def driver_config(data: dict) -> tuple[dict, list]:
    """(ρύθμιση του driver με ΜΟΝΟ τις ενεργές ομάδες, ονόματα ανενεργών αρθρώσεων).

    ValueError αν καμία άρθρωση δεν είναι ενεργή.
    """
    from arm_calibration import arm_config
    controller = data['controller']
    keys = ('segment', 'i2c_bus', 'tca_address', 'pca', 'control')
    config = {key: controller[key] for key in keys}
    config['groups'] = {}
    disabled = []
    for group in arm_config.groups(data):
        if not group.enabled:
            disabled += group.joints
            continue
        config['groups']['+'.join(group.joints)] = _group(data, group)
    if not config['groups']:
        raise ValueError('καμία ενεργή άρθρωση στο arm.yaml (enabled: true)')
    return config, disabled


def _group(data: dict, group) -> dict:
    from arm_calibration import arm_config
    joints = {}
    for name in group.joints:
        joint = data['joints'][name]
        joints[name] = {'mux_channel': data['encoders'][joint['encoder']]['channel'],
                        'law': dict(joint['law']), **joint_fields(data, name)}
    signs = arm_config.servo_signs(data, group)
    motors = {}
    for servo in group.servos:
        motors[servo] = {'servo': servo, 'channel': data['servos'][servo]['channel'],
                         'mix': {j: s for j, s in signs[servo].items() if s}, 'mirrored': False,
                         **servo_fields(data, servo)}
    result = {'enabled': True, 'joints': joints, 'motors': motors}
    sensors = [s for j in group.joints for s in data['joints'][j].get('force_sensors', [])]
    if any('force_sensors' in data['joints'][j] for j in group.joints):
        if len(group.joints) != 1:
            raise ValueError(f'{group.joints}: αισθητήρες πίεσης μόνο σε άρθρωση με δικά της '
                             'servo')
        channels = {data['force_sensors'][s]['adc_channel'] for s in sensors}
        if len(channels) > 1:
            raise ValueError(f'{group.joints[0]}: αισθητήρες πίεσης σε διαφορετικά κανάλια '
                             f'{channels}')
        adc = data['controller']['adc']
        result.update(controller='gripper', contact=contact_levels(data, group.joints[0]),
                      adc={'mux_channel': channels.pop() if channels else None, **adc},
                      force_sensors={s: {'input': data['force_sensors'][s]['input']}
                                     for s in sensors})
    return result


def parameter_change(data: dict, name: str, value) -> Change | None:
    """Μια παράμετρος του /arm_calibration άλλαξε: γράφεται στα δεδομένα → Change (ή None)."""
    from arm_calibration import arm_config
    try:
        arm_config.set_value(data, name, value)
    except KeyError:
        return None
    parts = name.split('.')
    section, item, leaf = parts[0], parts[1] if len(parts) > 1 else '', parts[-1]
    if leaf in IGNORED:
        return None
    if section == 'joints' and len(parts) == 3 and leaf in LIVE_JOINT:
        return Change(JOINT, item, joint_fields(data, item))
    if section == 'servos' and leaf in LIVE_SERVO:
        return Change(SERVO, item, servo_fields(data, item))
    pressure = (section, leaf) in (('joints', 'force_margin'), ('force_sensors', 'max_v'))
    if pressure:
        owners = [j for j, joint in data['joints'].items()
                  if section == 'joints' and j == item or item in joint.get('force_sensors', [])]
        if not owners:
            return None
        return Change(GRIPPER_GROUP, owners[0], {'contact': contact_levels(data, owners[0])})
    return Change(RESTART, name, {})
