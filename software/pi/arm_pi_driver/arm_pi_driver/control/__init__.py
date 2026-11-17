"""Ο έλεγχος του βραχίονα, χωρίς ROS και χωρίς υλικό.

interfaces.py: Controller (ομάδα αρθρώσεων), ControlLaw (νόμος ελέγχου), LimitError.
laws.py: PID, BangBang· LAWS = όνομα στο YAML → κλάση.
pulses.py: ταχύτητα → παλμός, με αντιστάθμιση νεκρής ζώνης ανά μοτέρ.
mix.py: MixController (διαφορικό, κατοπτρικά, ένα servo· μίξη από το YAML).
gripper.py: GripperController (MixController + στάση του κλεισίματος στην επαφή).
"""

from arm_pi_driver.control.gripper import GripperController
from arm_pi_driver.control.interfaces import ControlLaw, Controller, LimitError
from arm_pi_driver.control.laws import BangBang, LAWS, make_law, PID
from arm_pi_driver.control.mix import MixController


def build_controllers(config: dict, force_sensors: dict | None = None) -> list:
    """Ένας controller ανά ομάδα του YAML (groups)· κάθε άρθρωση σε μία μόνο ομάδα.

    force_sensors: {όνομα: ForceSensor} για τις ομάδες με `force_sensors` (αρπάγη).
    """
    force_sensors = force_sensors or {}
    controllers, seen_joints, seen_channels = [], set(), set()
    for name, group in config['groups'].items():
        kind = group.get('controller', 'mix')
        if kind == 'mix':
            controller = MixController(group, config['control'], config['pca'])
        elif kind == 'gripper':
            names = list(group.get('force_sensors', {}))
            missing = [n for n in names if n not in force_sensors]
            if missing:
                raise ValueError(f'ομάδα {name}: λείπουν οι αισθητήρες πίεσης {missing}')
            controller = GripperController(group, config['control'], config['pca'],
                                           {n: force_sensors[n] for n in names})
        else:
            raise ValueError(f"ομάδα {name}: άγνωστος controller '{kind}' (mix, gripper)")
        twice = seen_joints & set(controller.joints)
        shared = seen_channels & set(controller.channels)
        if twice or shared:
            raise ValueError(f'ομάδα {name}: αρθρώσεις {sorted(twice)} / κανάλια {sorted(shared)} '
                             f'ανήκουν ήδη σε άλλη ομάδα')
        seen_joints |= set(controller.joints)
        seen_channels |= set(controller.channels)
        controllers.append(controller)
    return controllers


def joint_configs(config: dict) -> dict:
    """{άρθρωση: ρύθμιση} από όλες τις ομάδες (για encoders και unwrapping)."""
    return {name: joint for group in config['groups'].values()
            for name, joint in group['joints'].items()}


def force_sensor_configs(config: dict) -> dict:
    """{αισθητήρας: ρύθμιση} από όλες τις ομάδες που έχουν `force_sensors`."""
    return {name: sensor for group in config['groups'].values()
            for name, sensor in group.get('force_sensors', {}).items()}


__all__ = ['BangBang', 'Controller', 'ControlLaw', 'GripperController', 'LAWS', 'LimitError',
           'MixController', 'PID', 'build_controllers', 'force_sensor_configs', 'joint_configs',
           'make_law']
