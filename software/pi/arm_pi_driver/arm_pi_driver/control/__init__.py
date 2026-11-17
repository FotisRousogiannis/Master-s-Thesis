"""Ο έλεγχος του βραχίονα, χωρίς ROS και χωρίς υλικό.

interfaces.py: Controller (ομάδα αρθρώσεων), ControlLaw (νόμος ελέγχου), LimitError.
laws.py: PID, BangBang· LAWS = όνομα στο YAML → κλάση.
pulses.py: ταχύτητα → παλμός, με αντιστάθμιση νεκρής ζώνης ανά μοτέρ.
mix.py: MixController (διαφορικό, κατοπτρικά, ένα servo· μίξη από το YAML).
"""

from arm_pi_driver.control.interfaces import ControlLaw, Controller, LimitError
from arm_pi_driver.control.laws import BangBang, LAWS, make_law, PID
from arm_pi_driver.control.mix import MixController


def build_controllers(config: dict) -> list:
    """Ένας controller ανά ομάδα του YAML (groups)· κάθε άρθρωση σε μία μόνο ομάδα."""
    controllers, seen_joints, seen_channels = [], set(), set()
    for name, group in config['groups'].items():
        kind = group.get('controller', 'mix')
        if kind != 'mix':
            raise ValueError(f"ομάδα {name}: άγνωστος controller '{kind}'")
        controller = MixController(group, config['control'], config['pca'])
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


__all__ = ['BangBang', 'Controller', 'ControlLaw', 'LAWS', 'LimitError', 'MixController', 'PID',
           'build_controllers', 'joint_configs', 'make_law']
