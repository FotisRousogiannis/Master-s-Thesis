"""GripperController: η αρπάγη (ένα servo, encoder θέσης, αισθητήρες πίεσης στα δάχτυλα).

Είναι ένας MixController με ένα επιπλέον κανόνα: όσο κλείνει, αν κάποιο δάχτυλο νιώσει πίεση
πάνω από το όριο επαφής (contact) ή αν ένας αισθητήρας δεν μετρά (fault), το κλείσιμο σταματά·
το άνοιγμα επιτρέπεται πάντα. Έτσι η αρπάγη κρατά το αντικείμενο χωρίς να το συνθλίβει.

Θέση (ADR-0020): 0° = πλήρως κλειστή, open_deg = πλήρως ανοιχτή (από το calibration.yaml).
"""

from arm_pi_driver.control.mix import MixController


class GripperController(MixController):
    """Αρπάγη: θέση με PID (όπως κάθε άρθρωση) + στάση του κλεισίματος στην επαφή."""

    def __init__(self, group: dict, control: dict, pca: dict, sensors: dict):
        super().__init__(group, control, pca)
        if len(self._joints) != 1:
            raise ValueError('η αρπάγη έχει μία άρθρωση')
        (self.joint,) = self.joints
        self.sensors = sensors                      # {όνομα: ForceSensor}
        self.contact_level = group.get('contact')   # None: χωρίς ανίχνευση επαφής (μόνο θέση)
        self.readings = {}                          # {όνομα: (τιμή, κατάσταση)}, τελευταίο step
        self.state = 'ελεύθερη'

    def closing(self, speed: float) -> bool:
        """Αν η ταχύτητα κινεί την αρπάγη προς το κλειστό (0°)."""
        joint = self._joints[self.joint]
        open_side = 1 if joint['upper_deg'] > 0 or joint['lower_deg'] >= 0 else -1
        return speed * open_side < 0

    def joint_speeds(self, angles: dict, dt: float) -> dict:
        """Όπως κάθε άρθρωση· το κλείσιμο μηδενίζεται σε επαφή ή βλάβη αισθητήρα."""
        speeds = super().joint_speeds(angles, dt)
        self.readings = {name: sensor.read() for name, sensor in self.sensors.items()}
        faults = [f'{n}: {s}' for n, (_, s) in self.readings.items() if s != 'OK']
        touching = self.contact_level is not None and any(
            v >= self.contact_level for v, s in self.readings.values() if s == 'OK')
        if faults:
            self.state = 'αισθητήρας: ' + ', '.join(faults)
        elif touching:
            self.state = 'επαφή'
        else:
            self.state = 'ελεύθερη'
        if (faults or touching) and self.closing(speeds[self.joint]):
            speeds[self.joint] = 0.0                # δεν κλείνει άλλο· ανοίγει κανονικά
        return speeds

    def diagnostics(self) -> list:
        """Μία κατάσταση ανά αισθητήρα πίεσης και μία για την αρπάγη (για /diagnostics)."""
        result = []
        for name, (value, status) in self.readings.items():
            level = 'info' if status == 'OK' else 'error'
            values = {} if value != value else {'value': f'{value:.3f}'}    # nan → τίποτα
            result.append((f'force/{name}', level, status, values))
        level = 'warning' if self.state.startswith('αισθητήρας') else 'info'
        contact = 'όχι' if self.contact_level is None else f'{self.contact_level:g}'
        result.append((f'gripper/{self.joint}', level, self.state, {'contact_level': contact}))
        return result
