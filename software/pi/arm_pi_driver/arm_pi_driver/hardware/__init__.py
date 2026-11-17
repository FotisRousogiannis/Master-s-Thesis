"""Το υλικό του βραχίονα στο I²C, χωρίς ROS.

Ένα αρχείο ανά chip· τα interfaces (συμβόλαια) στο interfaces.py, οι ψεύτικες υλοποιήσεις
για tests στο fake.py. Όποιος το χρησιμοποιεί γράφει απλώς:

    from arm_pi_driver.hardware import AS5600, Encoder, TCA9548A

Κλάσεις υλοποίησης με το όνομα του chip (AS5600, TCA9548A)· interfaces με το όνομα του ρόλου
(Encoder).
"""

from arm_pi_driver.hardware.as5600 import AS5600
from arm_pi_driver.hardware.interfaces import Encoder, NO_RESPONSE
from arm_pi_driver.hardware.tca9548a import TCA9548A

__all__ = ['AS5600', 'Encoder', 'NO_RESPONSE', 'TCA9548A']
