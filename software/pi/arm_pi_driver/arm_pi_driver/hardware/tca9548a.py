"""TCA9548A: πολυπλέκτης I²C 8 καναλιών (διεύθυνση 0x70 έως 0x77)."""


class Mux:
    """Συνδέει ένα από τα 8 κανάλια του TCA9548A με τον κύριο δίαυλο."""

    def __init__(self, bus, address: int = 0x70):
        self.bus = bus
        self.address = address
        self.current = None

    def select(self, channel: int) -> None:
        """Ανοίγει το κανάλι `channel` (0–7)· αν είναι ήδη ανοιχτό, δεν στέλνει τίποτα."""
        if not 0 <= channel <= 7:
            raise ValueError(f'κανάλι {channel}: το TCA9548A έχει κανάλια 0–7')
        if channel == self.current:
            return
        self.bus.write_byte(self.address, 1 << channel)
        self.current = channel
