"""TCA9548A: πολυπλέκτης I²C 8 καναλιών (διεύθυνση 0x70 έως 0x77)."""


class TCA9548A:
    """Πολυπλέκτης (mux): συνδέει ένα από τα 8 κανάλια του με τον κύριο δίαυλο."""

    def __init__(self, bus, address: int = 0x70):
        self.bus = bus
        self.address = address
        self.current = None

    def select(self, channel: int) -> None:
        """Ανοίγει το κανάλι `channel` (0–7)· αν είναι ήδη ανοιχτό, δεν στέλνει τίποτα.

        Αν το TCA9548A δεν απαντήσει, σκάει με OSError (όπως το smbus2).
        """
        if not 0 <= channel <= 7:
            raise ValueError(f'κανάλι {channel}: το TCA9548A έχει κανάλια 0–7')
        if channel == self.current:
            return
        try:
            self.bus.write_byte(self.address, 1 << channel)
        except OSError:
            self.current = None     # άγνωστο τι είναι ανοιχτό τώρα: την επόμενη φορά ξαναστέλνουμε
            raise
        self.current = channel

    def probe(self, channel: int, addresses) -> list[int]:
        """Ποιες από τις `addresses` απαντούν (ACK) στο κανάλι `channel`, όπως το i2cdetect.

        OSError αν δεν απαντά ο ίδιος ο TCA9548A.
        """
        self.select(channel)
        found = []
        for address in addresses:
            try:
                self.bus.read_byte(address)
            except OSError:
                continue
            found.append(address)
        return found
