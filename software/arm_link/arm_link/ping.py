"""Το μήνυμα Ping: αριθμός σειράς + χρόνος αποστολής, σε bytes για το δίκτυο."""

import struct
from dataclasses import dataclass
from typing import ClassVar

U32_MAX = 2**32 - 1
U64_MAX = 2**64 - 1


@dataclass(frozen=True)
class Ping:
    """Ένα μήνυμα Ping. Ο server το επιστρέφει όπως είναι, ο client μετρά πόσο άργησε."""

    FORMAT: ClassVar[str] = "!IQ"
    SIZE: ClassVar[int] = struct.calcsize(FORMAT)

    seq: int
    sent_ns: int

    def __post_init__(self):
        if not 0 <= self.seq <= U32_MAX:
            raise ValueError(f"seq={self.seq}: πρέπει να είναι αριθμός από 0 έως {U32_MAX}")
        if not 0 <= self.sent_ns <= U64_MAX:
            raise ValueError(f"sent_ns={self.sent_ns}: πρέπει να είναι από 0 έως {U64_MAX}")

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT, self.seq, self.sent_ns)

    @classmethod
    def from_bytes(cls, data: bytes) -> "Ping":
        if len(data) != cls.SIZE:
            raise ValueError(f"περίμενα {cls.SIZE} bytes, ήρθαν {len(data)}")
        seq, sent_ns = struct.unpack(cls.FORMAT, data)
        return cls(seq, sent_ns)
    