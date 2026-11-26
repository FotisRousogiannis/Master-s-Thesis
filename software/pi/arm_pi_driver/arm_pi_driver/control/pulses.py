"""Ταχύτητα μοτέρ (−1..1) → παλμός servo (µs), με αντιστάθμιση της νεκρής ζώνης κάθε μοτέρ.

Από τον πάγκο (teleop.py): κάθε μοτέρ ξεκινά από την άκρη της δικής του νεκρής ζώνης (teleop
[10]), με κοινό εύρος για όλα· έτσι η ίδια εντολή δίνει περίπου ίδια ταχύτητα σε μοτέρ με
διαφορετική νεκρή ζώνη (σύζευξη βάσης 2.86° → 0.37°).
"""

from arm_pi_driver.control.laws import clamp


def compensation_span(motors: list, min_us: float, max_us: float, max_offset_us: float) -> float:
    """Το εύρος παλμού πάνω από την άκρη της νεκρής ζώνης που έχουν ΟΛΑ τα μοτέρ και οι φορές.

    motors = [(neutral_us, (down_us, up_us))].
    """
    spans = []
    for neutral, (down, up) in motors:
        spans.append(min(max_us, neutral + max_offset_us) - up)
        spans.append(down - max(min_us, neutral - max_offset_us))
    return max(0.0, min(spans))


def compensated_pulse(speed: float, neutral_us: float, edges, span_us: float) -> float:
    """Ταχύτητα −1..1 → παλμός: 0 → νεκρό σημείο (φρένο)· αλλιώς από την άκρη της νεκρής ζώνης."""
    if speed == 0:
        return neutral_us
    speed = clamp(speed, -1.0, 1.0)
    down, up = edges
    return up + speed * span_us if speed > 0 else down + speed * span_us


def plain_pulse(speed: float, neutral_us: float, min_us: float, max_us: float,
                max_offset_us: float) -> float:
    """Χωρίς αντιστάθμιση: −1..1 → νεκρό ± ίδιο εύρος προς τις δύο μεριές (το μικρότερο που
    χωρά σε min, max και στο όριο ασφαλείας). Με διαφορετικό εύρος ανά μεριά, δύο μοτέρ μιας
    άρθρωσης που γυρίζουν αντίθετα έπαιρναν διαφορετική ταχύτητα (joint_3, 2026-11-26)."""
    speed = clamp(speed, -1.0, 1.0)
    return neutral_us + speed * min(max_offset_us, max_us - neutral_us, neutral_us - min_us)
