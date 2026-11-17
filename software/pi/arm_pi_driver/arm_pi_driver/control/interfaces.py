"""Τα συμβόλαια του ελέγχου, χωρίς ROS και χωρίς υλικό.

Τι ζητά ο Driver από μια ομάδα αρθρώσεων (Controller), και ο controller από έναν νόμο (ControlLaw).
"""

from abc import ABC, abstractmethod


class LimitError(RuntimeError):
    """Μια άρθρωση βγήκε από τα όρια της + το περιθώριο: στάση."""


class ControlLaw(ABC):
    """Νόμος ελέγχου μιας άρθρωσης: σφάλμα → ταχύτητα (PID σήμερα, άλλος αύριο).

    Η ανοχή, η ελάχιστη εντολή (νεκρή ζώνη) και το μέγιστο ΔΕΝ είναι δουλειά του νόμου: τα
    εφαρμόζει ο controller, ίδια για κάθε νόμο, ώστε μια σύγκριση νόμων να είναι δίκαιη.
    """

    @abstractmethod
    def update(self, error: float, measurement: float, dt: float) -> float:
        """Ταχύτητα −1..1 για αυτό το σφάλμα (μοίρες) και τη μέτρηση, μετά από dt δευτερόλεπτα."""

    @abstractmethod
    def reset(self) -> None:
        """Ξεχνά ό,τι θυμάται από τα προηγούμενα (νέος στόχος, στάση)."""


class Controller(ABC):
    """Μια ομάδα αρθρώσεων και τα μοτέρ της (διαφορικό, κατοπτρικά, εργαλείο…).

    Ο Driver έχει μια λίστα από αυτούς και δεν ξέρει ποιος είναι ποιος.
    """

    @property
    @abstractmethod
    def joints(self) -> tuple:
        """Οι αρθρώσεις της ομάδας."""

    @property
    @abstractmethod
    def channels(self) -> tuple:
        """Τα κανάλια PWM των μοτέρ της ομάδας."""

    @property
    @abstractmethod
    def targets(self) -> dict:
        """Οι στόχοι που ισχύουν (μοίρες)· όποια άρθρωση λείπει δεν έχει στόχο."""

    @abstractmethod
    def set_targets(self, targets: dict) -> dict:
        """Νέοι στόχοι (μοίρες), κομμένοι στα όρια· επιστρέφει όλους όσους ισχύουν."""

    @abstractmethod
    def clear_targets(self) -> None:
        """Ξεχνά όλους τους στόχους (στάση ασφαλείας)."""

    @abstractmethod
    def step(self, angles: dict, dt: float) -> dict:
        """Γωνίες (μοίρες) → {κανάλι: παλμός µs ή None}· LimitError εκτός ορίων."""

    @abstractmethod
    def hold(self) -> dict:
        """Οι παλμοί της στάσης: {κανάλι: νεκρό σημείο (φρένο) ή None (κανένας παλμός)}."""

    def diagnostics(self) -> list:
        """[(όνομα, επίπεδο info/warning/error, μήνυμα, {κλειδί: τιμή})]· προαιρετικό."""
        return []
