# Μετρήσεις σύνδεσης PC ↔ Pi

Εργαλείο: πακέτο `software/arm_link` (γραμμένο από τον Φώτη, branch `feature/pi-communication`).
`EchoServer` στο Pi (θύρα UDP 9999) επιστρέφει κάθε πακέτο· ο `LinkProbe` στο PC στέλνει μηνύματα `Ping`
(12 bytes: seq + χρόνος αποστολής, `time.monotonic_ns`) ένα-ένα και μετρά τον χρόνο πήγαινε-έλα (RTT).
Jitter = μέσος όρος των απόλυτων διαφορών διαδοχικών RTT.

Πώς τρέχει:

    # Pi
    cd ~/master-s-thesis/software/arm_link && python3 -m arm_link.server
    # PC
    cd ~/master-s-thesis/software/arm_link
    python3 -c "from arm_link.probe import LinkProbe; from arm_link.transport import UdpTransport; \
      t = UdpTransport(peer=('10.42.0.116', 9999)); s = LinkProbe(t).run(100, 0.5); \
      print(s.loss_percent, s.avg_ms, s.max_ms, s.jitter_ms)"

## Αποτελέσματα

| Ημερομηνία | Σύνδεση | Πακέτα | Απώλειες | RTT μέσος | RTT max | Jitter |
|---|---|---|---|---|---|---|
| 2026-11-16 | localhost (PC → PC, έλεγχος κώδικα) | 100 | 0 % | 0.046 ms | 0.136 ms | 0.009 ms |
| 2026-11-16 | **Ethernet απευθείας** (PC 10.42.0.1 ↔ Pi 10.42.0.116, NetworkManager «shared») | 100 | **0 %** | **0.174 ms** | **0.409 ms** | **0.024 ms** |
| — | WiFi | — | — | — | — | — (δεν έχει μετρηθεί) |

Ερμηνεία: σε βρόχο ελέγχου 100 Hz (10 ms ανά κύκλο) η χειρότερη καθυστέρηση του Ethernet είναι < 5 % του κύκλου·
η σύνδεση με καλώδιο δεν περιορίζει τον έλεγχο. Ένα δείγμα των 100 πακέτων, χωρίς φόρτο στο δίκτυο· για
συμπεράσματα για ακραίες τιμές χρειάζονται μεγαλύτερες σειρές (π.χ. 10 000) και μέτρηση με φόρτο.
