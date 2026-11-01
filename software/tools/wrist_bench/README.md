# wrist_bench — πάγκος δοκιμών του καρπού (Raspberry Pi)

Εργαλείο Python για τις δοκιμές T0–T5 του καρπού χωρίς ROS/ESP32 (ADR-0012).
Πρωτόκολλο και αποτελέσματα: [`docs/testing/wrist_bench.md`](../../../docs/testing/wrist_bench.md).

```
Raspberry Pi ── I²C ──┬── PCA9685 (0x40) ── servo ×4
                      └── TCA9548A (0x70) ── AS5600 ×3 (0x36, ένα ανά κανάλι)
```

## Εντολές
| Εντολή | Κίνηση | Περιγραφή |
|---|---|---|
| `scan` | ❌ | συσκευές I²C στο κύριο bus και σε κάθε κανάλι του mux |
| `enc [--duration S] [--csv F]` | ❌ | ροή γωνιών και κατάστασης μαγνήτη των encoders |
| `pwm SERVO OFFSET_US [--duration S]` | ✅ | ένα servo με απόκλιση από το νεκρό σημείο |
| `diff PITCH_US ROLL_US [--duration S]` | ✅ | διαφορικό: A = p + r, B = sign·(p − r) |
| `deadband SERVO ENCODER` | ✅ | σάρωση παλμών, μέτρηση ταχύτητας → νεκρή ζώνη |
| `stop` | — | όλες οι έξοδοι off |

## Ασφάλεια (από `wrist_bench.json` → `safety`)
- `max_offset_us`: μέγιστη απόκλιση από το νεκρό σημείο (προεπιλογή ±150 µs).
- `max_duration_s`: μέγιστη διάρκεια κίνησης (3 s).
- `max_travel_deg`: διακοπή αν κάποιος encoder μετακινηθεί περισσότερο (30°).
- Κάθε κίνηση τελειώνει **πάντα** με όλες τις εξόδους off (και σε Ctrl+C ή σφάλμα).

## Tests (χωρίς υλικό)
```bash
python3 -m pytest software/tools/wrist_bench
```
