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
| `probe [--channels 0-15]` | ✅ | σύντομος παλμός σε κάθε κανάλι → ποιος encoder κινείται |
| `jog SERVO` ή `jog --channel N` [`--no-encoders`] | ✅ | χειροκίνητος έλεγχος με πληκτρολόγιο: **κράτα W/S** για κίνηση +/−, **A/D** πιο αργά/γρήγορα, SPACE στάση, Q έξοδος· ζωντανή ένδειξη encoders |
| `stop` | — | όλες οι έξοδοι off |

## Ασφάλεια (από `wrist_bench.json` → `safety`)
- `pulse_min_us` / `pulse_max_us` (στο `pca`): απόλυτα όρια παλμού από την προδιαγραφή του servo (500–2500 µs)· δεν ξεπερνιούνται ποτέ.
- `max_offset_us`: μέγιστη απόκλιση από το νεκρό σημείο (±900 µs = πλήρης ταχύτητα).
- `jog_start_offset_us` / `jog_step_us`: αρχική ταχύτητα και βήμα στο `jog` (300 / 90 µs ≈ 30% / 10%, όπως το παλιό εργαλείο).
- `max_duration_s`: μέγιστη διάρκεια κίνησης (3 s).
- `max_travel_deg`: διακοπή αν κάποιος encoder μετακινηθεί περισσότερο (30°).
- `jog_max_travel_deg`: στο `jog`, διακοπή αν κάποιος encoder απομακρυνθεί περισσότερο από την αρχική θέση (90°).
- Στο `jog` το servo κινείται **μόνο όσο φτάνουν συνεχώς W/S** (auto-repeat)· αλλιώς σταματά σε ~0.15 s.
- Κάθε κίνηση τελειώνει **πάντα** με όλες τις εξόδους off (και σε Ctrl+C ή σφάλμα).

## Tests (χωρίς υλικό)
```bash
python3 -m pytest software/tools/wrist_bench
```
