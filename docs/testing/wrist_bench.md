# Πρωτόκολλο δοκιμών — καρπός (bench, Raspberry Pi)

Εργαλείο: `software/tools/wrist_bench/` · Αποφάσεις: ADR-0012.

## Διάταξη υπό δοκιμή
- **Raspberry Pi** (master I²C) → **PCA9685** (servo) και **TCA9548A** (mux encoders).
- **Πάνω μέρος καρπού:** 2 servo, 1 AS5600.
- **Κάτω μέρος (διαφορικό):** 2 servo, 2 AS5600.
- Servo QY3240MG, **συνεχούς περιστροφής**: ο παλμός ορίζει ταχύτητα, όχι θέση.

## Κανόνες ασφαλείας
1. Πριν από κάθε test με κίνηση: τα χέρια μακριά, καλώδια ελεύθερα, εύκολη
   πρόσβαση στον διακόπτη τροφοδοσίας των servo.
2. Ξεκινάμε πάντα με **μικρές αποκλίσεις** (±20–40 µs) και **σύντομη διάρκεια** (≤1 s).
3. Αν κάτι κινείται απρόβλεπτα: **Ctrl+C** και κλείσιμο τροφοδοσίας servo.
4. Δεν αυξάνουμε τα όρια ασφαλείας στο `wrist_bench.json` πριν περάσει το T4.

## Προετοιμασία του Pi (μία φορά)
```bash
cat /etc/os-release | head -2      # λειτουργικό
ls /dev/i2c-*                      # πρέπει να υπάρχει /dev/i2c-1
sudo apt install -y i2c-tools python3-smbus2 python3-pytest git
git clone https://github.com/FotisRousogiannis/Master-s-Thesis.git
cd Master-s-Thesis && git checkout draft
python3 -m pytest software/tools/wrist_bench     # 11 passed
```
Αν λείπει το `/dev/i2c-1`: Raspberry Pi OS → `sudo raspi-config` → Interface → I2C·
Ubuntu → `dtparam=i2c_arm=on` στο `/boot/firmware/config.txt` και επανεκκίνηση.

Όλες οι εντολές παρακάτω τρέχουν από `software/tools/wrist_bench/`.

---

## T0 — Τροφοδοσία (χωρίς κίνηση)
**Στόχος:** σωστές τάσεις και κοινή γείωση πριν από οποιαδήποτε εντολή.

| Μέτρηση (πολύμετρο) | Αναμενόμενο | Μετρήθηκε |
|---|---|---|
| Τάση servo (V+ PCA9685) | 5.0–7.4 V | |
| VCC λογικής PCA9685 / TCA9548A / AS5600 | 3.3 V | |
| GND Pi ↔ GND τροφοδοτικού servo | συνέχεια (0 Ω) | |
| Τροφοδοτικό servo: μέγιστο ρεύμα | ≥ 4 × ~2 A (stall) | |

**Κριτήριο:** όλα εντός ορίων.

## T1 — Σάρωση I²C (χωρίς κίνηση)
```bash
i2cdetect -y 1
python3 wrist_bench.py scan
```
**Αναμενόμενο:** `root: 0x40` (PCA9685) — ο mux (0x70) εξαιρείται — και `0x36` σε
τρία κανάλια του mux. **Καταγραφή:** σε ποια κανάλια βρέθηκαν οι AS5600 → `encoders` στο JSON.

## T2 — Encoders με το χέρι (χωρίς κίνηση, servo χωρίς τροφοδοσία)
```bash
python3 wrist_bench.py enc --csv t2_encoders.csv
```
Γυρνάμε **με το χέρι** κάθε άξονα αργά, μία φορά προς κάθε κατεύθυνση.

**Ελέγχουμε:** κατάσταση μαγνήτη `OK` σε όλους (όχι `NO_MAGNET`/`TOO_WEAK`/`TOO_STRONG`),
συνεχής μεταβολή γωνίας χωρίς άλματα, θόρυβος σε ακινησία < ~0.2°, ποιος encoder
αντιστοιχεί σε ποιον άξονα, φορά (θετική προς τα πού).

## T3α — Εύρεση καναλιών servo (`probe`)
```bash
python3 wrist_bench.py probe --channels 0-15 --offset 30 --duration 0.5
```
Κάθε κανάλι του PCA9685 παίρνει διαδοχικά παλμό `default_neutral_us + offset` για
`duration` s· τυπώνεται η μετακίνηση κάθε encoder ανά κανάλι. Για servo χωρίς encoder
(π.χ. αρπάγη) η κίνηση παρατηρείται οπτικά. Αν κανένα κανάλι δεν κινεί τίποτα, το
`offset` ίσως είναι πολύ μικρό για να υπερνικηθεί η τριβή του μειωτήρα → επανάληψη με `--offset 100`.

**Καταγραφή:** κανάλι → servo → encoder(s) που κινήθηκαν, με πρόσημο → `servos` στο JSON.

## T3 — Νεκρό σημείο κάθε servo
Για κάθε servo, πρώτα σύντομα παλμικά βήματα για να δούμε ότι κινείται το σωστό:
```bash
python3 wrist_bench.py pwm diff_a 30 --duration 0.5
python3 wrist_bench.py pwm diff_a -30 --duration 0.5
```
Αν κινήθηκε το λάθος servo → διόρθωση `channel` στο JSON. Μετά η σάρωση νεκρής ζώνης
(απαιτεί encoder που κινείται με αυτό το servo):
```bash
python3 wrist_bench.py deadband diff_a diff_1 --span 60 --step 5 --csv t3_deadband_diff_a.csv
```
**Καταγραφή:** εύρος παλμών με ταχύτητα ≈ 0 → το κέντρο του γίνεται `neutral_us`.

## T4 — Αντιστοίχιση κινητήρα → encoder και πρόσημα
```bash
python3 wrist_bench.py pwm <servo> 30 --duration 1 --csv t4_<servo>.csv
```
**Καταγραφή:** ποιοι encoders κινήθηκαν, πόσο, με ποιο πρόσημο (θετική απόκλιση → ±).

## T5 — Διαφορικό χωρίς κλειστό βρόχο
```bash
python3 wrist_bench.py diff 30 0 --duration 1 --csv t5_pitch.csv   # καθαρό pitch
python3 wrist_bench.py diff 0 30 --duration 1 --csv t5_roll.csv    # καθαρό roll
```
**Αναμενόμενο:** στο καθαρό pitch κινείται μόνο η κίνηση pitch (και το roll μένει ≈ σταθερό), και αντίστροφα.
Αν το «pitch» δίνει roll → αλλαγή `b_sign` στο JSON.

---

## Αποτελέσματα
_(συμπληρώνονται ανά test με ημερομηνία, ρυθμίσεις, μετρήσεις και CSV στο `docs/testing/data/`)_

### T1 — 2026-11-01, πρώτη σάρωση (αυτόνομο script σάρωσης)
```
main bus       0x70 (TCA9548A / PCA9685 all-call)
mux channel 0  0x40 (PCA9685)
mux channel 1-7  -
```
- Ο **PCA9685 είναι πίσω από το κανάλι 0 του mux**, όχι στο κύριο bus → `pca.mux_channel = 0` στο JSON.
- Κανένας AS5600 στην πρώτη σάρωση· μετά από έλεγχο συνδέσεων εντοπίστηκαν όλοι (βλ. παρακάτω).
- Σημείωση: με ανοιχτό το κανάλι 0, ο PCA9685 ακούει και στη διεύθυνση all-call 0x70
  (ίδια με του TCA9548A) μέχρι να αρχικοποιηθεί· ο οδηγός μας την απενεργοποιεί (MODE1 = AI).

### T1 — 2026-11-01, σάρωση με `wrist_bench.py scan`
```
root  : -
mux0  : 0x40
mux1  : 0x36
mux6  : 0x36
mux7  : 0x36
```
✅ PCA9685 στο κανάλι 0, **τρεις AS5600 στα κανάλια 1, 6, 7**. Ονομάζονται προσωρινά
`enc_ch1`, `enc_ch6`, `enc_ch7` μέχρι το T2 να δείξει ποιος άξονας είναι ο καθένας.

### Παράμετροι servo — 2026-11-01
QY3240MG: νεκρό σημείο **1600 µs**, εύρος **500–2500 µs** (≈ ±900 µs για μέγιστη ταχύτητα).
Με τα αρχικά όρια (±30 µs στο jog/probe, ≈ 3% της ταχύτητας) δεν κινήθηκε κανένα servo —
πιθανώς ανεπαρκής ροπή εκκίνησης μέσω του μειωτήρα 1:36. Νέα όρια: απόλυτο 500–2500 µs,
απόκλιση έως ±400 µs, jog από 100 µs με βήμα 20 µs.
