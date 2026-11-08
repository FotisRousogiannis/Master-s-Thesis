# Segment board v1 (KiCad 10)

Κοινή πλακέτα τμήματος του βραχίονα — προδιαγραφή: [`SPEC.md`](SPEC.md), BOM/διαθεσιμότητα: [`bom/`](bom/).

## Άνοιγμα
Άνοιξε το `segment_board.kicad_pro` στο **KiCad 10** (το PCB είναι αποθηκευμένο σε μορφή KiCad 10· το σχηματικό παράγεται σε μορφή KiCad 7). **Κλείσε το KiCad πριν από `git pull`**, αλλιώς το KiCad ξαναγράφει τις παλιές ρυθμίσεις του project πάνω στις νέες.
Τα project libraries `segment_board` δηλώνονται στο `sym-lib-table` (σύμβολα, `lib/segment_board.kicad_sym`) και στο
`fp-lib-table` (footprints, `lib/segment_board.pretty`).

## Το σχηματικό παράγεται από κώδικα
Όλο το κύκλωμα ορίζεται στο `gen/build.py`. **Μην** αλλάζεις συνδέσεις στο KiCad — άλλαξε το `build.py` και ξαναπαράγε:
```bash
cd gen && python3 build.py
kicad-cli sch export netlist -o /tmp/board.net ../segment_board.kicad_sch && python3 check_netlist.py /tmp/board.net
```
Ελεύθερα στο KiCad: μετακίνηση/τακτοποίηση συμβόλων για αναγνωσιμότητα (θα χαθεί αν ξαναπαραχθεί), layout PCB.

| Φύλλο | Περιεχόμενο |
|---|---|
| `power` | XT30 7.4 V, ασφάλεια, προστασία πολικότητας, TVS, eFuse TPS259827, INA219 · MSTB 5.1 V, eFuse TPS259472A · ιδανικές δίοδοι, buck 3.3 V · LED τροφοδοσίας |
| `mcu` | ESP32-S3-WROOM-1, USB-C + ESD, RESET/BOOT, LED κατάστασης, header GPIO |
| `i2c` | jumpers επιλογής master, PCA9685, TCA9548A, ADS1115, pull-ups, header επέκτασης |
| `connectors` | 9 servo, 6 encoders, 2 αφής (MCV 3.5 mm), ανίχνευση σύνδεσης με LM339 + 17 LED |
| `pi_link` | header Raspberry Pi, UART/EN/BOOT Pi↔ESP32, RS-485 THVD1451 + LINK, οπές στερέωσης |

## Footprints του project (`gen/footprints.py` → `lib/segment_board.pretty`)
| Footprint | Εξάρτημα | Πηγή |
|---|---|---|
| `TI_RGE0024_2EP` | TPS259827 (VQFN-24 4×4, 2 thermal pads, θερμικά vias 0.2 mm) | TI RGE0024M land pattern (tps25982.pdf σ. 55–57) |
| `TI_RPW0010A` | TPS259472A (VQFN-HR-10 2×2, γωνιακά pads σε σχήμα L) | TI RPW0010A land pattern (tps25947.pdf σ. 72–74) |
| `L_4x4mm_SMNR4020` | πηνίο buck 2.2 µH 3.4 A (LCSC C135262) | land pattern LCSC |
| `RPi_Socket_2x20_FaceDown` | header Pi με **κατοπτρικές στήλες** (Pi ανάποδα) + περίγραμμα Pi 4/5 και οπές M2.5 στο User.Drawings | ίδιο με το `Raspberry_Pi_Zero_Socketed_THT_FaceDown` του KiCad |

## PCB (`segment_board.kicad_pcb`)
Δημιουργήθηκε με `python3 gen/pcb.py --force` (με **αυτόματη αρχική τοποθέτηση**: κάθε chip μαζί με τα μικρά εξαρτήματα που μοιράζονται σήμα μαζί του, ανά περιοχή) (δεν ξανατρέχει χωρίς `--force`, για να μη χαθεί δουλειά). Από εκεί
και πέρα το layout γίνεται στο KiCad. Αλλαγές του σχηματικού περνούν με **Tools → Update PCB from Schematic (F8)**.

- 100 × 120 mm, 4 στρώσεις: F.Cu σήματα · In1 **GND** (zone) · In2 τροφοδοσίες · B.Cu σήματα
- Κανόνες JLCPCB και net classes: `python3 gen/rules.py` (ασφαλές να ξανατρέξει)

  | Net class | Δίκτυα | Πλάτος | Γιατί |
  |---|---|---|---|
  | Power_Servo | `VS_*`, `VSERVO` | 2.5 mm (+ πολύγωνα) | έως ~10 A |
  | Servo_Ch | `SERVO*_GND` | 1.0 mm | επιστροφή servo, έως ~4 A στο μπλοκάρισμα |
  | Power_Logic | 5 V, 3.3 V, GND | 0.6 mm | |
  | Default | σήματα | 0.2 mm | |
- **Πρόταση τοποθέτησης** (μετακινείται ελεύθερα): connectors αριστερά σε 4 στήλες (έξω από το Pi, τα φις μπαίνουν
  από πάνω), Pi ανάποδα στο δεξί μισό με τις θύρες του στην κάτω άκρη, κεραία ESP32 στην πάνω άκρη (έξω από το Pi,
  με το keep-out του footprint), XT30 και MSTB στην πάνω άκρη, USB-C στη δεξιά άκρη. Τα υπόλοιπα εξαρτήματα είναι
  δίπλα στην πλακέτα, ομαδοποιημένα ανά φύλλο του σχηματικού.
- Κάτω από το Pi (≥ 18 mm ελεύθερο ύψος): μόνο SMD.
- Το CI (KiCad 10) τρέχει **DRC** με έλεγχο ισοτιμίας με το σχηματικό και βγάζει εικόνες της πλακέτας.

## Υλικά (`bom/`)
- `parts.csv`: κύρια εξαρτήματα · `passives.csv`: αντιστάσεις, πυκνωτές, LED κ.λπ. — επιλογή με `resolve_passives.py`
  (προτίμηση σε **basic** parts της JLC, χωρίς χρέωση setup). Το `build.py` βάζει τους κωδικούς LCSC στο σχηματικό.
- Χωρίς LCSC (κόλληση με το χέρι): connectors MCV/MSTB/XT30, header Pi, pin headers/jumpers, θήκη ασφάλειας.
