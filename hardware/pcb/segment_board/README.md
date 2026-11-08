# Segment board v1 (KiCad 9)

Κοινή πλακέτα τμήματος του βραχίονα — προδιαγραφή: [`SPEC.md`](SPEC.md), BOM/διαθεσιμότητα: [`bom/`](bom/).

## Άνοιγμα
Άνοιξε το `segment_board.kicad_pro` στο **KiCad 9** (τα αρχεία είναι σε μορφή KiCad 7 και αναβαθμίζονται στην αποθήκευση).
Το project library `segment_board` (`lib/segment_board.kicad_sym`) δηλώνεται στο `sym-lib-table`.

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
