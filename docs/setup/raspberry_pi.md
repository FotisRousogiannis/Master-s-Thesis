# Raspberry Pi: σύνδεση με τον υπολογιστή (ADR-0021)

> ⚠ **Το Pi του πάγκου έχει Raspberry Pi OS (Debian 13, NetworkManager), όχι Ubuntu Server** (ADR-0021, συμπλήρωμα).
> Το `software/pi/setup_network.sh` (βήμα 1) είναι για Ubuntu Server με systemd-networkd: **μην το τρέξεις** σε
> αυτό το Pi. Χωρίς αυτό, το Pi παίρνει ήδη διεύθυνση 10.42.0.x από τον υπολογιστή (DHCP) και τη βλέπεις με
> `sudo sh -c 'cat /var/lib/NetworkManager/dnsmasq-*.leases'`.

Το Pi είναι ο προσωρινός ελεγκτής του βραχίονα. Ο υπολογιστής του μιλά με **ROS 2** (έλεγχος) και **SSH**
(συντήρηση, πάνελ «Εργαλεία»), πάνω από **καλώδιο Ethernet** (κύρια σύνδεση) ή **WiFi**.

```
Υπολογιστής 10.42.0.1 ──── καλώδιο Ethernet ──── Pi 10.42.0.2 (armpi.local)
     │ WiFi (internet)                               │ I²C → TCA9548A → PCA9685, AS5600 ×7, ADS1115
```
Με το καλώδιο ο υπολογιστής **μοιράζει το internet του** στο Pi, σαν μικρός router. Αν αλλάξεις WiFi στον
υπολογιστή, το καλώδιο δεν επηρεάζεται.

## Κάθε μέρα: γρήγορα βήματα (πάγκος με καλώδιο)
Όλα στον **υπολογιστή**, σε τερματικό του Ubuntu (όχι στο Docker), εκτός αν γράφει «στο Pi».

1. Καλώδιο Ethernet συνδεδεμένο, Pi αναμμένο. Ενεργοποίηση της σύνδεσης προς το Pi:
   ```bash
   nmcli connection up arm-pi-direct
   ```
2. Ποια διεύθυνση πήρε το Pi (συνήθως `10.42.0.116`, η γραμμή με `e4:5f:01:e0:f2:1f`):
   ```bash
   sudo sh -c 'cat /var/lib/NetworkManager/dnsmasq-*.leases'
   ```
3. Μπες στο Pi (χωρίς κωδικό, με το κλειδί του `.local/ssh/`· αν ζητήσει, ο κωδικός του `fotis`):
   ```bash
   ssh fotis@10.42.0.116
   ```
4. **Στο Pi:** ενημέρωση του repo:
   ```bash
   cd ~/Master-s-Thesis && git pull --ff-only
   ```
5. **Στο Pi:** δοκιμές του πάγκου (`software/tools/wrist_bench/README.md`):
   ```bash
   python3 software/tools/wrist_bench/wrist_bench.py scan
   ```
   ```bash
   python3 software/tools/wrist_bench/teleop.py
   ```
   (μενού: σάρωση, μαγνήτες, επιλογή καναλιών, κίνηση διαφορικού)
   και πάντα στο τέλος:
   ```bash
   python3 software/tools/wrist_bench/wrist_bench.py stop
   ```
6. Έξοδος από το Pi: `exit`.

Αν το `ssh` δεν συνδέεται: «Αν δεν συνδέεται» παρακάτω (συνήθως βοηθά επανεκκίνηση του Pi ή ξανά το καλώδιο στο Pi).
Από το Arm Studio: πάνελ «Σύνδεση» (Host = η διεύθυνση του βήματος 2) → «Εύρεση και έλεγχος»· πάνελ «Εργαλεία».

## Βήμα 1 — Στο Pi (μία φορά, με οθόνη και πληκτρολόγιο, ή με SSH από το τωρινό σου δίκτυο)
```bash
cat /etc/os-release | head -2
```
Σημείωσε την έκδοση· το ROS 2 Jazzy θέλει Ubuntu 24.04.

Λήψη του repo (αν δεν υπάρχει) και ρύθμιση δικτύου:
```bash
git clone https://github.com/FotisRousogiannis/Master-s-Thesis.git && cd Master-s-Thesis && git checkout draft
```
```bash
sudo software/pi/setup_network.sh base
```
Το `base` κάνει τρία πράγματα:
1. δίνει στο Pi το όνομα `armpi`·
2. εγκαθιστά το mDNS (`avahi-daemon`), ώστε να βρίσκεται ως `armpi.local`·
3. ορίζει στη θύρα Ethernet DHCP **και** τη σταθερή 10.42.0.2.

Στο τέλος ρωτά: **πάτα ENTER αν η σύνδεση δουλεύει ακόμα**. Αν δεν πατήσεις, όλα επανέρχονται σε 120 s.

## Βήμα 2 — Στον υπολογιστή (μία φορά, στο τερματικό του Ubuntu, όχι στο Docker)
```bash
software/launcher/pi_link.sh setup-direct
```
Η θύρα Ethernet γίνεται η σύνδεση «arm-pi-direct» (10.42.0.1, μοιρασμένο internet). Σύνδεσε το καλώδιο και:
```bash
software/launcher/pi_link.sh find
```
Αναμενόμενο: `✔ Pi στο καλώδιο: 10.42.0.2`.

## Βήμα 3 — Στο Arm Studio, πάνελ «Σύνδεση»
Την πρώτη φορά `docker compose build` (το image θέλει `openssh-client`).
1. Ο host είναι ήδη `10.42.0.2` (καλώδιο). Συμπλήρωσε τον **χρήστη** του Pi.
2. **«Ρύθμιση κλειδιού SSH…»**: ζητά μία φορά τον κωδικό του χρήστη στο Pi. Ο κωδικός δεν αποθηκεύεται· μετά
   δεν ξαναζητείται.
3. **«Εύρεση και έλεγχος»**: ελέγχει με τη σειρά καλώδιο → Pi → SSH → ROS 2. Αν το Pi δεν είναι στο καλώδιο, το
   ψάχνει ως `armpi.local` στο δίκτυο και γράφει μόνο του τη διεύθυνσή του.

Μετά, στο πάνελ «Εργαλεία»: «Σάρωση I²C» → «Εκτέλεση».

Από τερματικό γίνονται τα ίδια με `software/launcher/pi_link.sh find` και `pi_link.sh key <χρήστης>`.

## WiFi (εναλλακτικά)
Πρόσθεση δικτύου στο Pi (καλύτερα **από το καλώδιο**, ώστε να μη χαθεί η σύνδεση αν κάτι πάει στραβά):
```bash
sudo software/pi/setup_network.sh add-wifi "Όνομα-WiFi"
```
Τα γνωστά δίκτυα μένουν· το Pi μπαίνει σε όποιο βρει. Στο ίδιο WiFi με τον υπολογιστή, το «Εύρεση και έλεγχος»
το βρίσκει ως `armpi.local` (mDNS, όπως οι 3D printers) και κρατά τη διεύθυνσή του.

## Αν δεν συνδέεται
| Σύμπτωμα | Τι ελέγχεις |
|---|---|
| «Καλώδιο: όχι» | καλώδιο συνδεδεμένο; `pi_link.sh setup-direct` έγινε; |
| `find`: δεν βρέθηκε, με καλώδιο | `pi_link.sh setup-direct` έγινε; `setup_network.sh base` στο Pi; λαμπάκια στη θύρα; |
| `find`: δεν βρέθηκε, με WiFi | ίδιο δίκτυο; στο Pi: `setup_network.sh show` |
| `Permission denied (publickey…)` | ξανά «Ρύθμιση κλειδιού SSH…» (ή `pi_link.sh key <χρήστης>`) |
| `Host key verification failed` | το Pi ξαναστήθηκε: σβήσε τη γραμμή του από το `.local/ssh/known_hosts` |

## Ασφάλεια
- Τα εργαλεία που κινούν servo (⚠) ζητούν επιβεβαίωση και έχουν πάντα περιορισμένη διάρκεια.
- Η «Διακοπή» κλείνει τη σύνδεση SSH, αλλά **δεν** εγγυάται ότι σταματά αμέσως το πρόγραμμα στο Pi. Γι' αυτό
  υπάρχει το «Όλες οι έξοδοι off», και το E-stop της τροφοδοσίας.
