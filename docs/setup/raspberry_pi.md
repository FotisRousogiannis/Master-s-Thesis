# Raspberry Pi: σύνδεση με τον υπολογιστή (ADR-0021)

Το Pi είναι ο προσωρινός ελεγκτής του βραχίονα. Ο υπολογιστής του μιλά με **ROS 2** (έλεγχος) και **SSH**
(συντήρηση, πάνελ «Εργαλεία»), πάνω από **καλώδιο Ethernet** (κύρια σύνδεση) ή **WiFi**.

```
Υπολογιστής 10.42.0.1 ──── καλώδιο Ethernet ──── Pi 10.42.0.2 (armpi.local)
     │ WiFi (internet)                               │ I²C → TCA9548A → PCA9685, AS5600 ×7, ADS1115
```
Με το καλώδιο ο υπολογιστής **μοιράζει το internet του** στο Pi, σαν μικρός router. Αν αλλάξεις WiFi στον
υπολογιστή, το καλώδιο δεν επηρεάζεται.

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

Κλειδί SSH για το Arm Studio· ζητά **μία** φορά τον κωδικό του χρήστη στο Pi:
```bash
software/launcher/pi_link.sh key <χρήστης-του-pi>
```

## Βήμα 3 — Στο Arm Studio
Πάνελ «Εργαλεία» → «Raspberry Pi (SSH)»: ο host είναι ήδη `10.42.0.2`· συμπλήρωσε τον χρήστη. Μετά:
«Σάρωση I²C» → «Εκτέλεση». Το image θέλει `openssh-client`, άρα την πρώτη φορά `docker compose build`.

## WiFi (εναλλακτικά)
Πρόσθεση δικτύου στο Pi (καλύτερα **από το καλώδιο**, ώστε να μη χαθεί η σύνδεση αν κάτι πάει στραβά):
```bash
sudo software/pi/setup_network.sh add-wifi "Όνομα-WiFi"
```
Τα γνωστά δίκτυα μένουν· το Pi μπαίνει σε όποιο βρει. Στο ίδιο WiFi με τον υπολογιστή το `pi_link.sh find`
το βρίσκει ως `armpi.local` και γράφει τη διεύθυνσή του· αυτή μπαίνει στο Arm Studio, γιατί μέσα στο Docker τα
ονόματα `.local` δεν λύνονται.

## Αν δεν συνδέεται
| Σύμπτωμα | Τι ελέγχεις |
|---|---|
| `find`: δεν βρέθηκε, με καλώδιο | `pi_link.sh setup-direct` έγινε; `setup_network.sh base` στο Pi; λαμπάκια στη θύρα; |
| `find`: δεν βρέθηκε, με WiFi | ίδιο δίκτυο; στο Pi: `setup_network.sh show` |
| `Permission denied (publickey…)` | ξανά `pi_link.sh key <χρήστης>` |
| `Host key verification failed` | το Pi ξαναστήθηκε: σβήσε τη γραμμή του από το `.local/ssh/known_hosts` |

## Ασφάλεια
- Τα εργαλεία που κινούν servo (⚠) ζητούν επιβεβαίωση και έχουν πάντα περιορισμένη διάρκεια.
- Η «Διακοπή» κλείνει τη σύνδεση SSH, αλλά **δεν** εγγυάται ότι σταματά αμέσως το πρόγραμμα στο Pi. Γι' αυτό
  υπάρχει το «Όλες οι έξοδοι off», και το E-stop της τροφοδοσίας.
