# Raspberry Pi από το Arm Studio (SSH)

Το πάνελ **«Εργαλεία»** του Arm Studio τρέχει εντολές του πάγκου (σάρωση I²C, encoders, έξοδοι off…)
**στο Raspberry Pi**, όπου είναι το I²C. Η σύνδεση γίνεται με **SSH μέσα από το δίκτυο**: WiFi (ίδιο router
με τον υπολογιστή) ή καλώδιο Ethernet. Τα κλειδιά SSH μένουν στο `.local/ssh/` του repo, εκτός git.

## 1. Στο Pi (μία φορά)
- Η προετοιμασία του `docs/testing/wrist_bench.md` (I²C ενεργό, `python3-smbus2`, clone του repo στο home).
- SSH ενεργό: Raspberry Pi OS → `sudo raspi-config` → Interface → SSH.
- Η διεύθυνσή του: `hostname -I` (π.χ. `192.168.1.20`), ή το όνομα `<hostname>.local`.

## 2. Στον υπολογιστή (μία φορά, από τη ρίζα του repo)
Κλειδί μόνο για αυτή τη σύνδεση, χωρίς κωδικό, ώστε το Arm Studio να συνδέεται χωρίς ερώτηση:
```bash
mkdir -p .local/ssh && ssh-keygen -t ed25519 -N "" -f .local/ssh/id_ed25519
```
Αντιγραφή του δημόσιου κλειδιού στο Pi (ζητά μία φορά τον κωδικό του χρήστη στο Pi):
```bash
ssh-copy-id -i .local/ssh/id_ed25519.pub <χρήστης>@<host>
```
Έλεγχος (πρέπει να τυπώσει τις συσκευές I²C χωρίς να ζητήσει κωδικό):
```bash
ssh -i .local/ssh/id_ed25519 <χρήστης>@<host> "cd ~/Master-s-Thesis && python3 software/tools/wrist_bench/wrist_bench.py scan"
```

## 3. Στο Arm Studio
Πάνελ «Εργαλεία» → πλαίσιο «Raspberry Pi (SSH)»: host, χρήστης, φάκελος του repo (προεπιλογή
`Master-s-Thesis` στο home). Οι τιμές αποθηκεύονται. Μετά: «Σάρωση I²C» → «Εκτέλεση».

Το image θέλει `openssh-client`: μετά από αυτή την αλλαγή `docker compose build` μία φορά.

## Αν δεν συνδέεται
| Μήνυμα | Αιτία |
|---|---|
| `Permission denied (publickey…)` | δεν έγινε το `ssh-copy-id`, ή λάθος χρήστης |
| `Could not resolve hostname` | το `.local` όνομα δεν λύνεται μέσα στο container· δώσε τη διεύθυνση IP |
| `Connection timed out` | Pi σβηστό, άλλο δίκτυο, ή κλειστό SSH |
| `Host key verification failed` | το Pi ξαναστήθηκε: σβήσε τη γραμμή του από το `.local/ssh/known_hosts` |

## Ασφάλεια
- Εργαλεία που κινούν servo (⚠) ζητούν επιβεβαίωση και έχουν πάντα περιορισμένη διάρκεια.
- Η «Διακοπή» κλείνει τη σύνδεση SSH· **δεν** εγγυάται ότι σταματά αμέσως το πρόγραμμα στο Pi. Γι' αυτό τα
  εργαλεία κίνησης τελειώνουν μόνα τους και υπάρχει το «Όλες οι έξοδοι off». Το E-stop της τροφοδοσίας μένει.
