# Arm Studio — εκκίνηση με εικονίδιο

Η εφαρμογή ελέγχου (`arm_gui`, ADR-0016) ανοίγει με διπλό κλικ, χωρίς γραμμή εντολών.
Το εικονίδιο τρέχει το `arm_studio.sh`, που:
1. ξεκινά το container (την πρώτη φορά χτίζει το image, 10–30 λεπτά),
2. χτίζει το workspace (`colcon build --symlink-install`, λίγα δευτερόλεπτα όταν δεν άλλαξε κάτι),
3. ανοίγει την εφαρμογή μέσα στο container.

Από την εφαρμογή ξεκινούν Gazebo και RViz (πάνελ «Σύστημα» → «Εκκίνηση»).
Μηνύματα και σφάλματα του launcher: `.local/arm_studio.log` (στη ρίζα του repo).

## Ubuntu (μία φορά)
```bash
cd ~/Desktop/Master-s-Thesis
./software/launcher/install_linux.sh
```
Εμφανίζεται το **Arm Studio** στο μενού εφαρμογών (μπορεί να καρφιτσωθεί στο dock) και στην επιφάνεια
εργασίας. Αν το εικονίδιο της επιφάνειας εργασίας δεν ανοίγει: δεξί κλικ → **Allow Launching**.

## Windows (μία φορά)
Προϋποθέσεις: WSL 2 με Ubuntu, Docker που τρέχει από μέσα από το WSL, το repo κλωνοποιημένο **μέσα στο WSL**
(ADR-0002), π.χ. `~/master-s-thesis`.
1. Explorer → `\\wsl.localhost\Ubuntu\home\<χρήστης>\master-s-thesis\software\launcher`
2. Διπλό κλικ στο **`install_windows.cmd`**.

Αν η διανομή ή η διαδρομή διαφέρουν: `install_windows.cmd -Distro Ubuntu-24.04 -RepoPath ~/code/master-s-thesis`.
Τα παράθυρα εμφανίζονται μέσω WSLg.

## Σύνδεση με τον βραχίονα μέσω USB
Ο launcher προσθέτει το `software/docker/compose.usb.yaml`: το container βλέπει τις σειριακές USB
(`/dev/ttyACM*`, `/dev/ttyUSB*`), ακόμα κι αν η πλακέτα συνδεθεί αφού ξεκίνησε.
Στα Windows η συσκευή περνά πρώτα στο WSL:
```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

## Εικονίδιο
`icons/arm_studio.png` (Linux) και `icons/arm_studio.ico` (Windows) παράγονται από το `icons/make_icons.py`.
