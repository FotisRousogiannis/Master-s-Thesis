# ADR-0010: Ρύθμιση host — Docker από τα αποθετήρια Ubuntu, Intel GPU, XWayland

- **Κατάσταση:** Αποδεκτή
- **Ημερομηνία:** 2026-10-20

## Πλαίσιο
Κατά την εγκατάσταση στον υπολογιστή ανάπτυξης (Lenovo Legion Y530, βλ.
`docs/setup/ubuntu.md`, Βήμα 1) διαπιστώθηκαν:
- **Ubuntu 26.04 LTS** — το ROS 2 Jazzy υποστηρίζεται μόνο σε 24.04.
- **Docker εγκατεστημένο από snap.**
- **Wayland** session.
- Υβριδικά γραφικά **Intel UHD 630 + NVIDIA GTX 1050 Mobile**.

## Εναλλακτικές

**Πηγή Docker**
1. **snap** (υπάρχον) — confinement που περιορίζει την πρόσβαση σε αρχεία,
   γνωστά προβλήματα με Dev Containers και NVIDIA Container Toolkit.
2. **Αποθετήριο της Docker Inc.** — νεότερες εκδόσεις, αλλά η υποστήριξη νέων
   εκδόσεων Ubuntu (26.04) καθυστερεί.
3. **`docker.io` + `docker-compose-v2` + `docker-buildx` από τα αποθετήρια του
   Ubuntu** — εγγυημένα διαθέσιμα στο 26.04, ενημερώσεις με `apt upgrade`.

**ROS 2 στο 26.04**
1. Native ROS 2 *Lyrical* (LTS για 26.04) — νέο, το οικοσύστημα (MoveIt 2,
   gz_ros2_control, micro-ROS, realsense-ros) ωριμάζει ακόμα.
2. **ROS 2 Jazzy σε Docker** (Ubuntu 24.04 μέσα στο container).

**Γραφικά**
1. **Intel μέσω `/dev/dri`** — χωρίς επιπλέον drivers.
2. NVIDIA — απαιτεί proprietary driver + NVIDIA Container Toolkit.

## Απόφαση
- Docker από τα **αποθετήρια του Ubuntu** (`docker.io` 29.1, Compose 2.40)· το snap αφαιρέθηκε.
- **ROS 2 Jazzy σε Docker** (επιβεβαιώνει ADR-0001, ADR-0002).
- Γραφικά μέσω **XWayland** (`xhost +local:docker`) και **Intel GPU** (`compose.gpu.yaml`).

## Αιτιολόγηση
Η λύση λειτούργησε με την πρώτη: image 6.66 GB, `colcon build` και 5/5 tests,
RViz με το μοντέλο του βραχίονα (βλ. οδηγό, Βήματα 2–6). Δεν απαιτήθηκε καμία
αλλαγή στο σύστημα πέρα από την αντικατάσταση του Docker.

## Συνέπειες
- Η εντολή `xhost +local:docker` επαναλαμβάνεται σε κάθε σύνδεση στον υπολογιστή.
- Αν το Gazebo αποδειχθεί αργό με την Intel, εξετάζεται η NVIDIA (νέο ADR).
- Η μετάβαση σε ROS 2 Lyrical επανεξετάζεται όταν ωριμάσει το οικοσύστημα.
