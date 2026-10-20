# Οδηγός εγκατάστασης — Ubuntu

Εγκατάσταση από το μηδέν του περιβάλλοντος ανάπτυξης (Docker + ROS 2 Jazzy)
στον υπολογιστή ανάπτυξης. Κάθε βήμα καταγράφει **τι τρέχουμε**, **γιατί**, και
το **αποτέλεσμα στο μηχάνημα αναφοράς**. Σχετικές αποφάσεις: ADR-0001, ADR-0002.

---

## Βήμα 1 — Έλεγχος συστήματος

### Εντολές
```bash
lsb_release -ds; uname -m
echo "Session: $XDG_SESSION_TYPE"
df -h ~ | tail -1
free -h | grep Mem
lspci | grep -iE 'vga|3d'
which git docker code
```

### Αποτέλεσμα στο μηχάνημα αναφοράς (Lenovo Legion Y530-15ICH, 2026-10-20)
```
Ubuntu 26.04 LTS
x86_64
Session: wayland
/dev/nvme0n1p4   97G   50G   42G  55% /
Mem:            14Gi       4.1Gi       6.3Gi       554Mi       5.4Gi        10Gi
00:02.0 VGA compatible controller: Intel Corporation CoffeeLake-H GT2 [UHD Graphics 630]
01:00.0 VGA compatible controller: NVIDIA Corporation GP107M [GeForce GTX 1050 Mobile] (rev a1)
/usr/bin/git
/snap/bin/docker
/snap/bin/code
```

### Τι ελέγχουμε και τι συμπεραίνουμε

| Έλεγχος | Αποτέλεσμα | Γιατί μας ενδιαφέρει / συμπέρασμα |
|---|---|---|
| Έκδοση Ubuntu | **26.04 LTS** | Το ROS 2 Jazzy υποστηρίζεται επίσημα μόνο σε **Ubuntu 24.04**· native εγκατάσταση σε 26.04 δεν είναι δυνατή. Το Docker image (Ubuntu 24.04 μέσα στο container) λύνει το πρόβλημα — επιβεβαιώνει την ADR-0002. Σημείωση: για το 26.04 υπάρχει το νέο LTS ROS 2 *Lyrical*, όμως το οικοσύστημα (MoveIt 2, gz_ros2_control, micro-ROS, realsense-ros) ωριμάζει ακόμα — παραμένουμε στο Jazzy (ADR-0001). |
| Αρχιτεκτονική | **x86_64** | Τα επίσημα images `osrf/ros:jazzy-desktop` υπάρχουν για x86_64. |
| Session γραφικών | **wayland** | Τα RViz/Gazebo μέσα στο container είναι εφαρμογές X11· τρέχουν μέσω **XWayland**. Απαιτείται `xhost +local:docker` ώστε το container να ανοίγει παράθυρα. |
| Ελεύθερος χώρος | **42 GB** | Το image με Gazebo + MoveIt πιάνει ~8–10 GB, το workspace λίγα GB. Επαρκεί. |
| RAM | **14 GiB** | Gazebo + MoveIt + RViz χρειάζονται ~4–6 GB. Επαρκεί. |
| GPU | **Intel UHD 630** + **NVIDIA GTX 1050 Mobile** (υβριδικό) | Ξεκινάμε με την **Intel** μέσω `/dev/dri` (`compose.gpu.yaml`), που δουλεύει χωρίς επιπλέον drivers. Η NVIDIA απαιτεί proprietary driver + NVIDIA Container Toolkit — προαιρετική βελτίωση αν το Gazebo είναι αργό. |
| git | `/usr/bin/git` | Εγκατεστημένο από apt. ✅ |
| Docker | **`/snap/bin/docker`** | Η έκδοση **snap** έχει γνωστά προβλήματα: περιορισμούς πρόσβασης σε αρχεία (confinement), ασυμβατότητα με NVIDIA Container Toolkit και Dev Containers. **Αντικαθίσταται** από το Docker των αποθετηρίων του Ubuntu (Βήμα 2). |
| VS Code | `/snap/bin/code` | Η snap έκδοση του VS Code λειτουργεί με Dev Containers· κρατιέται. |

---

## Βήμα 2 — Εγκατάσταση Docker (αντικατάσταση του snap)

### Εντολές
```bash
# 2α. Αφαίρεση του Docker από snap (διαγράφει και images/containers του snap Docker)
sudo snap remove --purge docker

# 2β. Docker από τα αποθετήρια του Ubuntu
sudo apt update
sudo apt install -y docker.io docker-compose-v2 docker-buildx

# 2γ. Χρήση του docker χωρίς sudo
sudo usermod -aG docker $USER
```
Μετά: **αποσύνδεση και επανασύνδεση** (ή επανεκκίνηση), ώστε να ισχύσει η ομάδα `docker`.

```bash
# 2δ. Επαλήθευση
docker --version
docker compose version
docker run --rm hello-world
```

### Γιατί
- **`docker.io` του Ubuntu αντί για το αποθετήριο της Docker Inc.:** είναι
  εγγυημένα διαθέσιμο και συμβατό με το Ubuntu 26.04 από την πρώτη μέρα,
  ενημερώνεται με το `apt upgrade` μαζί με το σύστημα, και είναι αρκετά
  σύγχρονο για τις ανάγκες μας (Compose v2, BuildKit).
- **`docker-compose-v2`:** παρέχει την εντολή `docker compose`, με την οποία
  περιγράφεται και ξεκινά το container μας (`software/docker/compose.yaml`).
- **`docker-buildx`:** ο σύγχρονος builder (BuildKit) για το `docker compose build`.
- **Ομάδα `docker`:** επιτρέπει `docker ...` χωρίς `sudo`. Απαραίτητο για το VS Code Dev Containers.

### Αποτέλεσμα στο μηχάνημα αναφοράς
```
Docker version 29.1.3, build 29.1.3-0ubuntu4.1
Docker Compose version 2.40.3+ds1-0ubuntu1
...
Hello from Docker!
This message shows that your installation appears to be working correctly.
```
✅ Docker Engine 29.1 και Compose 2.40 από τα αποθετήρια του Ubuntu· το
container `hello-world` κατέβηκε από το Docker Hub και έτρεξε χωρίς `sudo`,
άρα λειτουργούν daemon, δίκτυο και δικαιώματα ομάδας `docker`.

---

## Βήμα 3 — Λήψη του repository και χτίσιμο του image

### Εντολές
```bash
# 3α. Λήψη του repo και μετάβαση στο branch ανάπτυξης
#     (αν υπάρχει ήδη clone, π.χ. ~/Desktop/Master-s-Thesis, παραλείπεται το clone
#      και γίνεται: cd ~/Desktop/Master-s-Thesis && git fetch origin && git pull --ff-only)
cd ~/Desktop
git clone https://github.com/FotisRousogiannis/Master-s-Thesis.git
cd ~/Desktop/Master-s-Thesis
git checkout draft
id -u

# 3β. Χτίσιμο του image (μία φορά· ~10–30 λεπτά, κατεβάζει μερικά GB)
cd ~/Desktop/Master-s-Thesis/software/docker
docker compose build
docker images robotic-arm-dev
```

### Γιατί
- **Θέση του repo:** οπουδήποτε στον δίσκο (εδώ `~/Desktop/Master-s-Thesis`). Στο
  Linux τα ονόματα είναι case-sensitive (`Master-s-Thesis` ≠ `master-s-thesis`).
  Το repo **δεν** μπαίνει μέσα στο container: το container προσαρτά (mount) τον φάκελο·
  έτσι ό,τι αλλάζει μέσα στο container αλλάζει και στον δίσκο, και αντίστροφα.
- **`git checkout draft`:** η ανάπτυξη γίνεται στο `draft` (ADR-0003).
- **`id -u`:** ο χρήστης μέσα στο container (`ubuntu`) έχει uid 1000· αν και ο
  χρήστης του host έχει 1000, τα αρχεία που δημιουργούνται στο container ανήκουν σε εμάς και όχι στον root.
- **`docker compose build`:** χτίζει το image από το `software/docker/Dockerfile`
  (ROS 2 Jazzy, Gazebo Harmonic, ros2_control, MoveIt 2). Γίνεται μία φορά· οι
  επόμενες εκκινήσεις του container είναι άμεσες.

### Αποτέλεσμα στο μηχάνημα αναφοράς
Υπήρχε ήδη clone στο `~/Desktop/Master-s-Thesis` (καθαρό working tree):
```
$ git pull --ff-only
Updating 6ea32ab..4c94b14
Fast-forward
$ git checkout draft
branch 'draft' set up to track 'origin/draft'.
Switched to a new branch 'draft'
$ id -u
1000
$ docker images robotic-arm-dev
IMAGE                   ID             DISK USAGE   CONTENT SIZE
robotic-arm-dev:jazzy   8c4d7d850ad9       6.66GB         1.48GB
```
✅ uid 1000 = uid του χρήστη `ubuntu` στο container· image χτισμένο (6.66 GB στον δίσκο).

---

## Βήμα 4 — Εκκίνηση του container με γραφικά

### Εντολές
```bash
xhost +local:docker
cd ~/Desktop/Master-s-Thesis/software/docker
docker compose -f compose.yaml -f compose.gpu.yaml up -d
docker compose exec dev bash
```

### Γιατί
- **`xhost +local:docker`:** επιτρέπει σε τοπικά containers να ανοίγουν παράθυρα
  στον X server (εδώ XWayland). Ισχύει μέχρι την αποσύνδεση· επαναλαμβάνεται σε κάθε σύνδεση.
- **`compose.gpu.yaml`:** περνά το `/dev/dri` (Intel UHD 630) στο container για
  επιτάχυνση OpenGL σε RViz/Gazebo.
- **`up -d`:** το container τρέχει στο παρασκήνιο· **`exec dev bash`:** shell μέσα σε
  αυτό (prompt `ubuntu@...`). Το repo είναι στο `~/master-s-thesis` του container.
- Τερματισμός: `docker compose down` (από τον φάκελο `software/docker`).

### Αποτέλεσμα στο μηχάνημα αναφοράς
```
$ docker compose -f compose.yaml -f compose.gpu.yaml up -d
 ✔ Container robotic-arm-dev  Started
ubuntu@fotisrous-Lenovo-Legion-Y530-15ICH:~/master-s-thesis/software/ros2_ws$ echo $ROS_DISTRO
jazzy
$ ls ~/master-s-thesis
LICENSE  NOTES.md  README.md  docs  software  test.txt  thesis
```
✅ Το container ξεκινά, το ROS 2 Jazzy φορτώνεται και το repo είναι προσαρτημένο.
(Το `glxinfo` δεν υπήρχε στο image — προστέθηκε το `mesa-utils` στο Dockerfile.)


---

## Βήμα 5 — Χτίσιμο του workspace και tests (μέσα στο container)

### Εντολές
```bash
cd ~/master-s-thesis/software/ros2_ws
colcon build --symlink-install
colcon test && colcon test-result --verbose
source install/setup.bash
```

### Γιατί
- **`colcon build --symlink-install`:** χτίζει όλα τα packages του `src/`· με
  symlinks, αλλαγές σε Python/launch/URDF ισχύουν χωρίς νέο build.
- **`colcon test`:** τρέχει τα tests (π.χ. δομή του URDF).
- **`source install/setup.bash`:** κάνει τα packages μας ορατά στο `ros2`
  (γίνεται και αυτόματα σε κάθε νέο shell μέσω `~/.bashrc`).

### Αποτέλεσμα στο μηχάνημα αναφοράς
```
Summary: 1 package finished [2.14s]
...
Summary: 5 tests, 0 errors, 0 failures, 0 skipped
```
✅ 4 tests δομής URDF + 1 ctest που περιβάλλει το pytest.


---

## Βήμα 6 — Οπτικοποίηση του μοντέλου στο RViz

### Εντολές
```bash
ros2 launch arm_description display.launch.py
```

### Αναμενόμενο
Ανοίγουν το **RViz** (ο βραχίονας από κυλίνδρους, όρθιος) και το
**Joint State Publisher** (6 μπάρες, μία ανά άρθρωση). Η κίνηση μιας μπάρας
κινεί την αντίστοιχη άρθρωση.

### Αποτέλεσμα στο μηχάνημα αναφοράς
✅ Άνοιξαν RViz και Joint State Publisher· οι αρθρώσεις κινούνται με τις μπάρες.
Το περιβάλλον λειτουργεί από άκρη σε άκρη: Docker → ROS 2 → γραφικά (XWayland, Intel GPU) → δικός μας κώδικας.

