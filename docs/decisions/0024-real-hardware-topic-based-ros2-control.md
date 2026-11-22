# ADR-0024: `hardware:=real` — ros2_control ↔ Pi με `topic_based_ros2_control`

- **Κατάσταση:** Αποδεκτή (απόφαση Φώτη, 2026-11-22)
- **Ημερομηνία:** 2026-11-22
- **Επεκτείνει:** ADR-0016 (`hardware:=real`), ADR-0021 (στάδιο D: «ορίζεται στο επόμενο βήμα»)

## Πλαίσιο
Ο `arm_pi_driver` (ADR-0021) ακούει στόχους θέσης στο `/base/joint_commands` και δημοσιεύει θέσεις στο
`/base/joint_states` (`JointState`, ακτίνια). Πάνω από το `ros2_control` (`arm_controller`, MoveIt, Arm Studio) όλα
είναι ίδια για mock / gazebo / real· λείπει το hardware plugin που ενώνει το `ros2_control` με αυτά τα topics.

## Επιλογές
1. **`topic_based_ros2_control/TopicBasedSystem`** (PickNik, έτοιμο πακέτο): γράφει τις εντολές θέσης σε ένα topic
   `JointState` και διαβάζει τις θέσεις από άλλο.
2. **Δικό μας plugin C++** (`arm_hardware/ArmSystem`, όπως προέβλεπε το ADR-0016).
3. **Γέφυρα Python** (action server `follow_joint_trajectory` → topics), έξω από το `ros2_control`.

## Απόφαση
**(1).** Στο `arm.ros2_control.xacro`, `hardware == 'real'`: plugin `TopicBasedSystem` με
`joint_commands_topic = /base/joint_commands`, `joint_states_topic = /base/joint_states` και
`trigger_joint_command_threshold = -1`. Το `bringup.launch.py` δέχεται `hardware:=real`· το Arm Studio το προσφέρει.

## Αιτιολόγηση
- Τα topics του plugin είναι **ακριβώς** τα topics του Pi (και των ESP32 αργότερα): καμία αλλαγή στο Pi, κανένας κώδικας
  C++ για συντήρηση. Το (2) χρειάζεται όταν η επικοινωνία δεν είναι πια ROS topics· σήμερα δεν ισχύει.
- Το (3) θα παρέκαμπτε το `ros2_control`: διαφορετική συμπεριφορά από mock/gazebo, άρα όσα δοκιμάζονται σε
  προσομοίωση δεν θα ίσχυαν στον βραχίονα.

## Λεπτομέρειες που μετράνε (από τον κώδικα του plugin, v0.3.0)
- **Εντολή σε κάθε κύκλο (100 Hz).** Από προεπιλογή το plugin δεν στέλνει τίποτα όταν εντολή ≈ θέση. Ο driver του Pi
  σταματά μετά από 0.2 s χωρίς εντολή (watchdog): με το κατώφλι −1 η σιωπή σημαίνει μόνο «χάθηκε η σύνδεση».
- **Ο `arm_controller` ξεκινά μετά την πρώτη θέση του Pi.** Ο trajectory controller κρατά τη θέση που διαβάζει όταν
  ενεργοποιείται· πριν από το πρώτο `/base/joint_states` το plugin διαβάζει 0 σε όλες τις αρθρώσεις, άρα θα έστελνε
  τον βραχίονα απότομα στο μηδέν. Το launch περιμένει ένα μήνυμα (`ros2 topic echo --once`) και μετά ξεκινά τον
  controller.
- **Αρθρώσεις που το Pi δεν οδηγεί** (σήμερα joint_3–6): μένουν στο 0 στο `/joint_states` (δεν είναι μετρημένες), και ο
  driver αγνοεί τις εντολές τους (άγνωστα ονόματα).
- QoS: εντολές reliable (όπως ακούει το Pi), θέσεις best effort στο plugin (δέχεται τον reliable publisher του Pi).
- **STOP** του Arm Studio: ακύρωση τροχιάς → ο controller κρατά την τρέχουσα θέση → το Pi τη στέλνει ως στόχο. Δεν
  είναι λειτουργία ασφαλείας (ADR-0021: διακόπτης στο ρεύμα των servo).

## Συνέπειες
- Νέα εξάρτηση: `ros-jazzy-topic-based-ros2-control` στο Docker του PC (ξαναχτίσιμο μόνο του μικρού layer).
- Ο driver μηδένιζε τη «μνήμη» του νόμου ελέγχου (I) σε κάθε αλλαγή στόχου· σε τροχιά ο στόχος αλλάζει κάθε 10 ms,
  άρα το I δεν θα δούλευε ποτέ. **Διορθώθηκε:** reset μόνο στην αρχή μιας κίνησης ή σε στάση ασφαλείας (η παράγωγος
  είναι ήδη στη μέτρηση, χωρίς «κλωτσιά» στην αλλαγή στόχου).
- **Δεν δοκιμάστηκε** με ROS/Pi στη συνεδρία (χωρίς ROS εκεί). Ούτε επιβεβαιώθηκε ότι το πακέτο υπάρχει ως binary
  για Jazzy· αν όχι, χτίζεται από τον κώδικα (github.com/PickNikRobotics/topic_based_ros2_control) στο workspace.
