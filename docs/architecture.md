# Αρχιτεκτονική συστήματος (τρέχουσα εικόνα)

Σύνοψη των αποδεκτών αποφάσεων. Η αιτιολόγηση κάθε επιλογής βρίσκεται στα
ADR του φακέλου [`decisions/`](decisions/README.md).

```
                     Υπολογιστής (Docker, Ubuntu 24.04, ROS 2 Jazzy)
     ros2_control · micro-ROS Agent · MoveIt 2 · όραση · RViz / Gazebo (sim)
        │ USB 3                          │ Ethernet
   RealSense D435if                  Router 2.4 GHz (αποκλειστικό)
   (σταθερή, eye-to-hand)                ┆ WiFi — micro-ROS over UDP
                       ┌─────────────────┴─────────────────┐
               ESP32 link 1 (PCB)                    ESP32 wrist (PCB)
        I²C: TCA9548A → AS5600                I²C: TCA9548A → AS5600
             PCA9685  → servo                      PCA9685  → servo
        PID θέσης, μίξη διαφορικού,           PID θέσης, μίξη διαφορικού,
        όρια, watchdog                        όρια, watchdog
        (USB σειριακό για debugging)          (USB σειριακό για debugging)
```

## Τι τρέχει πού

| Μονάδα | Λογισμικό | Ρόλος | ADR |
|---|---|---|---|
| Υπολογιστής | ros2_control (controller_manager, joint_trajectory_controller, hardware interface) | συντονισμός των 6 αρθρώσεων, εκτέλεση τροχιών | 0004, 0005 |
| Υπολογιστής | micro-ROS Agent | γέφυρα ESP32 ↔ ROS 2 | 0007 |
| Υπολογιστής | MoveIt 2 | IK, σχεδιασμός τροχιάς χωρίς συγκρούσεις | 0001 |
| Υπολογιστής | realsense-ros, κόμβος όρασης | εικόνα/βάθος, εντοπισμός και θέση αντικειμένων | 0008 |
| Υπολογιστής | κόμβος εργασίας | λογική «pick & place» | — |
| Υπολογιστής | Gazebo + gz_ros2_control | προσομοίωση — αντικαθιστά τα ESP32 | 0001 |
| ESP32 ×2 | firmware (PlatformIO, micro-ROS, FreeRTOS) | βρόχος θέσης ~500 Hz, μίξη διαφορικού, ασφάλεια | 0005, 0007 |

## Λογισμικό στον υπολογιστή — στρώματα (ADR-0016)

```
 Διεπαφή     arm_gui «Arm Studio» (Qt, Python) · RViz · Gazebo — τρία δυναμικά παράθυρα
 ─────────── μόνο ROS interfaces ───────────────────────────────────────────
 Εποπτεία    arm_manager: IDLE / MANUAL / AUTO / FAULT / STOP          (επόμενο)
 Εργασίες    arm_tasks (pick & place) · arm_perception                  (επόμενο)
 Κίνηση      MoveIt 2 · MoveIt Servo                                    (επόμενο)
 Έλεγχος     ros2_control: joint_state_broadcaster, arm_controller      ✅ arm_bringup
 Υλικό       hardware:=mock ✅ │ gazebo ✅ │ real: τώρα Raspberry Pi (ROS 2, Ethernet, ADR-0021),
                                            μετά ESP32 (micro-ROS: WiFi ή USB)
```

| Package | Ρόλος | Γλώσσα | Κατάσταση |
|---|---|---|---|
| `arm_description` | URDF + ros2_control tags | xacro | ✅ (προσωρινή γεωμετρία, ADR-0009) |
| `arm_bringup` | controllers, `bringup.launch.py hardware:=mock\|gazebo`, παράθυρα `rviz.launch.py` / `gazebo_gui.launch.py` (ADR-0018) | launch/YAML | ✅ |
| `arm_calibration` | ρυθμίσεις (παλμοί servo, κανάλια, μηδενικά encoders, όρια) από `calibration.yaml` ως παράμετροι ROS 2 (ADR-0017, ADR-0019) | Python | ✅ (δεν τις διαβάζει ακόμα το υλικό) |
| `arm_logbook` | ημερολόγιο σε SQLite: τελευταία θέση, ιστορικό calibration, καταγραφές, σφάλματα· `logbook` για ανάγνωση (ADR-0020) | Python | ✅ (αδοκίμαστο στο Docker) |
| `arm_gui` | εφαρμογή ελέγχου: Σύστημα, Αρθρώσεις, Αισθητήρες, Συσκευές (`/diagnostics`), Παράμετροι, ROS 2, Εργαλεία (`tools.yaml`), Log· «● REC»· σκούρο/φωτεινό θέμα· εικονίδιο εκκίνησης (`software/launcher/`) | Python | ✅ v0.2 |
| `arm_interfaces`, `arm_manager` | καταστάσεις, ποιος κινεί τον βραχίονα | msg / Python | ⬜ |
| `arm_moveit_config` | MoveIt 2 | YAML | ⬜ |
| `arm_perception`, `arm_tasks` | όραση, pick & place | Python | ⬜ |
| `arm_hardware` | plugin ros2_control → ESP32 | C++ | ⬜ |

## Δίαυλοι

| Σύνδεση | Πρωτόκολλο | ADR |
|---|---|---|
| ESP32 ↔ αισθητήρες / οδηγός servo | I²C (τοπικά, ≤ ~30 cm) | 0006 |
| ESP32 ↔ υπολογιστής | WiFi (micro-ROS/UDP) ή USB σειριακό (micro-ROS serial) — επιλογή στο Arm Studio | 0006, 0016 |
| Υπολογιστής ↔ router | Ethernet | 0006 |
| Κάμερα ↔ υπολογιστής | USB 3 | 0008 |

## Κανόνες σχεδίασης που προκύπτουν
- Κάθε διαφορικός μηχανισμός (2 κινητήρες + 2 encoders) συνδέεται σε **ένα** ESP32.
- Το I²C δεν βγαίνει από το τμήμα του βραχίονα στο οποίο βρίσκεται το ESP32.
- Πάνω από το ros2_control υπάρχουν μόνο **λογικές** αρθρώσεις· το υλικό είναι αθέατο.
- Προσομοίωση και πραγματικός βραχίονας διαφέρουν **μόνο** στο hardware interface.
- E-stop υλικού στην τροφοδοσία των servo. Το STOP του Arm Studio είναι λογισμικό και δεν το αντικαθιστά.
- Το GUI δεν περιέχει λογική ελέγχου· κάθε λειτουργία του είναι ROS interface που δοκιμάζεται και από τερματικό.

## Απορριφθείσες εναλλακτικές (σύνοψη)

| Εναλλακτική | Γιατί απορρίφθηκε | ADR |
|---|---|---|
| Raspberry Pi onboard | ο υπολογιστής είναι δίπλα· δεν προσθέτει αυτονομία· επιπλέον μηχάνημα | 0004 |
| Ένα node ανά άρθρωση | τα διαφορικά συζευγνύουν αρθρώσεις· συγχρονισμός | 0005 |
| Ένα PCB για όλο τον βραχίονα | I²C σε ~1 m καλώδιο, πολλά καλώδια στις αρθρώσεις | 0005 |
| I²C ανάμεσα στις πλακέτες | μόνο για εκατοστά, θόρυβος, όχι micro-ROS transport | 0006 |
| UART / RS-485 / CAN προς υπολογιστή | καλώδια στις αρθρώσεις· CAN χωρίς επίσημο micro-ROS | 0006 |
| EtherCAT | απαιτεί ειδικό υλικό, εκτός κλίμακας | 0006 |
| Δικό μας πρωτόκολλο αντί micro-ROS | λιγότερη ανεκτικότητα σε αλλαγές υλικού | 0007 |
| Κάμερα στην αρπάγη | ελάχιστο βάθος ~0.3 m, θάμπωμα, βάρος | 0008 |
