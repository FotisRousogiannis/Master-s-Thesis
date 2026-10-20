# ADR-0002: Περιβάλλον ανάπτυξης σε Docker

- **Κατάσταση:** Αποδεκτή
- **Ημερομηνία:** 2026-10-20

## Πλαίσιο
Η ανάπτυξη γίνεται σε Ubuntu και σε Windows. Το ROS 2 Jazzy απαιτεί Ubuntu
24.04 και πολλές εξαρτήσεις· θέλουμε ίδιο, επαναλήψιμο περιβάλλον παντού και στο CI.

## Εναλλακτικές
1. **Docker image + VS Code Dev Container** — ίδιο περιβάλλον σε Ubuntu,
   Windows (WSL2) και GitHub Actions.
2. **Native εγκατάσταση** σε κάθε μηχάνημα — ταχύτερο γραφικά, αλλά απόκλιση
   εκδόσεων και αδύνατο σε Windows.
3. **Εικονική μηχανή (VM)** — βαριά, αργά γραφικά για Gazebo.

## Απόφαση
Docker image βασισμένο στο `osrf/ros:jazzy-desktop` με Gazebo, ros2_control,
MoveIt 2 (`software/docker/`), και Dev Container για VS Code (`.devcontainer/`).

## Αιτιολόγηση
Ένα περιβάλλον για όλους, που περιγράφεται σε κώδικα και αναπαράγεται με
μία εντολή· τα γραφικά (RViz, Gazebo) λειτουργούν μέσω X11 (Ubuntu) / WSLg (Windows).

## Συνέπειες
- Σε Linux απαιτείται `xhost +local:docker` για τα γραφικά.
- Σε Windows οι εντολές τρέχουν από το WSL Ubuntu και το repo κλωνοποιείται
  μέσα στο σύστημα αρχείων του WSL (όχι στο `C:`), για ταχύτητα.
- Προαιρετικά `compose.gpu.yaml` για επιτάχυνση με GPU σε Linux.
