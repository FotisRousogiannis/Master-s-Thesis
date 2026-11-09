#!/usr/bin/env bash
# Arm Studio launcher (host side, Ubuntu or WSL). Started by the desktop icon (docs/decisions/0016):
#   1. starts the Docker container (builds the image the first time),
#   2. builds the ROS 2 workspace (incremental, a few seconds),
#   3. opens the application inside the container.
# Output goes to <repo>/.local/arm_studio.log.
set -uo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
DOCKER_DIR="$REPO/software/docker"
LOG_DIR="$REPO/.local"
LOG="$LOG_DIR/arm_studio.log"
mkdir -p "$LOG_DIR"
exec >>"$LOG" 2>&1
echo "===== $(date '+%F %T') Arm Studio ====="

notify() {
    echo "$1"
    command -v notify-send >/dev/null && notify-send -a "Arm Studio" "Arm Studio" "$1" || true
}

fail() {
    echo "ΣΦΑΛΜΑ: $1"
    if command -v zenity >/dev/null; then
        zenity --error --title="Arm Studio" --text="$1\n\nΛεπτομέρειες: $LOG" 2>/dev/null
    else
        notify "$1 (δες $LOG)"
    fi
    exit 1
}

command -v docker >/dev/null || fail "Δεν βρέθηκε το Docker."
docker info >/dev/null 2>&1 || fail "Το Docker δεν τρέχει ή ο χρήστης δεν ανήκει στην ομάδα docker."

# Allow the container to open windows on this display (X11 / XWayland / WSLg).
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null; then
    xhost +local:docker >/dev/null || true
fi

cd "$DOCKER_DIR" || fail "Δεν βρέθηκε ο φάκελος $DOCKER_DIR"
COMPOSE=(docker compose -f compose.yaml -f compose.usb.yaml)
[ -e /dev/dri ] && COMPOSE+=(-f compose.gpu.yaml)

if ! docker image inspect robotic-arm-dev:jazzy >/dev/null 2>&1; then
    notify "Πρώτη εκκίνηση: χτίζεται το περιβάλλον Docker (10–30 λεπτά)…"
fi
"${COMPOSE[@]}" up -d --build || fail "Δεν ξεκίνησε το container."

notify "Προετοιμασία της εφαρμογής…"
"${COMPOSE[@]}" exec -T dev bash -c '
    source /opt/ros/jazzy/setup.bash &&
    cd "$WS" &&
    colcon build --symlink-install --packages-up-to arm_gui &&
    source install/setup.bash &&
    exec ros2 run arm_gui arm_gui
' || fail "Η εφαρμογή δεν ξεκίνησε ή τερμάτισε με σφάλμα."
