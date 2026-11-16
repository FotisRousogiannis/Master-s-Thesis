#!/usr/bin/env bash
# Arm Studio launcher (host side, Ubuntu or WSL). Started by the desktop icon (docs/decisions/0016):
#   1. starts the Docker container (builds the image the first time, or when the Dockerfile changed),
#   2. builds the ROS 2 workspace (incremental, a few seconds),
#   3. opens the application inside the container.
# Meanwhile a small window (zenity) shows a moving bar and the current step.
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

# ---- progress window (zenity): a moving bar with the current step; closes when the app opens.
# Docker gives no percentage, so the bar pulsates and the text below it says what is happening.
PROGRESS_FD=""
progress_start() {
    [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v zenity >/dev/null || return 0
    exec {PROGRESS_FD}> >(zenity --progress --pulsate --auto-close --no-cancel --width=480 \
        --title="Arm Studio" --text="Ξεκινά…" 2>/dev/null)
}
show() {       # show "text": only the line under the bar
    [ -n "$PROGRESS_FD" ] && echo "# ${1:0:90}" >&"$PROGRESS_FD" 2>/dev/null
    return 0
}
progress() {   # progress "text": the line under the bar, and the log
    echo "$1"
    show "$1"
}
progress_end() {
    [ -n "$PROGRESS_FD" ] || return 0
    echo 100 >&"$PROGRESS_FD" 2>/dev/null
    exec {PROGRESS_FD}>&-
    PROGRESS_FD=""
}
follow() {     # follow "prefix": log every line of a command's output, show the useful ones in the window
    local line
    while IFS= read -r line; do
        echo "$line"
        case "$line" in
            *"#"*"["*"/"*"]"*|*"Starting >>>"*|*"Finished <<<"*|*"Pulling"*|*"Creating"*|*"Recreat"*|*"Started"*)
                show "$1: $(echo "$line" | sed -E 's/^#[0-9]+ //; s/ +/ /g')" ;;
        esac
    done
}

fail() {
    progress_end
    echo "ΣΦΑΛΜΑ: $1"
    if command -v zenity >/dev/null; then
        zenity --error --title="Arm Studio" --text="$1\n\nΛεπτομέρειες: $LOG" 2>/dev/null
    else
        notify "$1 (δες $LOG)"
    fi
    exit 1
}

# One launcher at a time: a second click while Arm Studio is starting or open only says so (otherwise two
# launchers race to recreate the same container and one fails with "Conflict").
exec {LOCK_FD}>"$LOG_DIR/arm_studio.lock"
if ! flock -n "$LOCK_FD"; then
    echo "Ήδη σε εξέλιξη: δεύτερο κλικ, δεν κάνω τίποτα."
    msg="Το Arm Studio ήδη ξεκινά ή είναι ανοιχτό."
    if command -v zenity >/dev/null; then
        zenity --info --title="Arm Studio" --text="$msg" --timeout=5 2>/dev/null
    else
        notify "$msg"
    fi
    exit 0
fi

command -v docker >/dev/null || fail "Δεν βρέθηκε το Docker."
docker info >/dev/null 2>&1 || fail "Το Docker δεν τρέχει ή ο χρήστης δεν ανήκει στην ομάδα docker."

# Allow the container to open windows on this display (X11 / XWayland / WSLg).
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null; then
    xhost +local:docker >/dev/null || true
fi

cd "$DOCKER_DIR" || fail "Δεν βρέθηκε ο φάκελος $DOCKER_DIR"
COMPOSE=(docker compose -f compose.yaml -f compose.usb.yaml)
[ -e /dev/dri ] && COMPOSE+=(-f compose.gpu.yaml)

progress_start
if ! docker image inspect robotic-arm-dev:jazzy >/dev/null 2>&1; then
    notify "Πρώτη εκκίνηση: χτίζεται το περιβάλλον Docker (10–30 λεπτά)…"
fi
progress "Docker: έλεγχος (αν άλλαξε, ξαναχτίζεται: έως 30 λεπτά)"
"${COMPOSE[@]}" up -d --build 2>&1 | follow "Docker"
[ "${PIPESTATUS[0]}" -eq 0 ] || fail "Δεν ξεκίνησε το container."

progress "ROS 2: χτίσιμο της εφαρμογής"
"${COMPOSE[@]}" exec -T dev bash -c '
    source /opt/ros/jazzy/setup.bash &&
    cd "$WS" &&
    colcon build --symlink-install --packages-up-to arm_gui
' 2>&1 | follow "ROS 2"
[ "${PIPESTATUS[0]}" -eq 0 ] || fail "Το χτίσιμο της εφαρμογής απέτυχε."

progress "Άνοιγμα του Arm Studio…"
if [ -n "$PROGRESS_FD" ]; then
    # the window closes a few seconds after the app starts: only this background job keeps it open
    ( sleep 4; progress_end ) &
    exec {PROGRESS_FD}>&-
    PROGRESS_FD=""
fi
"${COMPOSE[@]}" exec -T dev bash -c '
    source /opt/ros/jazzy/setup.bash &&
    cd "$WS" &&
    source install/setup.bash &&
    exec ros2 run arm_gui arm_gui
' || fail "Η εφαρμογή δεν ξεκίνησε ή τερμάτισε με σφάλμα."
