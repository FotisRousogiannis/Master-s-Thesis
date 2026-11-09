#!/usr/bin/env bash
# Adds the Arm Studio icon to the application menu and the desktop (Ubuntu). Run once:
#   ./software/launcher/install_linux.sh
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
APPS="$HOME/.local/share/applications"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
ENTRY="$APPS/arm-studio.desktop"

mkdir -p "$APPS"
cat >"$ENTRY" <<DESKTOP
[Desktop Entry]
Type=Application
Name=Arm Studio
Comment=Έλεγχος και προσομοίωση του ρομποτικού βραχίονα
Exec="$HERE/arm_studio.sh"
Icon=$HERE/icons/arm_studio.png
Terminal=false
Categories=Development;Science;
StartupWMClass=arm_gui
DESKTOP
chmod +x "$ENTRY" "$HERE/arm_studio.sh"

if [ -d "$DESKTOP_DIR" ]; then
    cp "$ENTRY" "$DESKTOP_DIR/"
    chmod +x "$DESKTOP_DIR/arm-studio.desktop"
    # GNOME: mark as trusted so it runs on double click ("Allow Launching").
    gio set "$DESKTOP_DIR/arm-studio.desktop" metadata::trusted true 2>/dev/null || true
fi
update-desktop-database "$APPS" 2>/dev/null || true

echo "Εγκαταστάθηκε: μενού εφαρμογών → «Arm Studio» και εικονίδιο στην επιφάνεια εργασίας."
echo "Αν το εικονίδιο της επιφάνειας εργασίας δεν ανοίγει: δεξί κλικ → «Allow Launching»."
