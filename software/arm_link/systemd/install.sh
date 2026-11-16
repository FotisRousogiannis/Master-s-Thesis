#!/usr/bin/env bash
# Εγκαθιστά/ενημερώνει την υπηρεσία arm-link-echo στο Pi. Τρέχει από τον χρήστη του Pi (όχι με sudo):
#   bash ~/master-s-thesis/software/arm_link/systemd/install.sh
# Αφαίρεση:  bash .../install.sh --remove
set -euo pipefail

NAME=arm-link-echo
HERE="$(cd "$(dirname "$0")" && pwd)"
DIR="$(dirname "$HERE")"                      # .../software/arm_link
TARGET=/etc/systemd/system/$NAME.service

if [ "${1:-}" = "--remove" ]; then
    sudo systemctl disable --now "$NAME" 2>/dev/null || true
    sudo rm -f "$TARGET"
    sudo systemctl daemon-reload
    echo "Η υπηρεσία $NAME αφαιρέθηκε."
    exit 0
fi

sed -e "s|@USER@|$USER|" -e "s|@DIR@|$DIR|" "$HERE/$NAME.service" | sudo tee "$TARGET" > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable "$NAME"                 # ξεκινά σε κάθε εκκίνηση του Pi
sudo systemctl restart "$NAME"                # ξεκινά/ξαναξεκινά τώρα, με τον νέο κώδικα
sleep 1
systemctl --no-pager --lines=3 status "$NAME"
