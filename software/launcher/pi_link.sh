#!/usr/bin/env bash
# Connection between this computer and the arm's Raspberry Pi (ADR-0021). Run ON THE COMPUTER (not in Docker).
#
#   ./pi_link.sh setup-direct        # once: the Ethernet port becomes the "arm" network (cable to the Pi)
#   ./pi_link.sh find                # where is the Pi now? (direct cable, then <name>.local on any network)
#   ./pi_link.sh key USER [HOST]     # once: SSH key for Arm Studio's Tools panel (asks the Pi password once)
#
# Direct cable: this computer is 10.42.0.1 and shares its own internet (whatever WiFi it is on) with the Pi;
# the Pi is always 10.42.0.2. Changing the computer's WiFi does not affect the cable.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SSH_DIR="$REPO/.local/ssh"
PI_DIRECT="10.42.0.2"
PC_DIRECT="10.42.0.1/24"
PI_NAME="${ARM_PI_NAME:-armpi}"
CONNECTION="arm-pi-direct"

reachable() { timeout 2 bash -c "</dev/tcp/$1/22" 2>/dev/null; }   # SSH port open?

cmd_setup_direct() {
    command -v nmcli >/dev/null || { echo "Χρειάζεται NetworkManager (nmcli)." >&2; exit 1; }
    local dev
    dev=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "ethernet" { print $1; exit }')
    [ -n "$dev" ] || { echo "Δεν βρέθηκε θύρα Ethernet." >&2; exit 1; }
    if nmcli -t -f NAME connection show | grep -qx "$CONNECTION"; then
        echo "Η σύνδεση «$CONNECTION» υπάρχει ήδη."
    else
        # "shared": this computer gives the Pi an address (DHCP) and its internet (NAT), like a small router.
        nmcli connection add type ethernet ifname "$dev" con-name "$CONNECTION" \
            ipv4.method shared ipv4.addresses "$PC_DIRECT" ipv6.method disabled \
            connection.autoconnect yes
        echo "Δημιουργήθηκε η «$CONNECTION» στη θύρα $dev."
    fi
    nmcli connection up "$CONNECTION" || echo "Θα ενεργοποιηθεί μόλις συνδεθεί το καλώδιο."
}

cmd_find() {
    if reachable "$PI_DIRECT"; then
        echo "✔ Pi στο καλώδιο: $PI_DIRECT"
        return 0
    fi
    local ip=""
    if command -v avahi-resolve-host-name >/dev/null; then
        ip=$(avahi-resolve-host-name -4 "$PI_NAME.local" 2>/dev/null | awk '{ print $2 }')
    fi
    [ -n "$ip" ] || ip=$(getent ahostsv4 "$PI_NAME.local" 2>/dev/null | awk 'NR == 1 { print $1 }')
    if [ -n "$ip" ] && reachable "$ip"; then
        echo "✔ Pi στο δίκτυο ως $PI_NAME.local: $ip"
        echo "  (στο Arm Studio → Εργαλεία → Host γράψε $ip· μέσα στο Docker τα ονόματα .local δεν λύνονται)"
        return 0
    fi
    echo "✖ Δεν βρέθηκε το Pi."
    echo "  Καλώδιο: συνδεδεμένο; έτρεξε το ./pi_link.sh setup-direct εδώ και το setup_network.sh base στο Pi;"
    echo "  WiFi: είναι στο ίδιο δίκτυο με αυτόν τον υπολογιστή; (στο Pi: setup_network.sh show)"
    return 1
}

cmd_key() {
    local user="${1:?Δώσε τον χρήστη του Pi: key USER [HOST]}" host="${2:-$PI_DIRECT}"
    mkdir -p "$SSH_DIR"
    [ -f "$SSH_DIR/id_ed25519" ] || ssh-keygen -t ed25519 -N "" -C "arm-studio" -f "$SSH_DIR/id_ed25519"
    ssh-copy-id -i "$SSH_DIR/id_ed25519.pub" -o UserKnownHostsFile="$SSH_DIR/known_hosts" "$user@$host"
    ssh -i "$SSH_DIR/id_ed25519" -o BatchMode=yes -o UserKnownHostsFile="$SSH_DIR/known_hosts" \
        "$user@$host" 'echo "✔ SSH χωρίς κωδικό προς $(hostname)"'
}

case "${1:-}" in
    setup-direct) cmd_setup_direct ;;
    find) cmd_find ;;
    key) shift; cmd_key "$@" ;;
    *) sed -n '2,10p' "$0"; exit 1 ;;
esac
