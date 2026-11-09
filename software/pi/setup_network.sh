#!/usr/bin/env bash
# Network of the Raspberry Pi (Ubuntu Server) for the arm (ADR-0021). Normally run by Arm Studio
# (panel «Σύνδεση») over SSH; it can also be run by hand ON THE PI with sudo:
#
#   sudo ./setup_network.sh base [HOSTNAME]   # once: name, mDNS, fixed address on the cable
#   sudo ./setup_network.sh add-wifi "SSID"   # add a WiFi network (asks for the password)
#   sudo ./setup_network.sh show              # addresses and configured networks
#
# Safety: a change is kept only if it is confirmed within 120 s, otherwise the previous network
# settings come back by themselves. By hand: `netplan try` (press ENTER). From Arm Studio:
# --confirm-file F (the app reconnects after the change and creates F). Existing netplan files
# (e.g. 50-cloud-init.yaml) are never modified: ours are 60-arm-*.yaml and 61-arm-*.yaml.
set -euo pipefail

DIRECT_ADDRESS="10.42.0.2/24"   # fixed address on the direct cable (the computer is 10.42.0.1)
NETPLAN="${ARM_NETPLAN_DIR:-/etc/netplan}"   # ARM_NETPLAN_DIR: only for the tests
CONFIRM_FILE=""                  # set by --confirm-file: non-interactive confirmation (Arm Studio)
PASSWORD_FILE=""                 # set by --password-file: WiFi password without a prompt
CONFIRM_TIMEOUT="${ARM_CONFIRM_TIMEOUT:-120}"
BACKUP=""

need_root() { [ "$(id -u)" -eq 0 ] || [ -n "${ARM_NETPLAN_DIR:-}" ] || { echo "Τρέξε το με sudo." >&2; exit 1; }; }

snapshot() {   # our files as they are now, to come back to
    BACKUP=$(mktemp -d)
    cp -a "$NETPLAN"/60-arm-*.yaml "$NETPLAN"/61-arm-*.yaml "$BACKUP"/ 2>/dev/null || true
}

rollback() {
    rm -f "$NETPLAN"/60-arm-*.yaml "$NETPLAN"/61-arm-*.yaml
    cp -a "$BACKUP"/. "$NETPLAN"/ 2>/dev/null || true
    netplan apply
}

apply_safely() {
    if ! netplan generate; then   # syntax check before anything changes
        rollback
        echo "FAILED: netplan generate"
        exit 1
    fi
    if [ -z "$CONFIRM_FILE" ]; then
        echo
        echo "Εφαρμογή με 'netplan try': αν η σύνδεση συνεχίζει να δουλεύει, πάτα ENTER."
        echo "Αν χαθεί, οι προηγούμενες ρυθμίσεις επανέρχονται μόνες τους σε $CONFIRM_TIMEOUT s."
        netplan try --timeout "$CONFIRM_TIMEOUT"
        return
    fi
    rm -f "$CONFIRM_FILE"
    netplan apply
    echo "APPLIED: waiting ${CONFIRM_TIMEOUT}s for $CONFIRM_FILE"
    for _ in $(seq "$CONFIRM_TIMEOUT"); do
        if [ -e "$CONFIRM_FILE" ]; then
            rm -f "$CONFIRM_FILE"
            echo "OK: confirmed"
            return
        fi
        sleep 1
    done
    rollback
    echo "ROLLED BACK: no confirmation in ${CONFIRM_TIMEOUT}s"
    exit 1
}

ethernet_iface() {
    for dev in /sys/class/net/*; do
        name=$(basename "$dev")
        [ -d "$dev/device" ] && [ ! -d "$dev/wireless" ] && { echo "$name"; return; }
    done
    echo eth0
}

wifi_iface() {
    for dev in /sys/class/net/*; do
        [ -d "$dev/wireless" ] && { basename "$dev"; return; }
    done
    echo wlan0
}

cmd_base() {
    need_root
    local host="${1:-armpi}" eth
    eth=$(ethernet_iface)
    snapshot
    hostnamectl set-hostname "$host"
    grep -q "127.0.1.1" /etc/hosts && sed -i "s/^127\.0\.1\.1.*/127.0.1.1 $host/" /etc/hosts \
        || echo "127.0.1.1 $host" >> /etc/hosts

    # mDNS (Avahi): the Pi announces itself as <host>.local on every network it joins.
    if ! dpkg -s avahi-daemon >/dev/null 2>&1; then
        apt-get update && apt-get install -y avahi-daemon \
            || echo "⚠ Χωρίς internet: το avahi-daemon θα εγκατασταθεί αργότερα (ξανατρέξε το base)."
    fi
    systemctl enable --now avahi-daemon 2>/dev/null || true

    # Cable: DHCP when plugged into a router, plus a fixed address for the direct cable to the computer.
    cat > "$NETPLAN/60-arm-ethernet.yaml" <<YAML
# ADR-0021: Ethernet of the arm's Raspberry Pi (written by software/pi/setup_network.sh)
network:
  version: 2
  ethernets:
    $eth:
      dhcp4: true
      optional: true
      addresses: [$DIRECT_ADDRESS]
YAML
    chmod 600 "$NETPLAN/60-arm-ethernet.yaml"
    echo "Όνομα: $host.local · καλώδιο ($eth): DHCP + ${DIRECT_ADDRESS%/*}"
    apply_safely
}

cmd_add_wifi() {
    need_root
    local ssid="${1:?Δώσε το όνομα του WiFi: add-wifi \"SSID\"}" pass wlan file
    if [ -n "$PASSWORD_FILE" ]; then
        pass=$(cat "$PASSWORD_FILE")
        rm -f "$PASSWORD_FILE"
    else
        read -r -s -p "Κωδικός για «$ssid»: " pass
        echo
    fi
    case "$pass$ssid" in *'"'*|*\\*) echo "Τα σύμβολα \" και \\ δεν υποστηρίζονται στο όνομα/κωδικό." >&2; exit 1 ;; esac
    wlan=$(wifi_iface)
    snapshot
    file="$NETPLAN/61-arm-wifi-$(echo -n "$ssid" | md5sum | cut -c1-8).yaml"
    # Each network in its own file; netplan merges them, so the networks already known stay.
    cat > "$file" <<YAML
# ADR-0021: WiFi "$ssid" (written by software/pi/setup_network.sh)
network:
  version: 2
  wifis:
    $wlan:
      dhcp4: true
      optional: true
      access-points:
        "$ssid":
          password: "$pass"
YAML
    chmod 600 "$file"   # contains the password
    echo "Προστέθηκε το «$ssid» ($file)."
    apply_safely
}

cmd_show() {
    echo "Όνομα: $(hostname).local"
    ip -4 -brief address
    echo
    echo "Γνωστά WiFi (αρχεία του setup_network.sh):"
    grep -h -A0 '^        "' "$NETPLAN"/61-arm-wifi-*.yaml 2>/dev/null | tr -d ' ":' || echo "  κανένα"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --confirm-file) CONFIRM_FILE="$2"; shift 2 ;;
        --password-file) PASSWORD_FILE="$2"; shift 2 ;;
        *) break ;;
    esac
done

case "${1:-}" in
    base) shift; cmd_base "$@" ;;
    add-wifi) shift; cmd_add_wifi "$@" ;;
    show) cmd_show ;;
    *) sed -n '2,14p' "$0"; exit 1 ;;
esac
