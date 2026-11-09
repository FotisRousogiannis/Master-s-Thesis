#!/usr/bin/env bash
# Network of the Raspberry Pi (Ubuntu Server) for the arm (ADR-0021). Run ON THE PI with sudo.
#
#   sudo ./setup_network.sh base [HOSTNAME]   # once: name, mDNS, fixed address on the cable
#   sudo ./setup_network.sh add-wifi "SSID"   # add a WiFi network (asks for the password)
#   sudo ./setup_network.sh show              # addresses and configured networks
#
# Safety: every change is applied with `netplan try`. If the connection is lost and you cannot
# press ENTER within 120 s, the previous network settings come back by themselves.
# Existing netplan files (e.g. 50-cloud-init.yaml) are never modified: ours are 60-arm-*.yaml.
set -euo pipefail

DIRECT_ADDRESS="10.42.0.2/24"   # fixed address on the direct cable (the computer is 10.42.0.1)
NETPLAN=/etc/netplan

need_root() { [ "$(id -u)" -eq 0 ] || { echo "Τρέξε το με sudo." >&2; exit 1; }; }

apply_safely() {
    netplan generate   # syntax check before anything changes
    echo
    echo "Εφαρμογή με 'netplan try': αν η σύνδεση συνεχίζει να δουλεύει, πάτα ENTER."
    echo "Αν χαθεί, οι προηγούμενες ρυθμίσεις επανέρχονται μόνες τους σε 120 s."
    netplan try --timeout 120
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
    read -r -s -p "Κωδικός για «$ssid»: " pass
    echo
    wlan=$(wifi_iface)
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

case "${1:-}" in
    base) shift; cmd_base "$@" ;;
    add-wifi) shift; cmd_add_wifi "$@" ;;
    show) cmd_show ;;
    *) sed -n '2,12p' "$0"; exit 1 ;;
esac
