#!/usr/bin/env bash
# ============================================================================
# WiFi Access Point via NetworkManager (Pi OS Bookworm's native stack — no
# hostapd daemon). Reads the SSID/passphrase from the hostapd-FORMAT
# credentials file so qr_generator.py can read the very same file unchanged.
#
# Usage: setup_ap.sh <path/to/ap_credentials.conf>
#
# NOTE: this turns wlan0 into an AP — a wifi-based ssh session dies here.
# Use the ethernet cable (the intended workflow).
# ============================================================================
set -euo pipefail

AP_CONF="${1:?usage: setup_ap.sh <ap_credentials.conf>}"
AP_IFACE="${RE4LIFE_AP_IFACE:-wlan0}"
AP_IP="${RE4LIFE_AP_IP:-192.168.4.1}"
CON_NAME="re4life-ap"

# '|| true' inside the substitutions: under set -e, grep's exit-1 on a missing
# line would kill the script HERE instead of reaching the friendly error below.
SSID=$(grep -E '^ssid=' "$AP_CONF" | head -1 | cut -d= -f2- || true)
PASS=$(grep -E '^wpa_passphrase=' "$AP_CONF" | head -1 | cut -d= -f2- || true)

if [ -z "$SSID" ] || [ -z "$PASS" ]; then
    echo "[-] $AP_CONF must contain ssid= and wpa_passphrase= lines."
    exit 1
fi
if [ "${#PASS}" -lt 8 ]; then
    echo "[-] WPA2 passphrase must be at least 8 characters."
    exit 1
fi

# Make sure NetworkManager actually manages wlan0 (it does by default on
# Bookworm; this guards against leftover dhcpcd/wpa_supplicant configs).
sudo systemctl enable --now NetworkManager

# Recreate the hotspot connection from scratch (idempotent)
sudo nmcli connection delete "$CON_NAME" >/dev/null 2>&1 || true
sudo nmcli connection add type wifi ifname "$AP_IFACE" con-name "$CON_NAME" \
    autoconnect yes ssid "$SSID" \
    802-11-wireless.mode ap 802-11-wireless.band bg \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$PASS" \
    ipv4.method shared ipv4.addresses "$AP_IP/24" \
    ipv6.method disabled

# 'shared' method = NetworkManager runs its own DHCP for clients. Phones that
# join get an address in the AP subnet and can reach $AP_IP:8883/:8443/:8080.
sudo nmcli connection up "$CON_NAME" || \
    echo "[-] Hotspot will come up on reboot (wlan may be busy right now)."

echo "[+] AP configured: SSID='$SSID' on $AP_IFACE at $AP_IP"
