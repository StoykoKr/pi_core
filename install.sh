#!/usr/bin/env bash
# ============================================================================
# Re4Life robot brain — master installer for Raspberry Pi 5 + Hailo AI HAT
# ----------------------------------------------------------------------------
# Intended workflow (fresh Pi OS Lite 64-bit, headless):
#   plug ethernet -> ssh in -> git clone <repo> -> cd pi_core -> bash install.sh
#   -> reboot -> stack starts on its own (systemd).
#
# RUN OVER ETHERNET, NOT WIFI SSH: this script reconfigures wlan0 into an
# Access Point, which kills any wifi-based ssh session mid-install.
#
# Run as the normal user (NOT with sudo in front — the script sudo's where
# needed; it must know the real user for docker group + paths).
#
# TWO-PHASE DESIGN: everything is installed and the app image is BUILT in this
# run, but the stack is NOT started. /dev/hailo0 only exists after the
# post-driver reboot, so the systemd unit brings the stack up on boot instead.
# ============================================================================
set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
    echo "[-] Run as the normal user, not root/sudo (the script sudo's internally)."
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"
echo "[+] Repo: $REPO_DIR"

# ── 1. Base system ──────────────────────────────────────────────────────────
echo "[+] 1/9 System packages..."
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y git curl ca-certificates openssl chrony network-manager

# ── 2. Docker (official convenience script — Debian's docker.io/compose
#      packaging on Pi OS is inconsistent across releases) ──────────────────
echo "[+] 2/9 Docker..."
if ! command -v docker >/dev/null 2>&1; then
    curl -fsSL https://get.docker.com | sudo sh
fi
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
echo "    (docker group takes effect at next login — the reboot covers it)"

# ── 3. Hailo host driver ────────────────────────────────────────────────────
# VERIFY-ON-HARDWARE ITEM: 'hailo-all' is the official Raspberry Pi OS package
# for the AI HAT family. Confirm it covers the Hailo-10H on this image; after
# reboot run:  hailortcli fw-control identify
# The HailoRT version installed here must later match the HEF compile version.
echo "[+] 3/9 Hailo driver..."
if sudo apt-get install -y dkms hailo-all; then
    echo "    hailo-all installed (active after reboot)."
else
    echo "[-] WARNING: hailo-all install failed — NPU disabled for now."
    echo "    Stack still runs (depth inference uses the loud-fail stub)."
fi

# ── 4. udev rules (ESP32 serial mapping) ────────────────────────────────────
# Default variant: classic ESP32 DevKitC mapped BY PHYSICAL USB PORT.
# The rules file ships with TODO placeholders — see config/99-robot-hardware.rules
# for the one-time procedure to fill in this Pi's actual port paths.
echo "[+] 4/9 udev rules..."
sudo cp "$REPO_DIR/config/99-robot-hardware.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# ── 5. Persistent data dirs ─────────────────────────────────────────────────
# Deliberately NOT a recursive chown of all of /robot_data: the broker key in
# certs/ must stay owned by uid 1883 (mosquitto) — a blanket chown on a re-run
# would silently revert it and crash-loop the broker. init-ca.sh owns the
# certs/ permission story (and re-asserts it on every run).
echo "[+] 5/9 /robot_data..."
sudo mkdir -p /robot_data/certs /robot_data/maps
sudo chown "$USER":"$USER" /robot_data /robot_data/certs
sudo chown -R "$USER":"$USER" /robot_data/maps

# ── 6. WiFi Access Point credentials + hotspot ──────────────────────────────
# Single source of truth: config/ap_credentials.conf (hostapd key=value format).
# NetworkManager creates the actual hotspot FROM it; the provisioning container
# reads the SAME file (RE4LIFE_HOSTAPD) so the QR always matches reality.
echo "[+] 6/9 Access Point..."
AP_CONF="$REPO_DIR/config/ap_credentials.conf"
if [ ! -f "$AP_CONF" ]; then
    PASS="$(openssl rand -base64 12 | tr -dc 'a-zA-Z0-9' | head -c 16)"
    sed "s/^wpa_passphrase=.*/wpa_passphrase=$PASS/" \
        "$REPO_DIR/config/ap_credentials.conf.example" > "$AP_CONF"
    echo "    Generated new AP credentials (gitignored): $AP_CONF"
fi
bash "$REPO_DIR/scripts/setup_ap.sh" "$AP_CONF"

# ── 7. Certificates (one-time trust-root ceremony) ──────────────────────────
echo "[+] 7/9 Root CA + broker cert..."
bash "$REPO_DIR/scripts/init-ca.sh"

# ── 8. systemd unit: stack up on every boot ─────────────────────────────────
echo "[+] 8/9 systemd unit..."
sed "s|__REPO_DIR__|$REPO_DIR|g" "$REPO_DIR/systemd/robot-stack.service.template" \
    | sudo tee /etc/systemd/system/robot-stack.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable robot-stack.service
echo "    Enabled (starts after reboot; 'sudo systemctl start robot-stack' to force)."

# ── 9. Build images now so the first boot is fast ───────────────────────────
# Pull is restricted to the genuinely-remote images BY NAME: the image-reuse
# services (provisioning, web_video_server share the locally built ros2_app
# image) have no build: key, so a blanket pull would try Docker Hub for
# re4life/ros2_app and fail — and an '|| true' there would also swallow REAL
# pull failures. Named pulls fail loudly, as they should.
echo "[+] 9/9 Pulling + building container images (first build: 30-60 min on a Pi)..."
cd "$REPO_DIR/docker"
sudo docker compose --profile '*' pull mosquitto agent_drive agent_sensors
sudo docker compose --profile '*' build

echo
echo "============================================================"
echo "  Install complete. REBOOT NOW:  sudo reboot"
echo "  After reboot, verify with:     bash scripts/verify.sh"
echo "  Enable hardware services in:   docker/.env (COMPOSE_PROFILES)"
echo "============================================================"
