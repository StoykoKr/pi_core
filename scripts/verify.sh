#!/usr/bin/env bash
# ============================================================================
# Post-reboot health check — run after install.sh + reboot.
# Prints PASS/FAIL per subsystem; failures are expected for hardware that
# is not plugged in yet (profiles off) — those lines say SKIP.
# ============================================================================
set -uo pipefail
ok()   { echo "  [PASS] $1"; }
bad()  { echo "  [FAIL] $1"; }
skip() { echo "  [SKIP] $1"; }

echo "── Docker ──────────────────────────────────────────────"
docker info >/dev/null 2>&1 && ok "docker daemon reachable (as $USER)" \
    || bad "docker unreachable — did you re-login/reboot after install? (docker group)"

echo "── Stack ───────────────────────────────────────────────"
systemctl is-enabled robot-stack.service >/dev/null 2>&1 \
    && ok "robot-stack.service enabled" || bad "robot-stack.service not enabled"
for c in mqtt_broker signing_server provisioning ros2_robot_app; do
    if [ "$(docker inspect -f '{{.State.Running}}' "$c" 2>/dev/null)" = "true" ]; then
        ok "container $c running"
    else
        bad "container $c NOT running (docker logs $c)"
    fi
done

echo "── Hailo NPU ───────────────────────────────────────────"
if command -v hailortcli >/dev/null 2>&1; then
    hailortcli fw-control identify >/dev/null 2>&1 \
        && ok "Hailo responds (hailortcli fw-control identify)" \
        || bad "hailortcli present but device not responding — check HAT seating / dmesg | grep hailo"
else
    skip "hailortcli not installed (hailo-all failed or pending decision)"
fi

echo "── Access Point ────────────────────────────────────────"
nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null | grep -q re4life-ap \
    && ok "hotspot re4life-ap active" || bad "hotspot not active (nmcli connection up re4life-ap)"

echo "── Certificates ────────────────────────────────────────"
[ -f /robot_data/certs/root_ca.crt ] && ok "root CA present" || bad "root CA missing (scripts/init-ca.sh)"
curl -sk https://127.0.0.1:8443/health 2>/dev/null | grep -q '"ok"' \
    && ok "signing server /health" || bad "signing server not answering on :8443"

echo "── ESP32 serial symlinks (profile-dependent) ───────────"
for d in /dev/esp_drive /dev/esp_sensors; do
    [ -e "$d" ] && ok "$d present" || skip "$d absent (ESP unplugged or udev rule placeholders unfilled)"
done

echo "── ROS graph (inside the app container) ────────────────"
docker exec ros2_robot_app bash -c "source /opt/ros/jazzy/setup.bash && timeout 10 ros2 topic list" 2>/dev/null \
    && ok "ros2 CLI alive in container" || bad "ros2 CLI failed in ros2_robot_app"

echo "Done."
