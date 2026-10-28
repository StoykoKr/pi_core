#!/usr/bin/env bash
# ============================================================================
# BASIC CONNECTION TEST — Pi + esp_drive + esp_sensors + CYD screen.
# No camera, no lidar, no mobile app required (phone optional at the end).
#
# Prerequisites:
#   * install.sh done + reboot; stack running (scripts/verify.sh green-ish)
#   * both ESPs flashed with firmware/esp_drive + firmware/esp_sensors (v1)
#   * udev TODO ports filled -> /dev/esp_drive and /dev/esp_sensors exist
#   * docker/.env: COMPOSE_PROFILES=drive,sensors  (then restart the stack:
#     sudo systemctl restart robot-stack)
#   * !!! DRIVE WHEELS OFF THE GROUND for the motion steps !!!
#
# Interactive steps: press Enter to run, or type s+Enter to skip.
# Exit code: number of FAILed automatic checks (0 = all passed).
# ============================================================================
set -uo pipefail
FAILS=0
R() { docker exec ros2_robot_app bash -c "source /opt/ros/jazzy/setup.bash && $1"; }
ok()  { echo "  [PASS] $1"; }
bad() { echo "  [FAIL] $1"; FAILS=$((FAILS+1)); }
hr()  { echo "────────────────────────────────────────────────────────"; }
ask() {  # ask "prompt" -> returns 0 to run, 1 to skip
  echo "$1  [Enter = run / s = skip]"
  read -r a || true
  [ "${a:-}" != "s" ]
}
# ros2 topic pub blocks forever waiting for a matching subscription when the
# ESP side is down — exactly the case this script diagnoses — so every pub is
# wrapped in a hard timeout.
PUB() { R "timeout $1 ros2 topic pub $2" >/dev/null 2>&1; }

hr; echo "BENCH MODE: are the VESC motor controllers POWERED for this run?"
echo "  (first connection test = Pi + ESPs + screen only, VESCs off, zero motion risk)"
echo "  [y = VESCs powered / anything else = bench mode, VESC checks skipped]"
read -r VESC_ANSWER || true
VESC_ON=false; [ "${VESC_ANSWER:-}" = "y" ] && VESC_ON=true

hr; echo "1. Device symlinks + agent containers"
[ -e /dev/esp_drive ]   && ok "/dev/esp_drive"   || bad "/dev/esp_drive missing (udev TODO ports? ESP unplugged?)"
[ -e /dev/esp_sensors ] && ok "/dev/esp_sensors" || bad "/dev/esp_sensors missing (udev TODO ports? ESP unplugged?)"
for c in micro_ros_agent_drive micro_ros_agent_sensors; do
  [ "$(docker inspect -f '{{.State.Running}}' $c 2>/dev/null)" = "true" ] \
    && ok "agent container $c running" \
    || bad "$c not running (COMPOSE_PROFILES=drive,sensors set? docker logs $c)"
done
echo "   (ESP onboard LEDs should be SOLID = agent connected; blinking = still searching)"

hr; echo "2. ESP nodes visible in the ROS graph"
R "timeout 10 ros2 node list" 2>/dev/null | tee /tmp/ct_nodes
grep -q esp_drive   /tmp/ct_nodes && ok "/esp_drive node"   || bad "/esp_drive missing"
grep -q esp_sensors /tmp/ct_nodes && ok "/esp_sensors node" || bad "/esp_sensors missing"

hr; echo "3. Sensor data flowing"
if $VESC_ON; then
  R "timeout 8 ros2 topic echo --once /wheel_telemetry" >/dev/null 2>&1 \
    && ok "/wheel_telemetry (JointState from VESC tachometers)" \
    || bad "/wheel_telemetry silent — firmware only publishes it once a VESC has answered"
  R "timeout 8 ros2 topic echo --once /battery_voltage" >/dev/null 2>&1 \
    && ok "/battery_voltage" || bad "/battery_voltage silent (published only after a successful VESC read)"
else
  echo "  [SKIP] /wheel_telemetry + /battery_voltage — EXPECTED silent with VESCs off"
  echo "         (by design: the firmware never publishes zeros; silence = no VESC link."
  echo "          Note: unpowered VESCs also make esp_drive's loop sluggish — ~100 ms"
  echo "          blocking read per VESC per cycle. Expected on the bench, gone when powered.)"
fi
R "timeout 8 ros2 topic echo --once /ultrasonic/mid" >/dev/null 2>&1 \
  && ok "/ultrasonic/mid (Range)" || bad "/ultrasonic/mid silent"

hr
if ! $VESC_ON; then
  echo "4–5. MOTION + E-STOP — auto-skipped (VESCs off). Re-run with powered"
  echo "     VESCs and wheels OFF THE GROUND when the drive test is due."
elif ask "4. MOTION — wheels OFF THE GROUND. Spins the wheels slowly for 2 s, then the publisher dies: the wheels must HARD-STOP within ~0.5 s (that stop IS the watchdog test)."; then
  PUB 25 "-r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'"
  echo "   Did the wheels turn opposite ways, then stop at once when publishing ended?"
  echo "   If they kept spinning: STOP EVERYTHING — the watchdog is not working."
fi

hr
if $VESC_ON && ask "5. E-STOP latch: asserts /e_stop, then commands motion — NOTHING should move (brake engaged). Then releases: robot must stay idle (release != resume)."; then
  PUB 6 "--once /e_stop std_msgs/msg/Bool '{data: true}'"
  PUB 12 "-r 10 -t 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'"
  echo "   Wheels stayed still + resisted turning by hand (brake)? good."
  PUB 6 "--once /e_stop std_msgs/msg/Bool '{data: false}'"
  echo "   Released. Robot idle until a fresh /cmd_vel — confirm nothing moved."
fi

hr
if ask "6. QR chain. Preferred: touch 'GET QR CODE' on the CYD and watch a QR render (~3 s). This step instead SIMULATES the touch from the Pi side and checks the payload."; then
  # Listener starts BEFORE the request: /ui/qr_payload is deliberately
  # non-latched, so a fast generation would otherwise beat the subscription.
  R "timeout 20 ros2 topic echo --once /ui/qr_payload" >/tmp/ct_qr 2>/dev/null &
  LPID=$!
  sleep 3
  PUB 6 "--once /ui/request_qr std_msgs/msg/Empty '{}'"
  wait $LPID && ok "QR payload produced (provisioning + signing + token chain alive)" \
             || bad "no /ui/qr_payload in 20 s (docker logs provisioning / signing_server)"
  echo "   If the CYD is wired to esp_sensors, the same payload should have rendered on it."
  echo "   Phone test: scan -> join AP -> app provisions -> mTLS connect."
fi

hr; echo "Done. Automatic FAILs: $FAILS"
echo "Failures in 2–3 are usually: agent on the wrong device, firmware not"
echo "flashed, or RMW interop — check 'docker logs micro_ros_agent_drive'."
exit $FAILS
