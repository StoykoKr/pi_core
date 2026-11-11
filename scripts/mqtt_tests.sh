#!/usr/bin/env bash
# ============================================================================
# MQTT regression harness — exercises the bridge the way the mobile app will,
# independent of the app (contract Stage 2 deliverable). Uses the broker
# container's own mosquitto clients with the bridge's client cert over mTLS.
#
#   mqtt_tests.sh watch            # live view of everything robot -> app
#     (note: Ctrl-C leaves a mosquitto_sub inside the broker container —
#      harmless; `docker restart mqtt_broker` clears strays if they pile up)
#   mqtt_tests.sh status           # one-shot: system status (10 s window)
#   mqtt_tests.sh teleop           # 2 s of gentle WASD 'W' at 10 Hz, then stop
#   mqtt_tests.sh goal U V         # send goal in percent coords (e.g. 0.5 0.5)
#   mqtt_tests.sh stop             # stop_autonomy
#   mqtt_tests.sh estop / release  # e-stop on / off
#   mqtt_tests.sh maps             # list maps
#   mqtt_tests.sh qr               # pairing-style check: watch ops+status
#
# Requires: stack up with COMPOSE_PROFILES including 'bridge'.
# ============================================================================
set -uo pipefail
CERTS=/robot_data/certs
AUTH="-h 127.0.0.1 -p 8883 --cafile $CERTS/root_ca.crt --cert $CERTS/bridge_client.crt --key $CERTS/bridge_client.key"
PUB() { docker exec mqtt_broker mosquitto_pub $AUTH -t "$1" -m "$2"; }
SUB() { docker exec mqtt_broker mosquitto_sub $AUTH -v "$@"; }

case "${1:-}" in
  watch)
    echo "robot -> app, live (Ctrl-C to quit). Map image shown as byte count."
    SUB -t 'robot/#' -F '%t  %p' | sed -E 's/(robot\/map\/image  ).{60,}.*/\1<PNG bytes>/' ;;
  status)
    SUB -t 'robot/status/system' -W 10 -C 1 ;;
  teleop)
    echo "Teleop W for 2 s (wheels off the ground!)..."
    for i in $(seq 1 20); do PUB location/commands '{"Command":"W"}'; sleep 0.1; done
    PUB location/commands '{"Command":"N"}'
    echo "sent. Robot must stop within ~0.4 s of the last message anyway (deadman)." ;;
  goal)
    PUB robot/commands/goal "{\"u\": ${2:?u}, \"v\": ${3:?v}}"
    echo "goal sent — watch robot/status/autonomy:"
    SUB -t 'robot/status/autonomy' -W 15 ;;
  stop)    PUB robot/commands/stop_autonomy '{}' ;;
  estop)   PUB robot/commands/emergency_stop '{}' ;;
  release) PUB robot/commands/emergency_stop '{"value": false}' ;;
  maps)
    PUB robot/commands/list_maps '{}'
    SUB -t 'robot/status/maps' -W 5 -C 1 ;;
  qr)
    echo "Watching ops/status for 30 s — touch the CYD button now:"
    SUB -t 'robot/status/#' -W 30 ;;
  *)
    grep -E '^#   mqtt' "$0" | sed 's/^# *//'; exit 1 ;;
esac
