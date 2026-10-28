# pi_core — Re4Life robot brain deployment

One-command setup of the robot's Raspberry Pi 5 (+ Hailo AI HAT) from a blank
**Pi OS Lite 64-bit** image. This repo is **Pi-only** — laptop/sim development
lives in the dev workspace; the interface contract and Phase A plan documents
are the design authority this repo implements.

## Quick start (fresh Pi)

```bash
# 1. Flash Pi OS Lite (64-bit), boot, connect ETHERNET (not WiFi — the
#    installer turns wlan0 into an Access Point and would cut a wifi ssh).
ssh <user>@<pi-address>

# 2. Clone + install (run as the normal user, NOT sudo — it sudo's internally)
git clone <this-repo-url> && cd pi_core
bash install.sh          # ~30-60 min first time (image build on the Pi)

# 3. Reboot — the stack auto-starts via systemd
sudo reboot

# 4. Verify
cd pi_core && bash scripts/verify.sh

# 5. Basic connection test (Pi + 2 flashed ESPs + CYD; wheels OFF the ground):
#    - flash firmware/esp_drive + firmware/esp_sensors (see firmware/README.md)
#    - fill the udev TODO ports (config/99-robot-hardware.rules), replug ESPs
#    - set COMPOSE_PROFILES=drive,sensors in docker/.env,
#      then: sudo systemctl restart robot-stack
bash scripts/connection_test.sh
```

## What install.sh does

| Step | What | Where it lands |
|---|---|---|
| system pkgs | git, curl, chrony, NetworkManager | host |
| Docker | official get.docker.com script | host |
| Hailo driver | `apt install hailo-all` (**verify it covers the 10H** — see below) | host |
| udev rules | ESP32 serial symlinks `/dev/esp_drive`, `/dev/esp_sensors` | host |
| data dirs | `/robot_data/{certs,maps}` | host |
| Access Point | NetworkManager hotspot built from `config/ap_credentials.conf` | host |
| certificates | Root CA + broker cert, SAN pins 192.168.4.1 (one-time; idempotent) | `/robot_data/certs` |
| systemd unit | `robot-stack.service` → `docker compose up -d` on boot | host |
| images | pull agents/mosquitto + build the ROS2 app image | docker |

**Two-phase on purpose:** `/dev/hailo0` only exists after the post-driver
reboot, so the stack starts on boot, never inside install.sh.

## The container stack (docker/)

Core (always on): `mosquitto` (mTLS :8883) · `signing` (:8443, issues client
certs) · `provisioning` (CYD touch → QR payload) · `ros2_app` (Nav2, SLAM,
perception, bridge — as code lands in `src/`).

Behind **profiles** (`docker/.env` → `COMPOSE_PROFILES=drive,sensors,camera`),
so the stack survives absent hardware:
`agent_drive` · `agent_sensors` (micro-ROS agents) · `web_video_server` (:8080).

## One-time hardware mapping (classic ESP32 DevKitC — current default)

Classic DevKitC boards can't set their own USB names, so the two ESPs are
mapped **by physical USB socket**. Fill the `TODO_*_PORT` placeholders in
`config/99-robot-hardware.rules` once per Pi (procedure is in the file), then
re-run steps 4 of install.sh (or just the two udev commands in the file).
Any board pair works in those sockets afterwards — swapping pairs needs nothing.

**Pending decision:** moving to ESP32-S3 (firmware-set USB names) makes the
udev rules unnecessary — the commented S3 variants sit in both the rules file
and the compose file. Decide at firmware-port time.

## Provisioning / pairing flow

CYD touch → `<REQ_QR>` over UART → esp_sensors publishes `/ui/request_qr` →
`provisioning` container subprocesses `qr_generator.py` (unchanged legacy
code) → fetches a single-use token from `signing` → publishes
`/ui/qr_payload` (≤600 B) → esp_sensors forwards `QR:{json}` to the CYD →
phone scans → joins the AP → `/provision` on :8443 → gets its mTLS client
cert → connects to mosquitto :8883. Failures never reach the CYD; they land
on `/ui/provisioning_status` (→ app, via the bridge once it exists).

`config/ap_credentials.conf` (gitignored, generated on first install) is the
single source of truth for the hotspot: NetworkManager builds the AP from it
and `qr_generator.py` reads the same file. Change the passphrase there, re-run
`scripts/setup_ap.sh config/ap_credentials.conf`, done.

## Verify-on-hardware items (do these on first bring-up)

1. **Hailo 10H**: `hailortcli fw-control identify` after reboot. `hailo-all`
   is the official Pi package for the AI HAT family — confirm it actually
   drives the 10H on this OS image; the installed HailoRT version must match
   the version the HEF model gets compiled with later.
2. **web_video_server**: confirm `ros-jazzy-web-video-server` installed in the
   image build (fallback note in `docker/Dockerfile` if the mirror lacks it).
3. **Agents ↔ app RMW interop**: the micro-ROS agent image uses its own DDS;
   the app uses CycloneDDS. Cross-vendor RTPS normally interoperates — verify
   ESP topics show up in `ros2 topic list` inside `ros2_robot_app`.
4. Cooling: official Pi 5 Active Cooler under the HAT;
   `vcgencmd measure_temp` under load, `vcgencmd get_throttled`.

## Current repo status

In and working: deploy fundamentals; provisioning/security subsystem;
`src/robot_bringup` (sim-configured, imported from the laptop),
`src/robot_interfaces` (frozen contract msgs), `src/base_controller`;
**micro-ROS firmware v1** for both ESPs (`firmware/esp_drive`,
`firmware/esp_sensors` — standard msgs only, see `firmware/README.md` for the
deliberate v1/v2 split); CYD firmware unchanged-by-design; the basic
connection test (`scripts/connection_test.sh`).

Not yet: **mqtt_bridge** (Stage 2 — the biggest remaining software item);
twist_mux launch wiring + Pi-variant bringup (Stage 3/6, done against the
laptop sim first per the plan); firmware v2 (contract custom msgs + time
sync); Hailo HEF + HailoRT — the model zoo ships a precompiled
`scdepthv3.hef` for HAILO10H (DFC v5.4.0, NYU, 256×320) → `models/`
(user-added) + HailoRT matching v5.4.0 in the image; lidar/camera driver
configs when the hardware exists.

## Security notes

- Nothing secret is committed: certs/keys/tokens/AP passphrase are generated
  on-device and gitignored.
- The QR (AP passphrase + pairing token) travels **only over the wired UART**
  to the CYD — never over WiFi (WPA2-PSK lets past-paired devices decrypt).
- Containers run privileged/host-net **by design** — Docker here is packaging
  and reproducibility, not isolation. The mTLS boundary is the security layer.
