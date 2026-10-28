# firmware/ — ESP32 + CYD firmware

| Dir | Board | Status |
|---|---|---|
| `esp_drive/` | ESP32 DevKitC | **micro-ROS port v1** — flash this. VESC drive via /cmd_vel (IK onboard), JointState telemetry, **watchdog 500 ms**, /e_stop active brake |
| `esp_sensors/` | ESP32 DevKitC | **micro-ROS port v1** — flash this. Ultrasonic Range topics, CYD UART passthrough (`/ui/qr_payload` ↔ `QR:`/`<REQ_QR>`) |
| `screen/` | CYD (ESP32-2432S028R) | current (non-ROS by design). `User_Setup_CYD.h` = the hard-won TFT_eSPI config (inversion OFF, BGR) — without it: negative colors, swapped red/blue |
| `legacy/` | — | the old ASCII-protocol firmwares + ESP32-CAM experiment, reference only |

## v1 scope (deliberate)

v1 uses **standard ROS messages only**, so it builds against the stock
micro_ros_arduino library — no library rebuild needed before the first
connection test. Consequences:

- `/motor_status` (robot_interfaces/MotorStatus) is replaced by a TEMPORARY
  `/battery_voltage` (std_msgs/Float32) — display value only.
- `/servo_cmd`, `/servo_state` (arm — deferred anyway) and `/actuator_cmd`
  are NOT subscribed; servos hold their start positions, relays stay idle
  (their driver classes are compiled in, ready).
- Timestamps are zero (no agent time-sync yet); base_controller stamps on
  arrival. Fine until ultrasonics feed collision_monitor.

**v2 (when the arm/actuators/MotorStatus are needed):** rebuild the
micro_ros_arduino static library with `src/robot_interfaces` included
(docker-based builder: clone micro_ros_arduino @ jazzy, drop robot_interfaces
into `extras/library_generation/extra_packages/`, run
`docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:jazzy`
— check the repo's current README for the exact invocation, it shifts between
releases), then swap the temporary topics for the frozen contract messages.
Add `rmw_uros_sync_session()` for real stamps at the same time.

## Build & flash (Arduino IDE)

1. Boards manager: **esp32 by Espressif**; board "ESP32 Dev Module".
2. Library: **micro_ros_arduino** — install the newest **jazzy**-tagged
   release from https://github.com/micro-ROS/micro_ros_arduino/releases
   (tags look like `vX.Y.Z-jazzy`). The distro MUST match the agent images.
3. esp_drive also needs: **VescUart**, **ESP32Servo** (same as legacy).
4. Flash `esp_drive/esp_drive.ino` to the VESC board,
   `esp_sensors/esp_sensors.ino` to the sensor board.
5. **USB Serial is the micro-ROS transport** — the Arduino serial monitor is
   useless (and harmful) once flashed. Status = onboard LED:
   slow blink = waiting for agent, solid = agent connected.

The CYD (`screen/screen.ino`) still builds with TFT_eSPI (+ `User_Setup_CYD.h`
contents into TFT_eSPI's `User_Setup.h`), TJpg_Decoder, XPT2046_Touchscreen;
QR lib is vendored (`qrcode.c/h`). Before flashing set the WiFi creds
(CHANGE_ME placeholders) to the values in the Pi's
`config/ap_credentials.conf` — creds→NVS is a later tweak, and the stream URL
switch to web_video_server only matters once a camera exists.

## Safety behaviors burned into esp_drive (contract §3b)

| Trigger | Response |
|---|---|
| no /cmd_vel for 500 ms | IMMEDIATE zero current — hard stop, no ramp-down (the legacy firmware had the watchdog COMMENTED OUT — mandatory now) |
| /e_stop true | active brake (8 A), /cmd_vel ignored until false; release ≠ resume; brake stays asserted even through an agent disconnect |
| agent/Pi death | entities torn down + motors zeroed (or braked if e-stopped), reconnects automatically |
| dead VESC link | /wheel_telemetry + /battery_voltage go SILENT (never publish zeros) — silence is the honest failure signal; note getVescValues blocks ~100 ms per failed read, so a powered-down VESC makes the loop sluggish on the bench — expected |
| mechanical constants | wheel Ø 165 mm, 15 pole pairs, right-motor sign flip (HERE only), **WHEEL_TRACK_M = 0.50 PLACEHOLDER — measure, and keep equal to base_controller's wheel_track param** |
