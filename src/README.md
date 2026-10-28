# src/ — the ROS2 workspace

Volume-mounted into the `ros2_app` container as `/robot_ws/src`. Build inside
the container with `scripts/build_ws.sh` (or `colcon build --symlink-install`
in `/robot_ws`). Python node edits need no rebuild; new files/packages do.

| Package | Status |
|---|---|
| `robot_bringup/` | **imported from the laptop workspace** — launch, nav2/slam params, BT xml, spatial_projector + camera_memory (+ the 57 MB scdepthv3.onnx for the CPU backend). Still the SIM configuration: `robot.launch.py` assumes O3DE topics/clock and is NOT used in the basic connection test. The Pi variant (real lidar/camera, `use_sim_time:=false`, twist_mux wiring) comes with Stage 3/6 work. |
| `robot_interfaces/` | **done** — the three frozen contract messages (`MotorStatus`, `ServoCommand`, `ActuatorCommand`). The Pi builds them now; firmware adopts them in v2 (see `firmware/README.md`). |
| `base_controller/` | **done** — `/wheel_telemetry` (JointState) → `/odom` + odom→base_link TF. Run manually: `ros2 run base_controller base_controller`. ⚠ `wheel_track` param = 0.50 placeholder, must match firmware's `WHEEL_TRACK_M` — measure both from the same tape measure. |
| `robot_bringup/config/twist_mux.yaml` | config committed, **not yet wired into the launch** (Stage 3: add the node + flip collision_monitor's `cmd_vel_in_topic`). |
| `mqtt_bridge/` | **NOT WRITTEN** — Phase A Stage 2, the biggest remaining software item (app ↔ ROS translation per contract §7–8). |

`cyclonedds.xml` is referenced by every ROS container via `CYCLONEDDS_URI`
(commented unicast-peers block inside for two-machine testing).

Note: `scdepthv3.onnx` (57 MB) is committed for the CPU plumbing backend —
over GitHub's 50 MB *warning* threshold, under the 100 MB hard limit. If the
repo should stay lighter, move it out and fetch like the HEF (`models/`).
