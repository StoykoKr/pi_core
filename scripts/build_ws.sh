#!/usr/bin/env bash
# Build the ROS workspace inside the running ros2_app container.
# NOTE: build artifacts live in the container's own filesystem — a container
# RECREATE (compose down/up, image rebuild) wipes them; re-run this script.
# Run after changing src/ packages (python node edits need no rebuild thanks
# to --symlink-install; new packages/files do).
set -euo pipefail
docker exec ros2_robot_app bash -c "
  source /opt/ros/jazzy/setup.bash &&
  cd /robot_ws &&
  colcon build --symlink-install &&
  echo '[+] workspace built — new shells inside the container pick it up automatically'
"
