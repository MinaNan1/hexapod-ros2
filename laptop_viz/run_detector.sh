#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# run_detector.sh — launch Phase 4 YOLO detection on the LAPTOP.
# Sets the ROS env to match the Jetson, then runs object_detector.py which
# subscribes to the robot's camera, runs YOLOv8 on the laptop GPU, and shows
# a live boxes window + publishes /detection/image/compressed + /detections.
#
# Prereqs: bash setup_detector.sh (once); robot stack running (launch_all.sh).
# ─────────────────────────────────────────────────────────────────────────────
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Optional model arg: bash run_detector.sh yolo11x.pt   (max accuracy)
# Default (no arg) uses the accurate yolo11m. Options, least→most accurate:
#   yolo11n.pt  yolo11s.pt  yolo11m.pt  yolo11l.pt  yolo11x.pt
if [ -n "${1:-}" ]; then
    export HEXAPOD_YOLO_MODEL="$1"
fi
echo "==> YOLO model: ${HEXAPOD_YOLO_MODEL:-yolo11m.pt (default)}"

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash

echo "==> Checking the laptop sees the camera feed…"
if ! ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw/compressed"; then
    echo "  ⚠ Can't see /camera/color/image_raw/compressed."
    echo "    Make sure the robot stack is up (launch_all.sh) and this laptop is"
    echo "    on the same network/domain. Opening anyway; it'll start when the"
    echo "    topic appears."
fi

python3 "$SCRIPT_DIR/object_detector.py"
