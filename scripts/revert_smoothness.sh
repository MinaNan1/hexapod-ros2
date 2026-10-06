#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# revert_smoothness.sh — undo the smoothness tweaks applied to the gait code.
#
# Restores the .bak copies of:
#   - hexapod_kinematics.py     (soft-landing Z curve change)
#   - hiwonder_servo_bridge.py  (MAX_SPEED_DEG_PER_SEC bump)
# in BOTH the kineamtics reference folder and the ws/src package that ROS runs.
#
# Run on the Jetson host:
#     bash ~/hexapod-ros2/scripts/revert_smoothness.sh
# Then rebuild + relaunch with bringup_servos.sh.
# ─────────────────────────────────────────────────────────────────────────────
set -u

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PAIRS=(
    "$ROOT_DIR/hexapod_ws/src/hexapod_control/hexapod_control/hexapod_kinematics.py"
    "$ROOT_DIR/hexapod_ws/src/hexapod_control/hexapod_control/hiwonder_servo_bridge.py"
    "$ROOT_DIR/hexapod_new/kineamtics/hexapod_kinematics.py"
    "$ROOT_DIR/hexapod_new/kineamtics/hiwonder_servo_bridge.py"
)

missing=0
for f in "${PAIRS[@]}"; do
    if [ ! -f "$f.bak" ]; then
        echo "MISSING backup: $f.bak"
        missing=1
    fi
done
[ $missing -ne 0 ] && { echo "Aborting — at least one .bak is missing."; exit 1; }

for f in "${PAIRS[@]}"; do
    cp -f "$f.bak" "$f"
    echo "  restored $f"
done

echo
echo "Done. Re-launch the robot to pick up the reverted code:"
echo "  sudo bash $ROOT_DIR/scripts/bringup_servos.sh"
