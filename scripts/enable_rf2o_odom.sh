#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# enable_rf2o_odom.sh — OPT-IN: replace gait dead-reckoning odometry with rf2o
# laser scan-matching odometry (far more accurate → better map + autonomy).
#
# What it does (after launch_all.sh is up):
#   1. Restarts hexapod_controller with publish_odom_tf:=false so it STOPS
#      publishing the open-loop odom→base_link.
#   2. Starts rf2o_laser_odometry, which publishes odom→base_link from /scan
#      by matching consecutive LIDAR scans (measures real motion incl. slip).
#
# RTAB-Map keeps map→odom on top; the chain map→odom→base_link is unchanged in
# shape, just driven by a better source. Reversible: re-run launch_all.sh to go
# back to the gait odom.
#
# Run on the Jetson host:  bash scripts/enable_rf2o_odom.sh
# ─────────────────────────────────────────────────────────────────────────────
set -u
CONTAINER=hexapod_ros
SESSION=hexapod

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container not running — start the stack first (launch_all.sh)."; exit 1
fi
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' not found — run launch_all.sh first."; exit 1
fi

SRC='source /opt/ros/humble/setup.bash && source /hexapodd/hexapod_ws/install/setup.bash && source /hexapodd/ros2_ws/install/local_setup.bash'

echo "==> Stopping the controller's open-loop odom (relaunch with publish_odom_tf:=false)"
docker exec "$CONTAINER" bash -lc "pkill -f 'hexapod_control hexapod_controller' 2>/dev/null; pkill -f hexapod_controller 2>/dev/null; true"
sleep 1
tmux kill-window -t "$SESSION:controller_rf2o" 2>/dev/null || true
tmux new-window -t "$SESSION" -n controller_rf2o \
    "docker exec -it $CONTAINER bash -lc '$SRC && ros2 run hexapod_control hexapod_controller --ros-args -p publish_odom_tf:=false'"
sleep 2

echo "==> Starting rf2o_laser_odometry (scan-matching odom→base_link)"
tmux kill-window -t "$SESSION:rf2o" 2>/dev/null || true
tmux new-window -t "$SESSION" -n rf2o \
    "docker exec -it $CONTAINER bash -lc '$SRC && ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py'"

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "  rf2o scan-matching odometry ENABLED."
echo "  Verify only ONE publisher of odom→base_link:"
echo "    ros2 run tf2_ros tf2_echo odom base_link   (should update as you drive)"
echo "  If the map/TF looks worse, revert: bash scripts/launch_all.sh"
echo "════════════════════════════════════════════════════════════════"
