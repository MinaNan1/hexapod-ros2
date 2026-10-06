#!/bin/bash
# TERMINAL — RTAB-Map 3D SLAM (LIDAR + depth, no color fusion).
#
# Prerequisites in launch order (handled by launch_all.sh):
#   t1_robot.sh       → odom→base_link from hexapod_controller
#   t1_servo_bridge.sh
#   t1_camera.sh      → /camera/color/image_raw, /camera/depth/image_raw
#   t1_lidar.sh       → /scan + base_link→laser static TF
#   t2_rtabmap.sh     → THIS — owns map→odom
#
# TF conflict handling:
# display_headless.launch.py (launched by t1_robot.sh) publishes a static
# world→odom. RTAB-Map will publish map→odom. Two parents on `odom` breaks
# the TF tree. We kill the conflicting publisher here. Restart of t1_robot.sh
# brings it back (for non-mapping development).

# ── Kill the conflicting world→odom static TF ───────────────────────────
pkill -f "__node:=world_to_odom" 2>/dev/null || true
sleep 0.3

# ── Kill any stale RTAB-Map from a previous launch ──────────────────────
# Specific patterns ONLY — naive `pkill -f rtabmap` would kill this script
# (whose command line `bash .../t2_rtabmap.sh` contains "rtabmap").
pkill -f "rtabmap_slam/rtabmap"           2>/dev/null || true
pkill -f "ros2 launch hexapod_gazebo rtabmap" 2>/dev/null || true
sleep 0.5

source /opt/ros/humble/setup.bash
source /hexapodd/hexapod_ws/install/setup.bash
source /hexapodd/ros2_ws/install/local_setup.bash

RTABMAP_LOG=/tmp/rtabmap_launch.log

# ── Fresh map vs resume ──────────────────────────────────────────────────
# Default = FRESH map every launch (-d wipes rtabmap.db). This matches the
# core use case ("hexapod enters a NEW place → builds a NEW map") and avoids
# the relocalization trap: resuming an old DB makes RTAB-Map withhold the
# grid until it matches the prior session's start pose, which looks like a
# hang. The DB still lives on the host at maps/rtabmap.db (bind-mounted via
# start_ros.sh) so it's inspectable/savable after a run.
#
# To RESUME a saved map instead of starting fresh, change the default below
# to :-1 (env vars don't cross the docker exec in launch_all.sh, so editing
# here is the reliable switch):
if [ "${HEXAPOD_RESUME_MAP:-0}" = "1" ]; then
    RTAB_ARGS=""
    echo "[t2_rtabmap.sh] RESUMING saved map (rtabmap.db kept)."
else
    RTAB_ARGS="-d"
    echo "[t2_rtabmap.sh] FRESH map (rtabmap.db wiped). Set HEXAPOD_RESUME_MAP=1 to resume."
fi

# ── Launch RTAB-Map ─────────────────────────────────────────────────────
(
    ros2 launch hexapod_gazebo rtabmap.launch.py rtabmap_args:="$RTAB_ARGS"
) 2>&1 | tee "$RTABMAP_LOG"

echo ""
echo "────────────────────────────────────────────────────────────────"
echo "  RTAB-Map exited. Log: $RTABMAP_LOG"
echo "  This window will stay open — Ctrl-b & to kill it from tmux."
echo "────────────────────────────────────────────────────────────────"
# read -r doesn't work without a TTY inside `docker exec`.
sleep infinity
