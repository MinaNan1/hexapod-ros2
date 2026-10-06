#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# view_3d.sh — launch RViz on the LAPTOP with the 3D-mapping layout:
#   RobotModel + LaserScan + 2D Map + 3D MapCloud + trajectory + camera image.
#
# Prereqs (once):  bash setup_laptop_viz.sh
# Prereqs (every session): the Jetson stack is running (launch_all.sh) and this
# laptop is on the same network with ROS_DOMAIN_ID=42 + rmw_fastrtps_cpp.
# ─────────────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/hexapod_viz_ws"
JETSON="${JETSON:-jetson@192.168.0.20}"   # set to <user>@<your Jetson IP>
RVIZ_CFG="$SCRIPT_DIR/hexapod_3d_mapping.rviz"

# ── Pull the canonical RViz layout from the Jetson so the laptop copy can't
#    drift stale (the #1 cause of "I have to add MapCloud manually"). The Jetson
#    copy is the source of truth. Skip with SKIP_RVIZ_SYNC=1 (e.g. offline), and
#    it falls back to whatever local copy exists. A 5s connect timeout means a
#    missing Jetson doesn't hang the launch.
if [ "${SKIP_RVIZ_SYNC:-0}" != "1" ]; then
    echo "==> Syncing RViz layout from $JETSON ..."
    if scp -o ConnectTimeout=5 -o BatchMode=no \
        "$JETSON:~/hexapod-ros2/laptop_viz/hexapod_3d_mapping.rviz" \
        "$RVIZ_CFG" 2>/dev/null; then
        echo "    ✓ layout up to date"
    else
        echo "    ⚠ couldn't reach $JETSON — using local copy"
    fi
fi

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" || {
    echo "WARNING: $WS not built yet — run setup_laptop_viz.sh first."
    echo "         RViz will still open but the robot model won't render."
}

echo "==> Checking the laptop can see the Jetson's topics..."
if ! ros2 topic list 2>/dev/null | grep -q "/rtabmap/map"; then
    echo ""
    echo "  ⚠ Cannot see /rtabmap/map. Either the Jetson stack isn't running,"
    echo "    or DDS multicast is blocked on this WiFi. If ping works but topics"
    echo "    don't, try discovery-server mode:"
    echo "       export ROS_DISCOVERY_SERVER=192.168.0.20:11811"
    echo "    (and start the server on the Jetson — ask Claude to wire it in)."
    echo ""
    echo "  Opening RViz anyway; displays will populate if/when topics appear."
fi

ros2 run rviz2 rviz2 -d "$RVIZ_CFG"
