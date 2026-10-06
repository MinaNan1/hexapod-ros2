#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_laptop_viz.sh — ONE-TIME setup on the Ubuntu LAPTOP so RViz can render
# the hexapod's 3D map + robot model coming from the Jetson over the network.
#
# Why this is needed: RViz renders the robot from /robot_description (published
# by the Jetson), but that URDF references its meshes as
# "package://hexapod_description/meshes/*.STL". RViz must resolve those paths
# LOCALLY — so the laptop needs the hexapod_description package (meshes + URDF)
# installed in a ROS 2 workspace. Without it, RViz shows only TF axes, no model.
#
# Run from the directory that contains this script (the laptop_viz folder):
#     bash setup_laptop_viz.sh
# ─────────────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/hexapod_viz_ws"

echo "==> Creating viz workspace at $WS"
mkdir -p "$WS/src"

echo "==> Copying hexapod_description package"
cp -r "$SCRIPT_DIR/hexapod_description" "$WS/src/"

echo "==> Building (ament_cmake — installs meshes so package:// resolves)"
cd "$WS"
source /opt/ros/humble/setup.bash
# --allow-overriding: you may already have a hexapod_description in another
# workspace (e.g. ~/ros2_ws). This viz copy overrides it for RViz mesh
# resolution; that's intentional and safe (meshes only, no code).
colcon build --packages-select hexapod_description --allow-overriding hexapod_description

echo ""
echo "================================================================"
echo "  Laptop viz setup complete."
echo ""
echo "  Every new terminal that wants to visualize must source BOTH:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source $WS/install/setup.bash"
echo ""
echo "  (Add those two lines to ~/.bashrc to make it automatic.)"
echo ""
echo "  Then launch the 3D view:"
echo "    bash $SCRIPT_DIR/view_3d.sh"
echo "================================================================"
