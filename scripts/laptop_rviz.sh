#!/bin/bash
# LAPTOP — RViz viewer for the hexapod running on the Jetson.
#
# Prerequisites (already done once):
#   - ROS 2 Humble installed on the laptop
#   - hexapod_ws built on the laptop (so meshes + rviz config resolve locally)
#   - ~/.bashrc exports ROS_DOMAIN_ID=42 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   - Laptop and Jetson on the same network (LAN cable or WiFi)
#
# The Jetson publishes /robot_description, /joint_states and TF over the network.
# This script just opens RViz pointed at that data — it runs no robot nodes.

# Find the repo root from this script's location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/hexapod_ws/install/setup.bash"

RVIZ_CFG="$REPO_ROOT/hexapod_ws/install/hexapod_description/share/hexapod_description/rviz/hexapod.rviz"
ros2 run rviz2 rviz2 -d "$RVIZ_CFG"
