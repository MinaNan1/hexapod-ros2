#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────
# laptop_camera_view.sh — quick proof the camera link works from the laptop.
# Runs on the LAPTOP (not the Jetson). Pulls the compressed color stream
# via image_transport over DDS and shows it in rqt_image_view.
#
# Pre-reqs (one time on the laptop):
#   sudo apt install ros-humble-rqt-image-view ros-humble-image-transport-plugins
#   # ~/.bashrc already exports ROS_DOMAIN_ID=42 + FastRTPS (see LAPTOP_COMMANDS.md)
#
# Usage:
#   bash scripts/laptop_camera_view.sh
# ─────────────────────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash
echo "Opening rqt_image_view → subscribe to /camera/color/image_raw"
echo "  (image_transport plugin should auto-negotiate the compressed wire format)"
echo "Also try: /camera/depth/image_raw  (colormap shows the depth map)"
ros2 run rqt_image_view rqt_image_view
