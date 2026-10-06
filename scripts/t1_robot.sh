#!/bin/bash
# TERMINAL 1 — Robot bring-up, HEADLESS (always run this first)
# No RViz here — RViz runs on the laptop over the network.
# See scripts/laptop_rviz.sh for the laptop side.

# Kill any stale nodes from a previous launch to prevent duplicate publishers
pkill -f hexapod_controller    2>/dev/null || true
pkill -f hiwonder_servo_bridge 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f imu_node              2>/dev/null || true
pkill -f rviz2                 2>/dev/null || true
sleep 0.5

source /opt/ros/humble/setup.bash

# Rebuild hexapod_control with --symlink-install so the latest controller /
# bridge / teleop code is live (and stays live for future src edits without a
# rebuild). Cheap (one package) and keeps the web UI stack in sync with src.
( cd /hexapodd/hexapod_ws && colcon build --packages-select hexapod_control --symlink-install ) || true

source /hexapodd/hexapod_ws/install/setup.bash
ros2 launch hexapod_description display_headless.launch.py
