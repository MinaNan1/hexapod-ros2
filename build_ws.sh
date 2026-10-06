#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────
# build_ws.sh — Build both ROS 2 workspaces inside the container
#
# Run this INSIDE the container (after ./start_ros.sh), first time only:
#   /hexapodd/build_ws.sh
# ─────────────────────────────────────────────────────────────────────────

set -e

source /opt/ros/humble/setup.bash

echo "============================================================"
echo "  Building Workspace 1: hexapod_ws (URDF + gait controller)"
echo "============================================================"
cd /hexapodd/hexapod_ws
colcon build
source install/setup.bash

echo ""
echo "============================================================"
echo "  Building Workspace 2: ros2_ws (navigation + Gazebo nodes)"
echo "============================================================"
cd /hexapodd/ros2_ws
colcon build --symlink-install \
    --packages-ignore \
        hexapod_description \
        rtabmap \
        rtabmap_ros \
        rtabmap_msgs \
        rtabmap_costmap_plugins \
        rtabmap_conversions \
        rtabmap_launch \
        rtabmap_sync \
        rtabmap_util \
        rtabmap_odom \
        rtabmap_slam \
        rtabmap_rviz_plugins \
        rtabmap_python \
        rtabmap_viz \
        rtabmap_examples \
        rtabmap_demos
source install/setup.bash

echo ""
echo "============================================================"
echo "  Both workspaces built successfully!"
echo ""
echo "  To run the known-map demo, open TWO terminals and run:"
echo "  (use: docker exec -it hexapod_ros bash  to open more terminals)"
echo ""
echo "  Terminal 1 — gait controller + URDF + RViz:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source /hexapodd/hexapod_ws/install/setup.bash"
echo "    ros2 launch hexapod_description display.launch.py"
echo ""
echo "  Terminal 2 — path planner:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source /hexapodd/hexapod_ws/install/setup.bash"
echo "    source /hexapodd/ros2_ws/install/local_setup.bash"
echo "    ros2 run hexapod_nav known_map_demo"
echo "============================================================"
