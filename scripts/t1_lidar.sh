#!/bin/bash
# TERMINAL — LDS-01 LiDAR driver + base_link → laser static TF.
#
# The LDS-01 enumerates as /dev/ttyUSB0 inside the container (passed
# through by `--privileged -v /dev:/dev` in start_ros.sh).
#
# LiDAR mount offset on the printed body (from mapping.launch.py):
#   x = 0, y = 0, z = +0.08 m, no rotation (LiDAR sits 8 cm above base_link
#   at the body centre). MEASURE the actual offset and edit if needed.
#
# This script does NOT start any SLAM/mapping node — that's t2_rtabmap.sh.

pkill -f hlds_laser_publisher 2>/dev/null || true
pkill -f "static_transform_publisher.*base_link.*laser" 2>/dev/null || true
sleep 0.3

source /opt/ros/humble/setup.bash
source /hexapodd/hexapod_ws/install/setup.bash

# Static TF: base_link → laser
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.08 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id laser &

sleep 0.3

# LDS-01 driver
LIDAR_LOG=/tmp/lidar_launch.log
(
    ros2 run hls_lfcd_lds_driver hlds_laser_publisher \
        --ros-args -p port:=/dev/ttyUSB0 -p frame_id:=laser
) 2>&1 | tee "$LIDAR_LOG"

echo ""
echo "────────────────────────────────────────────────────────────────"
echo "  LDS-01 driver exited. Log: $LIDAR_LOG"
echo "  Most likely cause: LIDAR not plugged in (no /dev/ttyUSB0)."
echo "  This window will stay open — Ctrl-b & to kill it from tmux."
echo "────────────────────────────────────────────────────────────────"
# read -r doesn't work without a TTY inside `docker exec`. sleep keeps the
# window alive so the operator can see the error message above.
sleep infinity
