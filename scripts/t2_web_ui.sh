#!/bin/bash
# TERMINAL 2 — rosbridge + ui_teleop (for web UI)
source /opt/ros/humble/setup.bash
source /hexapodd/hexapod_ws/install/setup.bash
source /hexapodd/ros2_ws/install/local_setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run hexapod_nav ui_teleop
