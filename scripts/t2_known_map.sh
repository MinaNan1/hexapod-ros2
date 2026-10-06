#!/bin/bash
# TERMINAL 2 — A* known-map path planner
source /opt/ros/humble/setup.bash
source /hexapodd/hexapod_ws/install/setup.bash
source /hexapodd/ros2_ws/install/local_setup.bash
ros2 run hexapod_nav known_map_demo
