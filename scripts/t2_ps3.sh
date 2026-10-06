#!/bin/bash
# TERMINAL 2 — PS3 controller via USB
source /opt/ros/humble/setup.bash
source /hexapodd/hexapod_ws/install/setup.bash
source /hexapodd/ros2_ws/install/local_setup.bash
ros2 run joy joy_node &
ros2 run hexapod_nav joy_teleop
