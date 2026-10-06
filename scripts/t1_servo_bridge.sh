#!/bin/bash
# TERMINAL — Hiwonder HX-35HM bus-servo bridge (Jetson side).
# Subscribes to /joint_states from hexapod_controller and drives the 18
# servos via USB-HID. Run AFTER t1_robot.sh.
#
# Device-path strategy:
#   1. Prefer /dev/hiwonder (created by the udev rule —
#      see LAPTOP_COMMANDS.md "I just plugged the Jetson..." section).
#   2. Otherwise, fall back to /dev/hidraw1 (legacy assumption).
#
# If neither exists, the user needs to:
#   - install the udev rule (one-time), OR
#   - find the correct hidraw with:
#        ros2 run hexapod_control hiwonder_servo_bridge --list
#     then pass it via --ros-args -p hid_device:=/dev/hidrawN.

# Kill any stale bridge from a previous launch
pkill -f hiwonder_servo_bridge 2>/dev/null || true
sleep 0.5

if [ -e /dev/hiwonder ]; then
    HID_PATH=/dev/hiwonder
elif [ -e /dev/hidraw1 ]; then
    HID_PATH=/dev/hidraw1
    chmod 666 "$HID_PATH" 2>/dev/null || true
else
    echo "WARNING: neither /dev/hiwonder nor /dev/hidraw1 is present —"
    echo "         is the Hiwonder controller plugged in and enumerated?"
    echo "Available HID devices: $(ls /dev/hidraw* 2>/dev/null || echo 'none')"
    echo "Falling back to default; the bridge will print available devices on error."
    HID_PATH=/dev/hiwonder
fi

source /opt/ros/humble/setup.bash
source /hexapodd/hexapod_ws/install/setup.bash

# ── Scheduling priority (anti-jitter) ────────────────────────────────────────
# The servo bridge writes HID packets on a 50 Hz cadence. When RTAB-Map + the
# camera saturate the Nano's 4 cores, the default-priority bridge can miss its
# scheduling slot → uneven packet timing → motor jitter. Raise its priority so
# the kernel always schedules it on time. Prefer real-time round-robin (chrt);
# fall back to a strong negative nice if chrt isn't permitted.
RUN_PREFIX=""
if command -v chrt >/dev/null 2>&1 && chrt -r 10 true 2>/dev/null; then
    RUN_PREFIX="chrt -r 10"        # SCHED_RR priority 10 — above normal tasks
elif command -v nice >/dev/null 2>&1; then
    RUN_PREFIX="nice -n -10"       # high-priority normal scheduling
fi
echo "[t1_servo_bridge.sh] launching with priority: ${RUN_PREFIX:-default}"

exec $RUN_PREFIX ros2 run hexapod_control hiwonder_servo_bridge \
    --ros-args -p hid_device:="$HID_PATH"
