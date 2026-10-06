#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# shutdown_servos.sh — clean stop of the running hexapod stack on the JETSON.
#
# Runs on the Jetson host. Does three things, in order:
#   1. Publishes 'stop' on /hexapod/cmd so the controller settles legs to
#      the standing pose before the bridge dies (no slamming).
#   2. Sends Ctrl-C to each tmux window so the bridge's destroy_node()
#      runs (sweeps servos to standing, closes the HID handle cleanly).
#   3. Kills the tmux session.
#
# From the LAPTOP you'd normally invoke it through ssh:
#   ssh -t jetson@192.168.0.20 'sudo bash ~/hexapod-ros2/scripts/shutdown_servos.sh'
# ─────────────────────────────────────────────────────────────────────────────
set -u

CONTAINER=hexapod_ros
SESSION=servo_bringup

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' isn't running — nothing to stop."
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    exit 0
fi

echo "==> Asking the controller to settle to standing pose…"
docker exec "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /hexapodd/hexapod_ws/install/setup.bash &&
    ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: '\''stop'\''"' \
    2>/dev/null || true
sleep 1.2

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "==> Ctrl-C to controller + bridge (lets the bridge settle servos)…"
    for win in controller bridge; do
        tmux send-keys -t "$SESSION:$win" C-c 2>/dev/null || true
    done
    sleep 2

    echo "==> Killing tmux session '$SESSION'…"
    tmux kill-session -t "$SESSION" 2>/dev/null || true
fi

echo "==> Killing any stray nodes inside the container (belt-and-braces)…"
docker exec "$CONTAINER" bash -lc '
    pkill -f hexapod_controller 2>/dev/null;
    pkill -f hiwonder_servo_bridge 2>/dev/null;
    true' || true

echo "Done. Robot is no longer receiving commands."
echo "To turn it back on, re-run:  sudo bash scripts/bringup_servos.sh"
