#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# bringup_servos.sh — one-command servo bring-up on the JETSON HOST.
#
# Replicates the laptop "chmod + two terminals" flow, but does everything the
# Jetson needs that the laptop didn't:
#   1. Hiwonder device: install udev rule + rebind the HID interface if the
#      Nano left it unbound  (needs root).
#   2. Make sure the ROS 2 docker container is running.
#   3. Rebuild hexapod_control so the auto-detect bridge is installed.
#   4. Launch hexapod_controller + hiwonder_servo_bridge in a tmux session
#      ('servo_bringup'), the two "terminals" from the laptop flow.
#
# Run on the Jetson host (the project lives here):
#     sudo bash ~/hexapod-ros2/scripts/bringup_servos.sh
#
# Watch the two nodes:   tmux attach -t servo_bringup    (Ctrl-b n to switch,
#                        Ctrl-b d to detach).  Stop:     tmux kill-session -t servo_bringup
# ─────────────────────────────────────────────────────────────────────────────
set -u

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER=hexapod_ros
SESSION=servo_bringup

if [ "$(id -u)" -ne 0 ]; then
    echo "Run as root (it installs a udev rule + rebinds USB):  sudo bash $0"
    exit 1
fi

echo "==> 1/4  Hiwonder device setup"
bash "$ROOT_DIR/scripts/setup_hiwonder.sh" || {
    echo "    Device not ready. Replug the Hiwonder USB cable and re-run."
    exit 1
}

for bin in docker tmux; do
    command -v "$bin" >/dev/null || { echo "Missing '$bin' on the host — install it first (e.g. sudo apt install $bin)."; exit 1; }
done

echo "==> 2/4  Ensuring docker container '$CONTAINER' is up"
if ! docker image inspect hexapod-ros2-humble >/dev/null 2>&1; then
    echo "    Image 'hexapod-ros2-humble' not built yet — building (5-10 min, first time only)…"
    docker build -f "$ROOT_DIR/hexapod.dockerfile" -t hexapod-ros2-humble "$ROOT_DIR" \
        || { echo "    image build failed"; exit 1; }
fi
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "    Container not running — starting it detached…"
    docker rm -f "$CONTAINER" 2>/dev/null || true
    docker run -d --name "$CONTAINER" --network host --privileged \
        -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -v /dev:/dev -v "$ROOT_DIR:/hexapodd" \
        hexapod-ros2-humble sleep infinity
    sleep 3
fi
echo "    ✓ $CONTAINER running"

echo "==> 3/4  Building hexapod_control (auto-detect bridge)"
docker exec "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash &&
    cd /hexapodd/hexapod_ws &&
    colcon build --packages-select hexapod_control --symlink-install' \
    || { echo "    build failed"; exit 1; }

echo "==> 4/4  Launching controller + bridge in tmux '$SESSION'"
RUN='source /opt/ros/humble/setup.bash && source /hexapodd/hexapod_ws/install/setup.bash &&'
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session  -d -s "$SESSION" -n controller \
    "docker exec -it $CONTAINER bash -lc '$RUN ros2 run hexapod_control hexapod_controller'"
sleep 2
tmux new-window -t "$SESSION" -n bridge \
    "docker exec -it $CONTAINER bash -lc '$RUN ros2 run hexapod_control hiwonder_servo_bridge'"

echo ""
echo "Done. Two nodes are running in tmux session '$SESSION'."
echo "  Watch them:   tmux attach -t $SESSION   (Ctrl-b n switches, Ctrl-b d detaches)"
echo "  Drive it:     ros2 topic pub --once /hexapod/cmd std_msgs/msg/String \"data: 'walk'\""
echo "  Stop:         tmux kill-session -t $SESSION"
