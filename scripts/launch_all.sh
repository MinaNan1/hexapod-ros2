#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────
# launch_all.sh — bring up the full hexapod stack on the Jetson.
# Runs four components inside a single tmux session so they survive
# after SSH disconnects.
#
# From the laptop:
#   ssh jetson@hexapod-desktop.local bash ~/hexapod-ros2/scripts/launch_all.sh
#
# To watch the logs:
#   ssh -t jetson@hexapod-desktop.local tmux attach -t hexapod
#   (press Ctrl-b then d to detach)
#
# To stop everything:
#   ssh jetson@hexapod-desktop.local bash ~/hexapod-ros2/scripts/stop_all.sh
# ─────────────────────────────────────────────────────────────────────────

set -e
SESSION="hexapod"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Already running? Just print info and exit.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Hexapod stack is already running."
    echo "Attach with: tmux attach -t $SESSION"
    echo "Stop with:   $PROJECT_DIR/scripts/stop_all.sh"
    exit 0
fi

# Clean up any orphan Flask / docker exec processes from a manual session
pkill -f "python3 app.py" 2>/dev/null || true
docker exec hexapod_ros bash -c "pkill -f hexapod_controller; pkill -f robot_state_publisher; pkill -f rosbridge; pkill -f ui_teleop; pkill -f hiwonder_servo_bridge; pkill -f orbbec_camera_node; pkill -f component_container; pkill -f v4l2_camera_node; pkill -f hlds_laser_publisher; pkill -f rtabmap; pkill -f static_transform_publisher" 2>/dev/null || true
sleep 1

# 1. Ensure the container is running (start it if not)
if ! docker ps --format '{{.Names}}' | grep -q '^hexapod_ros$'; then
    echo "Starting container..."
    # Run start_ros.sh in a detached tmux window so its interactive
    # bash session stays alive.
    tmux new-session -d -s "$SESSION" -n container \
        "bash $PROJECT_DIR/start_ros.sh"
    # Wait for container to come up
    for i in $(seq 1 30); do
        if docker ps --format '{{.Names}}' | grep -q '^hexapod_ros$'; then
            break
        fi
        sleep 1
    done
    sleep 2
else
    # Container is up — open the tmux session with a placeholder window
    tmux new-session -d -s "$SESSION" -n container "echo 'container already running'; sleep infinity"
fi

# 2. Robot stack (hexapod_controller + robot_state_publisher, headless)
tmux new-window -t "$SESSION" -n robot \
    "docker exec hexapod_ros bash /hexapodd/scripts/t1_robot.sh"

sleep 3

# 3. Hiwonder bus-servo bridge — drives the 18 HX-35HM servos from
#    /joint_states. Started after the robot stack so the controller is
#    already publishing.
tmux new-window -t "$SESSION" -n servos \
    "docker exec hexapod_ros bash /hexapodd/scripts/t1_servo_bridge.sh"

sleep 2

# 4. Astra Pro (Orbbec) depth camera — depth via orbbec_camera, color via
#    v4l2_camera (dual-driver pattern — see docs/05-astra-pro-camera.md).
#    Skip silently if the package isn't built yet (first-time run before
#    setup_orbbec.sh + build_ws.sh have been done).
if docker exec hexapod_ros bash -c "source /opt/ros/humble/setup.bash; [ -d /hexapodd/ros2_ws/install/orbbec_camera ]" 2>/dev/null; then
    tmux new-window -t "$SESSION" -n camera \
        "docker exec hexapod_ros bash /hexapodd/scripts/t1_camera.sh"
    sleep 2
else
    echo "NOTE: orbbec_camera not built yet — skipping camera window."
    echo "      Run scripts/setup_orbbec.sh on the host once, then build_ws.sh in the container."
fi

# 5. LDS-01 LiDAR — /scan + base_link→laser static TF (Phase 2).
tmux new-window -t "$SESSION" -n lidar \
    "docker exec hexapod_ros bash /hexapodd/scripts/t1_lidar.sh"
sleep 2

# 6. RTAB-Map 3D SLAM (Phase 2). Skip silently if rtabmap isn't installed
#    yet (first-time run before dockerfile rebuild with Phase 2 deps).
# Use a filesystem check — `ros2 pkg list` inside `docker exec bash -c`
# doesn't auto-source ROS, returns empty, and silently skipped the window
# on every launch. Filesystem check is reliable and doesn't need ROS.
if docker exec hexapod_ros bash -c "[ -d /opt/ros/humble/share/rtabmap_slam ]" 2>/dev/null; then
    tmux new-window -t "$SESSION" -n rtabmap \
        "docker exec hexapod_ros bash /hexapodd/scripts/t2_rtabmap.sh"
    sleep 2
else
    echo "NOTE: rtabmap_slam not installed yet — skipping rtabmap window."
    echo "      Rebuild the dockerfile to pick up ros-humble-rtabmap-ros."
fi

# 7. Web UI ROS side (rosbridge + ui_teleop)
tmux new-window -t "$SESSION" -n webui_ros \
    "docker exec hexapod_ros bash /hexapodd/scripts/t2_web_ui.sh"

sleep 2

# 8. Flask web server (host side, not container)
tmux new-window -t "$SESSION" -n flask \
    "cd $PROJECT_DIR/ros2_ws/src/hexapod_control/web_ui && python3 app.py"

echo ""
echo "================================================================"
echo "  Hexapod stack started in tmux session: $SESSION"
echo "================================================================"
echo "  Robot:   running headless (RViz lives on the laptop)"
echo "  Camera:  Astra Pro 640x480 — depth@orbbec, color@v4l2 — /camera/*"
echo "  LiDAR:   LDS-01 on /dev/ttyUSB0 — /scan"
echo "  SLAM:    RTAB-Map LIDAR+depth — /rtabmap/map, /cloud_map, /mapPath"
echo "  Web UI:  http://hexapod-desktop.local:5000"
echo "  ROS:     ROS_DOMAIN_ID=42, peers discover via DDS"
echo "----------------------------------------------------------------"
echo "  View logs:  tmux attach -t $SESSION       (Ctrl-b d to detach)"
echo "  Stop all:   $PROJECT_DIR/scripts/stop_all.sh"
echo "================================================================"
