#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────
# stop_all.sh — tear down the hexapod stack started by launch_all.sh.
# Kills the tmux session and stops the docker container.
# ─────────────────────────────────────────────────────────────────────────

SESSION="hexapod"

# Stop ROS nodes inside the container (graceful)
docker exec hexapod_ros bash -c "pkill -f hexapod_controller; pkill -f robot_state_publisher; pkill -f rviz2; pkill -f rosbridge; pkill -f ui_teleop; pkill -f hiwonder_servo_bridge; pkill -f orbbec_camera_node; pkill -f component_container; pkill -f v4l2_camera_node; pkill -f hlds_laser_publisher; pkill -f rtabmap; pkill -f static_transform_publisher" 2>/dev/null || true

# Stop the Flask app on the host
pkill -f "python3 app.py" 2>/dev/null || true

# Kill the tmux session (this also kills the container's interactive shell)
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Stop the docker container
docker stop hexapod_ros 2>/dev/null || true
docker rm hexapod_ros 2>/dev/null || true

echo "Hexapod stack stopped."
