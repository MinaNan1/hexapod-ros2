#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────
# start_ros.sh — Build and launch the hexapod ROS 2 Humble container
#
# Run this from the JETSON HOST terminal (not inside VS Code):
#   cd ~/hexapod-ros2
#   chmod +x start_ros.sh
#   ./start_ros.sh
# ─────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="hexapod-ros2-humble"
CONTAINER_NAME="hexapod_ros"

# Allow GUI apps (RViz) to use the host display
xhost +local:docker 2>/dev/null || true

# Make sure the maps directory exists on the host so we can bind-mount it
# into the container at /root/.ros. RTAB-Map writes its SLAM database there
# (rtabmap.db) — without this mount, the DB lives inside the container and
# gets wiped every time stop_all.sh destroys it. With the mount the map
# survives across sessions and is accessible to operators as plain files.
mkdir -p "$SCRIPT_DIR/maps"

# Build the image if it doesn't exist yet
if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
    echo "============================================================"
    echo "  Building hexapod ROS 2 Humble image (first time only)..."
    echo "  This takes 5-10 minutes. Grab a coffee."
    echo "============================================================"
    docker build -f "$SCRIPT_DIR/hexapod.dockerfile" -t "$IMAGE_NAME" "$SCRIPT_DIR"
    echo "============================================================"
    echo "  Image built successfully."
    echo "============================================================"
fi

# Stop any existing container with the same name
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

echo ""
echo "Starting hexapod ROS 2 container..."
echo "Your project is mounted at /hexapodd inside the container."
echo ""

docker run -it \
    --name "$CONTAINER_NAME" \
    --network host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v "$SCRIPT_DIR:/hexapodd" \
    -v "$SCRIPT_DIR/maps:/root/.ros" \
    "$IMAGE_NAME" \
    /bin/bash -c "
        source /opt/ros/humble/setup.bash

        echo '========================================================'
        echo '  Hexapod ROS 2 Humble container ready.'
        echo '  Project is at: /hexapodd'
        echo ''
        echo '  First time? Run:  /hexapodd/build_ws.sh'
        echo '  Then see README for demo launch commands.'
        echo '========================================================'
        exec /bin/bash
    "
