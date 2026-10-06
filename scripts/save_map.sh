#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# save_map.sh — export the current RTAB-Map map to portable files on the Jetson.
#
# RTAB-Map keeps its live map in /root/.ros/rtabmap.db (bind-mounted to the
# host at maps/rtabmap.db via start_ros.sh). That .db is the full map and
# already survives restarts, but it's not directly viewable. This script
# exports it to standard formats you can open anywhere:
#
#   maps/exports/<timestamp>/cloud.ply   — 3D point cloud (MeshLab/CloudCompare)
#   maps/exports/<timestamp>/map_2d.pgm  — 2D occupancy grid image
#   maps/exports/<timestamp>/map_2d.yaml — its metadata (Nav2-compatible)
#
# Run on the Jetson host while the stack is up (so the 2D grid is published):
#     bash ~/hexapod-ros2/scripts/save_map.sh
# ─────────────────────────────────────────────────────────────────────────────
set -u

CONTAINER=hexapod_ros
TS=$(date +%Y%m%d_%H%M%S)
OUT_HOST="$(cd "$(dirname "$0")/.." && pwd)/maps/exports/$TS"
OUT_CTNR="/root/.ros/exports/$TS"   # same dir, seen from inside the container

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' isn't running — start the stack first (launch_all.sh)."
    exit 1
fi

mkdir -p "$OUT_HOST"
docker exec "$CONTAINER" mkdir -p "$OUT_CTNR"

echo "==> 1/2  Exporting 3D point cloud from rtabmap.db → cloud.ply"
# rtabmap-export ships with the rtabmap apt package. --cloud writes a fused
# point cloud; we voxel-filter to keep the file reasonable.
docker exec "$CONTAINER" bash -lc "
    cd $OUT_CTNR &&
    rtabmap-export --cloud --voxel 0.02 --output cloud /root/.ros/rtabmap.db
" && echo "    ✓ cloud.ply written" \
  || echo "    ⚠ rtabmap-export failed (is there a map yet? walk the robot first)"

echo "==> 2/2  Saving 2D occupancy grid → map_2d.pgm / map_2d.yaml"
# Prefer nav2 map_saver if present; it subscribes to the live /rtabmap/map.
docker exec "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /hexapodd/ros2_ws/install/local_setup.bash 2>/dev/null;
    if ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
        cd $OUT_CTNR &&
        ros2 run nav2_map_server map_saver_cli \
            -t /rtabmap/map -f map_2d \
            --ros-args -p map_subscribe_transient_local:=true
    else
        echo 'nav2_map_server not installed — skipping 2D export'
        echo '(the 2D map is still inside rtabmap.db; install ros-humble-nav2-map-server to export it)'
    fi
" && echo "    ✓ 2D map saved (if nav2_map_server present)"

echo ""
echo "Done. Files on the host at:"
echo "  $OUT_HOST"
ls -la "$OUT_HOST" 2>/dev/null
