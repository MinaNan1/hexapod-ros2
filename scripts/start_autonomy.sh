#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# start_autonomy.sh — OPT-IN autonomous navigation (Phase 3).
#
# Adds an 'autonomy' tmux window to the already-running hexapod session and
# launches the frontier/coverage explorer. It binds to RTAB-Map's live map
# (/rtabmap/map) and RTAB-Map's TF (map→odom→base_link) — NOT the old
# lidar_mapper/scan_odom pipeline (RTAB-Map replaces both).
#
# SAFETY — this never moves the robot on its own:
#   • The explorer starts DISABLED. It only drives while the web UI is in AUTO.
#   • Click AUTO in the web UI (http://<jetson>:5000) to begin.
#   • Click STOP (or MANUAL) to halt instantly — STOP forces MANUAL.
#   • A heavy robot moving autonomously can hit things / fall off edges. Keep a
#     hand on STOP and clear the area before enabling AUTO.
#
# Usage (on the Jetson host, AFTER launch_all.sh is up):
#     bash scripts/start_autonomy.sh            # frontier exploration
#     bash scripts/start_autonomy.sh coverage   # boustrophedon area coverage
#
# Stop just autonomy:  tmux kill-window -t hexapod:autonomy
# ─────────────────────────────────────────────────────────────────────────────
set -u

CONTAINER=hexapod_ros
SESSION=hexapod
MODE="${1:-frontier}"     # frontier | coverage | goal

# Mode → explorer params:
#   frontier : UNKNOWN map — explore/fill by driving to boundaries until the
#              whole area is discovered (no frontiers left = room complete).
#   coverage : UNKNOWN map — boustrophedon sweep so the camera covers everything.
#   goal     : KNOWN (or in-progress) map — DON'T auto-explore; wait for a goal
#              (RViz "2D Goal Pose" → /goal_pose) and A*-navigate to it, avoiding
#              obstacles. Use with a resumed saved map (HEXAPOD_RESUME_MAP=1).
case "$MODE" in
    frontier|coverage) BEHAVIOR="$MODE"; AUTO_EXPLORE=true ;;
    goal)              BEHAVIOR=frontier; AUTO_EXPLORE=false ;;
    *) echo "Usage: $0 [frontier|coverage|goal]"; exit 1 ;;
esac
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' isn't running — start the stack first (launch_all.sh)."
    exit 1
fi
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' not found — run launch_all.sh first."
    exit 1
fi

# Replace any previous autonomy window
tmux kill-window -t "$SESSION:autonomy" 2>/dev/null || true

RUN="source /opt/ros/humble/setup.bash && \
     source /hexapodd/hexapod_ws/install/setup.bash && \
     source /hexapodd/ros2_ws/install/local_setup.bash && \
     ros2 run hexapod_nav lidar_explorer --ros-args \
         -p map_topic:=/rtabmap/map \
         -p behavior:=$BEHAVIOR \
         -p auto_explore:=$AUTO_EXPLORE \
         -p robot_radius_cells:=6"
# robot_radius_cells=6 → ~0.30 m clearance (map is 0.05 m/cell). The hexapod's
# sprawled-leg footprint is ~0.3 m radius; the old default of 2 (0.10 m) made
# A* route too close and clip obstacles. This is the core obstacle-avoidance
# tuning: the planner inflates every obstacle by this radius so planned paths
# keep the whole robot clear. Raise to 7-8 if it still grazes corners; lower to
# 5 if it refuses to fit through real doorways.

tmux new-window -t "$SESSION" -n autonomy \
    "docker exec -it $CONTAINER bash -lc '$RUN'"

echo "════════════════════════════════════════════════════════════════"
echo "  Autonomy window started — mode: $MODE"
echo "  The explorer is DISABLED until you switch the web UI to AUTO."
echo ""
echo "  1. Open the web UI:   http://192.168.0.20:5000"
if [ "$MODE" = "goal" ]; then
echo "  2. Click  AUTO        → planner waits for a goal"
echo "  3. In RViz: 2D Goal Pose tool → click a destination on the map"
echo "             → robot A*-navigates there, avoiding obstacles"
else
echo "  2. Click  AUTO        → robot begins ${MODE} exploration"
echo "             (drive it a few seconds first so a map exists to plan on)"
fi
echo "  • Click  STOP         → halts instantly (forces MANUAL)"
echo ""
echo "  Watch it:   tmux attach -t $SESSION   (Ctrl-b → 'autonomy' window)"
echo "  Stop it:    tmux kill-window -t $SESSION:autonomy"
echo "════════════════════════════════════════════════════════════════"
