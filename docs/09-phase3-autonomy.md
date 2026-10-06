# Phase 3 — Autonomous exploration (spec, not yet implemented)

## Goal

Robot enters a room, decides on its own where to walk, fills the map until
the room is fully explored, then stops. No human input during the run.

## Two distinct modes the operator can pick

Both already partially exist in `ros2_ws/src/hexapod_control/hexapod_nav/`:

| Mode | Existing file | What it does | Use case |
|---|---|---|---|
| **Frontier exploration** | `frontier_explorer.py` (partial) | Pick the closest cell on the boundary between known and unknown space. Walk there. Repeat. | "Explore an unknown room" |
| **Coverage planning** | needs new `coverage_planner.py` | Sweep the entire known free space in a boustrophedon (lawn-mower) pattern. | "Make sure every square cm has been seen by the camera" — the *project_instructions* goal |

The two are mutually exclusive in a single run. The web UI should expose a mode
toggle: `EXPLORE` vs `COVER`.

## Frontier-explorer pipeline

```
/rtabmap/map  →  frontier_explorer.py
                    │
                    ├─ pick frontier target (closest, weighted by size)
                    ├─ Publish geometry_msgs/PoseStamped on /explore/goal
                    │
                    ↓
                path_follower.py  (existing in repo as lidar_path_planner.py)
                    │
                    ├─ A* over /rtabmap/map
                    ├─ Emit /hexapod/cmd  walk / rotate_left / rotate_right / stop
                    │
                    ↓
                hexapod_controller (existing) → gait → motors
                    │
                    └─ /odom publishes current pose → loop closes
```

## Files to add or finish

| File | Status | Notes |
|---|---|---|
| `ros2_ws/src/hexapod_control/hexapod_nav/frontier_explorer.py` | partial | Already in the repo. Needs to be wired up to `/rtabmap/map` (currently uses an older `/map` topic). |
| `ros2_ws/src/hexapod_control/hexapod_nav/coverage_planner.py` | NEW | Boustrophedon over the occupancy grid. Picks the long axis, generates lanes spaced by camera FoV at expected distance, A*-routes between lane endpoints. |
| `ros2_ws/src/hexapod_control/hexapod_nav/path_follower.py` | partial | `lidar_path_planner.py` is close — refactor it to subscribe to either `/explore/goal` or `/cover/goal`, then publish gait commands. |
| `scripts/t1_autonomy.sh` | NEW | Tmux window wrapper for the explorer + path follower. |
| `scripts/launch_all.sh` | edit | Add autonomy window (default off; enabled by env var `HEXAPOD_AUTONOMY=1`). |
| Web UI mode toggle | edit | New `<button>` group in `index.html`. Sends `/ui/mode` String: `manual`/`explore`/`cover`. `ui_teleop` already has the multiplexing skeleton — extend it. |

## Safety + sanity

- **Watchdog**: frontier_explorer must publish `stop` if it loses `/rtabmap/map`
  for more than 2 seconds.
- **Deadman**: any web UI joystick input takes precedence and switches mode
  back to `manual` immediately.
- **Speed cap**: in explore/cover modes, the robot moves at ≤50% of normal
  gait speed. Walking into a wall is OK; walking *fast* into a wall snaps
  servos.
- **Recovery**: if the path follower fails to make progress for 5 seconds
  (no change in `/odom` while commanded to walk), publish `rotate_left` for
  1 second then re-plan.

## Test plan

1. Bench test with the robot in the air (no ground contact). Mode = explore.
   Verify frontier_explorer emits `/explore/goal` at the right cells.
2. With the robot on the ground in a 2×2 m clear area, mode = explore. The
   robot should walk to one wall, rotate, find another wall, eventually stop.
3. Same area, mode = cover. The robot should fill the area in lanes,
   visiting every cell.
4. In a real room with furniture — verify obstacle avoidance from the LIDAR.

## What's hard

- **Hexapod gait is not differential-drive.** The path follower can't just
  emit `cmd_vel`. We emit string commands (`walk`, `rotate_left`, …) and the
  gait controller translates. So path following is more like a state machine
  ("am I aligned with goal? if not rotate. else walk.").
- **Odometry is bad.** Gait dead-reckoning drifts by ~10% per meter. RTAB-Map
  corrects this via LIDAR ICP, but the corrected pose is in the `map` frame
  while the gait expects `odom`. The TF chain `map → odom → base_link` handles
  this if RTAB-Map publishes `map → odom` (it does).
- **LDS-01 is low quality.** Wall corners get rounded, narrow doorways may
  look like obstacles. The frontier detector needs to tolerate this — set
  `Grid/MaxObstacleHeight: 0.5` to ignore the noisy ceiling and `Grid/NoiseFilteringRadius`
  to clean up scan noise. Already set in `rtabmap.launch.py`.

## Defer this until

Phase 4 (YOLO) is at least scaffolded — `coverage_planner.py`'s lane spacing
depends on the camera's effective coverage cone, which is something we want to
visualize via Foxglove markers (Phase 4 deliverable) before we commit to a
specific lane width.
