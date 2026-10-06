# Demo 1 — Known Map + A* Path Simulation

The robot starts at world origin, picks a random goal on a randomly generated map, plans an A* path that keeps a safe distance from walls, and walks it. All in simulation. **No LiDAR required.**

## What you'll see

- A 200 × 200 cell occupancy grid (10 m × 10 m at 5 cm/cell) with random rectangular obstacles
- 3D walls extruded from the obstacles (Marker / `/walls_3d`)
- An orange path from the start to the goal that goes around obstacles with ≥ 60 cm of clearance
- The full hexapod URDF walking along the path with a tripod gait

## Files involved

| Path | What it does |
|---|---|
| [`ros2_ws/src/hexapod_control/hexapod_control/known_map_demo.py`](../ros2_ws/src/hexapod_control/hexapod_control/known_map_demo.py) | The demo node — generates map, plans, drives the robot |
| [`hexapod_path_planning_final/hexapod_path_planning/`](../hexapod_path_planning_final/hexapod_path_planning) | The pure-Python planning library this node imports |
| [`hexapod_ws/src/hexapod_description/urdf/crab_model.xacro`](../hexapod_ws/src/hexapod_description/urdf/crab_model.xacro) | URDF — body lifted to its standing height so leg tips touch the floor |
| [`hexapod_ws/src/hexapod_control/hexapod_control/hexapod_controller.py`](../hexapod_ws/src/hexapod_control/hexapod_control/hexapod_controller.py) | Gait controller — animates the legs and publishes `odom → base_link` |
| [`hexapod_ws/src/hexapod_description/launch/display.launch.py`](../hexapod_ws/src/hexapod_description/launch/display.launch.py) | One-shot bringup: URDF + controller + static TF + RViz |

## Build (first time)

```bash
cd ~/hexapod-ros2/hexapod_ws
colcon build
source install/setup.bash

cd ~/hexapod-ros2/ros2_ws
colcon build --packages-select hexapod_control
source install/setup.bash
```

## Run

Two terminals.

```bash
# Terminal 1 — bring up everything visual
source ~/hexapod-ros2/hexapod_ws/install/setup.bash
ros2 launch hexapod_description display.launch.py
```

```bash
# Terminal 2 — run the demo
cd ~/hexapod-ros2/ros2_ws
source install/setup.bash
ros2 run hexapod_nav known_map_demo
```

## RViz setup

In the RViz that `display.launch.py` opened:

1. **Global Options → Fixed Frame:** `odom`
2. **Add → RobotModel** → Description Topic: `/robot_description`
3. **Add → Map** → Topic: `/map`
4. **Add → Path** → Topic: `/planned_path`
5. **Add → Marker** → Topic: `/walls_3d` (this is the 3D extruded walls)

After the demo's startup delay (≈ 3 s), the robot starts walking the orange path. Terminal 2 logs progress and prints `■ DONE — goal reached` when finished.

## What the log tells you

```
Generating "random" map (seed=238203) 200×200 @ 0.05 m/cell
  Goal:  world (-2.70, +4.80) m
  A* path: 199 raw → 71 smooth waypoints (cost 215.4)
  Inflation: robot 8 + safety 4 = 12 cells (60 cm clearance)
▶ Path planned, robot will walk in 3.0 s
▶ WALKING — tracking 71 waypoints
  pose=(+1.20,+0.45, +12°) | wp 18/71 | dist_to_goal=4.85 m | yaw_cmd=+0.21
■ DONE — goal reached
```

The `Inflation:` line is the safety distance — `60 cm` means the path keeps the robot's centre that far from any wall cell, so leg tips (≈ 34 cm reach) stay ≈ 26 cm clear.

## Tunable parameters

Override at runtime with `--ros-args -p NAME:=VALUE`. Most useful:

| Parameter | Default | What it does |
|---|---|---|
| `map_type` | `random` | `random` / `walls` / `maze` / `shapes` — different obstacle layouts |
| `seed` | `-1` (random) | Reproducible — same seed → same map and same goal |
| `min_goal_dist_m` | `2.0` | Minimum distance from start to goal in metres |
| `robot_radius_cells` | `8` | Robot footprint in 5 cm cells (covers full leg span) |
| `safety_margin_cells` | `4` | Extra wall buffer in cells |
| `wall_height_m` | `0.30` | Height of the 3D wall cubes |
| `frame_id` | `odom` | Frame in which the map and path are published |
| `lookahead_m` | `0.30` | Steering look-ahead distance |
| `goal_tolerance_m` | `0.20` | How close the robot has to be before "DONE" |

Examples:

```bash
# Maze layout, deterministic
ros2 run hexapod_nav known_map_demo --ros-args -p map_type:=maze -p seed:=42

# Tighter clearance (50 cm) for crowded maps
ros2 run hexapod_nav known_map_demo --ros-args -p safety_margin_cells:=2

# Taller walls (0.5 m)
ros2 run hexapod_nav known_map_demo --ros-args -p wall_height_m:=0.5
```

## Common gotchas

| Symptom | Cause | Fix |
|---|---|---|
| Robot walks straight, doesn't stop | `hexapod_controller` not running, demo can't read TF | Make sure Terminal 1 is up |
| Map / Path doesn't appear in RViz "By topic" | Demo crashed during init (rare) or you opened RViz before publishers existed | Restart demo, click refresh on RViz topic dropdown |
| Legs sink into the floor | URDF body height wrong | Confirm `body_standing_height = 0.092` in `crab_model.xacro`, rebuild `hexapod_description` |
| Legs clip through walls | Inflation too tight | Bump `safety_margin_cells` higher |
| `odom` not in Fixed Frame dropdown | Static TF publisher hasn't fired yet | Just type `odom` directly into the field |
| `A* failed` repeatedly | Random goal is unreachable | Demo auto-retries 10 seeds; if all fail try `-p map_type:=random` or change `seed` |

## How it works (one paragraph)

`known_map_demo.py` first calls `MapGenerator` (from the planning library) to build a random occupancy grid, then `MapInflator` to grow obstacles by `(robot_radius + safety_margin)` cells. `AStarPlanner` runs over the inflated grid; `PathSmoother` simplifies with Ramer-Douglas-Peucker and re-interpolates. The smoothed path is published as `nav_msgs/Path`, the obstacle cells as a `Marker(CUBE_LIST)` for 3D visualization. A 20 Hz control loop reads `odom → base_link` from TF, finds the lookahead waypoint, and publishes a `Twist` whose `angular.z` (yaw rate) steers the gait controller's constant forward walk toward the next point. When the robot is within `goal_tolerance_m` of the last waypoint, the demo publishes `cmd: stop` and ends.
