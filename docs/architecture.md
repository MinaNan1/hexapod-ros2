# Architecture Overview

Where each piece lives, how data flows between them, and the conventions everything agrees on.

## The two ROS 2 workspaces (and why)

| Workspace | Purpose | Why separate |
|---|---|---|
| `hexapod_ws/` | Low-level: gait controller, URDF, robot description launch | Stable, rarely changes; matches a real robot's BSP |
| `ros2_ws/` | High-level: navigation, mapping, path planning, demos | Rapidly iterated; this is where new behaviour gets added |

`ros2_ws` depends on `hexapod_ws` being sourced first (so that `base_link`, `/joint_states`, `odom → base_link` TF and `/hexapod/body_pose` already exist when the navigation nodes come up).

## The standalone planning library

`hexapod_path_planning_final/hexapod_path_planning/` is a **pure-Python** library — no ROS imports — used by the ROS 2 nodes via `sys.path` injection. This means:

- It can be unit-tested without ROS
- It runs on a laptop without `colcon build`
- The same code is reused across `known_map_demo`, `lidar_path_planner`, `lidar_explorer_planner`

```
map_creation/
    map_generator.py      synthetic 200×200 maps (walls / maze / shapes / random)
    map_inflator.py       binary dilation by N cells (scipy.ndimage)
    map_validator.py      sanity-checks before A*
    frontier_explorer.py  finds boundary between known free and unknown

path_finding/
    astar_planner.py      8-connected A*, Euclidean heuristic
    path_smoother.py      Ramer-Douglas-Peucker + linear interpolation
    path_converter.py     grid → world → body_pose Twist commands
    reactive_planner.py   local replan around blockages

ros2_integration/
    hexapod_path_planner_node.py   older standalone ROS2 wrapper, uses /goal_pose + a saved .npy map
    path_follower.py               waypoint follower used by that wrapper
```

## TF chain — the contract everyone agrees on

```
world ── static identity (launch file) ──► odom
odom  ── dynamic dead-reckoning (hexapod_controller) ──► base_link
base_link ── static mount offset (launch file) ──► laser
```

- `world` is the global fixed frame. The static `world → odom` is identity for now (robot starts at origin); replace with a SLAM result if you want global localization.
- `odom → base_link` is dead-reckoned by `hexapod_controller` from gait parameters. The robot walks at constant `WALK_VEL_MS = 0.059 m/s`; only `body_pose.angular.z` (yaw rate) actually changes its heading.
- `base_link → laser` is set in `navigation.launch.py` / `exploration.launch.py` via the `LIDAR_MOUNT_*` constants. Default: 8 cm above the body, no rotation.

## Topic flow per demo

### Known map (Demo 1)

```
known_map_demo
    ↓ /map (OccupancyGrid)            → RViz Map display
    ↓ /planned_path (Path)            → RViz Path display
    ↓ /walls_3d (Marker, CUBE_LIST)   → RViz Marker display
    ↓ /hexapod/body_pose (Twist)      → hexapod_controller
    ↓ /hexapod/cmd (String)           → hexapod_controller   (walk / stop)
                ↓
hexapod_controller
    ↓ /joint_states                   → robot_state_publisher → URDF in RViz
    ↓ TF odom → base_link             → RViz, known_map_demo's pure-pursuit loop
```

### Live LiDAR (Demo 2)

```
lds-01 driver
    ↓ /scan (LaserScan)
                ↓
lidar_mapper                  reads /scan + TF (world ← laser)
    ↓ /map (OccupancyGrid)
                ↓
lidar_path_planner / lidar_explorer_planner
    ↓ /planned_path (Path)
    ↓ /hexapod/body_pose (Twist)
    ↓ /hexapod/cmd (String)
                ↓
hexapod_controller            ↺ feeds odom → base_link back into the loop
```

### Room corners (Demo 3)

```
room_corners
    ↓ /room_corners (MarkerArray)     → RViz
    ↓ /nearest_corner (PoseStamped)   → can be wired into a planner as a goal
```

## Coordinate conventions

- All world coordinates in **metres**
- Grid coordinates are `(row, col)` with `row` increasing downward (image-style)
- Cell `(r, c)` covers world `[origin + c·res, origin + (c+1)·res] × [origin + r·res, origin + (r+1)·res]`
- The OccupancyGrid `info.origin` field is the lower-left corner of cell `(0, 0)`
- For visualization markers and path waypoints, **cell centre = `origin + (c + 0.5)·res`** — keep this consistent or paths will look offset from walls
- Yaw is in **radians**, CCW positive, 0 = +x direction
- Body-pose Twist: `angular.z` = yaw rate (used by controller for steering); `linear.x/y` = body translation in mm/frame (used by some controllers but **ignored** by the current `hexapod_controller` for odometry)

## URDF height calibration

`base_link` sits at z = 0 (ground). The `thorax` is lifted by `body_standing_height` (≈ 0.092 m) so that the IK-computed foot tips land on z = 0. If you change `HIP_STANCE` or leg link lengths in the gait controller, recompute:

```
body_standing_height ≈ (femur + tibia) · sin(hip_stance°)
```

and update [crab_model.xacro](../hexapod_ws/src/hexapod_description/urdf/crab_model.xacro)'s `<xacro:property name="body_standing_height" ...>`.

## Build order

```bash
# 1. URDF + controller
cd ~/hexapod-ros2/hexapod_ws
colcon build
source install/setup.bash

# 2. Navigation + demos
cd ~/hexapod-ros2/ros2_ws
colcon build
source install/setup.bash
```

Source `hexapod_ws` *before* `ros2_ws` so that ros2_ws shadows any other workspace that happens to have a `hexapod_description` of the same name.

## Known sharp edges

- The IDE will warn that `map_creation`, `path_finding` etc. can't be resolved from inside the ROS 2 nodes. They can — the nodes inject the absolute path `~/hexapod-ros2/hexapod_path_planning_final/hexapod_path_planning` into `sys.path` at import time. The static analyzer doesn't see runtime path manipulation. Ignore the warnings.
- `hexapod_description` may be installed in more than one workspace on the same machine. If the URDF you edited isn't showing up, check `ros2 pkg prefix hexapod_description` and re-source workspaces in the right order.
- The `hexapod_controller` always walks at constant speed when its internal `_walking` flag is True (default at startup). High-level nodes have to publish `cmd: stop` first if they want the robot to stand still during a planning phase.
