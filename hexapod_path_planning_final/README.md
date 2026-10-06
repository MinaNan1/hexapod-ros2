# hexapod_path_planning_final

Pure-Python 2D path-planning library used by the ROS 2 navigation nodes. **No ROS dependencies** — can be imported from a plain Python script or unit-tested standalone.

## Layout

```
hexapod_path_planning/
  map_creation/
    map_generator.py       random / walls / maze / shapes 200×200 occupancy grids
    map_inflator.py        binary dilation of obstacles by N cells (scipy.ndimage)
    map_validator.py       sanity checks before A* runs
    frontier_explorer.py   finds boundary cells between known free and unknown
  path_finding/
    astar_planner.py       8-connected A*, Euclidean heuristic
    path_smoother.py       Ramer-Douglas-Peucker + linear interpolation
    path_converter.py      grid → world → body_pose Twist commands
    reactive_planner.py    local replan around newly-discovered blockages
  ros2_integration/
    hexapod_path_planner_node.py   older standalone ROS2 wrapper
    path_follower.py               waypoint follower used by that wrapper
  main.py                  Stage 1 demo (known map + A*)
  stage2_*.py              Stage 2 demos (unknown map + frontier exploration)
  outputs/                 saved maps and rendered path images
```

## Standalone usage (no ROS)

```bash
cd hexapod_path_planning
python3 main.py --map walls --save           # plan on a default map, save outputs
python3 main.py --load my_map.npy            # plan on a custom map
python3 stage2_explore_then_plan.py          # frontier exploration demo
```

Outputs land in `outputs/` (PNG visualizations, NPY map snapshots, command JSON).

## Library usage

```python
from map_creation.map_generator import MapGenerator
from map_creation.map_inflator   import MapInflator
from path_finding.astar_planner  import AStarPlanner
from path_finding.path_smoother  import PathSmoother

grid     = MapGenerator(200, 200).walls()
inflated = MapInflator(robot_radius_cells=8).inflate(grid)
result   = AStarPlanner(inflated, start=(10,10), goal=(180,180)).plan()
if result.found:
    smooth = PathSmoother().rdp_simplify(result.path, epsilon=2.0)
    smooth = PathSmoother().interpolate(smooth, spacing=3.0)
    # smooth is now a list of (row, col) waypoints
```

## How the ROS 2 nodes consume this library

The nodes inject this folder onto `sys.path` at runtime, so editing files here takes effect on the next `ros2 run` *without* needing `colcon build`. See the import block at the top of:

- `ros2_ws/src/hexapod_control/hexapod_control/known_map_demo.py`
- `ros2_ws/src/hexapod_control/hexapod_control/lidar_path_planner.py`
- `ros2_ws/src/hexapod_control/hexapod_control/lidar_explorer_planner.py`

(The IDE will flag `from map_creation.map_generator import ...` as unresolved — it's a static-analysis false positive, the runtime import works via the `sys.path` injection.)

## Dependencies

```
numpy
scipy        # for binary_dilation in map_inflator
matplotlib   # for visualization in main.py / stage2_*.py
Pillow       # for image-based map loading in map_generator
```

The standalone scripts run in `../hexapod_ai_env` (a Python venv at the project root).
