# Demo 2 — Live 2D LiDAR Mapping (moving robot)

Build a 2D occupancy-grid map of an unknown room with the LDS-01 360° LiDAR
while the hexapod walks around. **The LiDAR rides on the moving robot**, so
every scan is taken from a different, drifting pose — the central problem this
demo solves.

## The moving-LiDAR problem

A 2D scan is just ranges in the *laser's own frame*. To stitch consecutive
scans into one consistent map, you must know where the laser was in a fixed
frame at the instant of each scan. On a hexapod that is hard:

- There are **no wheel encoders** — the only "odometry" is the gait
  controller's dead-reckoning (constant walk speed + commanded yaw), which
  drifts quickly because legs slip and the body sways.
- If you trust that pose blindly and just paint each scan into a global grid
  (the naive approach), the map **smears** as drift accumulates.

The fix is **scan-matching SLAM**: don't trust the odometry pose, *correct* it.
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) (the supported
ROS 2 2D-SLAM library) matches each incoming scan against the map built so far,
figures out the laser's true pose, and publishes a `map → odom` correction on
top of whatever odometry it's given. The map is accumulated in the corrected
`map` frame, so it stays sharp even as the raw odometry drifts.

This is the same stack used on mobile robots that lack wheel odometry: a laser
odometry / dead-reckoning prior feeding SLAM Toolbox, which refines it
([rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry),
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)).

## TF chain (one global frame: `map`)

```
map  ──(slam_toolbox: scan-match correction)──►  odom
odom ──(odometry source, see below)──────────►  base_link
base_link ──(static mount, launch file)──────►  laser  ──►  /scan
```

⚠️ Unlike `display.launch.py`, the mapping launch publishes **no** static
`world→odom`. SLAM Toolbox owns `map→odom`; a second parent for `odom` would
break the TF tree. The global fixed frame here is `map`, not `world`.

## Files involved

| Path | What it does |
|---|---|
| [`ros2_ws/src/hexapod_gazebo/launch/mapping.launch.py`](../ros2_ws/src/hexapod_gazebo/launch/mapping.launch.py) | **Primary bringup** — controller + LDS-01 driver + `base_link→laser` TF + SLAM Toolbox |
| [`ros2_ws/src/hexapod_gazebo/config/mapper_params_online_async.yaml`](../ros2_ws/src/hexapod_gazebo/config/mapper_params_online_async.yaml) | SLAM Toolbox params, tuned to trust scan matching over the crude gait prior |
| [`ros2_ws/src/rf2o_laser_odometry/`](../ros2_ws/src/rf2o_laser_odometry/) | Optional higher-accuracy laser odometry (`odom_source:=rf2o`) |
| [`ros2_ws/src/hexapod_control/hexapod_nav/lidar_mapper.py`](../ros2_ws/src/hexapod_control/hexapod_nav/lidar_mapper.py) | Legacy homegrown log-odds mapper (no drift correction) — see "Alternative" below |

## Odometry source: `gait` (default) vs `rf2o`

`mapping.launch.py` takes an `odom_source` argument that picks who owns
`odom→base_link`. Only **one** node may own it.

| `odom_source` | Who owns `odom→base_link` | When to use |
|---|---|---|
| `gait` (default) | `hexapod_controller` dead-reckoning | Simplest; reuses the proven walking stack. SLAM corrects the drift. |
| `rf2o` | `rf2o_laser_odometry` (laser scan-matching) | More accurate prior. The controller runs legs-only via `publish_tf:=false`, so only rf2o publishes the transform. |

The `publish_tf` parameter was added to `hexapod_controller` precisely so the
gait controller can run legs-only when an external odometry node is in charge.

## Build (inside the ROS container)

Everything runs in the `hexapod_ros` container (started by `start_ros.sh`),
where the project is mounted at `/hexapodd` and the LDS-01 on `/dev/ttyUSB0`
is already accessible (`--privileged -v /dev:/dev`).

`ros-humble-slam-toolbox` and `ros-humble-hls-lfcd-lds-driver` are **already
baked into the image** (see `hexapod.dockerfile`), so no apt install is needed
for the default pipeline. Only the map saver is missing:

```bash
# Only needed if you want to save the map with map_saver_cli:
apt install -y ros-humble-nav2-map-server

# Build the workspaces. ros2_ws MUST select packages — a plain `colcon build`
# tries to build the vendored rtabmap (3D SLAM, needs OpenCV) and aborts.
cd /hexapodd/hexapod_ws && colcon build && source install/setup.bash
cd /hexapodd/ros2_ws \
    && colcon build --symlink-install --packages-select hexapod_nav hexapod_gazebo \
    && source install/setup.bash
# (add rf2o_laser_odometry to --packages-select for odom_source:=rf2o)
# Or just run /hexapodd/build_ws.sh, which ignores all rtabmap_* packages.
```

> Rebuild hexapod_ws even if you only touched ros2_ws — it picks up the new
> `publish_tf` parameter on the controller.
>
> Open extra container shells with `docker exec -it hexapod_ros bash`.

## Run

One terminal does everything (both workspaces sourced, hexapod_ws first):

```bash
ros2 launch hexapod_gazebo mapping.launch.py
# higher-accuracy odometry:
ros2 launch hexapod_gazebo mapping.launch.py odom_source:=rf2o
# custom LiDAR port / headless:
ros2 launch hexapod_gazebo mapping.launch.py lidar_port:=/dev/ttyUSB1 use_rviz:=false
```

In **RViz**: Fixed Frame `map`. Add **Map** (`/map`), **LaserScan** (`/scan`),
**TF**, **RobotModel**.

Drive the robot around the room (web UI, or from another terminal):

```bash
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'walk'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'stop'"
```

Walk it along the walls; the occupancy grid fills in and stays aligned because
SLAM is correcting the pose every scan.

## Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/hexapod_map
# → ~/hexapod_map.pgm + ~/hexapod_map.yaml
```

That saved map is what Demo 1's known-map A\* planner can consume.

## Key SLAM parameters (already tuned for this robot)

In `mapper_params_online_async.yaml`:

| Parameter | Value | Why |
|---|---|---|
| `mode` | `mapping` | Build a new map (vs `localization`) |
| `max_laser_range` | `3.0` | LDS-01 is noisy past ~3.5 m; clip for a clean map |
| `minimum_travel_distance` / `_heading` | `0.0` | Must be 0 with a crude/"fake" odometry prior so scans aren't dropped |
| `distance_variance_penalty` / `angle_variance_penalty` | `0.3` / `0.5` | Lowered → SLAM trusts scan matching **more** than the gait prior |
| `resolution` | `0.1` | Map cell size (m) |
| `do_loop_closing` | `true` | Re-aligns the map when the robot revisits a place |

If the map still smears while walking, the gait prior is too poor — switch to
`odom_source:=rf2o`.

## Common gotchas

| Symptom | Cause | Fix |
|---|---|---|
| `package 'hexapod_control' not found` from a launch file | Old launch used the pre-rename package name | Fixed — ros2_ws nodes are now `hexapod_nav` |
| TF error: `odom` has two parents | A static `world→odom` (e.g. from `display.launch.py`) is running alongside SLAM | Use `mapping.launch.py` only; don't also run `display.launch.py` |
| Robot model flickers / `/joint_states` has 2 publishers | Two controllers running | Run the controller only via `mapping.launch.py` |
| Map smears as the robot moves | Gait odometry drift outpacing SLAM | `odom_source:=rf2o` |
| Walls rotated 180° | LiDAR mount yaw wrong | Set `LIDAR_MOUNT_YAW = '3.14159'` in `mapping.launch.py` |
| No `/scan` | Wrong USB port | `lidar_port:=/dev/ttyUSBx` |

## Alternative: the homegrown `lidar_mapper`

`navigation.launch.py` and `exploration.launch.py` use a hand-written log-odds
mapper (`lidar_mapper.py`) that looks up `world ← laser` from TF per scan and
raycasts each beam. It's a useful, dependency-free reference and feeds the
`world`-frame planners (`lidar_path_planner`, `lidar_explorer_planner`), **but
it cannot correct odometry drift** — it blindly trusts the TF pose, so it
smears on a moving robot exactly as described above. Prefer SLAM Toolbox
(`mapping.launch.py`) for real mapping; keep `lidar_mapper` for static-LiDAR
tests or as a teaching reference.

## Next steps

- Run `mapping.launch.py` on the Jetson, walk a room, confirm the map stays
  sharp over a full loop (validates SLAM under real gait drift).
- Compare `odom_source:=gait` vs `:=rf2o` on the same room to quantify the
  accuracy gain from laser odometry.
- Wire the saved map back into the A\* planner (Demo 1) for plan-on-known-map.
