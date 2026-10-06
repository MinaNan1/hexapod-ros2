# Demo 3 — Room Corners

You declare the room dimensions and which corner the hexapod is standing at; the node computes the four corner positions in the world frame and continuously reports which one is currently nearest.

Useful as a localization anchor (you know where the corners are without doing SLAM) and as a quick navigation goal source ("go to nearest corner").

## Files involved

| Path | What it does |
|---|---|
| [`ros2_ws/src/hexapod_control/hexapod_control/room_corners.py`](../ros2_ws/src/hexapod_control/hexapod_control/room_corners.py) | The node |

## Corner labels

Top-down view, +x right, +y up:

```
        +y
         ▲
    B ───┼─── A
    │    │    │
    │    └────┼──► +x
    │         │
    D ─────── C
```

| Label | Position |
|---|---|
| **A** | top-right (max x, max y) |
| **B** | top-left  (min x, max y) |
| **C** | bottom-right (max x, min y) |
| **D** | bottom-left (min x, min y) |

Pick the label that matches where the robot is *physically* placed when it starts.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/room_corners` | `visualization_msgs/MarkerArray` | Four labelled spheres + a grey rectangle outline |
| `/nearest_corner` | `geometry_msgs/PoseStamped` | Updated at `publish_rate`; ready to feed straight into a planner as a goal |

## Run

Two terminals minimum.

```bash
# Terminal 1 — controller (publishes odom → base_link)
source ~/hexapod-ros2/hexapod_ws/install/setup.bash
ros2 run hexapod_control hexapod_controller
```

```bash
# Terminal 2 — the node, parameters chosen at launch time
cd ~/hexapod-ros2/ros2_ws
source install/setup.bash
ros2 run hexapod_nav room_corners --ros-args \
  -p room_width_m:=4.0 \
  -p room_height_m:=3.0 \
  -p start_corner:=A
```

If you also want the static `world → odom` TF (so you can use Fixed Frame `world` in RViz), run `display.launch.py` or `navigation.launch.py` instead of the bare controller. Otherwise use Fixed Frame `odom` and pass `-p world_frame:=odom`.

## RViz setup

- Fixed Frame: `world` (or `odom` if you skipped the static TF publisher)
- **Add → MarkerArray** → topic `/room_corners` — see four spheres labelled A/B/C/D plus a rectangle outline
- **Add → Pose** → topic `/nearest_corner` — arrow at the currently nearest corner

The terminal logs the nearest corner only when it changes (no spam).

## Tunable parameters

| Parameter | Default | What it does |
|---|---|---|
| `room_width_m` | 3.0 | Room extent along x in metres |
| `room_height_m` | 3.0 | Room extent along y in metres |
| `start_corner` | `A` | Which corner the robot is standing at: `A`, `B`, `C`, or `D` |
| `world_frame` | `world` | Frame the corners are published in |
| `robot_frame` | `base_link` | Frame whose pose is compared against the corners |
| `publish_rate` | 2.0 | Hz |

## Examples

```bash
# Robot at top-left of a 5 × 4 m room
ros2 run hexapod_nav room_corners --ros-args \
  -p start_corner:=B -p room_width_m:=5.0 -p room_height_m:=4.0

# Robot at bottom-right of a 6 × 6 m room, no static world TF available
ros2 run hexapod_nav room_corners --ros-args \
  -p start_corner:=C -p room_width_m:=6.0 -p room_height_m:=6.0 \
  -p world_frame:=odom
```

## How it works (one paragraph)

The node looks up the chosen `start_corner` in a hard-coded layout table and computes the other three corners' world coordinates. Each tick it reads `world ← base_link` (or `odom ← base_link`) from TF, computes Euclidean distance to each corner, picks the minimum, and publishes that as a `PoseStamped`. The marker array is rebuilt with each tick so RViz doesn't time them out.

## Next steps for this demo

- Subscribe `/nearest_corner` from `lidar_explorer_planner` so a parameter flip turns it into a "go to nearest corner" autonomous behaviour.
- Add a `/set_room` service to change room size and start corner at runtime without restarting the node.
- (Stretch) Auto-detect orientation from the LiDAR by RANSAC-fitting the longest two perpendicular wall lines.
