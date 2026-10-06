# Laptop 3D Visualization

Everything here runs on the **laptop** (not the Jetson). Goal: see the hexapod's
3D map, robot model, LiDAR scan, trajectory, and camera — like the RViz
reference photo — while the Jetson does the SLAM.

## Honest expectation: the 3D cloud will NOT be colored

The reference video used a Kinect2, whose RGB and depth are factory-registered,
so its 3D point cloud is photo-colored. **Your Astra Pro's color comes from a
separate UVC interface that is not registered with the depth sensor**, so
RTAB-Map fuses depth + LiDAR only. The result:

- 3D map (`/rtabmap/cloud_map`) is **height-colored** (turbo colormap by Z), not
  photo-colored. Geometrically correct, just not pretty RGB.
- The live RGB camera is still shown in its own panel, side by side.

Recovering colored 3D would need a one-time chessboard calibration of the
depth↔color extrinsics. Not done; not required for mapping.

---

## Path 1 — RViz2 (best quality, needs DDS working)

Use this if the laptop is on Ethernet or a network that passes UDP multicast.

```bash
# One-time: build the description package so RViz can resolve robot meshes
bash setup_laptop_viz.sh

# Every session (Jetson stack must be running):
bash view_3d.sh
```

`view_3d.sh` exports `ROS_DOMAIN_ID=42` + `rmw_fastrtps_cpp`, sources the viz
workspace, and opens RViz with `hexapod_3d_mapping.rviz` (RobotModel, LaserScan,
2D Map, 3D MapCloud, trajectory, camera image).

If RViz opens but everything is empty and `ros2 topic list` doesn't show
`/rtabmap/map`, your WiFi is blocking multicast → use Path 2, or ask Claude to
wire in a FastDDS discovery server on the Jetson.

## Path 2 — Foxglove Studio (works over WiFi, easier)

Use this if DDS discovery fails (common on home WiFi). Foxglove talks to the
Jetson's rosbridge over a WebSocket, which always works if you can open the web
UI.

1. Install Foxglove Studio (free): https://foxglove.dev/download
2. Open it → **Open connection** → **Rosbridge** →
   `ws://192.168.0.20:9090` → Open.
3. **Layout → Import from file** → pick `hexapod_foxglove_layout.json`.
4. You get a 3D panel (height-colored map cloud + 2D grid + scan + trajectory)
   next to the live camera.

Foxglove's robot-model/URDF mesh rendering is limited; the 3D **map** is the
main payload and renders fine. For a textured robot model, use Path 1 (RViz).

---

## Files

| File | Purpose |
|---|---|
| `hexapod_description/` | URDF + 4 STL meshes (extracted from the design ZIP, meshes only — no gcode/launch/package overwrites) |
| `setup_laptop_viz.sh` | One-time: builds the description package into `~/hexapod_viz_ws` |
| `view_3d.sh` | Launches RViz with the 3D-mapping layout |
| `hexapod_3d_mapping.rviz` | RViz layout matching the reference photo |
| `hexapod_foxglove_layout.json` | Foxglove layout (rosbridge path) |

## Note on `kinect+rplidar_bz2.bag`

That 8 GB file in the project root is the **reference recording from the video**
you watched — Kinect2 + RPLidar, ROS 1 `.bag` format. It is *not* directly
usable on this hexapod: different sensors, different topic names, and ROS 1 vs
your ROS 2 stack. It's a visual target, not an input. Don't try to replay it
into the live system — at most you'd replay it in a separate RViz to study what
"good" looks like, which needs ros1_bridge or a bag conversion. Recommend
leaving it aside.
