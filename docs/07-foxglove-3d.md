# Foxglove Studio — live 3D view from the laptop

The web UI shows the 2D occupancy grid and the RGB camera. To see the **3D
point-cloud map**, **LIDAR scan ring**, and **TF tree** rotating in real time,
use Foxglove Studio. It connects to the same `rosbridge_server` the web UI uses,
so no extra setup on the Jetson is needed.

## One-time setup on the laptop

1. Download **Foxglove Studio** from <https://foxglove.dev/download> (free).
   Available for Windows, Mac, Linux. ~150 MB.
2. Install and launch it.
3. Click **Open connection** → choose **Rosbridge (WebSocket)**.
4. Connection URL: `ws://192.168.0.20:9090` (replace IP if your Jetson moved).
   - The Jetson stack must be running (`launch_all.sh`) — Foxglove reaches
     ROS through the `webui_ros` tmux window's rosbridge.
5. Click **Open**. You should see "Connected" within ~2 seconds.

## Loading the layout

The repo ships a pre-built layout that opens three panels (3D + Camera + 2D Map).

1. In Foxglove: **Layouts** (top-left icon) → **Import from file**.
2. Select `~/hexapod-ros2/foxglove_layouts/hexapod_3d.json`
   (or wherever you keep the project on the laptop).
3. The window switches to a 3-panel layout:
   - **Left (65%)** — 3D scene: point cloud (`/rtabmap/cloud_map`), live LIDAR
     scan dots (`/scan` in orange), trajectory polyline (`/rtabmap/mapPath` in
     cyan), and the URDF robot model that follows the live `base_link` TF.
   - **Right top** — RGB camera feed (`/camera/color/image_raw/compressed`).
   - **Right bottom** — 2D occupancy grid (`/rtabmap/map`).

## Controls in the 3D panel

- **Drag** → orbit the camera around the robot
- **Scroll** → zoom
- **Shift + drag** → pan
- The view follows `base_link` by default (`followMode: "follow-pose"`),
  so the robot stays centered as it moves.

## When you should expect to see something

- **Point cloud appears**: about 5 seconds after RTAB-Map starts publishing.
  Initially small (one keyframe), grows as the robot moves.
- **LIDAR ring**: visible immediately if the LDS-01 is plugged in and
  `/scan` is publishing at ~5 Hz.
- **Robot model**: depends on `/robot_description` being latched. If it
  doesn't appear, run `ros2 topic echo /robot_description --once` inside
  the container to confirm RSP published it.

## Saving your own layout

Adjust panels however you like, then **Layouts → Export to file** to save
the JSON next to `hexapod_3d.json` for next session. Layouts are pure JSON,
so they're diff-friendly and safe to commit to git.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| "Connection failed" | Stack not running or wrong IP | `ssh jetson@192.168.0.20 tmux ls` — should list `hexapod` session |
| 3D scene is blank | TF tree gap; "fixed frame" target doesn't exist | Set fixed frame to `map` (not `world`); make sure RTAB-Map is publishing `/tf` `map → odom` |
| Point cloud appears but doesn't grow | RTAB-Map hasn't added new keyframes | Walk the robot a few steps; threshold is 2 cm of movement |
| LIDAR points blink in & out | Decay time set too short | In the 3D panel topic settings, set `/scan` decayTime to 1.0s |
| Performance laggy | WiFi bandwidth saturated by point cloud | In `/rtabmap/cloud_map` settings, increase `pointSize` and reduce `decayTime`, or hide the topic during driving |
