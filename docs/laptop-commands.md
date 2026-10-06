# Hexapod — Laptop Command Reference

Everything below runs on the **laptop**.
The Jetson runs the robot brain; the laptop runs the visualizer, the browser, the controller input, and any commands you send.

The Jetson is reached at `192.168.0.20` on the LAN. If you ever switch networks and the IP changes, update the address everywhere in this doc (or fall back to its mDNS name `hexapod-desktop.local`, which resolves to whatever IP it currently has).

---

## How the system is split

Mental model — what lives where:

```
┌─────────────── LAPTOP ─────────────────┐    ┌──────────── JETSON ─────────────┐
│                                        │    │                                 │
│   RViz   (visualizes the robot)        │◄───┤   hexapod_controller            │
│                                        │    │   (gait, TF, joint_states)      │
│   Browser   → http://192.168.0.20:5000   │◄───┤                                 │
│                                        │    │   robot_state_publisher         │
│   PS3 controller (USB) — eventually    │───►│   (URDF + link transforms)      │
│                                        │    │                                 │
│   `ros2 topic pub …` commands          │───►│   rosbridge (WebSocket :9090)   │
│                                        │    │                                 │
│                                        │    │   Flask web app (:5000)         │
└────────────────────────────────────────┘    └─────────────────────────────────┘
            both share ROS_DOMAIN_ID=42 → DDS discovers them automatically
```

You launch everything that lives on the Jetson with **one** SSH command (`launch_all.sh`).
You launch RViz and the browser yourself **on the laptop**.

---

## First time — one-time setup (already done)

These are written here so you can re-do them if you ever reinstall the laptop or use a different one.

### 1. Configure ROS 2 to talk to the Jetson

Open `~/.bashrc` and add at the bottom:
```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/hexapod-ros2/hexapod_ws/install/setup.bash
source ~/hexapod-ros2/ros2_ws/install/setup.bash
```
Then reload:
```bash
source ~/.bashrc
```
**Why:** every new terminal will join ROS network 42 and know about your locally-built hexapod packages.

### 2. Set up passwordless SSH to the Jetson

```bash
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
ssh-copy-id jetson@192.168.0.20
```
**Why:** so you don't have to type the Jetson's password every time the launcher SSHes in.

Test:
```bash
ssh jetson@192.168.0.20
```
Should log you straight in with no password prompt.

---

## Every day — full session, in order

This is the sequence to run any time you want to use the robot from a fresh boot.

### Step 1 — Start the robot on the Jetson

```bash
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/launch_all.sh
```

**What it does (in the background):**
- Boots the Docker container if it isn't already running
- Starts `hexapod_controller` (gait + odometry + TF)
- Starts `robot_state_publisher` (URDF → link transforms)
- Starts `hiwonder_servo_bridge` (drives the 18 Hiwonder HX-35HM servos)
- Starts `rosbridge_server` on port 9090 (used by the web UI)
- Starts `ui_teleop` (bridges web UI commands into the robot)
- Starts the Flask web server on port 5000

**Wait ~15 seconds**, then continue to Step 2. The command returns immediately — the services keep running inside a tmux session on the Jetson, surviving even if your laptop disconnects.

### Step 2 — Open RViz on the laptop

In a fresh laptop terminal:
```bash
ros2 run rviz2 rviz2
```

**First time only**, configure RViz:
1. Top-left, under "Global Options", set **Fixed Frame** = `world`
2. Click **Add** (bottom-left), pick **By topic**, find `/robot_description`, add **RobotModel**
3. Click **Add** again, **By display type**, add **TF**
4. Save the layout: **File → Save Config**

Now you'll see the hexapod standing in neutral pose, broadcast live from the Jetson.

### Step 3 — Open the web UI

In any browser on the laptop:
```
http://192.168.0.20:5000
```

You should see:
- Top-right indicator → **Connected to ROS** (green)
- Click **Walk** → hexapod walks in RViz
- Click **Stop** → legs settle to neutral
- Joystick on the left → speed / steering
- Sliders → roll / pitch / yaw / body height

If the indicator stays red ("Disconnected"), see Troubleshooting below.

### Step 4 — Stop everything when you're done

```bash
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/stop_all.sh
```

**What it does:** kills all ROS nodes, the Flask server, and tears down the Docker container. Clean slate for the next session.

---

## Mid-session — useful things to know

### Watch the robot's logs in real time

```bash
ssh -t jetson@192.168.0.20 tmux attach -t hexapod
```

You're now inside the tmux session on the Jetson. It has **5 windows**:

| Window | Number | What it shows |
|---|---|---|
| container | 0 | The container's bash shell (boring, mostly idle) |
| robot | 1 | hexapod_controller logs — gait events, frame count |
| servos | 2 | hiwonder_servo_bridge logs — per-joint servo positions |
| webui_ros | 3 | rosbridge connections, ui_teleop commands |
| flask | 4 | Flask access logs — every browser hit |

**Tmux keystrokes** (all start with `Ctrl-b`):
- `Ctrl-b` then `n` → next window
- `Ctrl-b` then `p` → previous window
- `Ctrl-b` then `0` / `1` / `2` / `3` → jump to that window
- `Ctrl-b` then `d` → **detach** (closes your view, keeps everything running)

**Important:** never press `Ctrl-c` inside tmux unless you actually want to kill that service. Use `Ctrl-b d` to leave.

### Drive the robot from the command line (no UI)

Sometimes faster than clicking. From any laptop terminal:

```bash
# Walk forward at default speed
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'walk'"

# Stop walking — legs settle to neutral pose
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'stop'"

# Walk backward
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'backward'"

# Trigger the salute emote
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'salute'"

# End the salute (legs return to neutral)
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'salute_stop'"

# Trump pump emote
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'trump'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'trump_stop'"

# Rotate in place
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'rotate_left'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'rotate_right'"

# Body-pose sliders (roll/pitch/yaw_tilt rad ; tx/ty/tz mm)
ros2 topic pub --once /hexapod/posture geometry_msgs/msg/Twist \
    "{angular: {x: 0.0, y: 0.1, z: 0.0}, linear: {x: 0.0, y: 0.0, z: 10.0}}"
```

**Tip:** `--once` means "publish exactly one message and exit". Without it, `ros2 topic pub` republishes forever.

**Reset-to-zero behaviour:** every cmd transition (walk → stop, stop → walk,
into/out of an emote) snaps the controller's gait phase to 0, so the robot
always begins each command from a known starting position rather than mid-
stride. Same idea as the legacy controller's `float_frame = 0.0` reset.

**Topic split** (new in the v3 controller):
- `/hexapod/posture` (Twist) — body-pose sliders (roll/pitch/yaw_tilt/tx/ty/tz).
- `/hexapod/body_pose` (Twist) — joystick / motion: `angular.z` = steering yaw,
  `linear.x` = 0..1 speed multiplier.
- The yaw slider on `/hexapod/posture` *also* steers the robot while walking
  (so a single yaw control does both jobs).
- The Tkinter teleop GUI publishes sliders on `/hexapod/posture`; the web UI
  and any joystick node should publish motion on `/hexapod/body_pose`.

### Check that the cross-machine link is healthy

```bash
# Should list ~10 topics including /joint_states, /tf, /hexapod/cmd
ros2 topic list

# Should print average rate: ~20 Hz
ros2 topic hz /joint_states

# Should show /joint_states has exactly ONE publisher (the Jetson's controller)
ros2 topic info /joint_states --verbose | grep "Publisher count"

# Should list nodes from both Jetson and laptop
ros2 node list
```

If any of these look wrong, jump to Troubleshooting.

### Open a manual shell on the Jetson

Whenever you need to poke around the Jetson directly:
```bash
ssh jetson@192.168.0.20
```
You're now sitting at the Jetson. Type `exit` to come back to the laptop.

---

## Common scenarios

### "I just want to look at the robot, not move it"
1. Step 1 (`launch_all.sh`)
2. Step 2 (RViz)
3. Skip the browser — the robot just stands in neutral pose

### One-time setup: run `setup_hiwonder.sh` (udev rule + rebind)

The Hiwonder controller's `/dev/hidrawN` number is **not** stable — it depends
on what else is on the USB bus and shifts on every re-enumeration. On the
laptop it was `hidraw1`; on the Jetson (wireless dongle + keyboard also on the
bus) it lands on `hidraw5` — and `hidraw1` there is actually a wireless dongle,
so the old hardcoded path was writing servo packets into the wrong device.

Two fixes are now in place:
1. **The bridge auto-detects the board by its USB VID:PID `0483:5750`** — so the
   `hidrawN` number no longer matters. It also prefers a `/dev/hiwonder` symlink
   if present, and accepts `--device=/dev/hidrawN` or `$HEXAPOD_HID_DEVICE` to
   override.
2. A one-time host setup script installs the udev rule (stable `/dev/hiwonder`
   + `0666` perms) and rebinds the HID interface if the Nano left it unbound:

```bash
ssh -t jetson@192.168.0.20 'sudo bash ~/hexapod-ros2/scripts/setup_hiwonder.sh'
```

It ends by printing `/dev/hiwonder -> hidrawN`. After this, no per-boot `chmod`
is needed. **Run on the host, not inside the container** — it touches
`/etc/udev` and `/sys`.

If it reports the board's HID interface has no driver / `/dev/hiwonder` never
appears: **unplug and replug the Hiwonder USB cable**, then re-run it. List
what the system currently sees with:
```bash
ssh -t jetson@192.168.0.20 "docker exec -it hexapod_ros \
    ros2 run hexapod_control hiwonder_servo_bridge --list"
```
Look for the line flagged `← HIWONDER`. If no line shows `0483:5750`, the board
isn't enumerated as HID (cable / power / the interface came unbound).

### "I just plugged the Jetson into the servo board and want to make it walk"

This is the *minimal* hardware bring-up — no web UI, no rosbridge, no Flask.
Just RSP + the Cartesian gait controller + the Hiwonder bridge in one launch.

**Step 1 — If you've installed the udev rule, skip device discovery.**
The bridge defaults to `/dev/hiwonder`. Otherwise find the right hidraw:
```bash
ssh -t jetson@192.168.0.20 "docker exec -it hexapod_ros \
    ros2 run hexapod_control hiwonder_servo_bridge --list"
```
Look for `HID_NAME=...Hiwonder...` — note its `/dev/hidrawN`.

**Step 2 — Launch.**
With udev rule installed:
```bash
ssh -t jetson@192.168.0.20 "docker exec -it hexapod_ros bash -lc '\
    source /opt/ros/humble/setup.bash && \
    source /hexapodd/hexapod_ws/install/setup.bash && \
    ros2 launch hexapod_description hardware.launch.py'"
```
Without the rule (override the device path):
```bash
ssh -t jetson@192.168.0.20 "docker exec -it hexapod_ros bash -lc '\
    chmod 666 /dev/hidrawN && \
    source /opt/ros/humble/setup.bash && \
    source /hexapodd/hexapod_ws/install/setup.bash && \
    ros2 launch hexapod_description hardware.launch.py hid_device:=/dev/hidrawN'"
```
(Replace `hidrawN` with whatever step 1 found.)

**Step 3 — On the laptop, drive it.**

Open the teleop GUI:
```bash
ros2 run hexapod_control hexapod_teleop
```
…or just publish commands directly:
```bash
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'walk'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'stop'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'salute'"
```

That's it. The bridge takes `/joint_states` from the controller, translates each leg's delta-from-standing into raw Hiwonder positions, and writes HID packets to the bus.

### "I broke something, restart cleanly"
```bash
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/stop_all.sh
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/launch_all.sh
```
Then re-open RViz and the browser.

### "The Jetson is unreachable / hung"
Power-cycle it. Wait for it to boot (~30 s), then go to Step 1.

### "I want to change the robot's code"
Edit the Python files on the Jetson (or use SSH/VS Code remote). After editing:
```bash
ssh jetson@192.168.0.20 docker exec hexapod_ros bash -c \
    "cd /hexapodd/hexapod_ws && colcon build --packages-select hexapod_control --symlink-install"
```
Then restart the stack (stop_all + launch_all).

---

## Troubleshooting

### `ping 192.168.0.20` times out
- LAN cable plugged at both ends?
- `nmcli device status` on both machines — both should say `connected`
- Try `nmcli connection up "Wired connection 1"` on whichever side shows `disconnected`

### `ssh jetson@192.168.0.20` hangs or asks for password
- Hangs: probably the IP changed. Find it: log in to the Jetson directly and run `hostname -I`
- Asks for password: re-run `ssh-copy-id jetson@192.168.0.20`

### `ros2 topic list` is empty
The two machines aren't seeing each other on ROS. Verify both have the same env vars:
```bash
# On the laptop:
echo "DOMAIN=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"

# On the Jetson (via SSH into container):
ssh jetson@192.168.0.20 docker exec hexapod_ros bash -c 'echo "DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"'
```
Both must say `DOMAIN=42 RMW=rmw_fastrtps_cpp`. If they differ, restart the offending side.

### Web UI page loads but says "Disconnected from ROS"
rosbridge isn't running. Restart the stack:
```bash
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/stop_all.sh
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/launch_all.sh
```

### RViz: "Error retrieving file [package://hexapod_description/meshes/...]"
Your terminal didn't source the workspace. Close RViz and:
```bash
source ~/hexapod-ros2/hexapod_ws/install/setup.bash
ros2 run rviz2 rviz2
```
(Already in your `~/.bashrc`, but you may have launched RViz from a shell that pre-dates the edit.)

### Robot legs glitching / flickering / disappearing
Means there are duplicate publishers — leftover processes. Always fix with:
```bash
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/stop_all.sh
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/launch_all.sh
```
Then confirm:
```bash
ros2 topic info /joint_states --verbose | grep "Publisher count"
```
Should be `1`. If it's `2`, kill processes manually:
```bash
ssh jetson@192.168.0.20 docker exec hexapod_ros pkill -f hexapod_controller
ssh jetson@192.168.0.20 docker exec hexapod_ros pkill -f robot_state_publisher
```
and re-run launch_all.

### Discovery flaky after switching to WiFi
Some home routers block UDP multicast (which DDS uses). Quick fix — switch to discovery-server mode. Add this to your laptop's `~/.bashrc`:
```bash
export ROS_DISCOVERY_SERVER=<jetson-wifi-ip>:11811
```
And add `-e ROS_DISCOVERY_SERVER=0.0.0.0:11811` to `start_ros.sh` so the container does the same.

---

## Bus servos (Hiwonder HX-35HM, v3 design)

The current build uses 18 × Hiwonder HX-35HM smart bus servos driven over USB
HID. The new controller (Cartesian foot-trajectory gait, JetHexa method)
publishes joint angles as **deltas from the standing pose** ("standing → 0 rad")
and `hiwonder_servo_bridge` translates each joint into a raw servo position.
The bridge **auto-detects** the controller board by its USB ID `0483:5750`
(it prefers the `/dev/hiwonder` udev symlink, then scans for the ID), so the
`/dev/hidrawN` number never has to be guessed.

### ✅ Everyday bring-up — ONE command (CONFIRMED WORKING)

From a **laptop** terminal (it runs on the Jetson host via ssh; type the
password when prompted):
```bash
ssh -t jetson@192.168.0.20 'sudo bash ~/hexapod-ros2/scripts/bringup_servos.sh'
```
This single command, on a cold Jetson, does **everything**:
1. Confirms the device (`/dev/hiwonder → /dev/hidraw5`, rebinds it if the Nano
   left the HID interface unbound, installs the udev rule on first run).
2. Ensures the `hexapod_ros` Docker container is running (builds the image if
   it's the very first time).
3. Rebuilds `hexapod_control`.
4. Launches `hexapod_controller` **and** `hiwonder_servo_bridge` in a tmux
   session called `servo_bringup`.

**The udev rule is permanent** — after the first run, reboots come up with
`/dev/hiwonder` already present, and re-running the script skips straight to
relaunching the two nodes. No more per-boot `chmod`.

Watch the two nodes:
```bash
ssh -t jetson@192.168.0.20 'sudo tmux attach -t servo_bringup'   # Ctrl-b n switches, Ctrl-b d detaches
```
Stop them (note **`sudo`** — the session is owned by root because the script
ran under sudo; a plain `tmux kill-session` as your user won't find it):
```bash
ssh -t jetson@192.168.0.20 'sudo tmux kill-session -t servo_bringup'
```

### Driving it from the laptop

In a laptop terminal that has ROS sourced (`ROS_DOMAIN_ID=42`,
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `source /opt/ros/humble/setup.bash` —
put these in `~/.bashrc` once so every terminal is ready):
```bash
ros2 topic list     # expect /hexapod/cmd, /joint_states, /tf  → confirms discovery
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'walk'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'stop'"
```
Or the full control UI / teleop GUI (see "Control UI" below).

Servo ID map (front-right going around): legs `r1`/`r2`/`r3` = right
front/middle/back use IDs 1–9; `l1`/`l2`/`l3` = left back/middle/front use
IDs 10–18. See [`hiwonder_servo_bridge.py`](hexapod_ws/src/hexapod_control/hexapod_control/hiwonder_servo_bridge.py).

### Permission / device node

If you ran `setup_hiwonder.sh` once (see above), nothing is needed per boot —
the udev rule gives `/dev/hiwonder` `0666` perms and the bridge finds it. If you
have NOT installed the rule, the bridge still auto-detects the board by VID:PID,
but you must make the node world-writable each boot. Find it and chmod it:

```bash
ssh jetson@192.168.0.20 "docker exec hexapod_ros ros2 run hexapod_control hiwonder_servo_bridge --list"
# then, for whatever node is flagged ← HIWONDER (e.g. hidraw5):
ssh jetson@192.168.0.20 "docker exec hexapod_ros bash -c 'chmod 666 /dev/hidraw5'"
```

### Bench-test the servos one joint at a time (no ROS)

Sweeps each servo through a safe range with confirmation prompts — use this
the first time you wire up legs or change `LEGS` / `SERVO_DIRECTION` in the
bridge.

```bash
ssh -t jetson@192.168.0.20 "docker exec -it hexapod_ros bash -c \
    'python3 /hexapodd/hexapod_ws/src/hexapod_control/hexapod_control/hiwonder_servo_bridge.py --test'"
```

If a joint moves the wrong way: flip its sign in `SERVO_DIRECTION` at the top
of the bridge and rebuild.

### Run the bridge as a ROS node (the normal path)

Once `hexapod_controller` is publishing `/joint_states`, the bridge drives the
servos to match:

```bash
ros2 run hexapod_control hiwonder_servo_bridge
```

`launch_all.sh` now starts this automatically in the `servos` tmux window
(via [`scripts/t1_servo_bridge.sh`](scripts/t1_servo_bridge.sh)) after
`t1_robot.sh` brings up the controller, so the daily startup command needs
no change. The bridge script `chmod 666`s `/dev/hidraw1` on boot.

### New teleop GUI (laptop side)

The new Tkinter teleop has Walk / Stop / Forward / Backward / Rotate L /
Rotate R buttons plus Roll / Pitch / Yaw / TX / TY / TZ sliders. Run it on
the laptop — it talks to the Jetson over the shared `ROS_DOMAIN_ID=42`:

```bash
ros2 run hexapod_control hexapod_teleop
```

### Tuning hooks

| Variable (file) | What it does |
|---|---|
| `STANDING_POS` ([hiwonder_servo_bridge.py](hexapod_ws/src/hexapod_control/hexapod_control/hiwonder_servo_bridge.py)) | Raw servo value that corresponds to "0 rad" per joint; tune so the robot stands flat |
| `SERVO_DIRECTION` (same file) | Per-joint direction flip; bench-test mode is the way to find these |
| `GRAVITY_COMP_UNITS` (same file) | Static upward bias on the femur to counter sag under load |
| `MAX_SPEED_DEG_PER_SEC` (same file) | Per-joint rate limit; lower if you see jerk-then-catch-up motion |
| `STRIDE_MM`, `LIFT_MM`, `CYCLE_FRAMES` ([hexapod_controller.py](hexapod_ws/src/hexapod_control/hexapod_control/hexapod_controller.py)) | Gait tuning: stride length, foot lift, cycle duration |
| `STANCE_EXT`, `STANCE_HEIGHT` ([hexapod_kinematics.py](hexapod_ws/src/hexapod_control/hexapod_control/hexapod_kinematics.py)) | Neutral stance footprint and ride height |

### Note on legacy features (IMU balancing + emotes)

The old controller's IMU self-balancing, salute, and trump-pump emotes are not
yet ported to the new Cartesian gait. The previous controller is preserved at
[`hexapod_controller_legacy.py`](hexapod_ws/src/hexapod_control/hexapod_control/hexapod_controller_legacy.py)
(and likewise `hexapod_teleop_legacy.py`) — port pieces forward as needed.

---

## Control UI — web dashboard, dances & posture

The browser UI at `http://192.168.0.20:5000` is the full control surface and is
now **fully wired to the kinematics**. It comes up with the rest of the stack
via `launch_all.sh` (rosbridge + `ui_teleop` + Flask). The motors must already
be reachable (the bus-servo bridge runs as part of the same stack).

**What each control does and how it reaches the legs:**

| UI control | ROS path | Effect |
|---|---|---|
| Joystick | `/ui/body_pose` → `ui_teleop` → `/hexapod/body_pose` | speed (linear.x) + steering yaw (angular.z) while walking |
| Walk / Back / Stop | `/ui/cmd` → `/hexapod/cmd` | gait mode |
| Roll / Pitch / Yaw / Height sliders | `/ui/body_posture` → `/hexapod/posture` | tilt / raise the body in place |
| 👋 Wave · 💃 Dance · 🎩 Bow · 🙆 Stretch · 🐛 Wiggle · 🎉 Excited | animate the posture sliders → `/hexapod/posture` | body-pose "dances" (no foot lift) |
| 🕺 Trump · 🫡 Salute | `/hexapod/cmd` `trump`/`salute` (+ `_stop`) | **foot-level** emotes done in the controller (front legs pump / right-front salute) with ease-in/out |

**Topic contract (one job per topic — so the joystick and the dances never
fight over the same field):**
- `/hexapod/body_pose` (Twist) = **motion only**: `linear.x` 0–1 speed,
  `angular.z` steering yaw (rad).
- `/hexapod/posture` (Twist) = **body pose only**: `angular.x/y/z` =
  roll/pitch/yaw (rad), `linear.x/y/z` = tx/ty/tz (mm).
- `/hexapod/cmd` (String) = `walk|forward|backward|stop|rotate_left|rotate_right|trump|trump_stop|salute|salute_stop`.

The pose-based dances (wave…excited) are pure body-pose moves driven from the
browser, so they show up on the servos even while standing — the bridge follows
`/joint_states` continuously now (it no longer ignores frames after a `stop`).
The two foot-level emotes (trump, salute) are computed in the controller as
foot-position overrides on top of the still standing body, with a soft
amplitude ramp so the motors never jerk.

**Drive the same things from the command line** (laptop, ROS sourced):
```bash
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'salute'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'salute_stop'"
# Lean the body: roll 0.1 rad, rise 10 mm
ros2 topic pub --once /hexapod/posture geometry_msgs/msg/Twist \
    "{angular: {x: 0.1, y: 0.0, z: 0.0}, linear: {x: 0.0, y: 0.0, z: 10.0}}"
```

> After editing controller / bridge / teleop code, the stack auto-rebuilds:
> `t1_robot.sh` now runs `colcon build --packages-select hexapod_control
> --symlink-install`, so a `stop_all.sh` + `launch_all.sh` (or re-running
> `bringup_servos.sh`) picks up the latest code. The first such build converts
> the install to symlinks, after which src edits are live with no rebuild.

---

## Self-balancing (MPU-9250 IMU)

> **Heads up:** the balancing code below references the *legacy* controller's
> parameters. The new Cartesian controller doesn't expose them yet. Use the
> legacy controller (or port the balance loop forward) if you need balancing
> for now.

Full details in `docs/04-imu-balancing.md`. Quick commands:

```bash
# Is the IMU publishing? (expect ~35-50 Hz)
ros2 topic hz /imu/data

# See the live orientation (flat = x,y,z near 0, w near 1)
ros2 topic echo /imu/data --once

# Turn balancing off / on at runtime
ros2 topic pub --once /hexapod/balance_enable std_msgs/msg/Bool "data: false"
ros2 topic pub --once /hexapod/balance_enable std_msgs/msg/Bool "data: true"

# THE key real-world test: if the body tilts the WRONG way, flip the sign
ros2 param set /hexapod_controller balance_sign -1.0

# Tune responsiveness (start kp, then add ki for steady slopes)
ros2 param set /hexapod_controller balance_kp 0.8
ros2 param set /hexapod_controller balance_ki 0.4
ros2 param set /hexapod_controller balance_max_rad 0.30
```

**Confirm the IMU on the I2C bus** (should show `68`):
```bash
ssh jetson@192.168.0.20 "docker exec hexapod_ros i2cdetect -y -r 1"
```

---

## Files and locations on the Jetson

| What | Where |
|---|---|
| Project root | `~/hexapod-ros2/` |
| Robot controller (Python) — new Cartesian gait | `hexapod_ws/src/hexapod_control/hexapod_control/hexapod_controller.py` |
| Kinematics (IK + foot trajectories) | `hexapod_ws/src/hexapod_control/hexapod_control/hexapod_kinematics.py` |
| Hiwonder HX-35HM servo bridge (USB-HID) | `hexapod_ws/src/hexapod_control/hexapod_control/hiwonder_servo_bridge.py` |
| Teleop GUI (Tkinter) | `hexapod_ws/src/hexapod_control/hexapod_control/hexapod_teleop.py` |
| Legacy controller (IMU balance + emotes) | `hexapod_ws/src/hexapod_control/hexapod_control/hexapod_controller_legacy.py` |
| URDF + STL meshes | `hexapod_ws/src/hexapod_description/` |
| Raw design files (Hiwonder HX-35HM v3 STL/STEP) | `hexapod_ws/src/hexapod_description/design/Hiwonder_HX-35HM_v3/` |
| Nav, teleop, joy nodes | `ros2_ws/src/hexapod_control/hexapod_nav/` |
| Web UI (Flask + JS) | `ros2_ws/web_ui/` |
| Scripts (launch / stop / etc.) | `scripts/` |
| Container launcher | `start_ros.sh` |
| Master launcher (used by laptop) | `scripts/launch_all.sh` |
| Master stopper | `scripts/stop_all.sh` |

---

## Quick reference — just the commands

```bash
# ── Servos only (the everyday path, CONFIRMED WORKING) ──
# Start controller + bridge on the Jetson (one command, runs on the host):
ssh -t jetson@192.168.0.20 'sudo bash ~/hexapod-ros2/scripts/bringup_servos.sh'
# Watch:
ssh -t jetson@192.168.0.20 'sudo tmux attach -t servo_bringup'
# Stop (sudo — root owns the session):
ssh -t jetson@192.168.0.20 'sudo tmux kill-session -t servo_bringup'

# ── Full stack (web UI + rosbridge + Flask + RViz support) ──
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/launch_all.sh
ssh jetson@192.168.0.20 bash ~/hexapod-ros2/scripts/stop_all.sh

# Visualize
ros2 run rviz2 rviz2

# Control UI (browser)
xdg-open http://192.168.0.20:5000

# Drive (laptop, ROS sourced)
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'walk'"
ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'stop'"

# Teleop GUI (laptop)
ros2 run hexapod_control hexapod_teleop

# One-time device setup / repair (Jetson host, root)
ssh -t jetson@192.168.0.20 'sudo bash ~/hexapod-ros2/scripts/setup_hiwonder.sh'

# List HID devices / find the board (inside container)
ssh -t jetson@192.168.0.20 "docker exec -it hexapod_ros ros2 run hexapod_control hiwonder_servo_bridge --list"

# Bench-test the servos joint-by-joint (no ROS needed)
ssh -t jetson@192.168.0.20 "docker exec -it hexapod_ros bash -c \
    'python3 /hexapodd/hexapod_ws/src/hexapod_control/hexapod_control/hiwonder_servo_bridge.py --test'"

# Manual shell on Jetson
ssh jetson@192.168.0.20
```
