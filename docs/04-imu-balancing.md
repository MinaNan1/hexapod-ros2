# Self-Balancing with the MPU-9250 IMU

Keeps the hexapod body horizontal on uneven terrain. The IMU measures the
body's true roll/pitch; a feedback loop counter-tilts the body through the
existing `apply_body_pose()` leg model, so legs on the low side extend and
legs on the high side retract until the body is level.

## Hardware

You have an **MPU-9250** (9-axis: accel + gyro + magnetometer). Self-balancing
only needs accel + gyro — roll/pitch are observable from gravity, so we skip
the magnetometer (it would only add yaw, which leveling doesn't use).

### Wiring — MPU-9250 → Jetson Nano 40-pin header

Use **I2C bus 1** (the standard GPIO header I2C).

| MPU-9250 pin | Connect to | Jetson pin | Why |
|---|---|---|---|
| VCC | 3.3V | pin 1 | Match I2C logic level (don't use 5V) |
| GND | GND | pin 6 | Common ground |
| SCL/SCLK | I2C1 SCL | pin 5 | Clock |
| SDA/SDI | I2C1 SDA | pin 3 | Data |
| NCS | 3.3V | pin 17 | **HIGH = I2C mode** (low/floating = SPI) |
| ADO/SDO | GND | pin 9 | Sets address to **0x68** (3.3V = 0x69) |
| FSYNC | GND | pin 25 | Avoid noise triggering frame sync |
| EDA, ECL, INT | — | — | Leave unconnected |

> Pins 1 and 17 are both 3.3V; pins 6, 9, 25 are all GND. Use any of each.

## Software pieces (already added)

| File | Role |
|---|---|
| `hexapod_ws/.../imu_node.py` | Reads MPU-9250 over I2C, complementary-filters accel+gyro, publishes `sensor_msgs/Imu` on `/imu/data` |
| `hexapod_ws/.../hexapod_controller.py` | Subscribes `/imu/data`, runs a PI loop, adds the correction to the body pose so the body stays level |
| `display_headless.launch.py` | Starts `imu_node` alongside the controller + a `base_link → imu_link` static TF |

The controller never depends on the IMU being present: if `/imu/data` is
absent or stale, the balance correction is held at zero and the robot behaves
exactly as before.

## Bringing it up

### 1. Install the I2C deps in the container (one-time, or rebuild the image)

The Dockerfile now includes `i2c-tools`, `python3-smbus`, and `smbus2`. To get
them into the **currently running** container without a full rebuild:

```bash
ssh jetson@hexapod-desktop.local docker exec hexapod_ros bash -c \
  "apt-get update && apt-get install -y i2c-tools python3-smbus && pip3 install smbus2"
```

(Or rebuild the image later with `docker build -f hexapod.dockerfile -t hexapod-ros2-humble .` to bake them in.)

### 2. Confirm the IMU is detected on the bus

```bash
ssh jetson@hexapod-desktop.local docker exec hexapod_ros i2cdetect -y -r 1
```

You should see `68` in the grid. If you see `69`, the AD0 pin is high — either
move it to GND or set the param `i2c_addr:=0x69`. If you see nothing, recheck
VCC/GND/SDA/SCL and that NCS is tied to 3.3V.

### 3. Rebuild the workspace (picks up imu_node + controller changes)

```bash
ssh jetson@hexapod-desktop.local docker exec hexapod_ros bash -c \
  "cd /hexapodd/hexapod_ws && colcon build --packages-select hexapod_control hexapod_description --symlink-install"
```

### 4. Relaunch and watch the IMU node

```bash
ssh jetson@hexapod-desktop.local bash ~/hexapod-ros2/scripts/stop_all.sh
ssh jetson@hexapod-desktop.local bash ~/hexapod-ros2/scripts/launch_all.sh
```

From the laptop, confirm the IMU is publishing:
```bash
ros2 topic hz /imu/data        # ~50 Hz
ros2 topic echo /imu/data --once
```

## Bench test (before walking)

With the robot standing still in RViz on the laptop:

1. **Verify orientation reading.** Tilt the IMU board by hand (or tilt the
   whole robot). In `ros2 topic echo /imu/data`, the orientation quaternion
   should change smoothly.

2. **Verify the body counter-tilts.** Watch the robot in RViz. Tilt the robot
   to one side — the legs should adjust so the **body** stays level relative to
   the world. If the body tilts *further* the wrong way (positive feedback),
   the sign is inverted — flip it:
   ```bash
   ros2 param set /hexapod_controller balance_sign -1.0
   ```
   Once you know the right sign, set it as the default in
   `display_headless.launch.py` so it persists.

3. **Toggle on/off** to compare:
   ```bash
   ros2 topic pub --once /hexapod/balance_enable std_msgs/msg/Bool "data: false"
   ros2 topic pub --once /hexapod/balance_enable std_msgs/msg/Bool "data: true"
   ```

## Tuning

All are live-settable with `ros2 param set /hexapod_controller <name> <value>`:

| Param | Default | Effect |
|---|---|---|
| `balance_enabled` | `true` | Master on/off |
| `balance_kp` | `0.8` | Proportional gain — higher = snappier, too high = oscillation |
| `balance_ki` | `0.4` | Integral gain — removes steady tilt on a constant slope |
| `balance_sign` | `1.0` | Flip to `-1.0` if it corrects the wrong way |
| `balance_max_rad` | `0.30` | Max correction (~17°), clamps how far legs adjust |
| `balance_imu_timeout` | `0.5` | Seconds without IMU before correction is frozen to 0 |

**Tuning order:** start with `ki=0`, raise `kp` until the body holds level
with a slight wobble, then add `ki` until a sustained slope is fully
corrected. If it oscillates, lower `kp`.

## How the leveling actually moves the legs

`apply_body_pose(frame, roll, pitch, …)` computes, per leg, a vertical foot
offset `delta_z = ax·sin(pitch) − ay·sin(roll)` from the leg's mounting
position `(ax, ay)`, then converts that to a femur-angle change. Commanding a
body roll/pitch therefore pushes some feet down and lifts others — exactly the
"some legs extend more than the others" behavior you wanted. The balance loop
just chooses that commanded roll/pitch automatically to cancel the measured
tilt, instead of you moving the sliders by hand.
