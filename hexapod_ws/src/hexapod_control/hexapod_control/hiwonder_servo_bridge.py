"""
hiwonder_servo_bridge.py

Bridges ROS2 /joint_states → Hiwonder Bus Servo Controller via USB HID.

Configured for 3-leg testing with safety features:
  - Mechanical angle limits (clamped per joint)
  - Angular rate limiting (prevents sudden jerks)
  - Slow startup ramp to standing position
  - Standalone --test mode for joint-by-joint verification

Currently configured for all 6 legs:
  - r1 front right,  servo IDs 1/2/3
  - r2 middle right, servo IDs 4/5/6
  - r3 back right,   servo IDs 7/8/9
  - l1 back left,    servo IDs 10/11/12
  - l2 middle left,  servo IDs 13/14/15
  - l3 front left,   servo IDs 16/17/18

Usage:
  # 0. Give yourself permission (once per boot):
  sudo chmod 666 /dev/hidraw1

  # 1. First-time safe test (no ROS needed, moves one joint at a time):
  python3 hiwonder_servo_bridge.py --test

  # 2. Bridge mode (needs hexapod_controller publishing /joint_states):
  #    Terminal A:  ros2 run hexapod_control hexapod_controller
  #    Terminal B:  ros2 run hexapod_control hiwonder_servo_bridge
"""

import glob
import math
import os
import select
import sys
import time

try:
    # used only for the leading-leg feedback stance gate (foot-lift via FK)
    from hexapod_control import hexapod_kinematics as kin
except Exception:
    kin = None

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION — edit these to match your physical setup
# ──────────────────────────────────────────────────────────────────────────────

# The Hiwonder Serial Bus Servo Controller is a USB-HID device.  Its /dev/hidrawN
# number is NOT stable — it depends on what else is plugged in and shifts on every
# re-enumeration (reboot, cable wiggle, hub reset).  On the laptop it happened to
# be hidraw1; on the Jetson (with a wireless dongle + keyboard also on the bus) it
# lands on hidraw5 — or wherever.  So we DON'T trust a fixed number: we look the
# board up by its USB vendor/product ID and find whichever hidraw it currently is.
HIWONDER_VID = '0483'              # USB idVendor  (MindMotion SOC / Hiwonder)
HIWONDER_PID = '5750'              # USB idProduct
HID_DEVICE = '/dev/hidraw5'        # last-resort fallback only; see resolve_hid_device()

# ── Default standing position per leg (raw servo values) ─────────────────────
# ROS 0 rad → this position.  Movements are relative to these values.
STANDING_POS = {
    # FRONT-leg femur "open up" 2026-06-02: r1 260→224, l3 235→199 (each −36
    # units ≈ −8.6°). This reproduces the front-leg posture the user confirmed
    # they want by dragging the UI Height slider to its full negative (tz=−20mm):
    # the front legs splay open / point outward instead of tucking inward. The
    # tibia half of that posture is done via TIBIA_OUT_TRIM_EXTRA (≈+19, below).
    # Originals were 260 / 235 — restore those to undo the open-up.
    'r1': {'coxa': 500, 'femur': 224, 'tibia': 740},   # front-right  (open: femur was 260)
    'r2': {'coxa': 500, 'femur': 235, 'tibia': 760},
    'r3': {'coxa': 500, 'femur': 235, 'tibia': 805},
    'l1': {'coxa': 500, 'femur': 255, 'tibia': 770},
    'l2': {'coxa': 500, 'femur': 235, 'tibia': 750},
    'l3': {'coxa': 500, 'femur': 199, 'tibia': 770},   # front-left   (open: femur was 235)
}

# ── Legs to drive ────────────────────────────────────────────────────────────
# Each entry: leg_name → {coxa, femur, tibia} servo IDs
LEGS = {
    'r1': {'coxa': 1,  'femur': 2,  'tibia': 3},    # front right
    'r2': {'coxa': 4,  'femur': 5,  'tibia': 6},    # middle right
    'r3': {'coxa': 7,  'femur': 8,  'tibia': 9},    # back right
    'l1': {'coxa': 10, 'femur': 11, 'tibia': 12},   # back left
    'l2': {'coxa': 13, 'femur': 14, 'tibia': 15},   # middle left
    'l3': {'coxa': 16, 'femur': 17, 'tibia': 18},   # front left
}

# Servo direction per leg per joint.  +1 = default, -1 = inverted.
# Run --test first; if a joint moves the WRONG way, flip its sign here.
SERVO_DIRECTION = {
    'r1': {'coxa':  1, 'femur': -1, 'tibia':  1},   # front right  — verified
    'r2': {'coxa':  1, 'femur': -1, 'tibia':  1},   # middle right — tibia flipped
    'r3': {'coxa':  1, 'femur': -1, 'tibia':  1},   # back right   — verified
    'l1': {'coxa':  1, 'femur': -1, 'tibia':  1},   # back left    — tibia flipped
    'l2': {'coxa':  1, 'femur': -1, 'tibia':  1},   # middle left  — verified
    'l3': {'coxa':  1, 'femur': -1, 'tibia':  1},   # front left   — tibia flipped
}

# ── Static tibia splay trim (servo units) ───────────────────────────────────
# Points the tibia (lowest leg link) OUTWARD a bit more at the resting/standing
# pose. Higher tibia servo units = straighter/more-extended tibia = foot reaches
# further out (verified by the gait's stance-push direction), so a positive trim
# splays the tibia outward. Applied to every leg's standing tibia via
# SERVO_DIRECTION, with the gait deltas riding on top (gait shape unchanged).
#   25 units ≈ 6° (1000 units = 240°).
#   NOTE (learned via the front open-up): LOWER tibia units = more OUT/open;
#   HIGHER = more tucked-in. So 35→25 actually opens the tibias slightly AND frees
#   servo headroom near the 1000 limit, which is what lets STRIDE_MM go to 175
#   without the stance-extreme tibia clipping. Lower further for bigger steps.
TIBIA_OUT_TRIM = 25   # 35→25: frees tibia range for the bigger stride

# Per-leg EXTRA tibia-out (servo units) added ON TOP of TIBIA_OUT_TRIM.
# Leg keys (see LEGS above): r1=front-right, l3=front-LEFT, r3=back-right,
# l1=back-LEFT, r2/l2=middle. The 2 FRONT legs (r1, l3) get a touch more so they
# point outward a little more through the whole walk (shifts their entire
# swing+stance range outward). When you reverse, the BACK legs lead — mirror the
# extra onto r3/l1 then (currently 0).  12 units ≈ 2.9°.
# (FIX: was wrongly on l1=back-left instead of l3=front-left.)
TIBIA_OUT_TRIM_EXTRA = {
    'r1': 19, 'l3': 19,   # FRONT legs: tibia half of the "open up" posture (was a
                          # wrong-direction +40 that CLOSED them inward). +19 lands
                          # the front tibia at the tz=−20 value (≈794/824) to match
                          # the femur open-up above. Lower this further to open more.
    'r2':  0, 'l2':  0,   # middle
    'r3':  0, 'l1':  0,   # back
}

for _leg, _dirs in SERVO_DIRECTION.items():
    _trim = TIBIA_OUT_TRIM + TIBIA_OUT_TRIM_EXTRA.get(_leg, 0)
    _t = STANDING_POS[_leg]['tibia'] + _dirs['tibia'] * _trim
    STANDING_POS[_leg]['tibia'] = max(0, min(1000, _t))

# Move duration sent with every command (ms).
# SPEED-PRIORITY: dropped 50→30. At 50 ms the servo always lagged ~50 ms behind
# the moving target, so at a fast cadence the stance legs never fully reached
# position before the next phase → the plate progressively sagged. 30 ms (just
# above the 20 ms tick) makes the servos TRACK the fast gait tightly so legs
# arrive on time and hold the body up. (Not <20 ms or the servo reaches+stops
# between commands → jitter.)
MOVE_TIME_MS = 30

# ── Tibia compensation ───────────────────────────────────────────────────────
# When True, overrides the kinematics-computed tibia angle and forces the tibia
# link perpendicular to the ground by cancelling femur rotation.  This puts
# significant load on the tibia motor.
#
# When False (current), the tibia follows the kinematic gamma angle naturally,
# pointing outward at its default position — much lower motor load.
TIBIA_COMPENSATION = False

# ── Ground floor clamp ────────────────────────────────────────────────────────
# Prevents the femur from pushing past STANDING_POS toward the ground, and when
# it triggers it also forces the tibia back to standing (at_floor logic below).
# ⚠ DISABLED 2026-06-02: during a normal stance sweep the femur LEGITIMATELY
# rotates past standing to keep the (horizontally-extended) foot on the ground —
# this clamp can't tell that apart from "pushing into the ground", so it fired on
# EVERY stance tick (proven by sim) and PINNED the femur AND tibia to standing the
# whole time the foot was down → "the tibia never swings on the ground". With it
# off, femur+tibia follow the IK → the foot properly tracks the stance line and
# the tibia articulates, and the intended STANCE_PRESS pre-load now actually
# happens. Re-enable (True) only if a foot pushes itself into the floor.
FEMUR_FLOOR_CLAMP = False

# ── Gravity compensation ──────────────────────────────────────────────────────
# Static upward bias applied to the femur servo (in raw servo units).
# Counteracts sag from the robot's weight — the servos can't hold the exact
# commanded position under load, so we pre-push the femur a little toward
# "lift".  Automatically uses SERVO_DIRECTION to figure out which way is up.
# Increase if the robot still sags; decrease if it stands too tall.
# HEAVY-BASE: raised from 20 to 40 for stronger anti-droop bias.
GRAVITY_COMP_UNITS = 50   # PROVEN value (old working ws). Applied to ALL femurs
                          # (see Pass 2, ~L523) — the swing/stance split was reverted.
# A femur is treated as SWING (lifted, no comp) when it's lifted more than this
# many servo units above standing; otherwise STANCE (gets comp). Must be ABOVE
# the small variation a stance femur sees as the foot sweeps front↔back (~20),
# and BELOW the big lift of a swing leg (~120). 45 cleanly separates them and is
# what keeps comp applied through the WHOLE stance (fixes the body collapsing
# mid-walk while still preserving full swing lift).
SWING_LIFT_THRESHOLD = 45

# ── Mechanical limits (radians) ──────────────────────────────────────────────
# Conservative values well inside the Hiwonder 0-240° (±120°) physical range.
JOINT_LIMITS_RAD = {
    'coxa':  (-2,  2),         # ±109°
    'femur': (-2,  2),         # ±109°
    'tibia': (-2.2,  2.2),         # ±109°
}

# ── Rate limiting ────────────────────────────────────────────────────────────
# Max angular change per tick.  Headroom matters here — if the gait demands
# more than this at any moment (e.g. peak velocity through swing), the limiter
# clamps it and produces visible "stuck then catch up" choppiness.
# 350 was strangling the now-FAST gait (CYCLE_FRAMES=50): the swing needs more
# peak velocity than 350°/s allows, so the limiter clipped it → legs barely
# lifted, lagged (looked slow), and stance legs didn't finish their push
# (dragging). 550 gives the fast swing room to complete fully.
MAX_SPEED_DEG_PER_SEC = 700.0   # 600→660→700: headroom for CYCLE_FRAMES=35 + the
                                # bigger STRIDE_MM=190 (peak gait demand ~12.9°/tick
                                # = 643°/s on the back femur; 700 = 14°/tick keeps
                                # margin so it isn't clipped). Power is 12V/100A
                                # (current not the limit). NOTE: this only RAISES
                                # the clip ceiling — servos still only move as fast
                                # as the gait actually demands.
TICK_RATE_HZ = 50.0
MAX_DELTA_RAD = math.radians(MAX_SPEED_DEG_PER_SEC / TICK_RATE_HZ)

# ── Derived joint-name map (all 3 legs) ──────────────────────────────────────
# Maps ROS joint name → (leg_name, joint_type)
JOINT_MAP = {}
for _leg, _ids in LEGS.items():
    for _jtype in ('coxa', 'femur', 'tibia'):
        JOINT_MAP[f'{_jtype}_joint_{_leg}'] = (_leg, _jtype)


# ──────────────────────────────────────────────────────────────────────────────
# Hiwonder angle conversion
# Servo range 0-1000 ↔ 0°-240°.  Scale: 1000 units / 240°.
# ROS 0 rad → standing position (per-leg offset, NOT always 500).
# ──────────────────────────────────────────────────────────────────────────────

def rad_to_hiwonder(rad: float, joint_name: str, leg_name: str = '') -> int:
    leg_dir = SERVO_DIRECTION.get(leg_name, {})
    direction = leg_dir.get(joint_name, 1)
    leg_stand = STANDING_POS.get(leg_name, {})
    centre = leg_stand.get(joint_name, 500)
    degrees = math.degrees(rad) * direction
    position = int(centre + (degrees / 240.0) * 1000)
    return max(0, min(1000, position))


def clamp_angle(rad: float, joint_name: str) -> float:
    lo, hi = JOINT_LIMITS_RAD.get(joint_name, (-1.57, 1.57))
    return max(lo, min(hi, rad))


def rate_limit(current: float, target: float) -> float:
    delta = target - current
    if abs(delta) > MAX_DELTA_RAD:
        return current + math.copysign(MAX_DELTA_RAD, delta)
    return target


# ──────────────────────────────────────────────────────────────────────────────
# Hiwonder HID protocol — CMD_SERVO_MOVE (0x03)
# ──────────────────────────────────────────────────────────────────────────────

def build_move_packet(servo_positions: dict, move_time_ms: int) -> bytes:
    num_servos = len(servo_positions)
    time_lo = move_time_ms & 0xFF
    time_hi = (move_time_ms >> 8) & 0xFF

    params = [num_servos, time_lo, time_hi]
    for servo_id, position in servo_positions.items():
        pos_lo = position & 0xFF
        pos_hi = (position >> 8) & 0xFF
        params.extend([servo_id, pos_lo, pos_hi])

    length = num_servos * 3 + 5
    payload = [0x55, 0x55, length, 0x03] + params

    # HID report: report-ID (0x00) + 64 bytes zero-padded
    report = [0x00] + payload + [0x00] * (64 - len(payload))
    return bytes(report[:65])


def _hidraw_info(node):
    """Return (vid, pid, name) for a /dev/hidrawN node by reading sysfs.

    HID_ID in uevent looks like '0003:00000483:00005750' (bus:vid:pid, hex,
    zero-padded).  Returns ('', '', '') if anything is unreadable.
    """
    name = os.path.basename(node)            # e.g. 'hidraw5'
    uevent = f'/sys/class/hidraw/{name}/device/uevent'
    vid = pid = hidname = ''
    try:
        with open(uevent) as f:
            for line in f:
                if line.startswith('HID_ID='):
                    parts = line.strip().split('=', 1)[1].split(':')
                    if len(parts) == 3:
                        vid = parts[1][-4:].upper()
                        pid = parts[2][-4:].upper()
                elif line.startswith('HID_NAME='):
                    hidname = line.strip().split('=', 1)[1]
    except OSError:
        pass
    return vid, pid, hidname


def list_hid_devices():
    """Print every hidraw node with its USB IDs / name; flag the Hiwonder."""
    nodes = sorted(glob.glob('/dev/hidraw*'))
    if not nodes:
        print('No /dev/hidraw* devices found.  Is anything plugged in?')
        return
    print('Available HID devices:')
    for node in nodes:
        vid, pid, hidname = _hidraw_info(node)
        flag = '  ← HIWONDER' if (vid == HIWONDER_VID.upper()
                                   and pid == HIWONDER_PID.upper()) else ''
        print(f'  {node:<16} {vid}:{pid}  {hidname}{flag}')


def find_hiwonder_device():
    """Auto-detect the Hiwonder controller's hidraw node by USB VID:PID.

    Returns '/dev/hidrawN' or None if the board isn't currently enumerated.
    """
    for node in sorted(glob.glob('/dev/hidraw*')):
        vid, pid, _ = _hidraw_info(node)
        if vid == HIWONDER_VID.upper() and pid == HIWONDER_PID.upper():
            return node
    return None


def resolve_hid_device():
    """Pick the device path to open, most-specific first:

      1. --device=/dev/X  or  HEXAPOD_HID_DEVICE env var (explicit override)
      2. /dev/hiwonder    (stable udev symlink, if the rule is installed)
      3. auto-detect by USB VID:PID 0483:5750  (survives hidraw renumbering)
      4. HID_DEVICE constant  (last-resort fallback)
    """
    for arg in sys.argv:
        if arg.startswith('--device='):
            return arg.split('=', 1)[1]
    env = os.environ.get('HEXAPOD_HID_DEVICE')
    if env:
        return env
    if os.path.exists('/dev/hiwonder'):
        return '/dev/hiwonder'
    found = find_hiwonder_device()
    if found:
        return found
    return HID_DEVICE


def open_hid():
    """Resolve + open the HID device, with a helpful error on failure."""
    path = resolve_hid_device()
    try:
        hid = open(path, 'r+b', buffering=0)   # r+b so we can READ feedback too
        print(f'🔌 Using HID device: {path}')
        return hid
    except PermissionError:
        print(f'❌ Permission denied: {path}')
        print(f'   Fix with:  sudo chmod 666 {path}')
        print(f'   (or install the udev rule — see LAPTOP_COMMANDS.md)')
        raise SystemExit(1)
    except FileNotFoundError:
        print(f'❌ Device not found: {path}')
        if find_hiwonder_device() is None:
            print('   The Hiwonder board (USB 0483:5750) is NOT enumerated as a')
            print('   hidraw device right now.  Check power and the USB cable, then:')
        list_hid_devices()
        print('   Override with:  --device=/dev/hidrawN')
        raise SystemExit(1)


def send(hid, servo_positions: dict, move_time_ms: int):
    """Build and write a move packet."""
    pkt = build_move_packet(servo_positions, move_time_ms)
    hid.write(pkt)
    hid.flush()


# ── Position FEEDBACK read — CMD_MULT_SERVO_POS_READ (0x15) ───────────────────
# Validated on the real board: a full read parses cleanly. Half-duplex, so a read
# briefly pauses writes — we read only 1–2 servos at a time to keep that tiny.
def build_read_packet(ids):
    count = len(ids)
    payload = [0x55, 0x55, count + 3, 0x15, count] + list(ids)
    report = [0x00] + payload + [0x00] * (64 - len(payload))
    return bytes(report[:65])


def parse_pos_reply(data):
    b = list(data)
    for i in range(len(b) - 4):
        if b[i] == 0x55 and b[i + 1] == 0x55 and b[i + 3] == 0x15:
            count = b[i + 4]
            out = {}
            p = i + 5
            for _ in range(count):
                if p + 2 >= len(b):
                    break
                out[b[p]] = b[p + 1] + (b[p + 2] << 8)
                p += 3
            return out
    return {}


def read_positions(hid, ids, timeout=0.06):
    """Request + read actual positions of `ids`. Returns {id: pos} (or {})."""
    fd = hid.fileno()
    while select.select([fd], [], [], 0)[0]:        # drain stale input
        try:
            os.read(fd, 64)
        except OSError:
            break
    hid.write(build_read_packet(ids))
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not select.select([fd], [], [], max(0.0, deadline - time.time()))[0]:
            break
        try:
            pos = parse_pos_reply(os.read(fd, 64))
        except OSError:
            break
        if pos:
            return pos
    return {}


# ──────────────────────────────────────────────────────────────────────────────
# STANDALONE TEST MODE  (python3 hiwonder_servo_bridge.py --test)
# Gently sweeps each joint one at a time so you can verify wiring & direction.
# No ROS required.
# ──────────────────────────────────────────────────────────────────────────────

# Raw servo positions for test mode (bypasses angle conversion entirely)
TEST_POS_LOW   = 200         # near one end of travel
TEST_POS_HIGH  = 800         # near other end of travel
TEST_MOVE_MS   = 1000        # 1 s per step
TEST_PAUSE_S   = 1.5         # pause between steps


def send_debug(hid, servo_positions: dict, move_time_ms: int):
    """Build, print, and write a move packet."""
    pkt = build_move_packet(servo_positions, move_time_ms)
    print(f'    HID packet ({len(pkt)} bytes): {pkt[:20].hex(" ")} …')
    hid.write(pkt)
    hid.flush()


def run_test_mode():
    leg_names = list(LEGS.keys())
    print('╔══════════════════════════════════════════════════════════╗')
    print('║           HIWONDER 3-LEG TEST                          ║')
    print('╠══════════════════════════════════════════════════════════╣')
    print(f'║  HID device : {resolve_hid_device():<40} ║')
    for lname, lids in LEGS.items():
        sp = STANDING_POS[lname]
        print(f'║  Leg {lname:<3}: coxa={lids["coxa"]:<3} femur={lids["femur"]:<3}'
              f' tibia={lids["tibia"]:<3}                    ║')
        print(f'║    stand: coxa={sp["coxa"]}  femur={sp["femur"]}'
              f'  tibia={sp["tibia"]}                         ║')
    print(f'║  Sweep range: pos {TEST_POS_LOW} → {TEST_POS_HIGH}  '
          f'(≈{(TEST_POS_HIGH-TEST_POS_LOW)*240//1000}° of servo travel)  ║')
    print('╚══════════════════════════════════════════════════════════╝')
    print()

    hid = open_hid()

    # ── Step 0: move all servos to standing position ─────────────────────
    print('\n▶ Moving all servos to standing position (2 s) …')
    standing = {}
    for lname, lids in LEGS.items():
        sp = STANDING_POS[lname]
        for jtype in ('coxa', 'femur', 'tibia'):
            standing[lids[jtype]] = sp[jtype]
    send_debug(hid, standing, 2000)
    time.sleep(2.5)
    print('  ✔ Servos at standing\n')

    # ── Sweep each joint on each leg ─────────────────────────────────────
    for leg_name, leg_ids in LEGS.items():
        print(f'\n═══ LEG {leg_name.upper()} ═══')
        for joint_name in ('coxa', 'femur', 'tibia'):
            sid = leg_ids[joint_name]
            sp = STANDING_POS[leg_name]
            home = sp[joint_name]

            input(f'⏎  Press ENTER to sweep {leg_name.upper()}/{joint_name.upper()} '
                  f'(servo ID {sid}) …')

            # All servos for this leg stay at standing, except the one under test
            base = {leg_ids[j]: sp[j] for j in ('coxa', 'femur', 'tibia')}

            print(f'  → pos {TEST_POS_HIGH}')
            base[sid] = TEST_POS_HIGH
            send_debug(hid, base, TEST_MOVE_MS)
            time.sleep(TEST_PAUSE_S)

            print(f'  → pos {TEST_POS_LOW}')
            base[sid] = TEST_POS_LOW
            send_debug(hid, base, TEST_MOVE_MS)
            time.sleep(TEST_PAUSE_S)

            print(f'  → pos {home} (standing)')
            base[sid] = home
            send_debug(hid, base, TEST_MOVE_MS)
            time.sleep(TEST_PAUSE_S)

            print(f'  ✔ {leg_name.upper()}/{joint_name.upper()} done.\n')

    print('✅ All legs tested.  Check the hex packets above — if the')
    print('   servos barely moved, the servo IDs might not match your wiring.')
    print('   Edit LEGS at the top of the script and re-run --test.')
    hid.close()


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 BRIDGE NODE
# ──────────────────────────────────────────────────────────────────────────────

def _ros_bridge():
    """Run the ROS2 bridge (imported lazily so --test works without ROS)."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class HiwonderBridge(Node):
        def __init__(self):
            super().__init__('hiwonder_servo_bridge')
            self.hid = open_hid()
            for lname, lids in LEGS.items():
                self.get_logger().info(
                    f'🦵 Leg {lname} | servo IDs '
                    f'coxa={lids["coxa"]} femur={lids["femur"]} '
                    f'tibia={lids["tibia"]}')

            # Track last commanded angle per leg/joint (for rate limiting)
            self._last = {}
            for leg in LEGS:
                self._last[leg] = {'coxa': 0.0, 'femur': 0.0, 'tibia': 0.0}

            # Last servo frame actually written to the bus. The controller
            # publishes /joint_states at 50 Hz even while standing still, which
            # would otherwise mean 50 identical HID packets/sec hammering the
            # USB bus + CPU for no motion. We skip the write when nothing
            # changed, so an idle robot sends ~0 packets and the Nana's USB/CPU
            # is free for the camera + SLAM. (Hiwonder servos hold their last
            # commanded position, so not re-sending is correct.)
            self._last_sent = None

            # Walking state — mirrors /hexapod/cmd.
            # When False, /joint_states is ignored and STANDING_POS is held.
            self._walking = True

            # ── Slowly move servos to standing position on startup ───────
            # HEAVY-BASE: slower 3.5 s ramp (was 2 s) so a heavy body lifts
            # gradually without slamming or stalling the servos.
            self.get_logger().info('Moving to standing position (3.5 s) …')
            self._send_standing(move_ms=3500)
            time.sleep(4.0)
            self.get_logger().info('Servos at standing.  Listening on /joint_states …')

            self.create_subscription(
                JointState, '/joint_states', self._on_js, 10)

            # Mirror the same /hexapod/cmd topic the controller uses
            from std_msgs.msg import String as StringMsg
            self.create_subscription(
                StringMsg, '/hexapod/cmd', self._on_cmd, 10)

            # ── Leading-leg anti-droop feedback ──────────────────────────────
            # Hold ONLY the leading pair: the FRONT legs (r1,l3) when walking
            # FORWARD, the BACK legs (r3,l1) when REVERSING. The other 4 legs get
            # NO correction. Per-leg integral on femur+tibia, learned only while
            # that leg is in STANCE (loaded), eased in smoothly. OFF by default;
            # 'antidroop_on'/'antidroop_off' on /hexapod/cmd.
            self._antidroop = False
            self._direction = 'forward'            # tracked from /hexapod/cmd
            self._lead_legs = {'forward': ['r1', 'l3'],   # front pair
                               'backward': ['r3', 'l1']}  # back pair
            self._corr_legs = ['r1', 'l3', 'r3', 'l1']    # the 4 that CAN be held
            self._kin_leg = {'r1': 'rightFront', 'r3': 'rightBack',
                             'l1': 'leftBack',  'l3': 'leftFront'}
            self._corr_ids = []
            for _bl in self._corr_legs:
                self._corr_ids += [LEGS[_bl]['femur'], LEGS[_bl]['tibia']]
            self._corr         = {s: 0.0 for s in self._corr_ids}   # integral target
            self._corr_applied = {s: 0.0 for s in self._corr_ids}   # smoothed, sent
            self._leg_lift = {bl: 0.0 for bl in self._corr_legs}    # FK foot lift, mm
            self._rr = 0
            self._last_desired = None
            self._stance_lift_mm = 18.0            # lift below this = loaded/stance
            # cache standing angles + standing foot-z (FK) per correctable leg
            self._stand_ang = {}
            self._footz_stand = {}
            if kin is not None:
                for _bl, _kl in self._kin_leg.items():
                    _sa = kin.standing_angles(_kl)
                    self._stand_ang[_bl] = _sa
                    self._footz_stand[_bl] = (kin.FEMUR * math.sin(_sa[1])
                                              + kin.TIBIA * math.sin(_sa[1] + _sa[2]))
            self.declare_parameter('antidroop_gain', 0.6)
            self.declare_parameter('antidroop_max', 160.0)
            self.declare_parameter('antidroop_slew', 2.0)
            self._ad_ki   = float(self.get_parameter('antidroop_gain').value)
            self._ad_max  = float(self.get_parameter('antidroop_max').value)
            self._ad_slew = float(self.get_parameter('antidroop_slew').value)
            from rcl_interfaces.msg import SetParametersResult
            self._SetParametersResult = SetParametersResult
            self.add_on_set_parameters_callback(self._on_set_params)
            self.create_timer(0.10, self._antidroop_tick)   # 10 Hz

        # ── Helpers ───────────────────────────────────────────────────────
        def _send_standing(self, move_ms=500):
            """Drive all active servos directly to STANDING_POS."""
            standing = {}
            for lname, lids in LEGS.items():
                sp = STANDING_POS[lname]
                for jtype in ('coxa', 'femur', 'tibia'):
                    standing[lids[jtype]] = sp[jtype]
            send(self.hid, standing, move_ms)

        def _on_cmd(self, msg):
            cmd = msg.data.strip().lower()
            if cmd == 'stop':
                self._walking = False
                self.get_logger().info('STOP — holding standing position')
                self._send_standing(move_ms=400)
            elif cmd in ('walk', 'forward', 'backward'):
                self._walking = True
                if cmd in ('walk', 'forward'):
                    self._direction = 'forward'
                elif cmd == 'backward':
                    self._direction = 'backward'
                self.get_logger().info(f'CMD: {cmd} — resuming joint tracking '
                                       f'(lead legs: {self._lead_legs[self._direction]})')
            elif cmd == 'antidroop_on':
                self._antidroop = True
                self.get_logger().info(
                    f'🛡 ANTI-DROOP ON — holding leading legs only '
                    f'(Ki={self._ad_ki}, max={self._ad_max}u, gate={kin is not None})')
            elif cmd == 'antidroop_off':
                self._antidroop = False
                for s in self._corr:
                    self._corr[s] = 0.0     # _on_js eases _corr_applied → 0
                self.get_logger().info('anti-droop OFF (easing corrections out)')

        # ── Callback ─────────────────────────────────────────────────────
        def _on_js(self, msg: JointState):
            # Always follow /joint_states. When the controller is "stopped" it
            # publishes the neutral standing pose (deltas ~0), so the servos
            # hold standing on their own — and crucially, body-pose / posture
            # sliders and the pose-based dances (which are published even while
            # standing) now reach the servos instead of being gated out here.
            servo_pos = {}

            # ── Pass 1: clamp + rate-limit, convert all joints to servo units ──
            # leg_servo_pos[leg][jtype] = raw servo position (direction applied,
            # NOT yet tibia-compensated).
            leg_servo_pos = {}
            leg_ang = {}                          # rate-limited joint angles (rad)
            for ros_name, (leg, jtype) in JOINT_MAP.items():
                if ros_name not in msg.name:
                    continue
                idx = msg.name.index(ros_name)
                raw = msg.position[idx]

                # 1. Clamp to mechanical limits
                clamped = clamp_angle(raw, jtype)

                # 2. Rate-limit angular change
                limited = rate_limit(self._last[leg][jtype], clamped)
                self._last[leg][jtype] = limited

                # 3. Convert to servo position (SERVO_DIRECTION applied here)
                pos = rad_to_hiwonder(limited, jtype, leg)
                leg_servo_pos.setdefault(leg, {})[jtype] = pos
                leg_ang.setdefault(leg, {})[jtype] = limited

            # Foot lift (FK) per correctable leg → stance(=loaded) detection for
            # the anti-droop loop. lift ≈ 0 in stance, large during swing.
            if kin is not None:
                for bl in self._corr_legs:
                    a = leg_ang.get(bl)
                    if a and 'femur' in a and 'tibia' in a:
                        sa = self._stand_ang[bl]
                        fa = sa[1] + a['femur']
                        ta = sa[2] + a['tibia']
                        fz = kin.FEMUR * math.sin(fa) + kin.TIBIA * math.sin(fa + ta)
                        self._leg_lift[bl] = fz - self._footz_stand[bl]

            # ── Pass 2: femur floor + tibia compensation in servo-unit space ────
            for leg, joints in leg_servo_pos.items():
                femur_pos   = joints.get('femur', STANDING_POS[leg]['femur'])
                femur_stand = STANDING_POS[leg]['femur']

                # Clamp femur so it never pushes past standing (into the ground).
                # SERVO_DIRECTION tells us which way "lift" goes:
                #   dir < 0 → lift = servo below standing → clamp above
                #   dir > 0 → lift = servo above standing → clamp below
                if FEMUR_FLOOR_CLAMP:
                    femur_dir = SERVO_DIRECTION[leg]['femur']
                    if femur_dir < 0:
                        femur_pos = min(femur_pos, femur_stand)
                    else:
                        femur_pos = max(femur_pos, femur_stand)

                # If femur is at the floor, tibia must also return to standing
                # so the foot tip stays flat on the ground (not curled under).
                at_floor = (femur_pos == femur_stand)
                femur_delta = femur_pos - femur_stand   # signed, direction-correct

                for jtype, pos in joints.items():
                    if jtype == 'femur':
                        # Gravity compensation — matches the PROVEN old working
                        # workspace exactly (applied to all legs; with the low
                        # 85mm stance the small lift loss is harmless and the
                        # body holds). Reverted from the swing/stance-split
                        # version to eliminate variables while restoring a known-
                        # good walk.
                        femur_dir = SERVO_DIRECTION[leg]['femur']
                        pos = femur_pos - femur_dir * GRAVITY_COMP_UNITS
                        pos = max(0, min(1000, pos))
                    elif jtype == 'tibia' and at_floor:
                        # Femur is at standing → force tibia to standing too
                        pos = STANDING_POS[leg]['tibia']
                    elif jtype == 'tibia' and TIBIA_COMPENSATION:
                        # (Legacy) Force tibia perpendicular to ground by
                        # cancelling femur rotation.  Disabled by default —
                        # causes high motor load.  Tibia now follows the
                        # kinematic gamma angle naturally.
                        tibia_dir = SERVO_DIRECTION[leg]['tibia']
                        femur_dir = SERVO_DIRECTION[leg]['femur']
                        ratio = tibia_dir / femur_dir   # +1 or -1
                        pos = int(STANDING_POS[leg]['tibia'] - femur_delta * ratio)
                        pos = max(0, min(1000, pos))

                    servo_pos[LEGS[leg][jtype]] = pos

            # Send when we have joints to command.
            # NOTE: no per-tick logging here — this callback runs at 50 Hz, and
            # logging a formatted 18-joint string every tick was starving the
            # HID write of CPU on the Nano (visible as motor jitter once the
            # camera + RTAB-Map were also running). Errors are still logged,
            # throttled, below.
            if servo_pos:
                self._last_desired = dict(servo_pos)   # gait target (pre-correction)
                # Ease the per-leg offsets toward target (toward 0 when off) at
                # <= slew units/frame → smooth, no step jumps. Only the leading
                # legs' ids ever carry a nonzero target.
                active = self._antidroop or any(
                    abs(v) > 0.5 for v in self._corr_applied.values())
                if active:
                    for sid in self._corr_ids:
                        tgt = self._corr[sid] if self._antidroop else 0.0
                        cur = self._corr_applied[sid]
                        if cur < tgt:
                            cur = min(tgt, cur + self._ad_slew)
                        elif cur > tgt:
                            cur = max(tgt, cur - self._ad_slew)
                        self._corr_applied[sid] = cur
                    sent_pos = {sid: max(0, min(1000,
                                    p + int(round(self._corr_applied.get(sid, 0.0)))))
                                for sid, p in servo_pos.items()}
                else:
                    sent_pos = servo_pos
                if sent_pos != self._last_sent:
                    try:
                        send(self.hid, sent_pos, MOVE_TIME_MS)
                        self._last_sent = dict(sent_pos)
                    except Exception as e:
                        self.get_logger().error(
                            f'HID write error: {e}', throttle_duration_sec=2.0)

        def _on_set_params(self, params):
            for p in params:
                if p.name == 'antidroop_gain':
                    self._ad_ki = float(p.value)
                elif p.name == 'antidroop_max':
                    self._ad_max = float(p.value)
                elif p.name == 'antidroop_slew':
                    self._ad_slew = float(p.value)
            return self._SetParametersResult(successful=True)

        def _antidroop_tick(self):
            # 10 Hz. Hold ONLY the leading pair for the current direction. Read one
            # leading leg that is in STANCE (loaded) and integrate a clamped offset
            # on its femur+tibia. Non-leading correctable legs ease to 0.
            if not self._antidroop or self._last_desired is None:
                return
            active = self._lead_legs.get(self._direction, [])
            active_ids = set()
            for bl in active:
                active_ids.add(LEGS[bl]['femur'])
                active_ids.add(LEGS[bl]['tibia'])
            for s in self._corr_ids:              # ease non-active legs out
                if s not in active_ids:
                    self._corr[s] = 0.0
            if kin is not None:                   # leading legs currently loaded
                stance = [bl for bl in active
                          if self._leg_lift.get(bl, 99.0) < self._stance_lift_mm]
            else:
                stance = list(active)
            if not stance:
                return
            bl = stance[self._rr % len(stance)]
            self._rr += 1
            fid, tid = LEGS[bl]['femur'], LEGS[bl]['tibia']
            actual = read_positions(self.hid, [fid, tid], timeout=0.06)
            for sid in (fid, tid):
                act, des = actual.get(sid), self._last_desired.get(sid)
                if act is None or des is None:
                    continue
                err = des - act                   # +err → joint sits below target
                self._corr[sid] = max(-self._ad_max, min(self._ad_max,
                                      self._corr[sid] + self._ad_ki * err))
            self._ad_n = getattr(self, '_ad_n', 0) + 1
            if self._ad_n % 12 == 1:
                parts = []
                for bl in active:
                    f, t = LEGS[bl]['femur'], LEGS[bl]['tibia']
                    parts.append(f'{bl}: f{self._corr[f]:+.0f} t{self._corr[t]:+.0f} '
                                 f'lift{self._leg_lift.get(bl, 0):.0f}')
                self.get_logger().info(f'🛡 {self._direction} lead  ' + '  '.join(parts))

        def destroy_node(self):
            if hasattr(self, 'hid') and not self.hid.closed:
                self.get_logger().info('Moving to standing before shutdown (1 s) …')
                try:
                    self._send_standing(move_ms=1000)
                    time.sleep(1.2)
                except Exception:
                    pass
                self.hid.close()
                self.get_logger().info('HID device closed.')
            super().destroy_node()

    rclpy.init()
    node = HiwonderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    if '--list' in sys.argv:
        list_hid_devices()
        return
    if '--test' in sys.argv:
        run_test_mode()
        return
    _ros_bridge()


if __name__ == '__main__':
    main()