"""
hexapod_controller.py

Cartesian foot-trajectory controller (JetHexa-style method).

Per tick the pipeline is:
    1. pick gait parameters (stride / turn) from the current mode + speed
    2. compute each leg's foot target — gait trajectory, neutral stance, or an
       active EMOTE override (trump / salute)
    3. overlay the body pose (roll / pitch / yaw / translation) on the targets
    4. inverse-kinematics each foot target -> coxa / femur / tibia angles
    5. publish the angle DELTA from the standing pose (standing -> 0 rad)

Publishes:
  /joint_states          — leg angles -> robot_state_publisher -> RViz / servos
  /trump_emote_active     — Bool, true while the trump emote runs (UI feedback)
  TF: odom -> base_link   — body odometry in the world frame

Subscribes (topic contract — one job per topic, so the web UI joystick and the
slider/dance posture never fight each other):
  /hexapod/cmd (String)      — "walk" | "forward" | "backward" | "stop"
                               | "rotate_left" | "rotate_right"
                               | "trump"/"trump_stop" | "salute"/"salute_stop"
  /hexapod/body_pose (Twist) — MOTION only: linear.x = 0..1 speed multiplier,
                               angular.z = steering yaw (rad) while walking.
  /hexapod/posture (Twist)   — BODY POSE only: angular.x/y/z = roll/pitch/yaw
                               (rad), linear.x/y/z = tx/ty/tz (mm). This is what
                               the slider panel and every pose-based dance
                               (wave/dance/bow/stretch/wiggle/excited) drive.

The published angles are deltas from the standing pose, which is exactly what
hiwonder_servo_bridge.py expects ("ROS 0 rad -> STANDING_POS").
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String, Bool
import tf2_ros

from hexapod_control import hexapod_kinematics as kin

# ── Leg ordering & URDF joint names ──────────────────────────────────────────
LEG_ORDER = [
    'rightMiddle', 'rightFront', 'leftFront',
    'leftMiddle',  'leftBack',   'rightBack',
]

# leftFront = l3 (front left), leftBack = l1 (back left) — matches the bridge.
_SUFFIX = {
    'rightFront': 'r1', 'rightMiddle': 'r2', 'rightBack': 'r3',
    'leftFront':  'l3', 'leftMiddle':  'l2', 'leftBack':  'l1',
}

URDF_NAMES = []
for _leg in LEG_ORDER:
    _s = _SUFFIX[_leg]
    URDF_NAMES.extend(
        [f'coxa_joint_{_s}', f'femur_joint_{_s}', f'tibia_joint_{_s}'])

# ── Gait parameters (tunable) ─────────────────────────────────────────────────
TIMER_HZ      = 50.0
DT            = 1.0 / TIMER_HZ

# Frames per full tripod cycle.  Larger = slower, smoother cadence.
# 35 = faster again (cycle ~0.70 s). Same foot trajectory/stride/lift — only the
# cadence changes, so it walks the same, just quicker. Worst-case joint velocity
# ~12.0°/tick needs the bridge rate cap raised to 660°/s (13.2°/tick) — done in
# hiwonder_servo_bridge.py — so it isn't clipped. ~33 is the floor at 660. Raise
# back to 40/46 to slow down. (Live-tunable: ros2 param set /hexapod_controller
# cycle_frames 35)
CYCLE_FRAMES  = 33                  # baked: a little more speed (was 35)

# Body translation per cycle from one tripod stance push (mm).
# 175 (was 150): bigger steps → the base/coxa motors sweep farther fore/aft
# (more horizontal swing) AND the body travels faster at the same cadence.
# Capped here by the TIBIA servo, not reach: at the stance extreme the splayed
# tibia nears its 1000 limit, so 175 needed TIBIA_OUT_TRIM lowered to 25 (bridge).
# To step bigger, lower TIBIA_OUT_TRIM more. Live: ros2 param set ... stride_mm 175
STRIDE_MM     = 200.0               # baked: more swing (was 175)

# Lateral translation per cycle when strafing (mm). Now matches the forward
# stride so sideways speed feels the same as walking (was 90 = noticeably slow).
STRAFE_MM     = 170.0               # baked: more sideways swing (was 130)

# Peak foot lift height during the swing phase (mm).
# Big lift so the swing legs clearly clear the ground and step. Now that the
# swing-leg gravity-comp bug is fixed, this value actually reaches the foot.
LIFT_MM       = 120.0  # baked: more vertical swing (was 85)
                       # to track at the faster cadence (less lag → less plate sag)

# Body yaw per cycle in the pure-rotate modes (rad).
# Reduced from 18° — slower rotation is much more stable under load.
TURN_PER_CYCLE = math.radians(15.0)

# How strongly the steering input turns the robot while walking (rad turn-per-
# cycle per rad of steer input). 0 = no steering.  Reduced for heavy base.
STEER_GAIN    = 0.50   # was 0.25 — turns the Steer slider into a real curved-walk
                       # steer (now folded into the gait turn). Live-tunable:
                       # ros2 param set /hexapod_controller steer_gain 0.5

# Odometry fudge factor — tune so RViz odom matches measured travel.
ODOM_RECTIFY  = 1.0

# ── Emote tuning ──────────────────────────────────────────────────────────────
# Emotes are foot-position overrides played on top of a still (standing) body,
# with an ease-in / ease-out amplitude envelope so the motors never jerk.
EMOTE_RAMP_SEC = 0.45          # seconds to ramp amplitude 0->1 and back
TRUMP_FREQ_HZ  = 1.2           # front-leg pump rate (slowed from 1.6 so the much
                               # bigger lift completes without rate-limit clipping)
TRUMP_LIFT_MM  = 150.0         # how high the front feet pump (WAY up — was 80)
TRUMP_REACH_MM = 45.0          # how far forward they reach at the top
TRUMP_ROCK_DEG = 7.0           # body roll amplitude — hips rock with each lift
SALUTE_LIFT_MM = 150.0         # how high the right-front leg is raised
SALUTE_IN_MM   = 25.0          # how far it tucks inward
SALUTE_WAVE_HZ = 1.4           # gentle wave while held up
SALUTE_WAVE_MM = 18.0

# ── New dances (all motor-safe: low frequency, sinusoidal, eased in/out) ──────
# BOUNCE: whole body bobs up/down on planted feet (a happy bob).
BOUNCE_HZ      = 0.9
BOUNCE_MM      = 28.0          # peak rise (body goes up then back to level)
# SWAY: body rolls side-to-side + a little counter-yaw — a slow hip sway.
SWAY_HZ        = 0.7
SWAY_DEG       = 9.0           # roll amplitude
SWAY_YAW_DEG   = 5.0           # subtle yaw twist with the sway
# STRETCH: slow "cat stretch" — nose dips forward and body lowers, holds, returns.
STRETCH_HZ     = 0.28          # one slow in-and-out cycle is very gentle
STRETCH_PITCH_DEG = 12.0       # nose-down pitch
STRETCH_DOWN_MM   = 25.0       # body lowers as it stretches
# WAVE: lift the LEFT-front leg and wave it slowly (distinct from salute's right).
WAVE_HZ        = 1.0
WAVE_LIFT_MM   = 120.0
WAVE_MM        = 32.0          # side-to-side sweep of the raised foot

FRONT_LEGS = ('rightFront', 'leftFront')

# Every emote the controller understands (foot- or body-pose-driven).
EMOTES = ('trump', 'salute', 'bounce', 'sway', 'stretch', 'wave')


def yaw_to_quat(yaw):
    """(qx, qy, qz, qw) for a pure Z-axis rotation."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.trump_pub = self.create_publisher(Bool, '/trump_emote_active', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            Twist, '/hexapod/body_pose', self._on_body_pose, 10)
        self.create_subscription(
            Twist, '/hexapod/posture', self._on_posture, 10)
        self.create_subscription(
            String, '/hexapod/cmd', self._on_cmd, 10)

        # ── Parameters (all default to the previous hard-coded behaviour) ──────
        # publish_odom_tf: set FALSE to let rf2o_laser_odometry own odom→base_link
        #   (scan-matching odometry — far better than gait dead-reckoning). When
        #   running rf2o, launch the controller with publish_odom_tf:=false so the
        #   two don't both publish the same transform.
        self.declare_parameter('publish_odom_tf', True)
        # IMU self-balancing (uses the already-published /imu/data). OFF by
        # default — enable + tune on real hardware. Adds a roll/pitch correction
        # to the body pose so the body stays level on uneven ground / slopes.
        self.declare_parameter('balance_enable', False)
        self.declare_parameter('balance_kp', 0.6)        # correction gain
        self.declare_parameter('balance_max_rad', 0.25)  # clamp (~14°)
        self.declare_parameter('balance_sign', 1.0)      # flip if it tilts the WRONG way
        # Fall cutoff: if the body tilts past this, stop commanding gait (protect
        # the servos / signal a tip). 0 = DISABLED (default — only enable once
        # the IMU is confirmed level at rest, else a bad reading freezes the gait).
        self.declare_parameter('fall_cutoff_deg', 0.0)

        # ── LIVE-TUNABLE gait params (RECOMMENDATION #2) ───────────────────────
        # Defaults equal the module constants above, so launch behaviour is
        # identical. Change any of these LIVE while it walks — no rebuild, no scp:
        #   ros2 param set /hexapod_controller lift_mm 95.0
        #   ros2 param set /hexapod_controller cycle_frames 42
        #   ros2 param set /hexapod_controller stride_mm 160.0
        # (floats need a decimal point; cycle_frames is an int). To revert, just
        # set them back — or relaunch (defaults restore the values shown here).
        self.declare_parameter('cycle_frames', int(CYCLE_FRAMES))
        self.declare_parameter('stride_mm', float(STRIDE_MM))
        self.declare_parameter('strafe_mm', float(STRAFE_MM))
        self.declare_parameter('lift_mm', float(LIFT_MM))
        self.declare_parameter('turn_deg_per_cycle', math.degrees(TURN_PER_CYCLE))
        self.declare_parameter('steer_gain', float(STEER_GAIN))
        # Femur DOWN-push into the ground during the stance phase (mm). Bigger =
        # the legs drive down harder → more ground authority/grip while moving.
        # Default = the kinematics constant, so launch behaviour is unchanged
        # (undoable: set it back, or relaunch). Live: ros2 param set ... stance_press_mm 32
        self.declare_parameter('stance_press_mm', float(kin.STANCE_PRESS_MM))

        self._publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self._balance_enable  = self.get_parameter('balance_enable').value
        self._balance_kp      = self.get_parameter('balance_kp').value
        self._balance_max     = self.get_parameter('balance_max_rad').value
        self._balance_sign    = self.get_parameter('balance_sign').value
        self._fall_cutoff     = math.radians(self.get_parameter('fall_cutoff_deg').value)

        self._cycle_frames    = max(1, int(self.get_parameter('cycle_frames').value))
        self._stride_mm       = float(self.get_parameter('stride_mm').value)
        self._strafe_mm       = float(self.get_parameter('strafe_mm').value)
        self._lift_mm         = float(self.get_parameter('lift_mm').value)
        self._turn_per_cycle  = math.radians(self.get_parameter('turn_deg_per_cycle').value)
        self._steer_gain      = float(self.get_parameter('steer_gain').value)
        kin.STANCE_PRESS_MM   = float(self.get_parameter('stance_press_mm').value)

        # Allow the gait params above to be changed live via `ros2 param set`.
        self.add_on_set_parameters_callback(self._on_set_params)

        # IMU state (roll/pitch measured from /imu/data); 0 until first message.
        self._imu_roll = self._imu_pitch = 0.0
        self._imu_ok = False
        self._fallen = False
        from sensor_msgs.msg import Imu
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)

        # Body-pose state (from /hexapod/posture)
        self._roll = self._pitch = self._yaw = 0.0
        self._tx = self._ty = self._tz = 0.0

        # Motion state (from /hexapod/body_pose)
        self._steer = 0.0           # steering yaw (rad), folded into the gait turn
        self._speed_mult = 1.0      # 0..1, scales cadence

        # Odometry state
        self._odom_x = self._odom_y = self._odom_heading = 0.0

        # Motion mode: walk | walk_back | rotate_cw | rotate_ccw | stop
        # Start STOPPED (stand still) — only walk on an explicit command from the
        # UI / controller. (Was 'walk', which made it stride the instant it
        # launched.)
        self._mode = 'stop'

        # Gait pattern: 'tripod' (default/main) | 'wave' | 'ripple'. Selected via
        # the "gait_<name>" command. Only changes the leg phasing + duty + cadence
        # scale — the foot trajectory / kinematics are identical, so the main
        # tripod walk is untouched.
        self._gait_pattern = 'tripod'

        # Emote state: None | 'trump' | 'salute'
        self._emote = None
        self._emote_active = False   # True while held, False while ramping out
        self._emote_amp = 0.0        # 0..1 ease envelope
        self._emote_t = 0.0          # seconds since emote start

        # Gait phase
        self._phase = 0.0

        # Standing-pose joint angles per leg — published angles are deltas
        # from these so that "standing == 0 rad" for the servo bridge.
        self._stand = {leg: kin.standing_angles(leg) for leg in LEG_ORDER}

        self.timer = self.create_timer(DT, self.tick)
        self.get_logger().info(
            'HexapodController ready — Cartesian tripod gait + posture + emotes | '
            f'stride={self._stride_mm:.0f}mm lift={self._lift_mm:.0f}mm '
            f'cycle={self._cycle_frames} frames | live-tunable via ros2 param set')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_body_pose(self, msg: Twist):
        """MOTION only — speed multiplier + steering yaw."""
        self._steer = msg.angular.z
        # linear.x is a 0–1 speed multiplier; ignore exact-0 so a button-driven
        # walk isn't frozen by an idle joystick still publishing 0.
        if msg.linear.x > 0.01:
            self._speed_mult = _clamp(msg.linear.x, 0.0, 1.0)

    def _on_posture(self, msg: Twist):
        """BODY POSE only — roll/pitch/yaw (rad) + tx/ty/tz (mm)."""
        self._roll  = msg.angular.x
        self._pitch = msg.angular.y
        self._yaw   = msg.angular.z
        self._tx    = msg.linear.x
        self._ty    = msg.linear.y
        self._tz    = msg.linear.z

    def _on_imu(self, msg):
        """Read body roll/pitch from the IMU quaternion (for balance + fall)."""
        q = msg.orientation
        # quaternion → roll (x) and pitch (y)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = _clamp(sinp, -1.0, 1.0)
        self._imu_pitch = math.asin(sinp)
        self._imu_ok = True

    def _on_set_params(self, params):
        """Apply live `ros2 param set` changes to the gait tuning params."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            try:
                if p.name == 'cycle_frames':
                    self._cycle_frames = max(1, int(p.value))
                elif p.name == 'stride_mm':
                    self._stride_mm = float(p.value)
                elif p.name == 'strafe_mm':
                    self._strafe_mm = float(p.value)
                elif p.name == 'lift_mm':
                    self._lift_mm = float(p.value)
                elif p.name == 'turn_deg_per_cycle':
                    self._turn_per_cycle = math.radians(float(p.value))
                elif p.name == 'steer_gain':
                    self._steer_gain = float(p.value)
                elif p.name == 'stance_press_mm':
                    kin.STANCE_PRESS_MM = float(p.value)
            except (TypeError, ValueError):
                return SetParametersResult(successful=False,
                                           reason=f'bad value for {p.name}')
        return SetParametersResult(successful=True)

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()

        # ── Emotes / dances ─────────────────────────────────────────────────
        # "<name>" starts it, "<name>_stop" (or any "*_stop"/"dance_stop") ends it.
        if cmd in EMOTES:
            self._start_emote(cmd)
            return
        if cmd.endswith('_stop') or cmd == 'dance_stop':
            self._stop_emote()
            return

        # ── Gait pattern select ("gait_tripod" | "gait_ripple" | "gait_wave") ─
        # Changes only the leg phasing/duty/cadence — not the motion mode. Best
        # switched while stopped (changing offsets mid-walk re-phases the legs).
        if cmd.startswith('gait_'):
            name = cmd[len('gait_'):]
            if name in kin.GAIT_PATTERNS:
                self._gait_pattern = name
                self._phase = 0.0
                self.get_logger().info(f'Gait pattern: {name.upper()}')
            else:
                self.get_logger().warn(f'Unknown gait pattern: {name}')
            return

        # ── Motion modes ────────────────────────────────────────────────────
        prev = self._mode
        if cmd == 'stop':
            self._mode = 'stop'
        elif cmd in ('walk', 'forward'):
            self._mode = 'walk'
        elif cmd == 'backward':
            self._mode = 'walk_back'
        elif cmd in ('strafe_left', 'left'):
            self._mode = 'strafe_left'
        elif cmd in ('strafe_right', 'right'):
            self._mode = 'strafe_right'
        elif cmd == 'rotate_left':
            self._mode = 'rotate_ccw'
        elif cmd == 'rotate_right':
            self._mode = 'rotate_cw'
        else:
            return

        # Any motion command cancels a running emote and starts the new motion
        # from a known gait phase (the Cartesian equivalent of float_frame=0).
        if self._emote is not None:
            self._stop_emote()
        if self._mode != prev:
            self._phase = 0.0
            self.get_logger().info(f'Mode: {self._mode.upper()}')

    # ── Emote helpers ───────────────────────────────────────────────────────

    def _start_emote(self, name):
        if self._emote == name and self._emote_active:
            return
        self._emote = name
        self._emote_active = True
        self._emote_t = 0.0
        self._mode = 'stop'           # body stays still; only the emote plays
        self._phase = 0.0
        self.get_logger().info(f'Emote: {name.upper()} start')
        if name == 'trump':
            self._publish_trump_active(True)

    def _stop_emote(self):
        if self._emote is None:
            return
        # Begin easing out; the envelope clears self._emote when amp hits 0.
        self._emote_active = False
        self.get_logger().info(f'Emote: {self._emote.upper()} stop')
        self._publish_trump_active(False)

    def _publish_trump_active(self, active: bool):
        m = Bool()
        m.data = bool(active)
        self.trump_pub.publish(m)

    def _emote_foot(self, leg):
        """Return an override foot (x,y,z) for `leg` during an emote, else None.

        Amplitude is scaled by self._emote_amp (the ease envelope), so the move
        grows in and shrinks out smoothly.
        """
        amp = self._emote_amp
        if amp <= 1e-3 or self._emote is None:
            return None
        nx, ny, nz = kin.neutral_foot(leg)
        t = self._emote_t

        if self._emote == 'trump' and leg in FRONT_LEGS:
            # STRICT alternation for balance: each front leg lifts only during
            # its OWN half of the cycle and stays fully planted the other half,
            # so one front foot is always on the ground (it waits, grounded,
            # until the other leg comes all the way back down).
            #   rightFront leads in [0, 0.5);  leftFront in [0.5, 1.0).
            phase = (TRUMP_FREQ_HZ * t) % 1.0
            local = phase if leg == 'rightFront' else (phase + 0.5) % 1.0
            if local < 0.5:
                # smooth single hump: 0 at both ends, peak mid-swing (no jerk).
                s = math.sin(math.pi * (local / 0.5)) ** 2
            else:
                s = 0.0                      # grounded, waiting for the other leg
            return (nx + amp * TRUMP_REACH_MM * s,
                    ny,
                    nz + amp * TRUMP_LIFT_MM * s)

        if self._emote == 'salute' and leg == 'rightFront':
            wave = SALUTE_WAVE_MM * math.sin(2.0 * math.pi * SALUTE_WAVE_HZ * t)
            return (nx - amp * SALUTE_IN_MM,
                    ny + amp * wave,
                    nz + amp * SALUTE_LIFT_MM)

        if self._emote == 'wave' and leg == 'leftFront':
            # Raise the left-front leg and sweep the foot side-to-side — a slow
            # greeting wave. sin() starts at 0 so the sweep eases in with no jump.
            sweep = WAVE_MM * math.sin(2.0 * math.pi * WAVE_HZ * t)
            return (nx,
                    ny + amp * sweep,
                    nz + amp * WAVE_LIFT_MM)

        return None

    def _emote_body_pose(self):
        """Body-pose overlay (roll, pitch, yaw, tx, ty, tz) for the active emote.

        These drive the whole body on planted feet (no foot override), so they
        are inherently smooth. Every term starts at 0 at t=0 and is scaled by the
        ease envelope (self._emote_amp), so nothing jerks on start/stop.
        Returns radians for roll/pitch/yaw and mm for tx/ty/tz.
        """
        amp = self._emote_amp
        if amp <= 1e-3 or self._emote is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        t = self._emote_t

        if self._emote == 'trump':
            # Hip-rock synced to the alternation phase: zero at the transitions
            # (both feet grounded) and peaks mid-lift, rocking toward the planted
            # side. sin() flips sign each half so it leans the right way for each
            # leg. (Flip TRUMP_ROCK_DEG sign if it rocks the wrong way.)
            phase = (TRUMP_FREQ_HZ * t) % 1.0
            roll = (math.radians(TRUMP_ROCK_DEG) * amp
                    * math.sin(2.0 * math.pi * phase))
            return (roll, 0.0, 0.0, 0.0, 0.0, 0.0)

        if self._emote == 'bounce':
            # Body bobs up and back to level (0..BOUNCE_MM, never below stance).
            tz = amp * BOUNCE_MM * (0.5 - 0.5 * math.cos(2.0 * math.pi * BOUNCE_HZ * t))
            return (0.0, 0.0, 0.0, 0.0, 0.0, tz)

        if self._emote == 'sway':
            phase = 2.0 * math.pi * SWAY_HZ * t
            roll = amp * math.radians(SWAY_DEG) * math.sin(phase)
            yaw  = amp * math.radians(SWAY_YAW_DEG) * math.sin(phase)
            return (roll, 0.0, yaw, 0.0, 0.0, 0.0)

        if self._emote == 'stretch':
            # One slow in-and-out: nose dips forward and body lowers, then returns.
            env = 0.5 - 0.5 * math.cos(2.0 * math.pi * STRETCH_HZ * t)
            pitch = amp * math.radians(STRETCH_PITCH_DEG) * env
            tz    = -amp * STRETCH_DOWN_MM * env
            return (0.0, pitch, 0.0, 0.0, 0.0, tz)

        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # ── Gait parameters for the current mode ──────────────────────────────────

    def _gait_params(self):
        """Return (stride_x, stride_y, turn) in mm/mm/rad per cycle.

        +stride_y = body moves toward +y = LEFT (body frame y points left).
        Strafe reuses the exact same tripod foot trajectory as forward walking,
        just along the lateral axis — the gait_foot() math already supports it.
        """
        if self._mode == 'walk':
            return self._stride_mm, 0.0, (self._steer + self._yaw) * self._steer_gain
        if self._mode == 'walk_back':
            return -self._stride_mm, 0.0, (self._steer + self._yaw) * self._steer_gain
        # Sign empirically flipped: on the real robot +stride_y drove it RIGHT,
        # so strafe_left = -strafe_mm, strafe_right = +strafe_mm.
        if self._mode == 'strafe_left':
            return 0.0, -self._strafe_mm, (self._steer + self._yaw) * self._steer_gain
        if self._mode == 'strafe_right':
            return 0.0,  self._strafe_mm, (self._steer + self._yaw) * self._steer_gain
        if self._mode == 'rotate_ccw':
            return 0.0, 0.0, self._turn_per_cycle
        if self._mode == 'rotate_cw':
            return 0.0, 0.0, -self._turn_per_cycle
        return 0.0, 0.0, 0.0          # stop

    # ── Main loop ─────────────────────────────────────────────────────────────

    def tick(self):
        now = self.get_clock().now().to_msg()

        # Advance the emote envelope + clock (ease amplitude toward target).
        target = 1.0 if self._emote_active else 0.0
        step = DT / EMOTE_RAMP_SEC
        self._emote_amp = _clamp(
            self._emote_amp + _clamp(target - self._emote_amp, -step, step),
            0.0, 1.0)
        if self._emote is not None:
            self._emote_t += DT
            if not self._emote_active and self._emote_amp <= 1e-3:
                self._emote = None            # fully eased out

        emoting = self._emote is not None

        # ── Fall detection (IMU) ──────────────────────────────────────────────
        # If the body tilts past the cutoff, freeze the gait (hold standing) to
        # protect the servos / signal a tip. Latches until it's back near level.
        if self._fall_cutoff > 0 and self._imu_ok:
            tilt = max(abs(self._imu_roll), abs(self._imu_pitch))
            if tilt > self._fall_cutoff and not self._fallen:
                self._fallen = True
                self.get_logger().warn(
                    f'FALL/tilt {math.degrees(tilt):.0f}° > cutoff — freezing gait.')
            elif self._fallen and tilt < self._fall_cutoff * 0.6:
                self._fallen = False
                self.get_logger().info('Tilt recovered — gait resumed.')

        walking = (self._mode != 'stop') and not emoting and not self._fallen

        stride_x, stride_y, turn = self._gait_params()

        # Selected gait pattern (phase offsets + swing duty + cadence scale).
        pat = kin.GAIT_PATTERNS.get(self._gait_pattern, kin.GAIT_PATTERNS['tripod'])

        # Advance the gait phase only while actually walking. The pattern's
        # cadence scale slows the lower-duty gaits (wave/ripple) so their shorter
        # swings stay within the servo speed limit (tripod = 1.0 = unchanged).
        phase_inc = (self._speed_mult / (self._cycle_frames * pat['cadence'])
                     if walking else 0.0)
        self._phase = (self._phase + phase_inc) % 1.0

        # Body-pose overlay for the active emote/dance (trump hip-rock, bounce,
        # sway, stretch...). Eased + sinusoidal so it's smooth; added on top of
        # any user roll/pitch/yaw/translation from the posture sliders.
        e_roll, e_pitch, e_yaw, e_tx, e_ty, e_tz = self._emote_body_pose()

        # ── IMU self-balance overlay ──────────────────────────────────────────
        # Counter-rotate the body by a fraction of the measured tilt so the legs
        # keep the body level on slopes / uneven ground. Off unless balance_enable.
        bal_roll = bal_pitch = 0.0
        if self._balance_enable and self._imu_ok:
            bal_roll  = _clamp(-self._balance_sign * self._balance_kp * self._imu_roll,
                               -self._balance_max, self._balance_max)
            bal_pitch = _clamp(-self._balance_sign * self._balance_kp * self._imu_pitch,
                               -self._balance_max, self._balance_max)

        # Build the joint frame: foot target -> body pose -> IK -> delta.
        positions = []
        for leg in LEG_ORDER:
            if emoting:
                foot = self._emote_foot(leg) or kin.neutral_foot(leg)
            elif walking:
                foot = kin.gait_foot(leg, self._phase, stride_x, stride_y,
                                     turn, self._lift_mm,
                                     leg_phase=pat['phase'][leg],
                                     swing_duty=pat['duty'])
            else:                              # stopped & no emote -> flat stand
                foot = kin.neutral_foot(leg)

            # While walking, the "Steer" slider (self._yaw) curves the GAIT path
            # (folded into the turn in _gait_params) instead of twisting the body,
            # so it actually steers. Dances still twist via e_yaw. Standing: the
            # yaw twists the body as before.
            manual_yaw = 0.0 if walking else self._yaw
            foot = kin.apply_body_pose(
                foot,
                self._roll + e_roll + bal_roll,
                self._pitch + e_pitch + bal_pitch,
                manual_yaw + e_yaw,
                self._tx + e_tx, self._ty + e_ty, self._tz + e_tz)
            coxa, femur, tibia = kin.leg_ik(leg, foot)
            s = self._stand[leg]
            # Per-gait tibia bias (e.g. wave pushes the curved tibia out so the
            # foot lands on its tip). Only while walking; 0 for tripod/ripple.
            tib_bias = pat.get('tibia_bias', 0.0) if walking else 0.0
            positions.extend([coxa - s[0], femur - s[1], tibia - s[2] + tib_bias])

        js = JointState()
        js.header.stamp = now
        js.name = URDF_NAMES
        js.position = [float(a) for a in positions]
        self.joint_pub.publish(js)

        # ── Odometry ──────────────────────────────────────────────────────────
        if walking:
            adv = 2.0 * phase_inc * ODOM_RECTIFY
            dx = stride_x / 1000.0 * adv          # body-frame, metres
            dy = stride_y / 1000.0 * adv
            self._odom_heading += 2.0 * turn * phase_inc
            ch, sh = math.cos(self._odom_heading), math.sin(self._odom_heading)
            self._odom_x += dx * ch - dy * sh
            self._odom_y += dx * sh + dy * ch

        # ── odom -> base_link TF ────────────────────────────────────────────
        # Skipped when publish_odom_tf:=false, so rf2o_laser_odometry can own
        # this transform with real scan-matching odometry instead of the
        # open-loop gait dead-reckoning above.
        if self._publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.translation.z = 0.0
            qx, qy, qz, qw = yaw_to_quat(self._odom_heading)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
