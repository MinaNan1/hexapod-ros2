"""
hexapod_kinematics.py

Cartesian, foot-position based kinematics for a 3-DOF-per-leg hexapod.

This replaces the old joint-angle-animation approach (mithi_kinematics) with
the method used by the well-working Hiwonder JetHexa stack:

    1. Every leg's foot has a TARGET POSITION (x, y, z) in the body frame.
    2. An analytic inverse-kinematics solver turns each foot target into the
       three joint angles (coxa, femur, tibia).
    3. Gaits are defined as FOOT TRAJECTORIES — the foot lifts in an arc and
       swings forward through the air, then pushes backwards along the ground
       to propel the body. This is physically what a real leg does, which is
       why it walks correctly where direct angle-animation does not.
    4. Body pose (roll / pitch / yaw / translation) is applied by transforming
       the foot targets, then re-solving IK.

Frames & conventions
--------------------
Body frame:  +x = forward, +y = left, +z = up, origin at body centre.
Leg frame:   origin at the coxa joint, +x points outward along the leg's
             mount direction (coxa angle = 0), +z up.

Angle conventions (radians, absolute):
    coxa  : rotation about +z. 0 = leg pointing straight out along its mount.
            positive = swing toward +x-ish (CCW seen from above).
    femur : rotation of the femur link in the vertical leg-plane.
            positive = lift the foot up.
    tibia : knee angle relative to the femur link extended.
            0 = leg fully straight, negative = knee folded under.

All distances are in millimetres.

The controller publishes the DIFFERENCE between the current angles and the
standing-pose angles (standing -> 0 rad), which is exactly what the Hiwonder
servo bridge expects ("ROS 0 rad -> STANDING_POS"). So this module's absolute
angle convention never has to match the servo zero — only its CHANGES matter.
"""

import math

# ── Robot dimensions (mm) — your measured values ──────────────────────────────
DIMENSIONS = {
    'coxia': 60.0,
    'femur': 136.8,
    'tibia': 141.16,
}
COXA  = DIMENSIONS['coxia']
FEMUR = DIMENSIONS['femur']
TIBIA = DIMENSIONS['tibia']

# ── Leg mount positions on the body (mm, body frame x=forward y=left) ─────────
# Same layout as the previous controller so the URDF / RViz still line up.
LEG_ATTACH = {
    'rightFront':  ( 112.27, -121.33),
    'rightMiddle': (   0.0,  -132.4),
    'rightBack':   (-112.27, -121.33),
    'leftFront':   ( 112.27,  121.33),
    'leftMiddle':  (   0.0,   132.4),
    'leftBack':    (-112.27,  121.33),
}

# Outward mount direction of each leg (coxa angle = 0 points this way).
MOUNT_ANGLE = {
    leg: math.atan2(y, x) for leg, (x, y) in LEG_ATTACH.items()
}

ALL_LEGS = list(LEG_ATTACH.keys())

# ── Alternating-tripod groups ─────────────────────────────────────────────────
# Group 0 and group 1 are 180° out of phase. Classic tripod: front-right,
# middle-left and rear-right move together; the other three move together.
GAIT_GROUP = {
    'rightFront': 0, 'leftMiddle': 0, 'rightBack': 0,
    'leftFront':  1, 'rightMiddle': 1, 'leftBack': 1,
}

# ── Gait patterns (selectable movement styles) ───────────────────────────────
# A pattern ONLY changes (a) each leg's phase offset (when it swings), (b) the
# swing DUTY (fraction of the cycle a leg is airborne), and (c) a CADENCE scale
# (so the shorter-duty swings of the slower gaits aren't too fast for the servos).
# The per-foot trajectory + IK are shared, so every pattern uses the IDENTICAL
# kinematics — 'tripod' is the proven main gait and is byte-for-byte unchanged
# (phase = 0.5*group, duty 0.5, cadence 1.0 == the old hard-coded behaviour).
#   tripod  : 3 legs swing together, fast (the default — do not change).
#   ripple  : 2 legs airborne at a time, medium speed/stability.
#   wave    : 1 leg airborne at a time (5 always planted) — slow, very stable,
#             metachronal "centipede" ripple travelling back→front.
# Metachronal order: back→front, alternating sides, so the lifted leg's
# neighbours are always planted.
_WAVE_ORDER = ['rightBack', 'leftBack', 'rightMiddle',
               'leftMiddle', 'rightFront', 'leftFront']
_WAVE_PHASE = {leg: i / 6.0 for i, leg in enumerate(_WAVE_ORDER)}

# 'tibia_bias' (radians, applied to the tibia angle while THIS gait walks) pushes
# the (curved) tibia further OUT so the foot lands on its TIP instead of the
# curved shin. NEGATIVE = "out / splayed" (same sense as the front-leg open-up:
# lower tibia servo units). Wave/centipede plants slowly leg-by-leg so a curved
# tibia tends to catch on its shin — give it a healthy outward bias there.
# ⚠ If it lands WORSE (more on the shin), flip this sign to positive.
GAIT_PATTERNS = {
    'tripod': {'phase': {leg: 0.5 * GAIT_GROUP[leg] for leg in ALL_LEGS},
               'duty': 0.5,       'cadence': 1.0, 'tibia_bias': 0.0},
    'ripple': {'phase': _WAVE_PHASE, 'duty': 1.0 / 3.0, 'cadence': 1.5, 'tibia_bias': 0.0},
    'wave':   {'phase': _WAVE_PHASE, 'duty': 1.0 / 6.0, 'cadence': 3.0,
               'tibia_bias': -0.20},   # ≈ -11.5°  (push the tip down/out)
}

# ── Neutral / standing stance (tunable) ───────────────────────────────────────
# STANCE_EXT   : horizontal distance from the coxa joint out to the foot (mm).
#                (the femur+tibia horizontal projection at standing)
# STANCE_HEIGHT: how far below the body the feet sit at standing (mm).
# Increase STANCE_HEIGHT for a taller stance; increase STANCE_EXT for a wider,
# more stable footprint. Both stay well inside the leg's reach.
#
# HEAVY-BASE TUNING: lowered stance height (legs more extended, stronger
# mechanical advantage) and widened stance for better load distribution.
# A lower stance height means the femur+tibia are closer to fully extended,
# so the servos carry the weight through the link structure rather than by
# pure torque.  The wider stance spreads the weight across a bigger polygon.
STANCE_EXT    = 145.0   # keep tucked (proven) — widening it raises femur torque/sag
STANCE_HEIGHT = 90.0    # SPEED-PRIORITY: pulled back from 95 toward the proven 85.
                        # "Keep the plate up" is the priority now, so we trade ~5mm
                        # of height for less femur torque (less sag) while still
                        # sitting above the old proven 85. Nudge: 85 = max hold,
                        # 95 = max height.

# ── Per-leg ground trim (mm, + = push that foot DOWN harder) ─────────────────
# The middle legs sit closest to the body's centre of mass, so on a heavy base
# they can end up barely loaded while the front/back legs carry everything.
# Pushing the middle feet a little lower extends those legs so they bear more
# weight and grip the floor. Applied to the neutral/standing foot Z, so it
# shifts both the standing reference and the gait stance consistently.
# Back legs pushed DOWN a bit so they bear more load and lift the rear — fixes
# the backside sag the user saw while walking. (+ = foot lower = that corner up.)
LEG_GROUND_TRIM_MM = {
    'rightFront': 0.0, 'rightMiddle': 0.0, 'rightBack': 18.0,
    'leftFront':  0.0, 'leftMiddle':  0.0, 'leftBack':  18.0,
}   # back legs pushed down harder (was 12) → back of the plate rides higher

# ── Ground-contact margin (heavy-base stability) ─────────────────────────────
# During the stance (grounded) phase of the gait, the foot is pushed slightly
# below the nominal standing Z. This "pre-loads" the legs against the ground
# so a heavy body doesn't bounce or lose traction mid-stride. Value in mm.
STANCE_PRESS_MM = 32.0   # baked: femur drives down harder for ground authority (was 10)

# ── CoG lean toward the planted tripod (heavy-base dynamic stability) ─────────
# Tripod gait lifts 3 legs at once; during that swing only the OTHER 3 legs
# support the body. For a heavy base the centre of gravity can drift to the
# edge of that support triangle and tip. Real insects pre-lean toward the
# planted tripod before lifting — we do the same: a small lateral body shift,
# synced to the gait phase, toward whichever tripod is currently on the ground.
#
# sin(2*pi*phase): +max at phase 0.25 (mid group-0 swing → lean toward the
# group-1 support, which sits left-of-centre) and -max at phase 0.75 (mid
# group-1 swing → lean toward group-0 support, right-of-centre). It is exactly
# 0 at the double-support transitions and at phase 0 (standing), so it never
# introduces a static lean. Set to 0.0 to disable. Costs no speed — it's a
# lateral overlay on top of the forward stride.
BODY_SHIFT_MM = 0.0   # zeroed to match proven old config (no CoG lean on forward walk).

# ── Strafe CoG lean (lateral stability) ──────────────────────────────────────
# When STRAFING (lateral stride, no forward stride), lean the body toward the
# currently-planted tripod each half-cycle so the centre of gravity stays inside
# the support polygon. Without this, the robot can fail to commit weight to the
# swing side in one lateral direction and just "stomps in place" (the gait math
# is perfectly symmetric, so a one-sided stomp is a CoG-margin problem, not a
# sign bug). Applied ONLY while strafing, so forward walking is unchanged.
#   Positive = lean toward the planted tripod (correct sense).  Set 0 to disable.
#   If strafing gets WORSE, flip the sign (the lean is going the wrong way).
STRAFE_LEAN_MM = 15.0

# ── Per-leg lift multiplier (heavy-base back-leg drag fix) ────────────────────
# Back legs tend to drag on heavy robots because the centre of gravity shifts
# rearward and the rear leg geometry produces less ground clearance at the same
# commanded lift height.  Give back legs extra swing lift to compensate.
# Front + back legs swing higher than the middle pair (more corner clearance).
# Middle stays 1.0. base lift = LIFT_MM (85mm), so:
#   front ×1.25 ≈ 106mm, back ×1.55 ≈ 132mm, middle ×1.0 = 85mm.
# ⚠ PULLED BACK from 1.45/1.75: those values made the corner femur servos
# (rightFront, leftBack — the two with the higher femur calibration) trip their
# HX-35HM over-load/over-temp protection after the first lift, so they'd swing up
# once then stay curled. Raise these again ONLY if those servos stay cool; if a
# leg curls-and-sticks, the femur tripped protection → lower its value here.
LEG_LIFT_MULT = {
    'rightFront': 1.4, 'rightMiddle': 1.0, 'rightBack': 1.55,
    'leftFront':  1.4, 'leftMiddle':  1.0, 'leftBack':  1.55,
}   # front UP-swing raised 1.25→1.4. ⚠ This is the same lever that tripped the
    # rightFront femur's over-load protection before (curl). If rightFront swings
    # up once then stays curled, drop this back toward 1.3 (or the servo's hot).

# ── Per-leg horizontal stride multiplier ─────────────────────────────────────
# Scales how far each leg reaches fore/aft (and laterally when strafing) per
# step, on top of the global STRIDE_MM. Front legs take a slightly longer step
# so their swing looks bigger. NOTE: legs that step farther than the others in
# the same time must scrub a little on a high-grip floor (the body has one
# speed); keep front modest (≤1.3) to limit it. 1.0 = no change.
LEG_STRIDE_MULT = {
    'rightFront': 1.1, 'rightMiddle': 1.0, 'rightBack': 1.0,
    'leftFront':  1.1, 'leftMiddle':  1.0, 'leftBack':  1.0,
}   # front step pulled back 1.3→1.1 to free tibia servo range for the much bigger
    # outward splay (TIBIA_OUT_TRIM_EXTRA, below). The tibia servo's 1000 limit is
    # shared between "splay out" and "reach far on the step" — can't max both. The
    # front legs still swing big via the up-lift + the articulating stance.


# ── Small math helpers ────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _rot2d(x, y, ang):
    """Rotate the point (x, y) about the origin by `ang` radians (CCW)."""
    c, s = math.cos(ang), math.sin(ang)
    return x * c - y * s, x * s + y * c


# ── Neutral foot positions ────────────────────────────────────────────────────

def neutral_foot(leg, height=STANCE_HEIGHT):
    """Body-frame (x, y, z) of a leg's foot at the standing pose."""
    mx, my = LEG_ATTACH[leg]
    ang = MOUNT_ANGLE[leg]
    reach = COXA + STANCE_EXT
    # Per-leg downward trim: push selected feet lower so they carry more load
    # (heavy-base middle-leg fix, #1).
    z = -(height + LEG_GROUND_TRIM_MM.get(leg, 0.0))
    return (mx + reach * math.cos(ang),
            my + reach * math.sin(ang),
            z)


# ── Inverse kinematics: foot position (body frame) -> joint angles ────────────

def leg_ik(leg, foot_body):
    """
    Solve the three joint angles that place `leg`'s foot at `foot_body`
    (a body-frame (x, y, z) tuple in mm).

    Returns (coxa, femur, tibia) in radians, absolute (see module docstring).
    Reach is clamped so an out-of-range target degrades gracefully instead
    of producing a math-domain error.
    """
    mx, my = LEG_ATTACH[leg]
    ang = MOUNT_ANGLE[leg]

    # Foot relative to the coxa joint, expressed in the leg frame
    # (rotate the body-frame offset by -mount_angle so +x points outward).
    rx = foot_body[0] - mx
    ry = foot_body[1] - my
    rz = foot_body[2]
    c, s = math.cos(ang), math.sin(ang)
    x =  rx * c + ry * s     # outward
    y = -rx * s + ry * c     # sideways
    z =  rz                  # up

    # Coxa yaw to face the foot.
    coxa = math.atan2(y, x)

    # Work in the vertical plane that now contains the leg.
    r = math.hypot(x, y) - COXA          # horizontal reach beyond the coxa
    d = math.hypot(r, z)                 # straight-line femur-joint -> foot
    d = _clamp(d, abs(FEMUR - TIBIA) + 1e-3, FEMUR + TIBIA - 1e-3)

    # Femur angle: aim at the foot, then open up by the femur/foot triangle.
    a1 = math.atan2(z, r)
    cos_a2 = (FEMUR * FEMUR + d * d - TIBIA * TIBIA) / (2.0 * FEMUR * d)
    a2 = math.acos(_clamp(cos_a2, -1.0, 1.0))
    femur = a1 + a2

    # Knee interior angle; tibia = 0 when the leg is fully straight.
    cos_knee = (FEMUR * FEMUR + TIBIA * TIBIA - d * d) / (2.0 * FEMUR * TIBIA)
    knee = math.acos(_clamp(cos_knee, -1.0, 1.0))
    tibia = knee - math.pi

    return coxa, femur, tibia


def standing_angles(leg, height=STANCE_HEIGHT):
    """Joint angles at the neutral standing pose — the controller's zero ref."""
    return leg_ik(leg, neutral_foot(leg, height))


# ── Body-pose transform ───────────────────────────────────────────────────────

def apply_body_pose(foot_body, roll, pitch, yaw, tx, ty, tz):
    """
    Re-express a nominal foot target after the BODY is moved by translation
    (tx, ty, tz) and rotated by (roll, pitch, yaw) — feet stay planted in the
    world, so the foot moves in the (moved) body frame by R^T * (foot - t).

    roll  > 0 : right side dips        (rotation about +x)
    pitch > 0 : nose dips              (rotation about +y)
    yaw   > 0 : body turns CCW         (rotation about +z)
    tz    > 0 : body rises
    Angles in radians, translation in mm.
    """
    x = foot_body[0] - tx
    y = foot_body[1] - ty
    z = foot_body[2] - tz

    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll).  We need R^T = Rx(-r) Ry(-p) Rz(-y).
    # Apply Rz(-yaw):
    x1 =  cy * x + sy * y
    y1 = -sy * x + cy * y
    z1 = z
    # Apply Ry(-pitch):
    x2 =  cp * x1 - sp * z1
    y2 =  y1
    z2 =  sp * x1 + cp * z1
    # Apply Rx(-roll):
    x3 =  x2
    y3 =  cr * y2 + sr * z2
    z3 = -sr * y2 + cr * z2
    return (x3, y3, z3)


# ── Tripod gait foot trajectory ───────────────────────────────────────────────

def gait_foot(leg, phase, stride_x, stride_y, turn, lift, height=STANCE_HEIGHT,
              leg_phase=None, swing_duty=0.5):
    """
    Foot position (body frame) for `leg` at a given gait `phase` in [0, 1).

    A full cycle (phase 0 -> 1) is one complete gait cycle: each leg swings once
    and stances once.

    stride_x, stride_y : body translation per cycle (mm). +x forward, +y left.
    turn               : body yaw per cycle (rad). +turn = CCW.
    lift               : peak foot lift during swing (mm).
    leg_phase          : this leg's phase offset in [0,1). None = tripod default
                         (0.5*GAIT_GROUP). Different per-leg offsets give ripple /
                         wave gaits.
    swing_duty         : fraction of the cycle the foot is airborne (0.5 = tripod).
                         Smaller = more legs planted at once (more stable, slower).

    Swing  : foot arcs from the back extreme to the front extreme, lifted.
    Stance : foot drives from the front extreme to the back extreme, grounded,
             which pushes the body the other way (forward).
    """
    if leg_phase is None:
        leg_phase = 0.5 * GAIT_GROUP[leg]      # tripod default
    nx, ny, nz = neutral_foot(leg, height)

    # Front / back extremes for THIS leg, combining translation and rotation.
    # Rotation extremes are the neutral foot swept ±turn/2 about the body centre.
    # Per-leg stride multiplier lengthens/shortens the linear step (front legs
    # reach a bit farther); the rotation sweep is NOT scaled (keeps turning true).
    sm = LEG_STRIDE_MULT.get(leg, 1.0)
    fx, fy = _rot2d(nx, ny, +turn / 2.0)
    fx += sm * stride_x / 2.0
    fy += sm * stride_y / 2.0
    bx, by = _rot2d(nx, ny, -turn / 2.0)
    bx -= sm * stride_x / 2.0
    by -= sm * stride_y / 2.0

    # Phase shifted by this leg's offset; swing for the first `swing_duty` of the
    # cycle, stance for the rest. (tripod = offset 0.5*group, duty 0.5.)
    local = (phase - leg_phase) % 1.0
    d = swing_duty

    if local < d:                         # swing — airborne, back -> front
        u = local / d
        # smoothstep xy easing: foot starts and ends swing at zero forward
        # velocity, so the moment of plant doesn't slam into the reversed
        # stance velocity. Eliminates the swing/stance jerk discontinuity.
        u_xy = (1.0 - math.cos(math.pi * u)) / 2.0
        x = bx + (fx - bx) * u_xy
        y = by + (fy - by) * u_xy
        z = nz + lift * LEG_LIFT_MULT.get(leg, 1.0) * math.sin(math.pi * u) ** 2
    else:                                 # stance — grounded, front -> back
        u = (local - d) / (1.0 - d)
        # Cosine-eased stance XY movement — prevents abrupt start/stop when
        # the foot is on the ground pushing a heavy body. Much smoother.
        u_stance = (1.0 - math.cos(math.pi * u)) / 2.0
        x = fx + (bx - fx) * u_stance
        y = fy + (by - fy) * u_stance
        # Press the foot slightly below nominal Z during stance to pre-load
        # against the ground. Smoothly ease in/out at the phase boundaries
        # so there's no discontinuity with swing Z.
        press = STANCE_PRESS_MM * math.sin(math.pi * u)
        z = nz - press

    # CoG lean toward the planted tripod (heavy-base anti-tip). Body-frame
    # lateral shift: moving the body +y means every foot target moves -y.
    # Applied to swing AND stance feet uniformly — it's a whole-body motion,
    # equivalent to the body translating laterally over the planted feet.
    y -= BODY_SHIFT_MM * math.sin(2.0 * math.pi * phase)

    # Strafe-only CoG lean toward the planted tripod (lateral anti-stomp).
    # Detect strafing by lateral-only stride. sin(2*pi*phase) is +max when the
    # group-1 tripod (left-of-centre) is planted and -max when group-0 (right-of-
    # centre) is, so moving the feet by -lean*sin shifts the body toward whichever
    # is on the ground. Forward/backward walking (stride_x != 0) is untouched.
    if STRAFE_LEAN_MM != 0.0 and abs(stride_y) > 1e-6 and abs(stride_x) < 1e-6:
        y -= STRAFE_LEAN_MM * math.sin(2.0 * math.pi * phase)

    return (x, y, z)
