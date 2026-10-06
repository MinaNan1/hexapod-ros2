"""
Hexapod Gait Controller with Emotes
Uses DIRECT JOINT ANGLES for emotes (no IK failure risk).
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import numpy as np
from .kinematics import HexapodKinematics


class GaitController(Node):

    def __init__(self):
        super().__init__('gait_controller')
        self.declare_parameter('gait_type', 'tripod')
        self.declare_parameter('gait_frequency', 1.0)
        self.gait_type = self.get_parameter('gait_type').value
        self.frequency = self.get_parameter('gait_frequency').value

        self.ik = HexapodKinematics()

        self.walking = False
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.phase = 0.0
        self.stance = self.ik.default_stance()

        # Emote state
        self.trump_active = False
        self.trump_phase = 0.0
        self.trump_lift_t = 0.0
        self.TRUMP_PUMP_HZ = 1.5
        self.TRUMP_LIFT_DUR = 0.6

        self.salute_active = False
        self.salute_lift_t = 0.0
        self.SALUTE_LIFT_DUR = 0.8

        self.LEG_MAPPING = {
            'fr': 'r1', 'mr': 'r2', 'rr': 'r3',
            'fl': 'l1', 'ml': 'l2', 'rl': 'l3'
        }
        self.ALL_LEGS = ['fr', 'mr', 'rr', 'fl', 'ml', 'rl']

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.trump_emote_pub = self.create_publisher(Bool, '/trump_emote_active', 10)

        self.create_subscription(Twist, '/hexapod/body_pose', self.cmd_vel_cb, 10)
        self.create_subscription(String, '/gait_mode', self.gait_mode_cb, 10)
        self.create_subscription(String, '/hexapod/cmd', self.hexapod_cmd_cb, 10)

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('Gait controller ready')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def cmd_vel_cb(self, msg):
        if self.trump_active or self.salute_active:
            return
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.vel_yaw = msg.angular.z
        self.walking = (abs(self.vel_x) > 0.01 or
                        abs(self.vel_y) > 0.01 or
                        abs(self.vel_yaw) > 0.01)

    def gait_mode_cb(self, msg):
        if msg.data in ['tripod', 'wave', 'ripple']:
            self.gait_type = msg.data

    def hexapod_cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'trump':
            self._stop_salute()
            self._start_trump()
        elif cmd == 'trump_stop':
            self._stop_trump()
        elif cmd == 'salute':
            self._stop_trump()
            self._start_salute()
        elif cmd == 'salute_stop':
            self._stop_salute()
        elif cmd == 'stop':
            self._stop_trump()
            self._stop_salute()
            self.walking = False

    # ── Stance joint angles for one leg ──────────────────────────────────
    def _stance_angles(self):
        return (self.ik.STANCE_THETA1, self.ik.STANCE_THETA2, self.ik.STANCE_THETA3)

    # ── Publish a full set of joint angles (dict: leg -> (t1,t2,t3)) ────
    def _publish_joint_angles(self, angles_dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for leg, (t1, t2, t3) in angles_dict.items():
            s = self.LEG_MAPPING[leg]
            msg.name.extend([f'coxa_joint_{s}', f'femur_joint_{s}', f'tibia_joint_{s}'])
            msg.position.extend([float(t1), float(t2), float(t3)])
        self.joint_pub.publish(msg)

    # ── Trump emote (direct joint angles) ────────────────────────────────
    def _start_trump(self):
        if self.trump_active:
            return
        self.get_logger().info('Trump emote: START')
        self.walking = False
        self.trump_active = True
        self.trump_phase = 0.0
        self.trump_lift_t = 0.0
        self.trump_emote_pub.publish(Bool(data=True))

    def _stop_trump(self):
        if not self.trump_active:
            return
        self.get_logger().info('Trump emote: STOP')
        self.trump_active = False
        self.trump_phase = 0.0
        self.trump_lift_t = 0.0
        self.trump_emote_pub.publish(Bool(data=False))

    def _trump_tick(self):
        dt = 0.02
        self.trump_lift_t = min(1.0, self.trump_lift_t + dt / self.TRUMP_LIFT_DUR)
        ease = 0.5 - 0.5 * math.cos(math.pi * self.trump_lift_t)

        if self.trump_lift_t >= 1.0:
            self.trump_phase = (self.trump_phase + self.TRUMP_PUMP_HZ * dt) % 1.0

        st1, st2, st3 = self._stance_angles()
        angles = {}

        # Support legs hold stance
        for leg in ['ml', 'mr', 'rl', 'rr']:
            angles[leg] = (st1, st2, st3)

        # Dancing legs: direct joint angles for elbow pump
        for i, leg in enumerate(['fl', 'fr']):
            phase_offset = 0.5 * i
            pump = (self.trump_phase + phase_offset) % 1.0
            pump_wave = 0.5 - 0.5 * math.cos(2 * math.pi * pump)

            # Pump femur between -0.3 (low) and -0.8 (high)
            target_t2 = -0.3 - 0.5 * pump_wave
            # Bent elbow look
            target_t3 = -1.0

            t1 = st1
            t2 = st2 + (target_t2 - st2) * ease
            t3 = st3 + (target_t3 - st3) * ease
            angles[leg] = (t1, t2, t3)

        self._publish_joint_angles(angles)

    # ── Salute emote (direct joint angles) ───────────────────────────────
    def _start_salute(self):
        if self.salute_active:
            return
        self.get_logger().info('Salute emote: START')
        self.walking = False
        self.salute_active = True
        self.salute_lift_t = 0.0

    def _stop_salute(self):
        if not self.salute_active:
            return
        self.get_logger().info('Salute emote: STOP')
        self.salute_active = False
        self.salute_lift_t = 0.0

    def _salute_tick(self):
        dt = 0.02
        self.salute_lift_t = min(1.0, self.salute_lift_t + dt / self.SALUTE_LIFT_DUR)
        ease = 0.5 - 0.5 * math.cos(math.pi * self.salute_lift_t)

        st1, st2, st3 = self._stance_angles()
        angles = {}

        # 5 legs hold stance
        for leg in ['fl', 'ml', 'mr', 'rl', 'rr']:
            angles[leg] = (st1, st2, st3)

        # Right front (fr/r1): raise with straight tibia
        # Target: coxa=0, femur raised up (-0.9), tibia straight (0)
        target_t1 = 0.0
        target_t2 = -0.9   # raised up ~50 degrees
        target_t3 = 0.0    # flat/straight out

        t1 = st1 + (target_t1 - st1) * ease
        t2 = st2 + (target_t2 - st2) * ease
        t3 = st3 + (target_t3 - st3) * ease
        angles['fr'] = (t1, t2, t3)

        self._publish_joint_angles(angles)

    # ── Main loop ────────────────────────────────────────────────────────
    def control_loop(self):
        if self.trump_active:
            self._trump_tick()
            return
        if self.salute_active:
            self._salute_tick()
            return
        if self.walking:
            self.phase = (self.phase + self.frequency * 0.02) % 1.0
            self.tripod_gait()
        else:
            # Always publish stance when idle
            angles = {leg: self._stance_angles() for leg in self.ALL_LEGS}
            self._publish_joint_angles(angles)

    # ── Walking gait ─────────────────────────────────────────────────────
    def tripod_gait(self):
        foot_positions = {}
        for leg in self.ik.TRIPOD_A:
            if self.phase < 0.5:
                foot_positions[leg] = self._swing(leg, self.phase / 0.5)
            else:
                foot_positions[leg] = self._propel(leg, (self.phase - 0.5) / 0.5)
        for leg in self.ik.TRIPOD_B:
            if self.phase >= 0.5:
                foot_positions[leg] = self._swing(leg, (self.phase - 0.5) / 0.5)
            else:
                foot_positions[leg] = self._propel(leg, self.phase / 0.5)
        self._publish_ik_positions(foot_positions)

    def _swing(self, leg, t):
        bx, by, bz = self.stance[leg]
        step = self.ik.STEP_LENGTH
        dx = step if abs(self.vel_x) < 0.01 and abs(self.vel_y) < 0.01 else (self.vel_x / max(0.01, np.sqrt(self.vel_x**2 + self.vel_y**2))) * step
        dy = 0.0 if abs(self.vel_x) < 0.01 and abs(self.vel_y) < 0.01 else (self.vel_y / max(0.01, np.sqrt(self.vel_x**2 + self.vel_y**2))) * step
        return (bx + dx * t, by + dy * t, bz + self.ik.STEP_HEIGHT * np.sin(np.pi * t))

    def _propel(self, leg, t):
        bx, by, bz = self.stance[leg]
        step = self.ik.STEP_LENGTH
        dx = step if abs(self.vel_x) < 0.01 and abs(self.vel_y) < 0.01 else (self.vel_x / max(0.01, np.sqrt(self.vel_x**2 + self.vel_y**2))) * step
        dy = 0.0 if abs(self.vel_x) < 0.01 and abs(self.vel_y) < 0.01 else (self.vel_y / max(0.01, np.sqrt(self.vel_x**2 + self.vel_y**2))) * step
        return (bx - dx * (t - 0.5), by - dy * (t - 0.5), bz)

    def _publish_ik_positions(self, foot_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for leg, (fx, fy, fz) in foot_positions.items():
            result = self.ik.solve_ik(leg, fx, fy, fz)
            if result is None:
                self.get_logger().warn(f'{leg}: IK unreachable ({fx:.3f},{fy:.3f},{fz:.3f})')
                t1, t2, t3 = self._stance_angles()  # fallback to stance
            else:
                t1, t2, t3 = result
            s = self.LEG_MAPPING[leg]
            msg.name.extend([f'coxa_joint_{s}', f'femur_joint_{s}', f'tibia_joint_{s}'])
            msg.position.extend([float(t1), float(t2), float(t3)])
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()