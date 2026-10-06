"""
Balance Controller Node

⚠️  NOTE (superseded for the live robot):
    This node was written for the OLDER controller lineage
    (ros2_ws/.../gait_controller.py) and publishes /balance_correction,
    which the currently-running controller (hexapod_ws/.../hexapod_controller.py)
    does NOT subscribe to. Self-balancing for the live robot is now built
    directly into hexapod_controller.py, fed by imu_node.py over /imu/data.
    See docs/04-imu-balancing.md. Kept here only for the gait_controller path.

Reads IMU data and adjusts body orientation to keep hexapod level

Trump Emote Mode:
  When trump_emote_active is True:
    - Front legs (fl, fr) are excluded from corrections (they're in the air dancing)
    - max_correction is boosted to handle the shifted CoM on 4 legs
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
import numpy as np

class BalanceController(Node):

    def __init__(self):
        super().__init__('balance_controller')

        self.declare_parameter('max_correction',        0.15)
        self.declare_parameter('max_correction_emote',  0.22)  # boosted for 4-leg stance
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('kd', 0.1)

        self.max_correction       = self.get_parameter('max_correction').value
        self.max_correction_emote = self.get_parameter('max_correction_emote').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value

        self.roll  = 0.0
        self.pitch = 0.0
        self.prev_roll  = 0.0
        self.prev_pitch = 0.0

        # ── Trump emote state ────────────────────────────────────────────────
        # When True: front legs are in the air — exclude them from corrections
        # and use a stronger correction to stabilise the 4-leg support polygon
        self.trump_emote_active = False
        # ────────────────────────────────────────────────────────────────────

        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # NEW: subscribe to emote flag published by gait_controller
        self.create_subscription(Bool, '/trump_emote_active', self.trump_emote_cb, 10)

        self.correction_pub = self.create_publisher(JointState, '/balance_correction', 10)

        self.get_logger().info('Balance controller started')

    # ── NEW: Trump emote flag callback ───────────────────────────────────────

    def trump_emote_cb(self, msg: Bool):
        self.trump_emote_active = msg.data
        if self.trump_emote_active:
            self.get_logger().info(
                'Balance: Trump emote ON — '
                'fl/fr excluded, max_correction boosted to '
                f'{self.max_correction_emote:.2f} rad'
            )
        else:
            self.get_logger().info(
                'Balance: Trump emote OFF — '
                'all 6 legs active, max_correction restored to '
                f'{self.max_correction:.2f} rad'
            )

    # ── IMU callback (modified) ───────────────────────────────────────────────

    def imu_cb(self, msg):
        # Extract roll and pitch from quaternion
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = np.arctan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = np.clip(sinp, -1.0, 1.0)
        self.pitch = np.arcsin(sinp)

        # PD controller
        roll_correction  = self.kp * self.roll  + self.kd * (self.roll  - self.prev_roll)
        pitch_correction = self.kp * self.pitch + self.kd * (self.pitch - self.prev_pitch)

        # Choose correction limit based on emote state
        # During emote: CoM is shifted forward/up → need stronger stabilisation
        clamp = self.max_correction_emote if self.trump_emote_active \
                else self.max_correction

        roll_correction  = np.clip(roll_correction,  -clamp, clamp)
        pitch_correction = np.clip(pitch_correction, -clamp, clamp)

        self.prev_roll  = self.roll
        self.prev_pitch = self.pitch

        if abs(roll_correction) > 0.01 or abs(pitch_correction) > 0.01:
            self.get_logger().debug(
                f'Balance correction: roll={roll_correction:.3f} '
                f'pitch={pitch_correction:.3f} '
                f'[emote={self.trump_emote_active}]'
            )

        self.publish_correction(roll_correction, pitch_correction)

    def publish_correction(self, roll, pitch):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.trump_emote_active:
            # ── EMOTE MODE: only correct the 4 support legs ───────────────
            # fl and fr are in the air — sending corrections to them would
            # fight the gait_controller's elbow-pump trajectory and cause
            # jitter or tip-over.
            corrections = {
                # Middle legs — primary lateral stabilisers
                'femur_joint_l2': -roll * 0.5,
                'femur_joint_r2':  roll * 0.5,
                # Rear legs — handle both pitch and roll
                'femur_joint_l3':  pitch * 0.5 - roll * 0.5,
                'femur_joint_r3':  pitch * 0.5 + roll * 0.5,
            }
        else:
            # ── NORMAL MODE: all 6 legs ───────────────────────────────────
            corrections = {
                'femur_joint_l1': -pitch * 0.5 - roll * 0.5,
                'femur_joint_r1': -pitch * 0.5 + roll * 0.5,
                'femur_joint_l2': -roll * 0.5,
                'femur_joint_r2':  roll * 0.5,
                'femur_joint_l3':  pitch * 0.5 - roll * 0.5,
                'femur_joint_r3':  pitch * 0.5 + roll * 0.5,
            }

        msg.name     = list(corrections.keys())
        msg.position = list(corrections.values())
        self.correction_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
