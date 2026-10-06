"""
joy_teleop.py
Bridges a PS3 (DualShock 3) controller connected via USB to the hexapod.

Controller layout:
  Left  stick Y   (axis 1)  → walk speed  (forward / backward)
  Left  stick X   (axis 0)  → steering yaw
  Right stick Y   (axis 3)  → body pitch
  Right stick X   (axis 2)  → body roll
  Cross   (btn 14)           → start walking
  Circle  (btn 13)           → stop
  Square  (btn 15)           → walk backward
  L1      (btn 10)           → lower body (TZ -)
  R1      (btn 11)           → raise body (TZ +)
  PS btn  (btn 16)           → reset body pose

Published topics:
  /hexapod/body_pose  (geometry_msgs/Twist)
  /hexapod/cmd        (std_msgs/String)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── PS3 DualShock 3 USB axis / button indices ─────────────────────────────────
AX_LX   = 0   # left  stick horizontal  → steering
AX_LY   = 1   # left  stick vertical    → speed  (axis inverted: push up = -1)
AX_RX   = 2   # right stick horizontal  → body roll
AX_RY   = 3   # right stick vertical    → body pitch

BTN_CROSS    = 14  # walk forward
BTN_CIRCLE   = 13  # stop
BTN_SQUARE   = 15  # walk backward
BTN_L1       = 10  # lower body
BTN_R1       = 11  # raise body
BTN_PS       = 16  # reset pose

# Tuning
MAX_STEER_DEG  = 40.0   # max yaw sent to controller (degrees)
MAX_ROLL_DEG   = 20.0
MAX_PITCH_DEG  = 20.0
TZ_STEP_MM     = 2.0    # mm per button press tick
TZ_MIN_MM      = -30.0
TZ_MAX_MM      =  30.0
DEADZONE       = 0.08   # ignore tiny stick drift


def _deadzone(val: float) -> float:
    return val if abs(val) > DEADZONE else 0.0


class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        self.pose_pub = self.create_publisher(Twist,  '/hexapod/body_pose', 10)
        self.cmd_pub  = self.create_publisher(String, '/hexapod/cmd',       10)

        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        self._prev_buttons = []
        self._tz_mm = 0.0
        self._walking = False

        self.get_logger().info(
            'Joy Teleop ready.\n'
            '  Cross=walk  Circle=stop  Square=backward\n'
            '  L1/R1=height  Left stick=steer  Right stick=tilt'
        )

    def _on_joy(self, msg: Joy):
        axes    = msg.axes
        buttons = msg.buttons

        def pressed(idx):
            """True on the rising edge (just pressed this tick)."""
            prev = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
            return buttons[idx] == 1 and prev == 0

        # ── Motion commands (rising-edge only) ────────────────────────────────
        if pressed(BTN_CROSS):
            self._send_cmd('walk')
            self._walking = True

        if pressed(BTN_CIRCLE):
            self._send_cmd('stop')
            self._walking = False

        if pressed(BTN_SQUARE):
            self._send_cmd('backward')
            self._walking = True

        # ── Height adjust (hold L1/R1) ────────────────────────────────────────
        if buttons[BTN_L1]:
            self._tz_mm = max(TZ_MIN_MM, self._tz_mm - TZ_STEP_MM)
        if buttons[BTN_R1]:
            self._tz_mm = min(TZ_MAX_MM, self._tz_mm + TZ_STEP_MM)

        # ── PS button: reset pose ─────────────────────────────────────────────
        if pressed(BTN_PS):
            self._tz_mm = 0.0

        # ── Body pose from sticks ─────────────────────────────────────────────
        lx = _deadzone(axes[AX_LX]) if AX_LX < len(axes) else 0.0
        ly = _deadzone(axes[AX_LY]) if AX_LY < len(axes) else 0.0
        rx = _deadzone(axes[AX_RX]) if AX_RX < len(axes) else 0.0
        ry = _deadzone(axes[AX_RY]) if AX_RY < len(axes) else 0.0

        twist = Twist()
        # Left stick Y → speed (axis up = -1 on most controllers, invert it)
        twist.linear.x  = -ly              # speed intensity (-1 to 1)
        # Left stick X → steering yaw
        twist.angular.z = math.radians(lx * MAX_STEER_DEG)
        # Right stick → body tilt
        twist.angular.x = math.radians(rx * MAX_ROLL_DEG)   # roll
        twist.angular.y = math.radians(-ry * MAX_PITCH_DEG)  # pitch
        # Height from L1/R1
        twist.linear.z  = self._tz_mm

        self.pose_pub.publish(twist)
        self._prev_buttons = list(buttons)

    def _send_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'CMD → {cmd}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
