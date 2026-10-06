import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

# Commands that count as "walking" for the deadman switch
WALKING_CMDS = {'walk', 'forward', 'backward',
                'strafe_left', 'strafe_right',
                'rotate_left', 'rotate_right'}

# Emotes/dances are pass-through — they don't reset the deadman timer and are
# allowed in both manual and auto mode. "<name>" starts; "<name>_stop" (or the
# generic "dance_stop") ends whichever is running.
EMOTE_NAMES = {'trump', 'salute', 'bounce', 'sway', 'stretch', 'wave'}


def _is_emote_cmd(cmd: str) -> bool:
    return cmd in EMOTE_NAMES or cmd.endswith('_stop') or cmd == 'dance_stop'

class UITeleopNode(Node):
    def __init__(self):
        super().__init__('ui_teleop_mux')

        self.declare_parameter('deadman_timeout_sec', 0.5)
        self.timeout = self.get_parameter('deadman_timeout_sec').value

        # Mode: True = Manual, False = Auto
        self.manual_mode = True

        self.last_cmd_time = self.get_clock().now()
        self.walking = False

        # Name of the emote/dance currently running (None = none), so we can
        # cancel it on stop / mode-switch and ignore duplicate start requests.
        self.active_emote = None

        # Publishers to actual hexapod controller
        self.body_pub    = self.create_publisher(Twist,  '/hexapod/body_pose', 10)
        self.posture_pub = self.create_publisher(Twist,  '/hexapod/posture',   10)
        self.cmd_pub     = self.create_publisher(String, '/hexapod/cmd',       10)

        # Subscribers from UI
        self.create_subscription(Twist,  '/ui/body_pose',    self.ui_pose_cb,    10)
        self.create_subscription(Twist,  '/ui/body_posture', self.ui_posture_cb, 10)
        self.create_subscription(String, '/ui/cmd',          self.ui_cmd_cb,     10)
        self.create_subscription(Bool,   '/ui/mode',         self.ui_mode_cb,    10)

        # Timer for deadman switch
        self.create_timer(0.1, self.deadman_check)

        self.get_logger().info('UI Teleop Mux started. Default: Manual mode.')

    def ui_mode_cb(self, msg: Bool):
        self.manual_mode = msg.data
        mode_str = "MANUAL" if self.manual_mode else "AUTO"
        self.get_logger().info(f'Mode switched to {mode_str}')
        if self.manual_mode:
            # Cancel any running emote and stop robot when switching to manual
            self._cancel_emote()
            self.stop_robot()

    def ui_pose_cb(self, msg: Twist):
        """Forward joystick motion data (speed + steering) to the controller."""
        if not self.manual_mode:
            return
        self.last_cmd_time = self.get_clock().now()

        twist_out = Twist()
        twist_out.linear.x  = msg.linear.x   # Speed intensity (0.0–1.0)
        twist_out.angular.z = msg.angular.z   # Steering yaw
        self.body_pub.publish(twist_out)

    def ui_posture_cb(self, msg: Twist):
        """Forward slider posture data directly to the controller."""
        if not self.manual_mode:
            return
        self.posture_pub.publish(msg)

    def ui_cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        # ── Emote commands: allowed regardless of mode, no deadman reset ──
        if _is_emote_cmd(cmd):
            self._handle_emote_cmd(cmd)
            return

        # ── All other commands require manual mode ─────────────────────────
        if not self.manual_mode:
            return

        self.last_cmd_time = self.get_clock().now()
        self.cmd_pub.publish(msg)

        if cmd in WALKING_CMDS:
            self.walking = True
        elif cmd == 'stop':
            # A hard stop also cancels any running emote
            self._cancel_emote()
            self.walking = False

    # ── Emote helpers ─────────────────────────────────────────────────────────

    def _handle_emote_cmd(self, cmd: str):
        # A stop request ("<name>_stop" or "dance_stop") ends whatever is running.
        if cmd.endswith('_stop') or cmd == 'dance_stop':
            self._cancel_emote()
            return

        # Otherwise it's a start request for a named emote.
        if cmd == self.active_emote:
            self.get_logger().info(f'{cmd} emote already active — ignoring.')
            return
        self._cancel_emote()           # cancel any other emote first
        self.get_logger().info(f'{cmd} emote: starting')
        self.active_emote = cmd
        self.walking = False
        fwd = String()
        fwd.data = cmd
        self.cmd_pub.publish(fwd)

    def _cancel_emote(self):
        if self.active_emote is None:
            return
        self.get_logger().info(f'Cancelling {self.active_emote} emote')
        stop = String()
        stop.data = f'{self.active_emote}_stop'
        self.cmd_pub.publish(stop)
        self.active_emote = None

    # ── Deadman switch ────────────────────────────────────────────────────────

    def deadman_check(self):
        # Never fire deadman during emote — the controller manages legs
        if not self.manual_mode or not self.walking or self.active_emote is not None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.timeout:
            self.get_logger().warn('Deadman switch triggered! Stopping hexapod.')
            self.stop_robot()

    def stop_robot(self):
        msg = String()
        msg.data = 'stop'
        self.cmd_pub.publish(msg)
        self.walking = False


def main(args=None):
    rclpy.init(args=args)
    node = UITeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()