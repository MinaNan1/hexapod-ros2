"""
hexapod_path_planner_node.py  (ros2_integration)
=================================================
ROS2 node that integrates A* path planning with hexapod motion control.

Architecture
------------
This node sits BETWEEN a goal source (RViz / topic) and the existing
hexapod_controller.py — it does NOT modify the controller.

Data flow
---------
/goal_pose  (PoseStamped)
    ↓  on_goal_callback()
A* planner  →  path (grid cells)
    ↓  plan_path()
PathSmoother → fewer waypoints
    ↓
PathConverter → world coordinates (metres)
    ↓
PathFollower → loaded with waypoints
    ↓  control_loop()  20 Hz
/hexapod/body_pose  (Twist)  ← hexapod_controller reads this unchanged
    ↓
/joint_states → Gazebo

Usage
-----
ros2 run hexapod_control hexapod_path_planner

To send a goal from the command line:
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 2.0, y: 2.0}}}"
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

import sys
import os

# Allow running from the ros2_integration folder directly during development
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from map_creation.map_inflator   import MapInflator
from path_finding.astar_planner  import AStarPlanner
from path_finding.path_smoother  import PathSmoother
from path_finding.path_converter import PathConverter
from ros2_integration.path_follower import PathFollower


class HexapodPathPlannerNode(Node):
    """
    ROS2 node for A*-based path planning on a 2D occupancy grid.

    ROS2 Parameters (set in launch file or via command line)
    --------------------------------------------------------
    map_path        : str    path to .npy map file        (default: 'my_map.npy')
    map_resolution  : float  metres per cell              (default: 0.05)
    robot_radius    : int    inflation radius in cells     (default: 2)
    step_speed      : float  mm per 50ms frame             (default: 10.0)
    max_yaw         : float  max yaw per frame (rad)       (default: 0.3)
    rdp_epsilon     : float  RDP simplification tolerance  (default: 2.0)
    """

    def __init__(self):
        super().__init__('hexapod_path_planner')

        # ── Declare ROS2 parameters ────────────────────────────────
        self.declare_parameter('map_path',       'my_map.npy')
        self.declare_parameter('map_resolution',  0.05)
        self.declare_parameter('robot_radius',    2)
        self.declare_parameter('step_speed',      10.0)
        self.declare_parameter('max_yaw',         0.3)
        self.declare_parameter('rdp_epsilon',     2.0)

        # ── Load parameters ────────────────────────────────────────
        map_path   = self.get_parameter('map_path').value
        resolution = self.get_parameter('map_resolution').value
        radius     = self.get_parameter('robot_radius').value
        speed      = self.get_parameter('step_speed').value
        max_yaw    = self.get_parameter('max_yaw').value
        epsilon    = self.get_parameter('rdp_epsilon').value

        # ── Load and inflate map ───────────────────────────────────
        try:
            raw_grid = np.load(map_path)
            self.get_logger().info(f'Map loaded: {raw_grid.shape}')
        except FileNotFoundError:
            self.get_logger().fatal(f'Map not found: {map_path}')
            raise SystemExit(1)

        inflator   = MapInflator(robot_radius_cells=radius)
        self.grid  = inflator.inflate(raw_grid)
        self.get_logger().info(
            f'Map inflated by {radius} cells '
            f'(robot radius ≈ {radius * resolution:.2f} m)'
        )

        # ── Planning helpers ───────────────────────────────────────
        self.resolution = resolution
        self.smoother   = PathSmoother()
        self.converter  = PathConverter(map_resolution=resolution)
        self.follower   = PathFollower(
            step_speed=speed,
            max_rotation_speed=max_yaw,
        )
        self.rdp_epsilon = epsilon

        # ── ROS2 Publishers ────────────────────────────────────────
        self.body_pose_pub = self.create_publisher(
            Twist, '/hexapod/body_pose', 10)
        self.path_viz_pub  = self.create_publisher(
            Path, '/planned_path', 10)

        # ── ROS2 Subscribers ───────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, 10)

        # ── Control loop: 20 Hz matches hexapod_controller ────────
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('HexapodPathPlannerNode ready — waiting for /goal_pose')

    # ------------------------------------------------------------------
    # Goal callback
    # ------------------------------------------------------------------

    def _on_goal(self, msg: PoseStamped) -> None:
        """Receive goal from RViz or command line, run A*."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # Convert world coords (metres) → grid (row, col)
        goal_grid = (
            int(gy / self.resolution),
            int(gx / self.resolution),
        )

        # Use hexapod's current simulated position as start
        start_grid = (
            int(self.follower.hexapod_y / 1000 / self.resolution),
            int(self.follower.hexapod_x / 1000 / self.resolution),
        )

        self.get_logger().info(
            f'Goal received: world({gx:.2f}, {gy:.2f}) → '
            f'grid{goal_grid}  from  {start_grid}'
        )
        self._plan(start_grid, goal_grid)

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _plan(self, start, goal) -> None:
        """Run A*, smooth, convert, and load into PathFollower."""
        planner = AStarPlanner(self.grid, start, goal)
        result  = planner.plan()

        if not result.found:
            self.get_logger().error(
                f'No path found from {start} to {goal}. '
                'Check that goal is reachable and not inside an obstacle.'
            )
            return

        # Smooth
        smooth_path = self.smoother.rdp_simplify(
            result.path, epsilon=self.rdp_epsilon)

        # Grid → world
        world_path = self.converter.grid_to_world(smooth_path)

        # Load into follower
        self.follower.load_path(world_path)

        # Publish path for RViz
        self._publish_path_viz(world_path)

        self.get_logger().info(
            f'Path planned: {len(result.path)} raw → '
            f'{len(smooth_path)} smoothed waypoints  '
            f'(cost={result.cost:.1f}, explored={result.explored_count})'
        )

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """20 Hz: compute and publish next body_pose command."""
        if not self.follower.waypoints:
            return

        cmd = self.follower.get_next_body_pose()

        twist             = Twist()
        twist.linear.x    = float(cmd['tx'])
        twist.linear.y    = float(cmd['ty'])
        twist.linear.z    = float(cmd['tz'])
        twist.angular.x   = float(cmd['roll'])
        twist.angular.y   = float(cmd['pitch'])
        twist.angular.z   = float(cmd['yaw'])

        self.body_pose_pub.publish(twist)

        idx, total = self.follower.progress()
        if idx % 10 == 0:
            self.get_logger().info(f'Waypoint {idx} / {total}')

        if self.follower.is_complete():
            self.get_logger().info('Path complete!')
            self.follower.waypoints = []

    # ------------------------------------------------------------------
    # RViz visualisation
    # ------------------------------------------------------------------

    def _publish_path_viz(self, world_path) -> None:
        """Publish nav_msgs/Path for display in RViz."""
        msg            = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp    = self.get_clock().now().to_msg()

        for x, y in world_path:
            pose                     = PoseStamped()
            pose.header.frame_id     = 'world'
            pose.pose.position.x     = float(x)
            pose.pose.position.y     = float(y)
            pose.pose.position.z     = 0.0
            msg.poses.append(pose)

        self.path_viz_pub.publish(msg)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = HexapodPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
