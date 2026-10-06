"""
navigation.launch.py
====================
Master launch file for LiDAR-based hexapod navigation.

TF chain
--------
    world  --static (this file)-->  odom
    odom   --dynamic (scan_odom_node, scan-matching)-->  base_link
    base_link  --static (this file)-->  laser

    The odom → base_link transform is produced by scan_odom_node, which
    matches consecutive laser scans using SVD to estimate how the robot
    moved.  This tracks actual LiDAR-observed movement (walls getting
    closer/further) rather than dead-reckoning from leg kinematics.

    The LiDAR rides on the hexapod, so `world ← laser` is dynamic.
    lidar_mapper looks that transform up per scan and raycasts from the
    laser's real world position.

Optional (only needed if you want the robot to walk)
----------------------------------------------------
    source ~/hexapod-ros2/hexapod_ws/install/setup.bash
    ros2 run hexapod_control hexapod_controller

    The gait controller animates the legs.  It is NOT needed for TF —
    scan_odom_node handles that via LiDAR scan matching.

Launches:
    1. LDS-01 LiDAR driver          → /scan
    2. Static TF: world → odom       (identity — robot starts at origin)
    3. Static TF: base_link → laser  (LiDAR mounted on top of hexapod)
    4. QoS scan relay
    5. scan_odom_node                → odom → base_link TF (scan matching)
    6. lidar_mapper                  → /map
    7. lidar_path_planner            → /hexapod/body_pose, /planned_path

Usage:
    ros2 launch hexapod_gazebo navigation.launch.py

Then in another terminal:
    rviz2
    - Add display: Map (topic /map,  fixed frame: world)
    - Add display: Path (topic /planned_path)
    - Add display: LaserScan (topic /scan)
    - Use "2D Goal Pose" button to set navigation goals
"""

from launch import LaunchDescription
from launch_ros.actions import Node


# ── Mount offset of the LiDAR relative to the hexapod body ──────────
# base_link is the hexapod body centre on the ground plane.
# Reasonable default: centred, 8 cm above the body, facing forward.
# Tune these if your physical mount is different.
LIDAR_MOUNT_X = '0.0'
LIDAR_MOUNT_Y = '0.0'
LIDAR_MOUNT_Z = '0.08'
LIDAR_MOUNT_YAW   = '0.0'
LIDAR_MOUNT_PITCH = '0.0'
LIDAR_MOUNT_ROLL  = '0.0'


def generate_launch_description():
    return LaunchDescription([

        # ── 1. LDS-01 LiDAR driver ─────────────────────────────────
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser'
            }],
            output='screen'
        ),

        # ── 2. Static TF: world → odom (identity, robot starts here) ─
        # hexapod_controller publishes the dynamic odom → base_link leg.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
            output='screen'
        ),

        # ── 3. Static TF: base_link → laser (mount on hexapod body) ─
        # The LiDAR moves with the robot — this transform stays constant
        # in the robot frame but moves in world frame as the robot walks.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_laser_tf',
            arguments=[
                LIDAR_MOUNT_X, LIDAR_MOUNT_Y, LIDAR_MOUNT_Z,
                LIDAR_MOUNT_YAW, LIDAR_MOUNT_PITCH, LIDAR_MOUNT_ROLL,
                'base_link', 'laser',
            ],
            output='screen'
        ),

        # ── 4. QoS relay for scan ──────────────────────────────────
        Node(
            package='topic_tools',
            executable='relay',
            name='scan_relay',
            arguments=['/scan', '/scan_reliable'],
            parameters=[{'qos_reliability': 'reliable'}],
            output='screen'
        ),

        # ── 5. Scan-matching odometry (odom → base_link) ──────────
        # Replaces hexapod_controller as the TF source.  Matches
        # consecutive laser scans via SVD to track real movement.
        Node(
            package='hexapod_nav',
            executable='scan_odom',
            name='scan_odom_node',
            output='screen'
        ),

        # ── 6. LiDAR Mapper ───────────────────────────────────────
        Node(
            package='hexapod_nav',
            executable='lidar_mapper',
            name='lidar_mapper',
            parameters=[{
                'grid_width':       200,
                'grid_height':      200,
                'resolution':       0.05,
                'map_publish_rate': 2.0,
            }],
            output='screen'
        ),

        # ── 7. LiDAR Path Planner ─────────────────────────────────
        Node(
            package='hexapod_nav',
            executable='lidar_path_planner',
            name='lidar_path_planner',
            parameters=[{
                'robot_radius_cells':  2,
                'rdp_epsilon':         2.0,
                'step_speed_mm':       10.0,
                'max_yaw_rad':         0.3,
            }],
            output='screen'
        ),

    ])
