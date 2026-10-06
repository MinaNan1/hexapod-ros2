import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Keep these in sync with navigation.launch.py.
LIDAR_MOUNT_X = '0.0'
LIDAR_MOUNT_Y = '0.0'
LIDAR_MOUNT_Z = '0.08'
LIDAR_MOUNT_YAW   = '0.0'
LIDAR_MOUNT_PITCH = '0.0'
LIDAR_MOUNT_ROLL  = '0.0'


def generate_launch_description():
    pkg_hexapod_gazebo = get_package_share_directory('hexapod_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params_file = os.path.join(pkg_hexapod_gazebo, 'config', 'mapper_params_online_async.yaml')

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

        # ── 2. Static TF: base_link → laser (mount on hexapod body) ─
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

        # ── 2.5 Scan-matching odometry (odom → base_link) ────────────
        # Matches consecutive laser scans via SVD to track real movement.
        # Gives SLAM Toolbox a proper odometry prior instead of a static
        # identity that says the robot never moves.
        Node(
            package='hexapod_nav',
            executable='scan_odom',
            name='scan_odom_node',
            output='screen'
        ),

        # ── 3. QoS relay for scan ──────────────────────────────────
        Node(
            package='topic_tools',
            executable='relay',
            name='scan_relay',
            arguments=['/scan', '/scan_reliable'],
            parameters=[{'qos_reliability': 'reliable'}],
            output='screen'
        ),

        # ── 4. SLAM Toolbox (replaces lidar_mapper AND world->odom TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'slam_params_file': slam_params_file}.items()
        ),

        # ── 5. LiDAR Explorer + Planner ──────────────────────────
        Node(
            package='hexapod_nav',
            executable='lidar_explorer',
            name='lidar_explorer_planner',
            parameters=[{
                'robot_radius_cells':    2,
                'rdp_epsilon':           2.0,
                'waypoint_spacing':      5.0,
                'step_speed_mm':        10.0,
                'max_yaw_rad':           0.3,
                'sensor_range_cells':   15,
                'max_exploration_steps': 100,
                'auto_explore':          False,
                'explore_delay_sec':     3.0,
            }],
            output='screen'
        ),

    ])
