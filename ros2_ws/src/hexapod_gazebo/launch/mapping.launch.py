"""
mapping.launch.py
=================
Build a live 2D occupancy-grid map of an unknown room while the hexapod
walks around — the LiDAR rides on the moving robot, so the map is built
with SLAM Toolbox, which scan-matches each scan against the accumulated
map and *corrects* odometry drift instead of blindly trusting the robot's
dead-reckoned pose. This is what keeps the map sharp on a legged robot
with no wheel encoders.

TF chain (one global frame: `map`)
----------------------------------
    map  --(slam_toolbox: scan-match correction)-->  odom
    odom --(odometry source, see below)-->           base_link
    base_link --(static mount, this file)-->         laser  -->  /scan

There is deliberately NO static `world→odom` here (unlike display.launch.py).
SLAM Toolbox owns `map→odom`; adding a second parent for `odom` would break
the TF tree.

Odometry source (`odom_source` argument)
----------------------------------------
    gait  (default) — the hexapod_controller's dead-reckoning provides the
                      odom→base_link prior; SLAM corrects its drift. The
                      mapper_params YAML is already tuned to trust the scan
                      matcher over this crude prior. Simplest, reuses the
                      proven walking stack.
    rf2o            — rf2o_laser_odometry (laser scan-matching odometry)
                      provides a much more accurate odom→base_link prior.
                      The gait controller then runs legs-only
                      (publish_tf:=false) so only ONE node owns odom→base_link.

Run
---
    # both workspaces sourced (hexapod_ws first, then ros2_ws)
    ros2 launch hexapod_gazebo mapping.launch.py
    # higher-accuracy odometry:
    ros2 launch hexapod_gazebo mapping.launch.py odom_source:=rf2o

Then drive the robot (web UI, or):
    ros2 topic pub --once /hexapod/cmd std_msgs/msg/String "data: 'walk'"

Save the finished map:
    ros2 run nav2_map_server map_saver_cli -f ~/hexapod_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


# ── Mount offset of the LiDAR relative to the hexapod body ──────────
# base_link is the body centre on the ground plane. Keep these in sync
# with navigation.launch.py / slam_exploration.launch.py.
LIDAR_MOUNT_X = '0.0'
LIDAR_MOUNT_Y = '0.0'
LIDAR_MOUNT_Z = '0.08'
LIDAR_MOUNT_YAW = '0.0'
LIDAR_MOUNT_PITCH = '0.0'
LIDAR_MOUNT_ROLL = '0.0'


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('hexapod_gazebo')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_description = get_package_share_directory('hexapod_description')

    slam_params_file = os.path.join(
        pkg_gazebo, 'config', 'mapper_params_online_async.yaml')

    # Robot description (URDF) for robot_state_publisher → link TFs in RViz
    xacro_file = os.path.join(pkg_description, 'urdf', 'crab_model.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── Launch arguments ────────────────────────────────────────────
    odom_source = LaunchConfiguration('odom_source')      # 'gait' | 'rf2o'
    use_rviz = LaunchConfiguration('use_rviz')
    lidar_port = LaunchConfiguration('lidar_port')

    use_gait_odom = PythonExpression(["'", odom_source, "' == 'gait'"])
    use_rf2o_odom = PythonExpression(["'", odom_source, "' == 'rf2o'"])

    return LaunchDescription([

        DeclareLaunchArgument(
            'odom_source', default_value='gait',
            description="odom->base_link source: 'gait' (controller dead-reckoning) "
                        "or 'rf2o' (laser odometry)."),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2 alongside the mapper.'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='Serial port of the LDS-01 LiDAR.'),

        # ── Robot description → link transforms ─────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── Gait controller (legs + odom→base_link prior) ───────────
        # In 'gait' mode it owns odom→base_link. In 'rf2o' mode it runs
        # legs-only (publish_tf:=false) so rf2o owns odom→base_link.
        Node(
            package='hexapod_control',
            executable='hexapod_controller',
            name='hexapod_controller',
            parameters=[{'publish_tf': ParameterValue(use_gait_odom, value_type=bool)}],
            output='screen',
        ),

        # ── LDS-01 LiDAR driver → /scan ─────────────────────────────
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': lidar_port, 'frame_id': 'laser'}],
            output='screen',
        ),

        # ── Static TF: base_link → laser (LiDAR mount) ──────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_laser_tf',
            arguments=[
                LIDAR_MOUNT_X, LIDAR_MOUNT_Y, LIDAR_MOUNT_Z,
                LIDAR_MOUNT_YAW, LIDAR_MOUNT_PITCH, LIDAR_MOUNT_ROLL,
                'base_link', 'laser',
            ],
            output='screen',
        ),

        # ── rf2o laser odometry (only in odom_source:=rf2o) ─────────
        # Owns odom→base_link via laser scan matching — more accurate than
        # the gait dead-reckoning prior. Provides SLAM a better starting guess.
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            condition=IfCondition(use_rf2o_odom),
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
            }],
            output='screen',
        ),

        # ── SLAM Toolbox (online async) → /map + map→odom ───────────
        # Scan-matches against the accumulated map to build the grid AND
        # correct odometry drift. The moving-LiDAR fix lives here.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': slam_params_file}.items(),
        ),

        # ── RViz2 ───────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            output='screen',
        ),

    ])
