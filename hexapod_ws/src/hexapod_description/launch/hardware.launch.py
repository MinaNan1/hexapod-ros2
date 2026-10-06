"""
hardware.launch.py
==================
Single command that brings up everything needed to drive the real robot:

    robot_state_publisher   ← URDF + link transforms
    hexapod_controller      ← Cartesian gait + posture + emotes
    hiwonder_servo_bridge   ← drives the 18 Hiwonder HX-35HM bus servos

No RViz, no rosbridge, no Flask — this is the minimal hardware bring-up
when you just want to plug the Jetson into the servo board and walk.

Usage (inside the Jetson container, after `colcon build`):
    ros2 launch hexapod_description hardware.launch.py
    # specify a different hidraw device:
    ros2 launch hexapod_description hardware.launch.py hid_device:=/dev/hidraw2
    # let an external odometry node own odom→base_link (e.g. for SLAM):
    ros2 launch hexapod_description hardware.launch.py publish_tf:=false

Tip: run `ros2 run hexapod_control hiwonder_servo_bridge --list` first to
see which /dev/hidrawN is the Hiwonder controller.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_description = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'crab_model.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    hid_device = LaunchConfiguration('hid_device')
    publish_tf = LaunchConfiguration('publish_tf')

    return LaunchDescription([

        DeclareLaunchArgument(
            'hid_device', default_value='/dev/hiwonder',
            description='HID path for the Hiwonder bus servo controller. '
                        'Defaults to the /dev/hiwonder udev symlink (see '
                        'LAPTOP_COMMANDS.md for the udev rule). If you have '
                        'not installed the rule, override with e.g. '
                        '`hid_device:=/dev/hidraw5` — '
                        '`ros2 run hexapod_control hiwonder_servo_bridge '
                        '--list` shows which hidrawN is the Hiwonder.'),

        DeclareLaunchArgument(
            'publish_tf', default_value='true',
            description='If false, the controller will not broadcast '
                        'odom→base_link (leave it to an external odom node, '
                        'e.g. rf2o during SLAM mapping).'),

        # ── URDF / TF chain ─────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── Cartesian gait controller (posture + dances + reset-to-zero) ─
        Node(
            package='hexapod_control',
            executable='hexapod_controller',
            name='hexapod_controller',
            parameters=[{
                'publish_tf': ParameterValue(publish_tf, value_type=bool),
            }],
            output='screen',
        ),

        # ── Hiwonder bus-servo bridge ───────────────────────────────────
        Node(
            package='hexapod_control',
            executable='hiwonder_servo_bridge',
            name='hiwonder_servo_bridge',
            parameters=[{'hid_device': hid_device}],
            output='screen',
        ),
    ])
