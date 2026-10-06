import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    """Headless robot bring-up for the Jetson.
    Same as display.launch.py but without RViz — RViz runs on the laptop."""
    pkg_description = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'crab_model.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
        ),
        # IMU mounted on the body centre (~8 cm above base_link). Tune if
        # your physical mount differs; only matters for an RViz Imu display.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='hexapod_control',
            executable='hexapod_controller',
            output='screen'
        ),
        # IMU driver for self-balancing (MPU-9250 on I2C bus 1 @ 0x68).
        # If the IMU isn't wired, this node idles harmlessly and the
        # controller just behaves as if balancing were off.
        Node(
            package='hexapod_control',
            executable='imu_node',
            name='imu_node',
            parameters=[{
                'i2c_bus': 1,
                'i2c_addr': 0x68,
                'rate_hz': 50.0,
            }],
            output='screen'
        ),
    ])
