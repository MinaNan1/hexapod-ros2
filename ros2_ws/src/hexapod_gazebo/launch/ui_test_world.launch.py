import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Load the robot's 3D model
    pkg_description = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'crab_model.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        # 1. rosbridge websocket server for the Web UI
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),

        # 2. UI Teleop Mux (Multiplexer and Deadman switch)
        Node(
            package='hexapod_control',
            executable='ui_teleop',
            name='ui_teleop_mux',
            output='screen'
        ),

        # 3. Hexapod Controller (Runs locally without actual hardware for testing)
        # Note: Requires sourcing hexapod_ws before running this launch file
        Node(
            package='hexapod_control',
            executable='hexapod_controller',
            name='hexapod_controller',
            output='screen'
        ),

        # 4. Static TF: world → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
            output='screen'
        ),

        # 5. Robot State Publisher (Loads the URDF so RViz can see the 3D model)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 6. RViz2 (The 3D window to see the robot moving)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

    ])
