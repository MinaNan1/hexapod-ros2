import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_description = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'crab_model.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),

        # 2. Static TF: world → odom (odom is the fixed ground frame)
        #    The controller broadcasts odom → base_link dynamically.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
        ),

        # 3. Hexapod walking controller + odometry TF
        Node(
            package='hexapod_control',
            executable='hexapod_controller',
            output='screen'
        ),

        # 4. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])