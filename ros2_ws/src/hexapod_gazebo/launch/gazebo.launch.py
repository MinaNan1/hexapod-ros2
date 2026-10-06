import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():

    pkg_desc   = get_package_share_directory('hexapod_description')
    pkg_gazebo = get_package_share_directory('hexapod_gazebo')

    urdf_file  = os.path.join(pkg_desc, 'urdf', 'hexapod.urdf.xacro')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'site.world')
    ctrl_file  = os.path.join(pkg_desc, 'config', 'controllers.yaml')

    robot_desc = xacro.process_file(urdf_file).toxml()

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hexapod',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.0'
        ],
        output='screen'
    )

    # Load joint state broadcaster
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Load joint trajectory controller
    load_jtc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    # Load controllers after robot spawns
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_jsb, load_jtc]
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot,
        load_controllers,
        rviz,
    ])
