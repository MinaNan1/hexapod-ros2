from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Set port permissions
        ExecuteProcess(
            cmd=['sudo', 'chmod', 'a+rw', '/dev/ttyUSB0'],
            output='screen'
        ),

        # LDS-01 LIDAR driver
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

        # QoS relay
        Node(
            package='topic_tools',
            executable='relay',
            name='scan_relay',
            arguments=['/scan', '/scan_reliable'],
            parameters=[{'qos_reliability': 'reliable'}],
            output='screen'
        ),

        # Static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'laser'],
            output='screen'
        ),

    ])
