from setuptools import setup

PKG = 'hexapod_nav'

setup(
    name=PKG,
    version='0.1.0',
    packages=[PKG],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG]),
        ('share/' + PKG, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gait_controller     = hexapod_nav.gait_controller:main',
            'balance_controller  = hexapod_nav.balance_controller:main',
            'lidar_mapper        = hexapod_nav.lidar_mapper:main',
            'lidar_path_planner  = hexapod_nav.lidar_path_planner:main',
            'lidar_explorer      = hexapod_nav.lidar_explorer_planner:main',
            'room_corners        = hexapod_nav.room_corners:main',
            'known_map_demo      = hexapod_nav.known_map_demo:main',
            'ui_teleop           = hexapod_nav.ui_teleop:main',
            'joy_teleop          = hexapod_nav.joy_teleop:main',
            'scan_odom           = hexapod_nav.scan_odom_node:main',
        ],
    },
)
