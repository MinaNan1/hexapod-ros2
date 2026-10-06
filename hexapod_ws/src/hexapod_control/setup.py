from setuptools import find_packages, setup

package_name = 'hexapod_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mina Maher',
    maintainer_email='minamaher9024@gmail.com',
    description='Hexapod walking controller with body pose overlay',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'hexapod_controller     = hexapod_control.hexapod_controller:main',
            'hexapod_teleop         = hexapod_control.hexapod_teleop:main',
            'hiwonder_servo_bridge  = hexapod_control.hiwonder_servo_bridge:main',
            'imu_node               = hexapod_control.imu_node:main',
        ],
    },
)