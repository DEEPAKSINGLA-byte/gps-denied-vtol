from setuptools import find_packages, setup

package_name = 'gps_denied_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remandey',
    maintainer_email='reman.airport@gmail.com',
    description='ROS 2 bridge for GPS-denied UAV navigation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mavlink_to_ros2_bridge = gps_denied_bridge.mavlink_to_ros2_bridge:main',
            'mavlink_cmd_bridge = gps_denied_bridge.mavlink_cmd_bridge:main',
            'lidar_udp_receiver = gps_denied_bridge.lidar_udp_receiver:main',
            'camera_tcp_receiver = gps_denied_bridge.camera_tcp_receiver:main',
            'metrics_monitor = gps_denied_bridge.metrics_monitor:main',
            'scan_to_3d = gps_denied_bridge.scan_to_3d:main',
        ],
    },
)
