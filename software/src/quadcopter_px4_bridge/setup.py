from setuptools import find_packages, setup

package_name = 'quadcopter_px4_bridge'

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
    description='Nav2 Twist to PX4 offboard bridge for the quadcopter GPS-denied VTOL platform.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'quadcopter_px4_bridge = quadcopter_px4_bridge.quadcopter_px4_bridge:main'
        ],
    },
)
