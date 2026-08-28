from setuptools import find_packages, setup

package_name = 'quadcopter_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/visual_odometry_bridge.launch.py',
        ]),
        ('share/' + package_name + '/params', [
            'params/localization.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remandey',
    maintainer_email='reman.airport@gmail.com',
    description='Visual-inertial odometry bridge and state estimation for GPS-denied UAV navigation',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'visual_odometry_bridge = quadcopter_localization.visual_odometry_bridge:main',
        ],
    },
)