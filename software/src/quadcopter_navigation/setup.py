from setuptools import find_packages, setup
from glob import glob

package_name = 'quadcopter_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remandey',
    maintainer_email='reman.airport@gmail.com',
    description='Nav2 navigation stack configuration and bringup launch for the quadcopter GPS-denied VTOL platform.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_tf_publisher = quadcopter_navigation.odom_tf_publisher:main',
        ],
    },
)
