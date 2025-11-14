from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holoocean_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='HoloOcean utilities for the CougUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_converter = holoocean_bridge.depth_converter_node:main',
            'gps_converter = holoocean_bridge.gps_converter_node:main',
            'cmd_vel_converter = holoocean_bridge.cmd_vel_converter_node:main',
            'dvl_converter = holoocean_bridge.dvl_converter_node:main',
            'heading_converter = holoocean_bridge.heading_converter_node:main',
            'fin_state_publisher = holoocean_bridge.fin_state_publisher_node:main',
            'truth_converter = holoocean_bridge.truth_converter_node:main',
            'hsd_commander = holoocean_bridge.hsd_commander_node:main',
            'imu_converter = holoocean_bridge.imu_converter_node:main',
        ],
    },
)
