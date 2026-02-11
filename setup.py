from setuptools import setup
import os
from glob import glob

package_name = 'gulse_scan_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'pillow'],
    zip_safe=True,
    maintainer='gulsekykc',
    description='ROS 2 Webots SLAM driver for autonomous robot navigation with LiDAR, GPS, and IMU integration',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_bridge = gulse_scan_driver.gps_bridge:main',
        ],
    },
)
