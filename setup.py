from setuptools import setup
import os
from glob import glob

package_name = 'gulse_scan_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')),
        # SLAM ayar dosyalarını (yaml) sisteme tanıtan kritik satır:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gulsekykc',
    description='Gulse ROS 2 Webots SLAM Driver',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_bridge = gulse_scan_driver.gps_bridge:main',
            'navigator = gulse_scan_driver.navigator:main',
        ],
    },
)
