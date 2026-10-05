from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mypkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.world')
        ),

        # URDF
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')
        ),

        # Config (slam_params, nav2_params, rviz configs)
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')
        ),

        # Worlds
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),

        # Map (saved after SLAM; empty on first build, filled after mapping)
        (
            os.path.join('share', package_name, 'map'),
            glob('map/*.yaml') + glob('map/*.pgm')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Pandhe',
    maintainer_email='harshpandhehome@gmail.com',
    description='MARS support package for SLAM and autonomous navigation',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
