from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mars_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'models', 'pioneer2dx'), glob(os.path.join('models', 'pioneer2dx', '*.*'))),
        (os.path.join('share', package_name, 'models', 'pioneer2dx', 'meshes'), glob(os.path.join('models', 'pioneer2dx', 'meshes', '*.*'))),
        (os.path.join('share', package_name, 'meshes', 'turtlebot3_burger'), glob(os.path.join('meshes', 'turtlebot3_burger', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harsh-pandhe',
    maintainer_email='harsh-pandhe@example.com',
    description='MARL Swarm Robotics in ROS 2 and Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_killer = mars_swarm.robot_killer:main',
            'tf_relay = mars_swarm.tf_relay:main',
            'evaluate_benchmarks = mars_swarm.evaluate_benchmarks:main',
            'sweep_robot_count = mars_swarm.sweep_robot_count:main',
            'semantic_vision = mars_swarm.semantic_vision:main',
            'mars_mcp_server = mars_swarm.mars_mcp_server:main',
        ],
    },
)
