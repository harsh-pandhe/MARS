import os
import tempfile
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    headless_str = context.perform_substitution(LaunchConfiguration('headless'))
    
    # 1. Get path to the cafe world SDF file
    mars_swarm_share = get_package_share_directory('mars_swarm')
    world_sdf_path = os.path.join(mars_swarm_share, 'worlds', 'cafe.sdf')
    
    # 2. Launch Gazebo Sim with the compiled world
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
    
    # Determine the gz arguments (headless option if true)
    gz_args = f'-r {world_sdf_path}'
    if headless_str.lower() == 'true':
        gz_args += ' -s'
        
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': gz_args}.items()
    )
    
    # 3. Spawn TurtleBot3
    spawn_tb3_launch = os.path.join(tb3_sim_share, 'launch', 'spawn_tb3.launch.py')
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_tb3_launch),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'namespace': LaunchConfiguration('namespace'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )
    
    return [gazebo, spawn_robot]

def generate_launch_description():
    tb3_sim_share = get_package_share_directory('nav2_minimal_tb3_sim')
    
    # Set Gazebo environment variables for resources
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_sim_share, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(tb3_sim_share).parent.resolve()))
    
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('headless', default_value='true', description='Run Gazebo headless (no GUI)'))
    ld.add_action(DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'))
    ld.add_action(DeclareLaunchArgument('robot_name', default_value='turtlebot3_waffle', description='Name of the robot'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-2.00', description='X position of the robot'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='-0.50', description='Y position of the robot'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.20', description='Z position of the robot'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.00', description='Yaw orientation of the robot'))
    
    # Add environment variables
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    
    # Add opaque function to compile xacro and include Gazebo/Spawning
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
