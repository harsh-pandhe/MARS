import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mars_swarm')
    
    # Default paths
    default_map_file = os.path.join(pkg_share, 'maps', 'sandbox_map.yaml')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Declare Launch Arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the nav2 stack'
    )

    # 1. Global Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ]
    )
    
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server']}
        ]
    )
    
    # Helper function to create Nav2 nodes for a specific robot namespace
    def create_nav2_robot(namespace):
        # The fully qualified names of the nodes for lifecycle manager
        lifecycle_nodes = [
            f'{namespace}/amcl',
            f'{namespace}/planner_server',
            f'{namespace}/controller_server',
            f'{namespace}/behavior_server',
            f'{namespace}/bt_navigator'
        ]
        
        # AMCL node
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        )
        
        # Planner Server (Global Planner)
        planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        )
        
        # Controller Server (Local Planner)
        controller_node = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[('cmd_vel', 'cmd_vel')]
        )
        
        # Behavior Server (Recovery Behaviors)
        behavior_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        )
        
        # BT Navigator
        bt_navigator_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        )
        
        # Namespaced Lifecycle Manager
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            namespace=namespace,
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes}
            ]
        )
        
        return [
            amcl_node,
            planner_node,
            controller_node,
            behavior_node,
            bt_navigator_node,
            lifecycle_manager_node
        ]
        
    # Generate nodes for all three robots
    tb1_nodes = create_nav2_robot('tb1')
    tb2_nodes = create_nav2_robot('tb2')
    tb3_nodes = create_nav2_robot('tb3')
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add Launch Arguments
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add Global Map Nodes
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_manager)
    
    # Add Robot Nodes
    for node in tb1_nodes:
        ld.add_action(node)
    for node in tb2_nodes:
        ld.add_action(node)
    for node in tb3_nodes:
        ld.add_action(node)
        
    return ld
