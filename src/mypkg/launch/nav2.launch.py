import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('mypkg')
    nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_path, 'config', 'nav2.rviz')

    # Look for the map in the package source folder first so we don't need a rebuild after saving
    ws_map = os.path.expanduser('~/GitHub/MARS/src/mypkg/map/coffee_shop_map.yaml')
    if os.path.exists(ws_map):
        map_yaml = ws_map
    else:
        map_yaml = os.path.join(pkg_path, 'map', 'coffee_shop_map.yaml')

    if not os.path.exists(map_yaml):
        raise FileNotFoundError(
            f"\n\n[nav2.launch] Map not found at:\n  {map_yaml}\n\n"
            "Run SLAM first:\n"
            "  ros2 launch mypkg slam.launch.py\n"
            "Then save:\n"
            "  ros2 run nav2_map_server map_saver_cli -f ~/GitHub/MARS/src/mypkg/map/coffee_shop_map\n"
        )

    # ── 1. Include Gazebo Launch (starts Gazebo Sim, Robot State Publisher, Entity Spawner, and Parameter Bridge)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
        )
    )

    # ── 2. Map Server
    map_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': map_yaml
                }]
            )
        ]
    )

    # ── 3. AMCL
    amcl = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    # ── 4. Lifecycle Manager: localization
    lifecycle_localization = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']
                }]
            )
        ]
    )

    # ── 5. Controller Server (Regulated Pure Pursuit)
    controller_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            )
        ]
    )

    # ── 6. Smoother Server
    smoother_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    # ── 7. Planner Server (NavFn)
    planner_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    # ── 8. Behavior Server
    behavior_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    # ── 9. BT Navigator
    bt_navigator = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    # ── 10. Waypoint Follower
    waypoint_follower = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    # ── 11. Velocity Smoother
    velocity_smoother = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')
                ]
            )
        ]
    )

    # ── 12. Lifecycle Manager: navigation
    lifecycle_navigation = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ]
                }]
            )
        ]
    )

    # ── 13. RViz2
    rviz = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'GALLIUM_DRIVER':        'llvmpipe'
                }
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        map_server,
        amcl,
        lifecycle_localization,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_navigation,
        rviz,
    ])
