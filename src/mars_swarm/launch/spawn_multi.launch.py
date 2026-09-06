import os
import re
import tempfile
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

_GUI_CONFIG_HEADER = """<?xml version="1.0"?>
<dialog name="quick_start" show_again="true"/>
<window>
  <width>1000</width>
  <height>845</height>
  <menus>
    <drawer default="false">
    </drawer>
  </menus>
  <dialog_on_exit>true</dialog_on_exit>
</window>
"""


def _build_gui_config(world_sdf_path):
    """Extract the world's <gui>...</gui> plugin list and wrap it in a
    standalone gui-config file (dialog/window header + plugins, no <gui>
    wrapper), since that's the format gz-sim's --gui-config flag expects."""
    try:
        with open(world_sdf_path, 'r') as f:
            world_xml = f.read()
    except OSError:
        return None

    match = re.search(r'<gui[^>]*>(.*)</gui>', world_xml, re.DOTALL)
    if not match:
        return None

    gui_body = match.group(1)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_gui.config', delete=False)
    tmp.write(_GUI_CONFIG_HEADER)
    tmp.write(gui_body)
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    headless_str = context.perform_substitution(LaunchConfiguration('headless'))
    tb3_sim_share = get_package_share_directory('nav2_minimal_tb3_sim')

    # 1. Get path to the world SDF file (cafe: furnished, ~90% max coverage due
    # to physically unreachable cells behind furniture. warehouse: verified
    # obstacle-free in the 12x12m region used here, case study for ~100% coverage)
    world_name = context.perform_substitution(LaunchConfiguration('world'))
    mars_swarm_share = get_package_share_directory('mars_swarm')
    world_sdf_path = os.path.join(mars_swarm_share, 'worlds', f'{world_name}.sdf')
    
    # 2. Launch Gazebo Sim with the compiled world and deterministic seed
    seed_str = context.perform_substitution(LaunchConfiguration('seed'))
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
    
    gz_args = f'-r {world_sdf_path} --seed {seed_str}'
    if headless_str.lower() == 'true':
        gz_args += ' -s'
    else:
        # gz-sim's GUI ignores a world file's embedded <gui> block whenever
        # ~/.gz/sim/<ver>/gui.config (or the installed default gui.config) is
        # present -- it always wins over the world's <gui>, so our
        # camera_pose never took effect. Work around it by extracting the
        # world's <gui>...</gui> plugin list and building a standalone
        # gui-config file (dialog/window header + world plugins) passed
        # explicitly via --gui-config, which does take priority.
        gui_config_path = _build_gui_config(world_sdf_path)
        if gui_config_path:
            gz_args += f' --gui-config {gui_config_path}'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': gz_args}.items()
    )
    
    # 3. Dynamic bridge generator helper to namespace all frame_ids
    from launch_ros.actions import Node
    
    def make_robot_nodes(namespace, robot_name, x, y, z, yaw, robot_type='waffle'):
        config_content = f"""- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/{namespace}/joint_states"
  gz_topic_name: "/{namespace}/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/{namespace}/odom"
  gz_topic_name: "/{namespace}/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
  frame_id: "{namespace}/odom"
  child_frame_id: "{namespace}/base_footprint"

- ros_topic_name: "/{namespace}/tf"
  gz_topic_name: "/{namespace}/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/{namespace}/imu"
  gz_topic_name: "/{namespace}/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
  frame_id: "{namespace}/imu_link"

- ros_topic_name: "/{namespace}/scan"
  gz_topic_name: "/{namespace}/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
  frame_id: "{namespace}/base_scan"

- ros_topic_name: "/{namespace}/cmd_vel"
  gz_topic_name: "/{namespace}/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/{namespace}/camera/image_raw"
  gz_topic_name: "/{namespace}/camera"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
"""
        temp_file = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)
        temp_file.write(config_content.encode('utf-8'))
        temp_file.close()

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            parameters=[{
                'config_file': temp_file.name,
                'use_sim_time': True,
            }],
            output='screen'
        )

        # Visual-only 1.6x scale on top of the stock (reliable) Waffle robot:
        # only the <visual> mesh <scale> tags are multiplied -- mass, joints,
        # wheel_separation/radius, and collision primitives are all untouched,
        # so kinematics/physics behave identically to the stock robot that's
        # confirmed to drive correctly. Tradeoff: the visual mesh is now
        # slightly larger than its own collision box, so it may look like it
        # clips into walls/other robots slightly before collision registers.
        if robot_type.lower() in ('pioneer', 'pioneer2dx'):
            robot_sdf = os.path.join(mars_swarm_share, 'urdf', 'pioneer2dx.sdf.xacro')
        else:
            robot_sdf = os.path.join(mars_swarm_share, 'urdf', 'gz_waffle_visual_big.sdf.xacro')

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            namespace=namespace,
            arguments=[
                '-name', robot_name,
                '-string', Command([
                    FindExecutable(name='xacro'), ' ', 'namespace:=',
                    namespace, ' ', robot_sdf]),
                '-x', x, '-y', y, '-z', z, '-Y', yaw
            ]
        )
        return bridge_node, spawn_node

    # 4. Determine number of robots to spawn
    num_robots_str = context.perform_substitution(LaunchConfiguration('num_robots'))
    multi_str = context.perform_substitution(LaunchConfiguration('multi'))
    if num_robots_str and num_robots_str.isdigit():
        num_robots = max(1, min(8, int(num_robots_str)))
    else:
        num_robots = 3 if multi_str.lower() == 'true' else 1

    robot_types_str = context.perform_substitution(LaunchConfiguration('robot_types'))
    robot_types = [t.strip().lower() for t in robot_types_str.split(',') if t.strip()]
    if not robot_types:
        robot_types = ['waffle']

    enable_static_tf_str = context.perform_substitution(LaunchConfiguration('enable_static_tf'))
    enable_static_tf = enable_static_tf_str.lower() == 'true'

    # Read the URDF file directly (robot_state_publisher requires URDF XML string, not SDF xacro output)
    urdf_file_path = os.path.join(tb3_sim_share, 'urdf', 'turtlebot3_waffle.urdf')
    try:
        with open(urdf_file_path, 'r') as f:
            robot_desc = f.read()
    except Exception as e:
        print(f"[spawn_multi] Error reading URDF file: {e}")
        robot_desc = ''

    tf_relay_node = Node(
        package='mars_swarm',
        executable='tf_relay',
        name='tf_relay',
        output='screen'
    )

    nodes = [gazebo, tf_relay_node]

    # Pre-calculated collision-free horizontal line coordinates at y=0, yaw=-1.5708
    # Spaced by 0.70m (safe margin > 0.45m inter-agent CBF distance) across all 5 worlds
    ROBOT_X_OFFSETS = [0.0, -0.7, 0.7, -1.4, 1.4, -2.1, 2.1, -2.8]

    for k in range(num_robots):
        ns = f'tb{k + 1}'
        xw = ROBOT_X_OFFSETS[k]
        rob_type = robot_types[k % len(robot_types)]
        bridge_node, spawn_node = make_robot_nodes(ns, ns, f'{xw:.2f}', '0.00', '0.20', '-1.5708', robot_type=rob_type)
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
                'frame_prefix': f'{ns}/'
            }]
        )
        nodes.extend([bridge_node, spawn_node, rsp_node])

        # Publish static transform between tb1/odom and tbK/odom
        # In tb1/odom frame: local_x = -y_world = 0.0, local_y = x_world = xw
        if k > 0 and enable_static_tf:
            static_tf = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_tb1_{ns}',
                arguments=['--x', '0.0', '--y', f'{xw:.2f}', '--z', '0.0', '--yaw', '0.0', '--frame-id', 'tb1/odom', '--child-frame-id', f'{ns}/odom']
            )
            nodes.append(static_tf)

    return nodes

def generate_launch_description():
    tb3_sim_share = get_package_share_directory('nav2_minimal_tb3_sim')
    mars_swarm_share_env = get_package_share_directory('mars_swarm')

    # Set Gazebo environment variables for resources
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_sim_share, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(tb3_sim_share).parent.resolve()))
    # gz-sim resolves package://<pkg>/... via GZ_SIM_RESOURCE_PATH, not
    # AMENT_PREFIX_PATH -- without this, package://mars_swarm/meshes/...
    # (used by gz_waffle_big.sdf.xacro) fails to load and Gazebo falls back
    # to blank/invisible geometry for the robot visuals.
    set_env_vars_resources3 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(mars_swarm_share_env).parent.resolve()))

    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('headless', default_value='true', description='Run Gazebo headless (no GUI)'))
    ld.add_action(DeclareLaunchArgument('world', default_value='cafe', description="World SDF to load (without extension): 'cafe', 'warehouse', 'depot', 'office', or 'maze'"))
    ld.add_action(DeclareLaunchArgument('enable_static_tf', default_value='true', description='Whether to enable static TF between robot odom frames'))
    ld.add_action(DeclareLaunchArgument('multi', default_value='true', description='Whether to spawn 3 robots (true) or just 1 (false)'))
    ld.add_action(DeclareLaunchArgument('num_robots', default_value='', description='Number of robots to spawn (1 to 8, overrides multi)'))
    ld.add_action(DeclareLaunchArgument('robot_types', default_value='waffle', description='Comma-separated robot types for swarm (e.g. waffle, or waffle,pioneer2dx)'))
    ld.add_action(DeclareLaunchArgument('seed', default_value='42', description='Deterministic PRNG seed for Gazebo physics and sensor noise'))
    
    # Add environment variables
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(set_env_vars_resources3)
    
    # Add opaque function to compile xacro and include Gazebo/Spawning
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
