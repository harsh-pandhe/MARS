#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Base directories
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WORKSPACE_DIR}"

# Source ROS 2 and workspace setup files
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] ROS 2 Jazzy setup.bash not found. Please ensure ROS 2 is installed."
    exit 1
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "[WARNING] install/setup.bash not found. Rebuilding workspace first..."
    colcon build --symlink-install
    source install/setup.bash
fi

show_help() {
    echo "=========================================================="
    echo "    Coffee Shop Robot (mypkg) Single-Robot Runner"
    echo "=========================================================="
    echo "Usage: ./run_single.sh [OPTION]"
    echo ""
    echo "Options:"
    echo "  --slam        Run Gazebo Sim, Robot Spawner, SLAM Toolbox & RViz"
    echo "  --nav         Run Gazebo Sim, Robot Spawner, Map Server, AMCL, Nav2 & RViz"
    echo "  --help        Show this help menu"
    echo ""
}

cleanup() {
    echo "Cleaning up lingering ROS 2 and Gazebo processes..."
    python3 -c "
import os, signal
current_pid = os.getpid()
for name in os.listdir('/proc'):
    if name.isdigit():
        pid = int(name)
        if pid == current_pid:
            continue
        try:
            with open(os.path.join('/proc', name, 'cmdline'), 'rb') as f:
                cmdline = f.read().decode('utf-8', errors='ignore').replace('\x00', ' ')
            lower_cmd = cmdline.lower()
            target_terms = [
                'gz sim', 'parameter_bridge', 'ros_gz_bridge', 
                'gazebo.launch.py', 'slam.launch.py', 'nav2.launch.py',
                'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'rviz2',
                'robot_state_publisher', 'static_transform_publisher',
                'slam_toolbox', 'amcl', 'planner_server', 'controller_server',
                'behavior_server', 'bt_navigator', 'lifecycle_manager', 'map_server'
            ]
            if any(term in lower_cmd for term in target_terms):
                os.kill(pid, signal.SIGKILL)
        except Exception:
            continue
" || true
}
trap cleanup EXIT

case "$1" in
    --slam)
        echo "=========================================================="
        echo "Starting Single Robot SLAM Mapping Session..."
        echo "=========================================================="
        echo "To teleoperate the robot, open a separate terminal and run:"
        echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
        echo ""
        echo "To save the map once mapping is complete, run:"
        echo "  ros2 run nav2_map_server map_saver_cli -f src/mypkg/map/coffee_shop_map"
        echo "=========================================================="
        
        # Build to ensure symlinks and paths are up to date
        colcon build --symlink-install
        source install/setup.bash
        
        # Launch SLAM system
        ros2 launch mypkg slam.launch.py
        ;;
        
    --nav)
        echo "=========================================================="
        echo "Starting Single Robot Nav2 Navigation Session..."
        echo "=========================================================="
        echo "Use the '2D Goal Pose' button in RViz to set target coordinates."
        echo "=========================================================="
        
        # Build to ensure symlinks and paths are up to date
        colcon build --symlink-install
        source install/setup.bash
        
        # Launch Nav2 system
        ros2 launch mypkg nav2.launch.py
        ;;
        
    --help|*)
        show_help
        ;;
esac
