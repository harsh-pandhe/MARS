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
    echo "MARS Swarm Robotics Unified Runner"
    echo "Usage: ./run_swarm.sh [OPTION]"
    echo ""
    echo "Options:"
    echo "  --demo              Run random swarm rollout demo in Gazebo GUI"
    echo "  --train             Train MAPPO policy on Ray RLlib (headless)"
    echo "  --evaluate <path>   Evaluate a trained policy checkpoint (headless)"
    echo "  --play <path>       Evaluate a trained policy checkpoint in Gazebo GUI"
    echo "  --resilience <path> Evaluate trained policy in GUI and inject failure using robot_killer"
    echo "  --record <path>     Evaluate trained policy (headless) and record ROS bag of odom/scans"
    echo "  --benchmark [path]  Run quantitative benchmarking across baselines & stress tests, and generate plots"
    echo "  --slam              Run SLAM Toolbox mapping on tb1 (with Gazebo GUI & RViz)"
    echo "  --nav               Run Nav2 Stack on the saved sandbox map for all 3 robots (with Gazebo GUI & RViz)"
    echo "  --help              Show this help menu"
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
                'spawn_multi.launch.py', 'spawn_tb3.launch.py', 
                'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'rviz2',
                'tf_relay', 'robot_state_publisher', 'static_transform_publisher',
                'evaluate_benchmarks.py', 'train_multi.py',
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
    --demo)
        echo "Running multi-robot random demo with Gazebo GUI..."
        python3 src/mars_swarm/mars_swarm/train_multi.py --demo --gui
        ;;
    --slam)
        echo "Running SLAM Toolbox mapping on tb1 (with Gazebo GUI & RViz)..."
        echo "To teleoperate tb1, run in a separate terminal:"
        echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tb1/cmd_vel"
        echo ""
        echo "To save the map once you are done mapping, run:"
        echo "  ros2 run nav2_map_server map_saver_cli -f src/mars_swarm/maps/my_saved_map"
        echo ""
        
        # Build workspace first
        colcon build --symlink-install
        source install/setup.bash
        
        # Start Gazebo sim with GUI
        ros2 launch mars_swarm spawn_multi.launch.py headless:=false enable_static_tf:=true &
        SIM_PID=$!
        
        sleep 8
        
        # Start SLAM Toolbox async mapping on tb1
        ros2 launch mars_swarm slam.launch.py &
        SLAM_PID=$!
        
        # Start RViz
        ros2 run rviz2 rviz2 -d install/mars_swarm/share/mars_swarm/rviz/namespaced_swarm.rviz &
        RVIZ_PID=$!
        
        wait $SIM_PID $SLAM_PID $RVIZ_PID
        ;;
    --nav)
        echo "Running Nav2 Stack on sandbox map for all 3 robots (with Gazebo GUI & RViz)..."
        echo "Use the '2D Goal Pose' tool in RViz to set navigation goals for the robots!"
        echo "Note: You must choose the goal topic corresponding to the robot namespace."
        echo ""
        
        # Build workspace first
        colcon build --symlink-install
        source install/setup.bash
        
        # Start Gazebo sim with GUI, static odom TF disabled (AMCL will publish map->tbX/odom)
        ros2 launch mars_swarm spawn_multi.launch.py headless:=false enable_static_tf:=false &
        SIM_PID=$!
        
        sleep 8
        
        # Start namespaced Nav2 stacks
        ros2 launch mars_swarm navigation.launch.py &
        NAV_PID=$!
        
        # Start RViz
        ros2 run rviz2 rviz2 -d install/mars_swarm/share/mars_swarm/rviz/nav2.rviz &
        RVIZ_PID=$!
        
        wait $SIM_PID $NAV_PID $RVIZ_PID
        ;;
    --train)
        echo "Starting MAPPO training loop..."
        python3 src/mars_swarm/mars_swarm/train_multi.py --train
        ;;
    --evaluate)
        if [ -z "$2" ]; then
            echo "[ERROR] Please specify a checkpoint path. Example: ./run_swarm.sh --evaluate ./checkpoints/checkpoint_000002"
            exit 1
        fi
        echo "Evaluating policy checkpoint (headless)..."
        python3 src/mars_swarm/mars_swarm/train_multi.py --evaluate --checkpoint "$2"
        ;;
    --play)
        if [ -z "$2" ]; then
            echo "[ERROR] Please specify a checkpoint path. Example: ./run_swarm.sh --play ./checkpoints/checkpoint_000002"
            exit 1
        fi
        echo "Evaluating policy checkpoint in Gazebo GUI..."
        python3 src/mars_swarm/mars_swarm/train_multi.py --evaluate --checkpoint "$2" --gui
        ;;
    --resilience)
        if [ -z "$2" ]; then
            echo "[ERROR] Please specify a checkpoint path. Example: ./run_swarm.sh --resilience ./checkpoints/checkpoint_000002"
            exit 1
        fi
        echo "Running resilience test: Evaluating policy in GUI and launching robot killer..."
        
        # Start evaluation in the background
        python3 src/mars_swarm/mars_swarm/train_multi.py --evaluate --checkpoint "$2" --gui &
        EVAL_PID=$!
        
        # Wait for Gazebo to boot up
        echo "Waiting for robots to spawn before starting killer..."
        sleep 18
        
        # Start failure injection
        echo "Injecting random robot failure..."
        ros2 run mars_swarm robot_killer &
        KILLER_PID=$!
        
        # Wait for evaluation to complete
        wait $EVAL_PID
        ;;
    --record)
        CHECKPOINT=""
        if [ ! -z "$2" ] && [ "$2" != "heuristic" ] && [ "$2" != "random" ]; then
            CHECKPOINT="$2"
        fi
        
        BAG_NAME="swarm_record_$(date +%Y%m%d_%H%M%S)"
        echo "Running evaluation and recording ROS 2 bag to ${BAG_NAME}..."
        
        # Start ros2 bag record in background (including tf, tf_static, and /map)
        ros2 bag record -o "${BAG_NAME}" /tb1/odom /tb2/odom /tb3/odom /tb1/scan /tb2/scan /tb3/scan /tf /tf_static /map &
        BAG_PID=$!
        
        # Start evaluation
        if [ ! -z "${CHECKPOINT}" ]; then
            python3 src/mars_swarm/mars_swarm/train_multi.py --evaluate --checkpoint "${CHECKPOINT}"
        else
            # If no checkpoint, run the benchmark suite (default to Frontier Heuristic/Random)
            python3 src/mars_swarm/mars_swarm/evaluate_benchmarks.py
        fi
        
        # Stop recording
        kill -INT $BAG_PID
        echo "ROS 2 Bag saved to ${BAG_NAME}."
        ;;
    --benchmark)
        CHECKPOINT=""
        GUI_FLAG=""
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --gui)
                    GUI_FLAG="--gui"
                    ;;
                *)
                    CHECKPOINT="$1"
                    ;;
            esac
            shift
        done

        if [ -z "${CHECKPOINT}" ]; then
            echo "No policy checkpoint provided. Running control baseline benchmarks only (Random & Heuristic Frontier)..."
            python3 src/mars_swarm/mars_swarm/evaluate_benchmarks.py ${GUI_FLAG}
        else
            echo "Running full quantitative benchmarking suite including MAPPO policy from ${CHECKPOINT}..."
            python3 src/mars_swarm/mars_swarm/evaluate_benchmarks.py --checkpoint "${CHECKPOINT}" ${GUI_FLAG}
        fi
        ;;
    --help|*)
        show_help
        ;;
esac
