#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Base directories
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WORKSPACE_DIR}"

# Disable TorchDynamo / Torch Compile to prevent segfault in torch.optim.Adam under Ray/RLlib
export TORCHDYNAMO_DISABLE=1
export TORCH_COMPILE_DISABLE=1

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
    echo "  --coverage-demo [N] [--world W] [--robots N] Run Frontier Heuristic (primary exploration engine)"
    echo "  --sweep-robots [--world W|all] [--steps S]   Run multi-world robot count scalability sweep (2, 3, 5, 8 robots)"
    echo "  --dynamic-test [--scenario S] [--world W]    Verify CBF safety against moving dynamic hazards in Gazebo"
    echo "  --render-heatmap [file] [--world W] [--out]  Render publication-grade coverage heatmap PNG from run data"
    echo "  --benchmark [path]  Run quantitative benchmarking across baselines & stress tests"
    echo "  --resilience <path> Evaluate swarm in GUI and inject failure using robot_killer"
    echo "  --demo              Run random swarm rollout demo in Gazebo GUI"
    echo "  --train             [Deprecated Baseline] Train MAPPO policy on Ray RLlib (negative baseline)"
    echo "  --evaluate <path>   [Deprecated Baseline] Evaluate trained MAPPO checkpoint (headless)"
    echo "  --play <path>       [Deprecated Baseline] Evaluate trained MAPPO checkpoint in Gazebo GUI"
    echo "  --record <path>     Evaluate policy (headless) and record ROS 2 bag of odom/scans"
    echo "  --slam              Run SLAM Toolbox mapping on tb1 (with Gazebo GUI & RViz)"
    echo "  --nav               Run Nav2 Stack on the saved sandbox map for all 3 robots (with Gazebo GUI & RViz)"
    echo "  --test              Run full automated unit and regression test suite"
    echo "  --mcp               Launch FastMCP server for AI agent introspection and control"
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
        shift
        echo "Starting MAPPO training loop..."
        python3 src/mars_swarm/mars_swarm/train_multi.py --train "$@"
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
        
        # Run evaluation headless
        echo "Running evaluation episode for recording..."
        python3 src/mars_swarm/mars_swarm/evaluate.py --checkpoint "${CHECKPOINT}" --episodes 1 --headless
        
        # Wait for file sync
        if ps -p $BAG_PID > /dev/null; then
           sleep 2
        fi
        
        # Stop recording
        kill -INT $BAG_PID
        echo "ROS 2 Bag saved to ${BAG_NAME}."
        ;;
    --benchmark)
        CHECKPOINT=""
        GUI_FLAG=""
        WORLD_ARG="cafe"
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --gui)
                    GUI_FLAG="--gui"
                    ;;
                --world)
                    shift
                    WORLD_ARG="$1"
                    ;;
                *)
                    CHECKPOINT="$1"
                    ;;
            esac
            shift
        done

        if [ -z "${CHECKPOINT}" ]; then
            echo "No policy checkpoint provided. Running control baseline benchmarks only (Random & Heuristic Frontier) for world=${WORLD_ARG}..."
            python3 src/mars_swarm/mars_swarm/evaluate_benchmarks.py --world "${WORLD_ARG}" ${GUI_FLAG}
        else
            echo "Running full quantitative benchmarking suite including MAPPO policy from ${CHECKPOINT} for world=${WORLD_ARG}..."
            python3 src/mars_swarm/mars_swarm/evaluate_benchmarks.py --checkpoint "${CHECKPOINT}" --world "${WORLD_ARG}" ${GUI_FLAG}
        fi
        ;;
    --coverage-demo)
        MAX_STEPS="1200"
        HEADLESS_FLAG=""
        WORLD="cafe"
        NUM_ROBOTS=""
        HEATMAP_ARG=""
        ROBOT_TYPES_ARG=""
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --headless)
                    HEADLESS_FLAG="--headless"
                    ;;
                --world)
                    shift
                    WORLD="$1"
                    ;;
                --robots)
                    shift
                    NUM_ROBOTS="--num-robots $1"
                    ;;
                --types|--robot-types)
                    shift
                    ROBOT_TYPES_ARG="--robot-types $1"
                    ;;
                --heatmap)
                    shift
                    HEATMAP_ARG="--export-heatmap $1"
                    ;;
                *)
                    MAX_STEPS="$1"
                    ;;
            esac
            shift
        done
        echo "Running Frontier Heuristic coverage demo (world=${WORLD}, Gazebo GUI + RViz)..."
        python3 src/mars_swarm/mars_swarm/evaluate_benchmarks.py --coverage-demo --max-steps "${MAX_STEPS}" --world "${WORLD}" ${HEADLESS_FLAG} ${NUM_ROBOTS} ${HEATMAP_ARG} ${ROBOT_TYPES_ARG}
        ;;
    --sweep-robots)
        WORLD="depot"
        MAX_STEPS="300"
        COUNTS=""
        GUI_FLAG=""
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --world)
                    shift
                    WORLD="$1"
                    ;;
                --steps)
                    shift
                    MAX_STEPS="$1"
                    ;;
                --gui)
                    GUI_FLAG="--gui"
                    ;;
                --counts)
                    shift
                    COUNTS="$1"
                    ;;
            esac
            shift
        done
        EXTRA_ARGS=""
        if [ ! -z "${COUNTS}" ]; then
            EXTRA_ARGS="--robot-counts ${COUNTS}"
        fi
        echo "Running robot count scalability sweep (world=${WORLD}, steps=${MAX_STEPS})..."
        python3 src/mars_swarm/mars_swarm/sweep_robot_count.py --world "${WORLD}" --max-steps "${MAX_STEPS}" ${GUI_FLAG} ${EXTRA_ARGS}
        ;;
    --dynamic-test)
        SCENARIO="head_on"
        WORLD="cafe"
        GUI_FLAG=""
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --gui)
                    GUI_FLAG="--gui"
                    ;;
                --world)
                    shift
                    WORLD="$1"
                    ;;
                --scenario)
                    shift
                    SCENARIO="$1"
                    ;;
            esac
            shift
        done
        echo "Running Dynamic Obstacle CBF test (scenario=${SCENARIO}, world=${WORLD})..."
        python3 src/mars_swarm/mars_swarm/dynamic_obstacle_test.py --scenario "${SCENARIO}" --world "${WORLD}" ${GUI_FLAG}
        ;;
    --render-heatmap)
        INPUT_FILE=""
        OUTPUT_FILE=""
        WORLD="depot"
        EXTRA_FLAGS=""
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --input|-i)
                    shift
                    INPUT_FILE="$1"
                    ;;
                --output|-o|--out)
                    shift
                    OUTPUT_FILE="$1"
                    ;;
                --world|-w)
                    shift
                    WORLD="$1"
                    ;;
                --density)
                    EXTRA_FLAGS="${EXTRA_FLAGS} --density"
                    ;;
                --demo)
                    EXTRA_FLAGS="${EXTRA_FLAGS} --demo"
                    ;;
                *)
                    if [ -f "$1" ]; then
                        INPUT_FILE="$1"
                    else
                        WORLD="$1"
                    fi
                    ;;
            esac
            shift
        done
        CMD_ARGS=""
        if [ ! -z "${INPUT_FILE}" ]; then
            CMD_ARGS="${CMD_ARGS} --input ${INPUT_FILE}"
        fi
        if [ ! -z "${OUTPUT_FILE}" ]; then
            CMD_ARGS="${CMD_ARGS} --output ${OUTPUT_FILE}"
        fi
        CMD_ARGS="${CMD_ARGS} --world ${WORLD} ${EXTRA_FLAGS}"
        echo "Rendering coverage heatmap (world=${WORLD})..."
        python3 src/mars_swarm/mars_swarm/coverage_heatmap_renderer.py ${CMD_ARGS}
        ;;
    --test)
        echo "Running full automated test suite with pytest..."
        PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/ -v
        ;;
    --mcp)
        echo "Starting MARS FastMCP Server for ROS 2 / Gazebo..."
        python3 src/mars_swarm/mars_swarm/mars_mcp_server.py
        ;;
    --help|*)
        show_help
        ;;
esac
