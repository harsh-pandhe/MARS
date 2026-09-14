#!/usr/bin/env bash
set -e

CONFIG_NAME="$1"
CHECKPOINT_PATH="$2"
shift 2
SEEDS=("$@")

if [ -z "$CONFIG_NAME" ] || [ -z "$CHECKPOINT_PATH" ] || [ ${#SEEDS[@]} -eq 0 ]; then
  echo "Usage: $0 <config_name> <checkpoint_path> <seed1> [seed2...]"
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source /home/harsh-pandhe/GitHub/MARS/install/setup.bash
cd /home/harsh-pandhe/GitHub/MARS

mkdir -p checkpoints/ablation_results

for seed in "${SEEDS[@]}"; do
  echo "=== Running evaluation: $CONFIG_NAME, seed $seed ==="
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "train_multi" 2>/dev/null || true
  pkill -f "ros_gz|robot_state_publisher|parameter_bridge" 2>/dev/null || true
  sleep 2
  
  python3 src/mars_swarm/mars_swarm/benchmark_mappo_multiworld.py \
    --single-run \
    --single-world cafe \
    --single-scenario mappo_nominal \
    --checkpoint "$CHECKPOINT_PATH" \
    --seed "$seed" \
    --export-single "./checkpoints/ablation_results/${CONFIG_NAME}_eval_seed${seed}.json"
    
  echo "=== Completed: $CONFIG_NAME, seed $seed ==="
  cat "./checkpoints/ablation_results/${CONFIG_NAME}_eval_seed${seed}.json"
  echo ""
done

pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros_gz|robot_state_publisher|parameter_bridge" 2>/dev/null || true
echo "All evaluations finished for $CONFIG_NAME."
