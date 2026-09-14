#!/usr/bin/env bash
set -e

export MARS_REWARD_COLLISION_PENALTY=8.0
export MARS_REWARD_DISCOVERY_BONUS=16.0
export TORCHDYNAMO_DISABLE=1

source /opt/ros/jazzy/setup.bash
source /home/harsh-pandhe/GitHub/MARS/install/setup.bash
cd /home/harsh-pandhe/GitHub/MARS

mkdir -p checkpoints/ablation_highdiscovery checkpoints/ablation_results

RESTORE_FLAG=""
if [ -f "./checkpoints/ablation_highdiscovery/rllib_checkpoint.json" ]; then
    RESTORE_FLAG="--restore ./checkpoints/ablation_highdiscovery"
    echo "Found checkpoint, resuming from checkpoint..."
fi

echo "Starting HighDiscovery ablation training (collision=8.0, discovery=16.0, 15 iterations)..."
python3 src/mars_swarm/mars_swarm/train_multi.py --train --iterations 15 --checkpoint-dir ./checkpoints/ablation_highdiscovery $RESTORE_FLAG
echo "Training finished cleanly."
