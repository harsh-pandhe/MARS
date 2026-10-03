#!/usr/bin/env bash
# Controlled comparison for Paper 2: the original RLlib-default configuration
# vs the corrected configuration, 15 iterations each, REPS independent training
# runs per arm, interleaved so any prefix of the results is balanced.
# Per-iteration metrics (entropy, returns, losses) go to
# checkpoints/paper2_ab/<arm>_rep<k>.jsonl

REPS="${1:-3}"
ITERS=15
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TORCHDYNAMO_DISABLE=1
OUT=checkpoints/paper2_ab
mkdir -p "$OUT"

cleanup() {
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "ros_gz|robot_state_publisher|parameter_bridge|tf_relay|static_transform" 2>/dev/null
  pkill -9 -f train_multi 2>/dev/null
  rm -rf /tmp/ray/session_* 2>/dev/null
  sleep 3
}

run_arm() {  # arm rep
  local arm="$1" rep="$2"
  local jl="$OUT/${arm}_rep${rep}.jsonl"
  if [ -f "$jl" ] && [ "$(wc -l < "$jl")" -ge "$ITERS" ]; then echo "skip $arm $rep (done)"; return; fi
  rm -f "$jl"
  cleanup
  if [ "$arm" = "old" ]; then
    export MARS_TRAIN_MAX_STEPS=150 MARS_TRAIN_BATCH_SIZE=300 MARS_TRAIN_LAMBDA=1.0 MARS_TRAIN_ENTROPY_COEFF=0.0
    export MARS_REWARD_COLLISION_PENALTY=20 MARS_REWARD_DISCOVERY_BONUS=2.0
  else
    export MARS_TRAIN_MAX_STEPS=90 MARS_TRAIN_BATCH_SIZE=700 MARS_TRAIN_LAMBDA=0.95 MARS_TRAIN_ENTROPY_COEFF=0.01
    export MARS_REWARD_COLLISION_PENALTY=8 MARS_REWARD_DISCOVERY_BONUS=4
  fi
  export MARS_METRICS_JSONL="$jl"
  echo "=== $(date +%T) start $arm rep$rep ==="
  python3 src/mars_swarm/mars_swarm/train_multi.py --train --iterations "$ITERS" \
      --checkpoint-dir "/tmp/paper2_ab_ckpt_${arm}_${rep}" > "/tmp/paper2_ab_${arm}_${rep}.log" 2>&1
  echo "=== $(date +%T) end $arm rep$rep: $(wc -l < "$jl" 2>/dev/null || echo 0) iterations logged ==="
  rm -rf "/tmp/paper2_ab_ckpt_${arm}_${rep}"
}

for rep in $(seq 1 "$REPS"); do
  run_arm old "$rep"
  run_arm corrected "$rep"
done
cleanup
echo "ALL DONE $(date +%T)"
