#!/usr/bin/env bash
# =============================================================================
# MARS v1.0.0 — Fast Research Reproduction Smoke Test
# =============================================================================
# Verifies environment, core dependencies, checkpoint loading, simulation
# assets, and unit tests in under 15 seconds without launching 3D GUI popups.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

echo "======================================================================"
echo " MARS v1.0.0 Smoke Test — Research Environment & Artifact Validator"
echo "======================================================================"

# 1. Environment variables
export TORCHDYNAMO_DISABLE=1
export TORCH_COMPILE_DISABLE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# 2. Check ROS 2 environment
echo "[1/5] Checking ROS 2 Jazzy environment..."
if [ -z "${ROS_DISTRO}" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "      Sourcing /opt/ros/jazzy/setup.bash..."
        source /opt/ros/jazzy/setup.bash
    else
        echo "      [WARN] /opt/ros/jazzy/setup.bash not found. Proceeding with system Python."
    fi
else
    echo "      Detected ROS_DISTRO=${ROS_DISTRO}"
fi

# 3. Check Python dependencies & critical imports
echo "[2/5] Verifying Python library dependencies..."
python3 -c "
import sys
modules = ['torch', 'ray', 'ray.rllib', 'gymnasium', 'pettingzoo', 'osqp', 'cv2', 'py_trees', 'numpy', 'scipy', 'matplotlib']
missing = []
for m in modules:
    try:
        __import__(m)
    except ImportError as e:
        missing.append((m, str(e)))

if missing:
    print('      [FAIL] Missing required modules:', missing)
    sys.exit(1)
else:
    import torch, ray, osqp, gymnasium, pettingzoo
    print(f'      [OK] PyTorch {torch.__version__}, Ray {ray.__version__}, OSQP {osqp.__version__}, Gymnasium {gymnasium.__version__}, PettingZoo {pettingzoo.__version__}')
"

# 4. Verify simulation assets & SDF worlds
echo "[3/5] Verifying simulation assets and world geometries..."
WORLDS=("cafe" "warehouse" "depot" "office" "maze")
for w in "${WORLDS[@]}"; do
    SDF="src/mars_swarm/worlds/${w}.sdf"
    if [ -f "${SDF}" ]; then
        # Verify single-threaded ODE configuration
        if grep -q "<thread_count>1</thread_count>" "${SDF}"; then
            echo "      [OK] World '${w}' found with single-threaded ODE solver config."
        else
            echo "      [WARN] World '${w}' missing explicit thread_count=1 setting."
        fi
    else
        echo "      [FAIL] World file ${SDF} not found!"
        exit 1
    fi
done

# 5. Verify MAPPO baseline checkpoint loading
echo "[4/5] Verifying pre-trained MAPPO checkpoint integrity..."
python3 -c "
import os, sys
sys.path.insert(0, os.path.abspath('src/mars_swarm/mars_swarm'))
from train_multi import TorchCentralizedCriticModel
from ray.rllib.models import ModelCatalog
ModelCatalog.register_custom_model('cc_model', TorchCentralizedCriticModel)
from ray.rllib.policy.policy import Policy

ckpt_dir = os.path.abspath('checkpoints/mappo_baseline/policies/shared_policy')
if not os.path.isdir(ckpt_dir):
    print(f'      [FAIL] Checkpoint directory {ckpt_dir} not found!')
    sys.exit(1)

policy = Policy.from_checkpoint(ckpt_dir)
print(f'      [OK] Loaded MAPPO policy {type(policy).__name__} from {os.path.relpath(ckpt_dir)}.')
"

# 6. Run core regression checks (CBF, A*, Voronoi, ACR fairness, Permutation Invariance)
echo "[5/5] Running core unit test suite (deterministic, headless)..."
pytest tests/test_acr_fairness.py \
       tests/test_cbf_safety.py \
       tests/test_frontier_astar.py \
       tests/test_decentralized_coordinator.py \
       tests/test_permutation_invariance.py \
       tests/test_deterministic_physics.py \
       -q --disable-warnings

echo "======================================================================"
echo " [SUCCESS] MARS v1.0.0 reproduction smoke test passed cleanly!"
echo "======================================================================"
