# MARS: Multi-Agent Robot Swarm Navigation, Area Coverage, and MAPPO Benchmark Suite

[![Release: v1.0.0](https://img.shields.io/badge/Release-v1.0.0--frozen-blue.svg)](https://github.com/harsh-pandhe/MARS/releases/tag/v1.0.0)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.22758286.svg)](https://doi.org/10.5281/zenodo.22758286)
[![Tests: 56/56 Passing](https://img.shields.io/badge/Tests-56%2F56%20Passing-brightgreen.svg)](tests/)
[![SSRN: 7556699](https://img.shields.io/badge/SSRN-7556699%20(Approved)-darkblue.svg)](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699)
[![SSRN: 7567220](https://img.shields.io/badge/SSRN-7567220%20(Review)-darkblue.svg)](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567220)
[![SSRN: 7567362](https://img.shields.io/badge/SSRN-7567362%20(Review)-darkblue.svg)](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567362)

MARS is an open-source ROS 2 Jazzy and Gazebo Harmonic research testbed for comparative evaluation, safety filtering, and reproducibility in multi-robot area coverage.

---

## What is MARS?

MARS (Multi-Agent Robot Swarm) is a reproducible simulation testbed designed to evaluate multi-robot autonomous exploration, cooperative area coverage, and safety-critical velocity filtering under realistic physical sensing and actuation constraints.

Multi-robot area coverage is frequently evaluated either in abstract grid-worlds lacking continuous dynamics and sensor occlusion, or in heavyweight robotics stacks that are difficult to reproduce across labs. MARS addresses this gap by coupling standard robotic middleware (**ROS 2 Jazzy**, **Gazebo Sim Harmonic**) with multi-agent reinforcement learning interfaces (**PettingZoo Parallel API**, **Ray RLlib**), offering an open, deterministic platform for comparing classical planners, stochastic baselines, and learned policies under identical physical observation and safety filtering pipelines.

---

## Research Questions / Purpose

MARS was developed as a comparative, reproducibility-oriented research testbed to investigate:
1. **Classical vs. Learned Area Coverage**: How does a decentralized frontier exploration heuristic (Dynamic Voronoi + A* + Behavior Trees) compare empirically to Multi-Agent PPO (MAPPO) and Random-Walk baselines when evaluated under identical sensor bounds and physical constraints?
2. **Policy Failure Modes under Safety Penalties**: Why do learned policies (such as MAPPO) fail to achieve viable area coverage in obstacle-dense environments, and does reward shaping or entropy regularization resolve policy-freezing behavior?
3. **Decoupled Safety-Filter Behavior**: How effectively can an external Quadratic Programming Control Barrier Function (QP-CBF) filter eliminate inter-agent and environmental collisions independently of nominal policy competence, and what are the practical implications of soft-slack solver relaxations?

---

## Main Components

- **Classical Controller**: A decentralized frontier exploration engine combining Dynamic Voronoi spatial cell partitioning, Consensus-Based Bundle Auction (CBAA) task assignment over simulated range-limited radio ($d_{\text{comm}} \le 3.0\,\text{m}$), A* path search with obstacle inflation, and a formal `py_trees` mission behavior tree managing stuck recovery and perimeter patrol.
- **Random-Walk Controller**: A stochastic baseline executing bounded random heading perturbations, providing a minimal non-learning comparison benchmark.
- **MAPPO Controller**: A centralized-critic Multi-Agent PPO policy trained under Ray RLlib (ModelV2 API stack) with 46-dimensional observation vectors and permutation-invariant neighbor pooling.
- **Shared Safety Stack**: A unified safety filter sitting between the controller output and the robot actuators, combining discrete rule-based collision overrides (inter-agent ACAS and emergency obstacle braking) with a C-accelerated OSQP Quadratic Programming Control Barrier Function (QP-CBF) solver.
- **ROS 2 / Gazebo Environment**: High-fidelity simulation utilizing Gazebo Harmonic, namespaced multi-robot spawning (`tb1`, `tb2`, `tb3`, ...), isolated `ros_gz_bridge` parameter bindings, and single-threaded ODE physics solvers (`<thread_count>1</thread_count>`) for deterministic execution.
- **PettingZoo Interface**: A parallel multi-agent Python environment wrapper (`multi_env_wrapper.py`) supporting vectorized OpenCV grid raycasting, standardized step/reset loops, and Gym spaces.
- **Metrics Engine**: An automated telemetry logger (`swarm_telemetry.py`) calculating live Discovered-Map Area Coverage Rate (D-ACR), cell overlap redundancy, normalized kinetic energy $\int (v^2 + \omega^2)\,dt$, minimum time between deadlocks (MTBD), and proximity event counts.

---

## Architecture

```mermaid
graph TD
    subgraph Decision & Planning Layer
        C[Classical Controller<br/>Dynamic Voronoi + CBAA + A* + BT]
        R[Random-Walk Baseline<br/>Stochastic Heading]
        M[MAPPO Baseline<br/>Ray RLlib Centralized Critic]
    end

    subgraph Shared Safety Filter
        SW[Controller Switch]
        OR[Discrete Overrides<br/>ACAS Inter-Agent Brake + Front E-Stop]
        CBF[Soft-Slack OSQP QP-CBF<br/>d_safe_obs=0.20m, d_safe_agent=0.45m]
    end

    subgraph Simulation Platform
        GZ[Gazebo Harmonic Sim<br/>Single-Threaded ODE, Seed 42]
        BR[ros_gz_bridge / ROS 2 Jazzy]
    end

    subgraph Physical Swarm World
        W[Multi-Robot Arena<br/>TurtleBot3 Waffle N in {2, 3, 5, 8}]
    end

    subgraph Evaluation & Telemetry
        DACR[D-ACR Coverage Metric<br/>Discovered-Map Denominator]
        TEL[Telemetry Engine<br/>run_summary.json + Heatmaps]
    end

    C --> SW
    R --> SW
    M --> SW
    SW --> |Nominal v, w| OR
    OR --> |Candidate v, w| CBF
    CBF --> |Filtered Safe v, w| BR
    BR <--> GZ
    GZ <--> W
    W --> |LiDAR, Odom, Contact| DACR
    DACR --> TEL
```

---

## Environments

MARS includes five deterministic Gazebo Harmonic worlds, each configured with single-threaded ODE solvers (`<thread_count>1</thread_count>`) and fixed PRNG seeds:

| World | Arena Dimensions | Cells ($0.4\,\text{m}$ grid) | Geometric Description |
| :--- | :---: | :---: | :--- |
| **`cafe`** | $20\,\text{m} \times 20\,\text{m}$ | 800 | Dense indoor café layout with tables, chairs, and perimeter walls. Primary benchmark arena. |
| **`warehouse`** | $30\,\text{m} \times 20\,\text{m}$ | 9,375 | Expansive open warehouse with perimeter boundaries and isolated pallet stacks. |
| **`depot`** | $25\,\text{m} \times 25\,\text{m}$ | 3,000 | Industrial logistics depot with parallel pallet-rack corridors and narrow aisles. |
| **`office`** | $22\,\text{m} \times 22\,\text{m}$ | 3,500 | Compartmentalized office environment with interior cubicle walls and doorways. |
| **`maze`** | $18\,\text{m} \times 18\,\text{m}$ | 4,800 | Constrained labyrinthian corridors testing deadlock recovery and tight turns. |

---

## Robot Configuration

- **Platform**: TurtleBot3 Waffle differential-drive mobile robot.
  - Track width / wheel base: $l = 0.287\,\text{m}$.
  - Sensor-to-bumper radius: $r_{\text{bumper}} \approx 0.14\,\text{m}$.
  - Sensor: 360° 2D LiDAR sub-sampled into 24 angular sectors (range $0.12\,\text{m}$ to $3.5\,\text{m}$).
  - Control bounds: Linear velocity $v \in [-0.22, 0.22]\,\text{m/s}$, angular velocity $\omega \in [-1.0, 1.0]\,\text{rad/s}$.
- **Swarm Sizes**: Supported and tested across $N \in \{2, 3, 5, 8\}$ robots.
- **Spawn Protocol**: Robots are spawned along $y=0.0\,\text{m}$, $z=0.20\,\text{m}$ with initial yaw $-1.5708\,\text{rad}$ ($-\pi/2$). World $x$-offsets are fixed across all worlds:
  $$\text{Offsets: } [0.0, -0.7, +0.7, -1.4, +1.4, -2.1, +2.1, -2.8]\,\text{m}\quad\text{for robots } 1 \dots 8$$

---

## Metrics

### Discovered-Map Area Coverage Rate (D-ACR)
Coverage is evaluated using the Discovered-Map Area Coverage Rate:
$$\mathrm{D\text{-}ACR} = \frac{|\text{visited} \setminus \text{obstacle}|}{N_{\text{cells}} - |\text{obstacle}|} \times 100\%$$

> [!IMPORTANT]
> **D-ACR Denominator Caveat**:
> D-ACR is a **discovered-map metric**, not a fixed-denominator physical coverage metric. The denominator excludes cells that have been actively registered as obstacles by LiDAR returns during the run. Consequently:
> - Controllers that discover obstacles at different rates have different denominators.
> - A controller that discovers few obstacles keeps a larger denominator (lowering its D-ACR).
> - A controller that rapidly hugs walls shrinks its denominator early.
> A ground-truth-denominator physical metric is not computed.

### Proximity Event Counts
- **Wall Proximity Event**: Registered when minimum LiDAR return is below $0.14\,\text{m}$ (bumper threshold proxy).
- **Inter-Robot Proximity Event**: Registered when Euclidean distance between two robots is below $0.20\,\text{m}$ after step 15.
- Both metrics are threshold-based sensor proxies, not verified physical contact forces.

---

## Safety Stack

The shared safety stack executes on every control step to filter nominal command velocities $(v_{\text{nom}}, \omega_{\text{nom}})$:

1. **Discrete Rule-Based Safeguards**:
   - *ACAS Inter-Agent Override*: If any peer robot is within $0.20\,\text{m}$, command velocities are clamped to zero.
   - *Front Obstacle Emergency Brake*: If frontal obstacles ($|\phi| \le 20^\circ$) are detected within $0.18\,\text{m}$, forward velocity is zeroed.
2. **Soft-Slack QP Control Barrier Function (QP-CBF)**:
   - Formulated as a Quadratic Program minimizing deviation from nominal commands plus a penalty on a slack relaxation variable $u_2$:
     $$\min_{v, \omega, u_2} \left[ (v - v_{\text{nom}})^2 + 0.05(\omega - \omega_{\text{nom}})^2 + P_{\text{slack}} u_2^2 \right]$$
     $$\text{subject to: } A_i v + B_i \omega + \gamma h_i + u_2 \ge 0, \quad u_2 \ge 0$$
   - Enforces dual lookahead front/rear obstacle barriers ($d_{\text{safe}}^{\text{obs}} = 0.20\,\text{m}$) and inter-agent relative barriers ($d_{\text{safe}}^{\text{agent}} = 0.45\,\text{m}$) with $\gamma = 2.0$ and $P_{\text{slack}} = 500.0$.

> [!CAUTION]
> **No Formal Safety Guarantee**:
> Because the QP includes a non-zero slack variable $u_2$ with finite penalty $P_{\text{slack}} = 500.0$ to guarantee solver feasibility in tight spaces, the safety barriers are **soft**. There is **NO formal mathematical safety guarantee**. In tight passages, the solver accepts temporary constraint violations rather than returning infeasible, resulting in observed minimum inter-agent clearances down to $0.26\,\text{m}$ (maze) and persistent wall grazing.

---

## MAPPO (Multi-Agent PPO)

- **Architecture**: Centralized Training with Decentralized Execution (CTDE) implemented under Ray RLlib (ModelV2 API stack, PyTorch).
- **Policy Model**: Shared Actor network (`[46] -> [128, 128] -> [4] (mean, log_std)`) and Centralized Critic network (`[46 * N] -> [256, 256] -> [1]`).
- **Observation Space (46-dim)**: 24 LiDAR sector distance minimums, relative goal vector (distance and heading angle), linear and angular velocities, and relative states to up to 9 neighbors (padded).
- **Archival Checkpoint**: Stored in `checkpoints/mappo_baseline/` (trained for 45 iterations on the `cafe` environment).
- **Evaluation & Statistical Limitations**:
  - The MAPPO evaluation results reported in the repository reflect specific experimental protocols:
    - *Protocol A* (`cafe` only): 10 episodes per condition, 150 steps/ep.
    - *Protocol B* (Cross-world): Single-seed probe ($n=1$) per condition across 5 worlds.
    - *Protocol C* (Reward ablation): $n=3$ seeds per condition.
  - These results demonstrate a low-displacement penalty-avoidance failure mode under the evaluated reward structure; they do **not** constitute a statistically representative benchmark across hyperparameter sweeps or diverse MARL algorithms.

---

## Reproducibility

### 1. Requirements & System Dependencies
- OS: Ubuntu 24.04 LTS (Host) or Ubuntu 22.04 LTS (Container)
- ROS 2: Jazzy Jalisco (`desktop` install)
- Simulator: Gazebo Sim Harmonic
- Python: 3.10 or 3.12 with pinned dependencies in `requirements.txt`
- PyTorch & Ray RLlib: PyTorch 2.12.0+, Ray 2.55.1

### 2. Environment Setup
```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Clone and navigate to repository
cd /path/to/MARS

# Build workspace packages
colcon build --symlink-install
source install/setup.bash

# Export required runtime environment variables
export TORCHDYNAMO_DISABLE=1
export TORCH_COMPILE_DISABLE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
```

### 3. Run Rapid Reproduction Smoke Test (<10 seconds)
Verify library dependencies, world files, MAPPO checkpoint loading, and core unit tests in a single headless command:
```bash
./scripts/smoke_test.sh
```

### 4. Run Full Regression Test Suite (56 Tests)
```bash
./run_swarm.sh --test
# Or directly via pytest:
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -v
```

### 5. Run Classical Frontier Coverage Demo
```bash
# Headless run in cafe world (1200 steps)
./run_swarm.sh --coverage-demo 1200 --world cafe --headless

# GUI rollout in warehouse with 3 robots
./run_swarm.sh --coverage-demo 1200 --world warehouse
```

### 6. Run Baseline Benchmark Comparison (Frontier vs. Random Walk vs. MAPPO)
```bash
# Evaluate Random Walk and Frontier Heuristic on cafe (150 steps)
./run_swarm.sh --benchmark --world cafe

# Evaluate all controllers including pre-trained MAPPO checkpoint
./run_swarm.sh --benchmark ./checkpoints/mappo_baseline --world cafe
```

### 7. Render High-Resolution Coverage Heatmaps
```bash
./run_swarm.sh --render-heatmap docs/heatmaps/warehouse_demo_heatmap.npz --out docs/heatmaps/my_heatmap.png
```

---

## Expected Results

The following values represent archived research results reported across project manuscripts and JSON manifests.

### 1. Protocol A: Café Distributional Comparison (10 Episodes, 150 Steps, $N=3$)
*From Paper 1 (`papers/paper1_negative_result_mappo/topic4_heuristic_beats_marl.pdf`)*:

| Controller / Scenario | Median D-ACR (%) | Mean D-ACR (%) | Redundancy | Swarm Dist. (m) | Wall Collisions | Agent Collisions |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Frontier Heuristic (A\* + BT)** | **38.6%** | $35.2 \pm 14.2\%$ | **16.77** | 5.4 m | 2.20 | 0.00 |
| **Random Walk** | **29.6%** | $31.4 \pm 4.5\%$ | 34.38 | 7.4 m | 0.80 | 0.00 |
| **MAPPO (Nominal, 45 iters)** | **14.5%** | $14.0 \pm 0.9\%$ | 43.91 | 1.5 m | 2.00 | 0.00 |
| **MAPPO (Sensor Noise)** | **12.0%** | $11.6 \pm 2.4\%$ | 23.55 | 1.8 m | 2.60 | 0.00 |
| **MAPPO (Agent Failure)** | **9.3%** | $9.9 \pm 0.9\%$ | 9.08 | 0.9 m | 3.00 | 0.00 |

*Status: 10-episode reference distribution. MAPPO experiences low-displacement freezing due to dense collision penalties.*

### 2. Protocol B: Single-Episode Cross-World Matrix ($n=1$, 150 Steps, $N=3$)
*From Paper 5 and `checkpoints/mappo_multiworld_comparison.json`*:

| World | Classical Frontier D-ACR | Random Walk D-ACR | MAPPO Nominal D-ACR |
| :--- | :---: | :---: | :---: |
| `cafe` | 57.0% | 32.8% | 28.2% |
| `warehouse` | 4.34% | 3.52% | 3.31% |
| `depot` | 13.28% | 9.40% | 6.69% |
| `office` | 3.80% | 2.87% | 2.95% |
| `maze` | 2.73% | 2.45% | 2.17% |

*Status: Exploratory single-run baselines illustrating relative controller performance across varying arena topologies.*

### 3. Long-Run Classical Controller Reference Runs (Single Runs, $N=3$)
*From Paper 5 and archived summary manifests*:

| World | Total Cells | Step Budget | Final D-ACR | Plateau Step | Swarm Dist. (m) | Wall Prox. | Agent Prox. | Min Clr. (m) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `warehouse` | 9,375 | 9,050 | 100.0% | 7,660 | 1263 m | 17,018 | 0 | 0.81 m |
| `office` | 3,500 | 3,500 | 98.2% | 3,466 | 377 m | 0 | 0 | 0.38 m |
| `depot` | 3,000 | 3,500 | 85.7% | 2,906 | 180 m | 477 | 0 | 0.55 m |
| `cafe` | 800 | 1,200 | 100.0% | 397 | 82 m | 738 | 0 | 0.46 m |
| `maze` | 4,800 | 4,000 | 25.0% | 3,961 | 316 m | 1,108 | 0 | 0.26 m |

*Status: Descriptive single-run reference values documenting late-stage D-ACR stabilization.*

---

## Research Outputs

The MARS project has produced three manuscripts submitted to SSRN:

1. **Testbed & Architecture Paper**:  
   *"MARS: A Safety-Filtered Multi-Robot Area-Coverage Testbed with Classical, Random-Walk and MAPPO Controllers"* (Pandhe, 2026).  
   SSRN Submission ID: [7567362](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567362) (Under Review) | [Compiled PDF](papers/paper5_safe_scalable_architecture/main.pdf)
2. **MAPPO Low-Displacement Failure Mode Paper**:  
   *"Diagnosing a Low-Displacement Failure Mode in MAPPO for Multi-Robot Area Coverage"* (Pandhe, 2026).  
   SSRN Submission ID: [7556699](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699) (Approved) | [Compiled PDF](papers/paper1_negative_result_mappo/topic4_heuristic_beats_marl.pdf)
3. **Entropy-Collapse Reproducibility Companion**:  
   *"A Failed Diagnosis of Entropy Collapse in Small-Budget MAPPO for Multi-Robot Area Coverage"* (Pandhe, 2026).  
   SSRN Submission ID: [7567220](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567220) (Under Review) | [Compiled PDF](papers/paper2_sample_efficient_mappo/topic1_sample_efficient_mappo.pdf)

All LaTeX sources, figures, and compiled PDFs are located in [`papers/`](papers/).

---

## Data / Artifacts

- Master verification table: [`docs/BENCHMARK_WORLDS.md`](docs/BENCHMARK_WORLDS.md)
- Complete artifact accounting: [`ARTIFACT_MANIFEST.md`](ARTIFACT_MANIFEST.md)
- Persistent Zenodo Archive: [10.5281/zenodo.22758286](https://doi.org/10.5281/zenodo.22758286)
- Checkpoints & Evaluation Manifests:
  - `checkpoints/mappo_baseline/`: Ray RLlib MAPPO checkpoint
  - `checkpoints/paper2_ab/`: Controlled A/B training logs
  - `checkpoints/run_summary.json`: Warehouse ceiling run
  - `checkpoints/mappo_multiworld_comparison.json`: 25-condition cross-world matrix
  - `checkpoints/sweep_scaling_results.json`: Robot count scaling sweep results

---

## Known Limitations

This research artifact operates under the following documented constraints:
1. **Discovered-Map D-ACR**: The metric denominator varies across controllers based on discovered obstacle cells; it does not measure coverage against a fixed ground-truth physical area.
2. **Soft Safety Bounds**: The QP-CBF filter uses soft-slack relaxation ($P_{\text{slack}} = 500.0$); no formal collision-prevention guarantee exists, and wall-grazing contacts occur.
3. **Evaluation Nondeterminism**: Asynchronous ROS-Gazebo bridge communication and OS thread scheduling introduce run-to-run timing variance.
4. **No Cross-Swarm MAPPO Transfer**: The MAPPO policy was trained strictly at $N=3$ in the `cafe` world; zero-shot transfer across team sizes was not evaluated.
5. **Missing Component Ablations**: Full architectural ablations of the classical coordinator (e.g. removing Voronoi partitioning or CBAA consensus independently) were not systematically benchmarked.
6. **Simulation-Only Scope**: The framework is validated purely in Gazebo simulation; no physical hardware transfer was performed.
7. **Single Primary Robot Model**: Primary benchmarking standardizes on the TurtleBot3 Waffle (with preliminary Pioneer 2DX model support).
8. **Small Environment-Step Training Budget**: Training was conducted on thousands of environment steps (due to physics-in-the-loop overhead) rather than millions of steps common in abstracted MARL research.
9. **Archival Gaps**: The raw telemetry from the initial exploratory entropy collapse run and two one-off hazard tests was unarchived; these have been superseded by controlled reproducible tests.

---

## Citation

If you use MARS in your research, please cite:

```bibtex
@article{pandhe2026mars,
  title   = {MARS: A Safety-Filtered Multi-Robot Area-Coverage Testbed with Classical, Random-Walk and MAPPO Controllers},
  author  = {Pandhe, Harsh},
  journal = {SSRN Electronic Journal},
  year    = {2026},
  doi     = {10.5281/zenodo.22758286},
  url     = {https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567362}
}

@article{pandhe2026mappo_failure,
  title   = {Diagnosing a Low-Displacement Failure Mode in MAPPO for Multi-Robot Area Coverage},
  author  = {Pandhe, Harsh},
  journal = {SSRN Electronic Journal},
  year    = {2026},
  doi     = {10.5281/zenodo.22758286},
  url     = {https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699}
}
```

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).

---

## AI Disclosure

Generative AI tools, including Anthropic's Claude, were used during project development and manuscript preparation for code and analysis-script assistance, language editing, structural critique and identification of potential methodological or statistical issues. The author reviewed and verified the results against the archived data and is solely responsible for the methodology, experiments, analysis and conclusions.

---

## Status

**Project State**: **v1.0.0 (Research Artifact Frozen)**.  
The v1.0.0 release defines the frozen research artifact; future development should occur in subsequent releases rather than modifying the published research baseline. The current codebase, simulation assets, pre-trained weights, and evaluation manifests represent the reference release for the cited manuscripts. No further feature development, reward retraining, or experimental changes will be made to this release branch.
