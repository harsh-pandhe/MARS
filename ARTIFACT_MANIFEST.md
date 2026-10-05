# MARS Artifact Manifest (v1.0.0)

This manifest provides a full transparency accounting of all software, simulation environments, neural network checkpoints, evaluation manifests, and historical data included or excluded in the **MARS v1.0.0** research release.

---

## 1. Included Artifacts

### 1.1 Source Code (`src/`)
- `src/mars_swarm/mars_swarm/`:
  - `cbf_qp_solver.py`: Fast OSQP-based Quadratic Programming Control Barrier Function solver with soft-slack relaxation.
  - `decentralized_coordinator.py`: Dynamic Voronoi partitioning and Consensus-Based Bundle Auction (CBAA) implementation.
  - `mission_behavior_tree.py`: `py_trees` mission state machine for frontier exploration and stuck recovery.
  - `scan_matching_localizer.py`: 2D distance-field scan matcher eliminating dead-reckoning drift.
  - `swarm_telemetry.py`: Telemetry logger exporting metrics, energy consumption, and `run_summary.json`.
  - `multi_env_wrapper.py`: PettingZoo parallel multi-agent environment with OpenCV grid raycasting.
  - `train_multi.py`: Ray RLlib MAPPO training pipeline with ModelV2 API stack and Centralized Critic.
  - `evaluate_benchmarks.py`: Multi-controller evaluation engine across Random Walk, Frontier Heuristic, and MAPPO.
  - `benchmark_mappo_multiworld.py`: Cross-environment $5 \times 5$ evaluation matrix runner.
  - `sweep_robot_count.py`: Scalability sweep across swarm sizes ($N \in \{2, 3, 5, 8\}$).
  - `dynamic_obstacle_test.py`: Dynamic hazard avoidance testing script.
  - `coverage_heatmap_renderer.py`: Publication-grade PNG heatmap generator from `.npz` and `.json` telemetry.
- `src/mypkg/`:
  - `clean_processes.py`: Cleanup utility for lingering ROS 2 and Gazebo processes.
  - Supporting launch, URDF, and navigation configurations.

### 1.2 Simulation Environments (`src/mars_swarm/worlds/`)
- Five deterministic Gazebo Harmonic worlds configured with single-threaded ODE solvers (`<thread_count>1</thread_count>`) and fixed PRNG seeds:
  - `cafe.sdf`: Dense obstacle coffee shop environment ($20\,\text{m} \times 20\,\text{m}$).
  - `warehouse.sdf`: High-ceiling industrial warehouse environment ($30\,\text{m} \times 20\,\text{m}$).
  - `depot.sdf`: Logistics depot with pallet racking corridors ($25\,\text{m} \times 25\,\text{m}$).
  - `office.sdf`: Multi-room office layout with cubicle partitions ($22\,\text{m} \times 22\,\text{m}$).
  - `maze.sdf`: Constrained labyrinthian corridors ($18\,\text{m} \times 18\,\text{m}$).

### 1.3 Configurations & Launch Files
- `src/mars_swarm/launch/`: Multi-robot spawning (`spawn_multi.launch.py`), namespaced parameter bridges, SLAM, and navigation launch files.
- `src/mars_swarm/config/`: ROS-Gazebo bridge parameter mapping files for 1, 2, 3, and 5 robots.
- `src/mars_swarm/rviz/`: RViz display configurations (`mars_swarm.rviz`, `multi_robot.rviz`, `namespaced_swarm.rviz`).

### 1.4 Policy Checkpoints (`checkpoints/`)
- `checkpoints/mappo_baseline/`:
  - Complete Ray RLlib algorithm state (`algorithm_state.pkl`, `rllib_checkpoint.json`).
  - Extracted policy directory (`policies/shared_policy/`) loaded via `Policy.from_checkpoint`.
- Controlled reward ablation checkpoints:
  - `checkpoints/ablation_baseline/`: Baseline reward configuration weights.
  - `checkpoints/ablation_lowpenalty/`: Reduced collision penalty ablation weights.
  - `checkpoints/ablation_highdiscovery/`: Increased frontier discovery reward weights.
- Controlled entropy A/B test logs:
  - `checkpoints/paper2_ab/`: Per-iteration JSONL metrics, summary JSON, and entropy return comparison curves.

### 1.5 Archived Empirical Results & Telemetry
- `checkpoints/run_summary.json`: 24,000-step ceiling run in warehouse world.
- `checkpoints/cafe_extended_summary.json`: 1,200-step ceiling run in cafe world.
- `checkpoints/depot_extended_summary.json`: 1,200-step ceiling run in depot world.
- `checkpoints/office_extended_summary.json`: 1,200-step ceiling run in office world.
- `checkpoints/maze_extended_summary.json`: 1,200-step ceiling run in maze world.
- `checkpoints/mappo_multiworld_comparison.json`: 25-condition cross-world matrix (5 worlds × 5 controller regimes).
- `checkpoints/sweep_scaling_results.json`: Swarm scalability sweep across 2, 3, 5, and 8 robots.
- `checkpoints/ablation_results/`: Paired statistical evaluation manifests across $n=3$ seeds.
- `checkpoints/repro_check.json`: Verification manifest.
- `docs/heatmaps/`: Pre-rendered high-resolution PNG coverage heatmaps.

### 1.6 Verification & Test Suite (`tests/`)
- 16 test modules with 56 unit and regression test cases passing at 100%:
  - `test_cbf_safety.py`: OSQP solver speed, boundary constraints, and slack variable response.
  - `test_dynamic_cbf.py`: Moving hazard deflection.
  - `test_frontier_astar.py`: A* detour routing and obstacle inflation.
  - `test_decentralized_coordinator.py`: Dynamic Voronoi partitioning and auction resolution.
  - `test_acr_fairness.py`: D-ACR metric calculation and obstacle exclusion.
  - `test_permutation_invariance.py`: Neighbor ordering invariance in observation processing.
  - `test_deterministic_physics.py`: ODE solver configuration and PRNG seed declarations.
  - `test_robot_sweep.py`: Swarm scalability mocking up to $N=8$.
  - `test_scan_matching_localizer.py`: Distance-field drift correction.
  - `test_swarm_telemetry.py`: Telemetry aggregation and JSON export.
  - `test_mappo_benchmark_runner.py`: Benchmark CLI and scenario registration.
  - `test_mappo_init.py`: Ray RLlib training step and Adam optimizer initialization.
  - `test_mission_behavior_tree.py`: State transition and escape behavior logic.
  - `test_new_worlds.py`: SDF existence and geometric bounds across all 5 benchmark worlds.
  - `test_coverage_heatmap.py`: Heatmap generation from array data.
  - `test_gui_config.py`: RViz and Gazebo GUI layout integrity.

### 1.7 Manuscripts & Research Outputs (`papers/`)
- Paper 1: *"Diagnosing a Low-Displacement Failure Mode in MAPPO for Multi-Robot Area Coverage"* (LaTeX source + compiled PDF, SSRN Abstract [7556699](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699)).
- Paper 2: *"A Failed Diagnosis of Entropy Collapse in Small-Budget MAPPO for Multi-Robot Area Coverage"* (LaTeX source + compiled PDF, SSRN Abstract [7567220](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567220)).
- Paper 3: *"Decoupling Safety Guarantees from Policy Competence"* (LaTeX source + compiled PDF).
- Paper 4: *"Environment Structure, Not Just Team Size"* (LaTeX source + compiled PDF).
- Paper 5: *"MARS: A Safety-Filtered Multi-Robot Area-Coverage Testbed with Classical, Random-Walk and MAPPO Controllers"* (LaTeX source + compiled PDF, SSRN Abstract [7567362](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567362)).

---

## 2. Excluded / Unavailable Historical Artifacts

To maintain full scientific honesty, the following historical intermediate materials are explicitly not included in this repository:

1. **Original Exploratory Entropy-Collapse Run Trace**:
   - The single unarchived exploratory training run that originally motivated the entropy-collapse hypothesis was overwritten prior to establishing the structured JSONL logging pipeline.
   - *Resolution*: Controlled $n=3$ seed A/B experiments were subsequently conducted, logged, and permanently archived in `checkpoints/paper2_ab/`, which form the empirical basis of Paper 2.
2. **Raw Telemetry Traces for Two Historical One-Off Dynamic Hazard Runs**:
   - The early manual Gazebo demonstration runs for head-on and crossing dynamic hazards were saved only as visual observations.
   - *Resolution*: A deterministic test script (`src/mars_swarm/mars_swarm/dynamic_obstacle_test.py`) and automated unit test (`tests/test_dynamic_cbf.py`) were created to reproduce the avoidance behavior programmatically.
3. **Pre-Portability Git History**:
   - Commits prior to the path-portability refactoring (which removed machine-specific absolute path dependencies) contained non-portable hardcoded system paths.
4. **Intermediate Raw TensorBoard Binary Event Logs**:
   - Intermediate binary event logs (`*.tfevents.*`) from discarded scratch runs are excluded via `.gitignore` to avoid repository bloat; all core metrics were preserved in structured JSON/JSONL manifests.
