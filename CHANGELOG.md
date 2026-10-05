# Changelog

All notable changes to the MARS (Multi-Agent Robot Swarm) project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] - 2026-10-06

### Summary
Initial frozen research software release accompanying the multi-robot area coverage studies:
1. *"Diagnosing a Low-Displacement Failure Mode in MAPPO for Multi-Robot Area Coverage"* (SSRN Abstract [7556699](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699)).
2. *"A Failed Diagnosis of Entropy Collapse in Small-Budget MAPPO for Multi-Robot Area Coverage"* (SSRN Abstract [7567220](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567220)).
3. *"MARS: A Safety-Filtered Multi-Robot Area-Coverage Testbed with Classical, Random-Walk and MAPPO Controllers"* (SSRN Abstract [7567362](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567362)).

The v1.0.0 release defines the frozen research artifact; future development should occur in subsequent releases rather than modifying the published research baseline. All simulation environments, policy checkpoints, and evaluation manifests serve as the citable reference baseline.

### Added
- **Multi-Robot Simulation Environment**: Built on ROS 2 Jazzy and Gazebo Harmonic, featuring 5 deterministic environments (`cafe`, `warehouse`, `depot`, `office`, `maze`) configured with single-threaded ODE solvers (`<thread_count>1</thread_count>`) and fixed PRNG seed support.
- **Control Stack**:
  - *Decentralized Frontier Heuristic*: Dynamic Voronoi partitioning + Consensus-Based Bundle Auction (CBAA) + `py_trees` Behavior Tree mission manager + A* path planning.
  - *Random Walk Baseline*: Bounded stochastic heading controller.
  - *Multi-Agent PPO (MAPPO)*: Centralized-critic MAPPO implementation under Ray RLlib (ModelV2 API stack) with 46-dimensional observation vectors and permutation-invariant neighbor pooling.
- **Shared Safety Filter**:
  - Quadratic Program Control Barrier Function (QP-CBF) solver using OSQP with slack-penalized relaxation ($P_{\text{slack}} = 500.0$) enforcing dual-lookahead front/rear collision barriers ($d_{\text{safe}}^{\text{obs}} = 0.20\,\text{m}$) and inter-agent barriers ($d_{\text{safe}}^{\text{agent}} = 0.45\,\text{m}$).
  - Discrete rule-based overrides including inter-agent ACAS braking and front obstacle emergency stopping.
- **Discovered-Map ACR Metric (D-ACR)**: Discovered-map Area Coverage Rate accounting for obstacle cell exclusion.
- **Automated Verification Suite**: 56 regression and unit tests in `tests/` covering CBF safety, A* path planning, Voronoi partitioning, telemetry export, GUI configurations, and deterministic physics parameters.
- **Reproduction Scripts**:
  - `scripts/smoke_test.sh`: Sub-10-second headless environment and checkpoint validator.
  - `run_swarm.sh`: Unified experiment runner supporting headless benchmarks, GUI rollouts, scalability sweeps, dynamic hazard testing, and heatmap rendering.
- **Archival Data & Checkpoints**:
  - Pre-trained MAPPO weights (`checkpoints/mappo_baseline/`).
  - Controlled A/B training traces (`checkpoints/paper2_ab/`).
  - Multi-world benchmark evaluation matrix (`checkpoints/mappo_multiworld_comparison.json`).
  - Swarm scalability sweeps across $N \in \{2, 3, 5, 8\}$ robots (`checkpoints/sweep_scaling_results.json`).
  - Warehouse ceiling benchmark log (`checkpoints/run_summary.json`).

### Disclosed Methodological Caveats
- D-ACR is evaluated over discovered space rather than a fixed global physical area denominator.
- Control Barrier Functions use soft-slack relaxation without formal mathematical collision-prevention guarantees.
- MAPPO experiences low-displacement penalty-avoidance freezing in dense obstacle worlds.
