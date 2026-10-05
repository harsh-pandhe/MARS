# MARS v1.0.0 — Research Release

## Summary
MARS (Multi-Agent Robot Swarm) is an open research testbed for multi-robot area coverage, autonomous exploration, and safety-critical control built on **ROS 2 Jazzy** and **Gazebo Sim (Harmonic)** with a parallel **PettingZoo** interface.

The v1.0.0 release defines the frozen research artifact; future development should occur in subsequent releases rather than modifying the published research baseline. The codebase provides ready-to-run baselines, simulation assets, pre-trained policy checkpoints, and evaluation manifests for comparative reproducibility.

---

## Included Components

- **Controllers**:
  - *Decentralized Frontier Heuristic*: Dynamic Voronoi cell partitioning + Consensus-Based Bundle Auction (CBAA) + `py_trees` Behavior Tree mission manager + A* path planning.
  - *Random Walk Baseline*: Bounded stochastic heading baseline.
  - *Multi-Agent PPO (MAPPO)*: Centralized-critic policy using Ray RLlib (ModelV2 API stack) with 46-dimensional observation vectors and permutation-invariant neighbor pooling.
- **Shared Safety Stack**:
  - C-accelerated OSQP Quadratic Program Control Barrier Function (QP-CBF) solver enforcing dual-lookahead front/rear collision barriers ($d_{\text{safe}}^{\text{obs}} = 0.20\,\text{m}$) and inter-agent relative distance barriers ($d_{\text{safe}}^{\text{agent}} = 0.45\,\text{m}$).
  - Discrete rule-based safeguards: inter-agent Active Collision Avoidance System (ACAS) braking and front obstacle emergency stopping.
- **Simulation Environments**:
  - Five Gazebo Harmonic SDF worlds (`cafe`, `warehouse`, `depot`, `office`, `maze`) configured with single-threaded ODE physics (`<thread_count>1</thread_count>`) and fixed PRNG seeds.
  - TurtleBot3 Waffle platform support with scalable swarm configurations ($N \in \{2, 3, 5, 8\}$ robots) and preliminary Pioneer 2DX support.
- **Metrics**:
  - Discovered-Map Area Coverage Rate (D-ACR) with obstacle exclusion.
  - Swarm trajectory length, overlap redundancy, and clearance distributions.
- **Testing & Reproduction**:
  - Fast headless reproduction smoke test: `scripts/smoke_test.sh` (<10 s).
  - 56 passing unit and regression tests in `tests/`.
  - Unified command runner: `run_swarm.sh`.
- **Archival Results & Checkpoints**:
  - Pre-trained MAPPO weights in `checkpoints/mappo_baseline/`.
  - Controlled reward ablation weights (`ablation_baseline`, `ablation_lowpenalty`, `ablation_highdiscovery`).
  - Controlled entropy A/B logs (`checkpoints/paper2_ab/`).
  - Master evaluation manifests (`mappo_multiworld_comparison.json`, `sweep_scaling_results.json`, `run_summary.json`).

---

## Research Publications & Preprints

1. **"Diagnosing a Low-Displacement Failure Mode in MAPPO for Multi-Robot Area Coverage"** (Pandhe, 2026)  
   - SSRN Abstract ID: [7556699](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699) (Approved)  
   - Manuscript PDF: [`papers/paper1_negative_result_mappo/topic4_heuristic_beats_marl.pdf`](papers/paper1_negative_result_mappo/topic4_heuristic_beats_marl.pdf)
2. **"A Failed Diagnosis of Entropy Collapse in Small-Budget MAPPO for Multi-Robot Area Coverage"** (Pandhe, 2026)  
   - SSRN Abstract ID: [7567220](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567220) (Under Review)  
   - Manuscript PDF: [`papers/paper2_sample_efficient_mappo/topic1_sample_efficient_mappo.pdf`](papers/paper2_sample_efficient_mappo/topic1_sample_efficient_mappo.pdf)
3. **"MARS: A Safety-Filtered Multi-Robot Area-Coverage Testbed with Classical, Random-Walk and MAPPO Controllers"** (Pandhe, 2026)  
   - SSRN Abstract ID: [7567362](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567362) (Under Review)  
   - Manuscript PDF: [`papers/paper5_safe_scalable_architecture/main.pdf`](papers/paper5_safe_scalable_architecture/main.pdf)

Companion internal notes are also archived in `papers/paper3_cbf_safety_bounds/` and `papers/paper4_generalization_scaling/`.

---

## Known Methodological Limitations & Explicit Disclosures

To prevent misinterpretation, researchers should note the following constraints documented in the papers:
1. **Discovered-Map D-ACR**: Coverage rate denominator is computed over *discovered* map area minus *discovered* obstacle cells, rather than a fixed global physical floorplan area.
2. **Soft Safety Stack**: The CBF solver employs soft-slack relaxation ($P_{\text{slack}} = 500.0$) to guarantee QP feasibility; there is **no formal mathematical safety guarantee**, and wall-grazing contacts persist.
3. **Low-Displacement MAPPO Failure Mode**: In dense obstacle environments, MAPPO collapses into penalty-avoidance policy freezing (agents hover in place to avoid collision penalties, traveling ~1.5 m vs 5.4 m for heuristic and 7.4 m for random walk).
4. **Evaluation Nondeterminism**: Thread scheduling and ROS-Gazebo bridge communication introduce run-to-run timing variance.
5. **Simulation Scope**: All experiments are conducted within ROS 2 Jazzy and Gazebo Harmonic simulation; no physical hardware transfer is evaluated.
6. **No MAPPO Swarm Size Transfer**: MAPPO was trained and evaluated at $N=3$; zero-shot transfer across swarm sizes was not evaluated.
7. **Archival Transparency**: The single exploratory run that initially motivated the entropy collapse hypothesis was unarchived; it has been superseded by the controlled $n=3$ A/B study in `checkpoints/paper2_ab/`.

---

## Archival Citation

- **DOI**: [10.5281/zenodo.22758286](https://doi.org/10.5281/zenodo.22758286)
- **Code Repository**: [https://github.com/harsh-pandhe/MARS](https://github.com/harsh-pandhe/MARS)
- **License**: Apache-2.0
