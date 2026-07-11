# MARS Paper — Experiment Run Plan

Every `[TODO(DATA)]` / red `\TODO{}` in `main.tex` maps to a command below.
Run these, then paste numbers into the matching table.

## 0. Prereqs
```bash
source /opt/ros/jazzy/setup.bash
cd <repo-root>
colcon build --symlink-install
source install/setup.bash
```
Note: `run_swarm.sh` unsets VS Code snap env vars to avoid the Gazebo GUI
libpthread crash (see commit history). Run from a plain terminal, not the
VS Code integrated snap terminal.

## 1. Train the MAPPO policy
```bash
./run_swarm.sh --train            # headless; writes ./checkpoints/checkpoint_XXXXXX
```
- Config: `train_multi.py` — 15 iterations default, `--iterations N` to change.
- Record final `episode_reward_mean` and policy loss curve for a training-curve
  figure (optional Fig.).

## 2. Main comparison + robustness tables (Tables I & II)
```bash
./run_swarm.sh --benchmark ./checkpoints/checkpoint_000002
```
Produces, per scenario, `ACR ± std | Overlap Redundancy | Energy(m)`:
- Random Walk, Frontier Heuristic (baselines)
- MAPPO Nominal, MAPPO Sensor Noise, MAPPO Agent Failure

Outputs:
- Console summary  -> fill **Table I** (nominal row per method) and **Table II**
  (MARS across 3 regimes).
- `./checkpoints/benchmark_results.png` -> copy to `paper/figures/` and
  uncomment the `figure` block in `main.tex` (Sec. Results).

Increase episodes for tighter std: `--episodes 10` (edit run_swarm.sh passthrough
or call `evaluate_benchmarks.py --episodes 10 --checkpoint <ckpt>`).

> Collision count is NOT currently printed by `evaluate_benchmarks.py`. To fill
> the "Coll." column, add a counter: in `run_benchmark_episode`, increment on
> `infos[agent]['status'] == 'FAILED'` and return it. Small patch — see §5.

## 3. Coverage-over-time / 100% coverage run (headline number)
```bash
./run_swarm.sh --evaluate ./checkpoints/checkpoint_000002   # continuous_exploration=True, 1200 steps
```
- `run_evaluation` uses `continuous_exploration=True`, `max_steps=1200`.
- Log final ACR from `env.visited_grid` for the abstract headline.

## 4. Safety-filter validation (Sec. VII-C)
Instrument `_apply_cbf` (multi_env_wrapper.py) to log:
- count of steps where `res.x != (v_nom,w_nom)` (CBF intervened),
- min-clearance pre-filter vs post-filter.
Then report interventions/episode and clearance distribution.

## 5. Ablations (Sec. VII-D)
Run the benchmark with each component disabled:
| Ablation | How |
|---|---|
| No CBF | early-return `(v_nom,w_nom)` at top of `_apply_cbf` |
| No consensus | comment the `d_comm` merge loop in `publish_coverage_map` |
| No GNN | replace masked mean-pool with fixed 3-neighbor concat; eval at N=5,10 |

GNN scale-invariance (N=5, N=10) already proven by:
```bash
python -m pytest scratch/test_swarm_logic.py -k gnn_scale_invariance -v
```

## 6. Unit-test suite (Methods correctness, cite in text if desired)
```bash
cd scratch && python -m unittest test_swarm_logic -v
```
Covers: coordinate transforms, frontier BFS, cooperative allocation, APF,
watchdog, CTDE postprocessing, CBF filter, consensus, GNN scale-invariance.

## 7. Compile the paper
```bash
cd paper
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

## Fill checklist (search main.tex for `\TODO`)
- [ ] Abstract headline sentence
- [ ] Table I — all baseline + MARS cells
- [ ] Table II — MARS 3 regimes
- [ ] Episodes/steps ($E$, $T$) in Experimental Setup
- [ ] CBF intervention stats
- [ ] Ablation numbers (No CBF / No consensus / No GNN)
- [ ] Conclusion headline restatement
- [ ] Uncomment figure block once `figures/benchmark_results.png` copied
