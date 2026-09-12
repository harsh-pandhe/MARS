# MARS Research Papers

Five papers derived from the MARS multi-robot swarm coverage project, each self-contained (source + compiled PDF + bibliography + figures where used).

| # | Folder | Title | Format | Core Finding |
|---|--------|-------|--------|--------------|
| 1 | [`paper1_negative_result_mappo/`](paper1_negative_result_mappo/) | When a Classical Heuristic Beats Multi-Agent Reinforcement Learning | SSRN-style | MAPPO loses to Frontier Heuristic in all 5 tested worlds — the headline negative result |
| 2 | [`paper2_sample_efficient_mappo/`](paper2_sample_efficient_mappo/) | Sample-Efficient MAPPO for Physics-in-the-Loop Multi-Robot Training | SSRN-style | Diagnoses & fixes training instability; postscript reports the completed (still negative) validation |
| 3 | [`paper3_cbf_safety_bounds/`](paper3_cbf_safety_bounds/) | Decoupling Safety Guarantees from Policy Competence | SSRN-style | CBF filter safety holds (0 collisions) even when the policy behind it is incompetent |
| 4 | [`paper4_generalization_scaling/`](paper4_generalization_scaling/) | Environment Structure, Not Just Team Size | SSRN-style | Coverage ceiling is a topology property, not a policy property — 5-world, 4-swarm-size sweep |
| 5 | [`paper5_safe_scalable_architecture/`](paper5_safe_scalable_architecture/) | MARS: Safe and Scalable Multi-Agent Reinforcement Learning | IEEEtran (conference) | Full system architecture paper — safety/coordination validated, MAPPO component honestly negative |

All PDFs compile cleanly with `pdflatex` (0 errors) as of the last build. Source `.bib` files and `figures/` (where applicable) are included per-paper so each folder can be zipped and submitted independently to SSRN, arXiv, or a conference.

## Data sources behind these papers

- `checkpoints/mappo_multiworld_comparison.json` — 25-run, 5-world MAPPO vs. heuristic matrix
- `checkpoints/sweep_scaling_results.json` — 20-run robot-count scaling sweep
- `checkpoints/run_summary.json` — 24,000-step warehouse ceiling run
- `docs/BENCHMARK_WORLDS.md` — consolidated master verification table
