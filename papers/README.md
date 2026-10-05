# MARS Research Papers

Five papers derived from the MARS multi-robot swarm coverage project, each self-contained (source + compiled PDF + bibliography + figures where used).

| # | Folder | Title | Format | Core Finding & SSRN Status |
|---|--------|-------|--------|----------------------------|
| 1 | [`paper1_negative_result_mappo/`](paper1_negative_result_mappo/) | Diagnosing a Low-Displacement Failure Mode in MAPPO for Multi-Robot Area Coverage | SSRN-style | MAPPO loses to Frontier Heuristic in all 5 tested worlds — the headline negative result, with explicit two-protocol evaluation, ACR definition, reward equation, and references. **Submitted to SSRN 2026-10-03, abstract ID [7556699](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699) (APPROVED).** |
| 2 | [`paper2_sample_efficient_mappo/`](paper2_sample_efficient_mappo/) | A Failed Diagnosis of Entropy Collapse in Small-Budget MAPPO for Multi-Robot Area Coverage | SSRN-style | Failure-to-reproduce report: the original entropy-collapse diagnosis (one unarchived run) was not reproduced in 3 controlled runs per arm; the corrected config raised entropy modestly (not significant at n=3). Data in `checkpoints/paper2_ab/`. **Submitted to SSRN 2026-10-05, abstract ID [7567220](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567220) (Under Review).** |
| 3 | [`paper3_cbf_safety_bounds/`](paper3_cbf_safety_bounds/) | Decoupling Safety Guarantees from Policy Competence | SSRN-style | Collision counts do not track competence: 69 of 72 archived runs had no agent contact while coverage spanned 1.1–98.6%; the soft CBF margin was not strictly held (min clearance 0.382 m vs 0.45 m). Rewritten on archived data and current filter; earlier frozen-policy headline withdrawn as unreproducible. Internal research note. |
| 4 | [`paper4_generalization_scaling/`](paper4_generalization_scaling/) | Environment Structure, Not Just Team Size | SSRN-style | Coverage ceiling is a topology property, not a policy property — 5-world, 4-swarm-size sweep. Internal research note. |
| 5 | [`paper5_safe_scalable_architecture/`](paper5_safe_scalable_architecture/) | MARS: A Safety-Filtered Multi-Robot Area-Coverage Testbed with Classical, Random-Walk and MAPPO Controllers | IEEEtran | System architecture and benchmark suite paper on archived data and the current testbed. **Submitted to SSRN 2026-10-05, abstract ID [7567362](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7567362) (Under Review).** |

All PDFs compile cleanly with `pdflatex` (0 errors). Papers 1, 2, and 5 have been submitted to SSRN. Source `.bib` files and `figures/` (where applicable) are included per-paper so each folder is self-contained.

## Data sources behind these papers

- `checkpoints/mappo_multiworld_comparison.json` — 25-run, 5-world MAPPO vs. heuristic matrix
- `checkpoints/sweep_scaling_results.json` — 20-run robot-count scaling sweep
- `checkpoints/run_summary.json` — 24,000-step warehouse ceiling run
- `docs/BENCHMARK_WORLDS.md` — consolidated master verification table
