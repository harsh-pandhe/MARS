# MARS Research Papers

Five papers derived from the MARS multi-robot swarm coverage project, each self-contained (source + compiled PDF + bibliography + figures where used).

| # | Folder | Title | Format | Core Finding |
|---|--------|-------|--------|--------------|
| 1 | [`paper1_negative_result_mappo/`](paper1_negative_result_mappo/) | Diagnosing a Low-Displacement Failure Mode in MAPPO for Multi-Robot Area Coverage | SSRN-style | MAPPO loses to Frontier Heuristic in all 5 tested worlds — the headline negative result, with explicit two-protocol evaluation, ACR definition, reward equation, and references (revised after external review). **Submitted to SSRN 2026-10-03, abstract ID [7556699](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7556699) (under SSRN review).** |
| 2 | [`paper2_sample_efficient_mappo/`](paper2_sample_efficient_mappo/) | Sample-Efficient MAPPO for Physics-in-the-Loop Multi-Robot Training | SSRN-style | Rewritten 2026-10-03 as a failure-to-reproduce report: the original entropy-collapse diagnosis (one unarchived run) was not reproduced in 3 controlled runs per arm; the corrected config raised entropy modestly (not significant at n=3). Data in `checkpoints/paper2_ab/`. Not yet submitted. |
| 3 | [`paper3_cbf_safety_bounds/`](paper3_cbf_safety_bounds/) | Decoupling Safety Guarantees from Policy Competence | SSRN-style | Collision counts do not track competence: 69 of 72 archived runs had no agent contact while coverage spanned 1.1–98.6%; the soft CBF margin was not strictly held (min clearance 0.382 m vs 0.45 m). Rewritten 2026-10-03 on archived data and the current filter; the earlier frozen-policy headline was withdrawn as unreproducible. Not yet submitted. |
| 4 | [`paper4_generalization_scaling/`](paper4_generalization_scaling/) | Environment Structure, Not Just Team Size | SSRN-style | Coverage ceiling is a topology property, not a policy property — 5-world, 4-swarm-size sweep |
| 5 | [`paper5_safe_scalable_architecture/`](paper5_safe_scalable_architecture/) | MARS: A Safety-Filtered Multi-Robot Area-Coverage Testbed with Classical and MAPPO Controllers | IEEEtran (conference) | Rewritten Oct 2026 on archived data (4 pp.); not yet submitted |

All PDFs compile cleanly with `pdflatex` (0 errors) as of the last build. Only Paper 1 has been through the full review-and-correction cycle; Papers 2–5 predate the evaluation non-determinism finding and the n=3 statistical correction and must be audited before any submission. Source `.bib` files and `figures/` (where applicable) are included per-paper so each folder can be zipped and submitted independently to SSRN, arXiv, or a conference.

## Data sources behind these papers

- `checkpoints/mappo_multiworld_comparison.json` — 25-run, 5-world MAPPO vs. heuristic matrix
- `checkpoints/sweep_scaling_results.json` — 20-run robot-count scaling sweep
- `checkpoints/run_summary.json` — 24,000-step warehouse ceiling run
- `docs/BENCHMARK_WORLDS.md` — consolidated master verification table
