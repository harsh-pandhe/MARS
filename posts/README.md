# MARS LinkedIn Post Series (10 Posts)

Each folder has a `post.md` with ready-to-paste copy and a specific attachment instruction. Suggested posting order below (roughly strongest → supporting).

| # | Folder | Angle | Attachment |
|---|--------|-------|-----------|
| 1 | [01_negative_result_headline](01_negative_result_headline/) | The headline: RL loses to a heuristic | `checkpoints/benchmark_results.png` |
| 2 | [02_frontier_vs_mappo_median](02_frontier_vs_mappo_median/) | It holds across 5 environments, not one | Table screenshot / bar chart |
| 3 | [03_zero_collision_cbf](03_zero_collision_cbf/) | The safety layer that *did* work | Gazebo/RViz screenshot of close-proximity robots |
| 4 | [04_wall_grazing_debug](04_wall_grazing_debug/) | Debugging a scary-looking metric | `docs/heatmaps/depot_extended_heatmap.png` |
| 5 | [05_ray_gcs_warfstory](05_ray_gcs_warfstory/) | Infra war story: Wi-Fi killed training | none / terminal screenshot |
| 6 | [06_torchdynamo_segfault](06_torchdynamo_segfault/) | Infra war story: PyTorch segfault | none / stack trace screenshot |
| 7 | [07_5world_benchmark](07_5world_benchmark/) | 5-world coverage comparison | 2x2 heatmap collage |
| 8 | [08_swarm_scaling](08_swarm_scaling/) | Non-linear scaling with robot count | Bar/line chart from sweep data |
| 9 | [09_heterogeneous_swarm](09_heterogeneous_swarm/) | Heterogeneous robot swarm | Gazebo screenshot, two robot types |
| 10 | [10_ai_verification_meta](10_ai_verification_meta/) | Meta: verifying AI-assisted claims | none / quote card |

**Suggested cadence:** 1 every 2-3 days, starting with #1 (highest engagement potential — negative results and honesty angles consistently outperform generic wins). Save #10 for after a few technical posts land, since it works best once you have credibility from the technical content.

**Images not yet created:** the bar/line charts referenced in posts 2 and 8 need to be built from `checkpoints/mappo_multiworld_comparison.json` and `checkpoints/sweep_scaling_results.json` respectively — say the word and I'll generate them.
