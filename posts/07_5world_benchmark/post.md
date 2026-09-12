# Post 7 — 5 Worlds, 20 Robots, 25 Benchmark Runs

**Attach:** collage of the 4 heatmap PNGs (`cafe_coverage_heatmap.png`, `depot_extended_heatmap.png`, `office_extended_heatmap.png`, `warehouse_demo_heatmap.png`) arranged as a 2x2 grid

---

What does "coverage" even mean when every room is a different shape?

We built (and downloaded) 5 completely different simulated environments to stress-test our multi-robot swarm: a café, a warehouse, an industrial depot, an office floor, and a maze. Same 3 robots, same algorithm, wildly different results:

🏭 Warehouse (open floor plan): 100% coverage achievable
🚪 Office (partitioned rooms): 98% achievable
📦 Depot (pillars + boxes): 86% achievable
☕ Café (furniture-packed): 56% ceiling — the rest is physically furniture
🌀 Maze (labyrinth): 17% — and that's actually ~100% of the walkable corridor space

The lesson that took us a few false starts to learn: a "56% coverage" number is meaningless without knowing whether that's a bad algorithm or a room that's 44% furniture. We had to run each world for far longer than expected just to find its true ceiling before we could compare anything fairly.

Attached: coverage heatmaps from four of the five worlds — you can see exactly where the robots went and where the walls stopped them.

#Robotics #Gazebo #SimulationEngineering #SwarmRobotics
