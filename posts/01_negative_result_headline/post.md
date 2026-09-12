# Post 1 — The Headline Negative Result

**Attach:** `checkpoints/benchmark_results.png` (box plot)

---

We trained a multi-robot swarm with reinforcement learning. It learned to sit still.

Three robots, one warehouse, one job: explore as much floor space as possible. We trained a Multi-Agent PPO policy for 45 iterations — the standard approach you'd read about in a dozen papers.

Then we benchmarked it against a 1980s-era idea: frontier exploration + A* pathfinding. No learning. No neural net. Just "go to the nearest unexplored cell."

The heuristic won. Not by a little.

→ Frontier Heuristic: 38.6% median coverage
→ Random Walk (no strategy at all): 29.6%
→ Our trained RL policy: 14.5%

Our "smart" policy lost to random noise.

Here's why: it learned that standing still and rotating slowly minimizes collision penalties. Moving is risky — bumping a wall costs points, discovering a cell earns a tiny fraction of a point. So it did the rational thing under that reward function: nothing.

We didn't hide this. We ran it to full training convergence, evaluated it properly across 50 episodes, and reported the number we got — not the number we wanted.

Most posts about robotics + RL show the win. This is the honest version.

#Robotics #ReinforcementLearning #MultiAgentSystems #NegativeResults #ROS2
