# Post 2 — It Wasn't Just One World

**Attach:** table image or simple bar chart built from this data (5 worlds side by side) — or use `docs/BENCHMARK_WORLDS.md` Section 10 table as a screenshot

---

When our RL policy lost to a classical heuristic, the obvious question was: "did you just get unlucky with one map?"

So we ran it again. And again. Five completely different environments:
🏢 a furnished café
🏭 an open logistics warehouse
📦 an industrial depot with pillars
🚪 a partitioned office floor
🌀 a narrow-corridor maze

Same result, every single time. The classical Frontier Heuristic beat our trained MAPPO policy in all 5 environments — margins from 1.3x to 2.7x.

That consistency is actually the interesting part. If it failed on just one map, you could blame the map. When it fails everywhere, you've found something structural: dense collision penalties + sparse exploration rewards reliably teaches a robot to stay put, regardless of what the room looks like.

This is the kind of result you only get by refusing to stop at the first convenient explanation.

#RoboticsResearch #Gazebo #ROS2 #SwarmRobotics #AIResearch
