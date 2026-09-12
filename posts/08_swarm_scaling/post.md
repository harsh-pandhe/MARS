# Post 8 — More Robots ≠ Linear Improvement (Sometimes It's Way Better)

**Attach:** simple line/bar chart of ACR vs. robot count for office & depot worlds (build from `checkpoints/sweep_scaling_results.json`)

---

Common intuition: double your robots, roughly double your coverage speed. Diminishing returns after that.

We tested 2, 3, 5, and 8 robots across 5 environments to check. In most worlds, that intuition held. In two, it didn't even come close.

In our office-floor environment, going from 5 to 8 robots didn't nudge coverage up a bit — it jumped 3.7x (12.7% → 46.8%) in the same time budget. Same jump pattern in our depot world.

Our best guess: below a certain robot count, a partitioning-based coordination strategy can't actually split the space into separate zones — there just aren't enough agents to claim every room simultaneously. Cross that threshold, and suddenly every hallway and office gets a robot at once instead of robots queuing through doorways one at a time.

The unglamorous takeaway: "how many robots do I need" isn't a smooth curve you can extrapolate — it can have a cliff, and you only find it by actually testing the counts in between.

#SwarmRobotics #MultiRobotSystems #Robotics #Scalability
