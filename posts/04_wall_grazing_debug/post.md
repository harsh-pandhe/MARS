# Post 4 — The Bug That Looked Like a Safety Failure (But Wasn't)

**Attach:** `docs/heatmaps/depot_extended_heatmap.png` (or a before/after pair if you generate one — depot before fix vs after)

---

Our depot-world benchmark showed 3,144 "collisions" in a single run. Alarming number.

Turns out: almost none of them were real. Here's the debugging trail.

Our collision counter tripped whenever the LiDAR's minimum reading dropped below a threshold. But the LiDAR sensor sits recessed inside the robot's chassis — so a completely safe robot, sitting 14cm from a wall, still reads "0.14m to nearest obstacle" and gets flagged as a collision.

We were measuring sensor geometry, not safety.

Once we split the metric into "wall grazing" (sensor proximity) vs. "actual agent-agent contact," the real picture emerged: 0 robot-to-robot collisions, and a wall-proximity number that dropped 85% (3,144 → 477) once we fixed the actual velocity-damping issue near obstacles.

The lesson: before you trust a scary number, ask what it's actually measuring.

#Debugging #Robotics #SoftwareEngineering #ROS2
