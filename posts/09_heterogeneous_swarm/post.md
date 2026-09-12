# Post 9 — Two Different Robots, One Swarm, Zero Extra Code Changes to the Safety Layer

**Attach:** Gazebo screenshot showing both robot models (TurtleBot3 Waffle + Pioneer 2DX) in the same scene

---

Most multi-robot demos use identical robots. Real fleets rarely do.

We wanted to know: does our safety and coordination stack actually care what robot it's controlling, or is it accidentally hard-coded to one chassis?

So we downloaded a completely different robot model (a Pioneer 2DX, different footprint, different sensor mounting) from an open model repository, wired up its differential-drive kinematics and LiDAR, and dropped it into a swarm alongside our usual TurtleBot3 — no changes to the collision-avoidance or coordination logic.

It worked. Both robot types explored the same space, respected the same safety margins, and reported clean telemetry — 0 wall and 0 agent collisions in the joint run.

Small result, but an important property: if your safety layer needs to be rewritten every time your hardware changes, it's not really a safety layer — it's a script.

#Robotics #MultiRobotSystems #Gazebo #ROS2
