# Post 3 — The Part That Actually Worked

**Attach:** Gazebo screenshot of the 3-robot swarm close together (like the one from the recording session), or an RViz screenshot showing overlapping trajectories with no collision

---

Our reinforcement-learning policy failed. Our safety layer didn't.

Every velocity command our robots ever send — whether from the RL policy, the classical planner, or a random walk — passes through a Control Barrier Function (CBF) safety filter first. It's a small optimization problem solved fresh at every single control step: "given where I am and where my neighbors are, what's the safest velocity close to what I wanted?"

Across every benchmark we ran — 5 environments, swarm sizes from 2 to 8 robots, moving obstacles thrown at them head-on and crossing their path — inter-agent collisions stayed at zero in 17 of 20 tested configurations. The three exceptions were isolated single-digit contacts in our two smallest, most cramped arenas at 5 and 8 robots. Our best guess is that the safety margins take up more of the free space as the arena fills, but we haven't tested that.

This is the boring-but-critical lesson: your safety layer and your intelligence layer are different problems, and you should be able to measure the first one even when the second one doesn't work. And a zero-collision record isn't proof: our filter is a soft constraint, and in one run the robots got closer than the margin we set without touching.

#RobotSafety #ControlTheory #Robotics #ROS2 #Gazebo
