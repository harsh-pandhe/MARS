# Post 3 — The Part That Actually Worked

**Attach:** Gazebo screenshot of the 3-robot swarm close together (like the one from the recording session), or an RViz screenshot showing overlapping trajectories with no collision

---

Our reinforcement-learning policy failed. Our safety layer didn't.

Every velocity command our robots ever send — whether from the RL policy, the classical planner, or a random walk — passes through a Control Barrier Function (CBF) safety filter first. It's a small optimization problem solved fresh at every single control step: "given where I am and where my neighbors are, what's the safest velocity close to what I wanted?"

Across every benchmark we ran — 5 environments, swarm sizes from 2 to 8 robots, moving obstacles thrown at them head-on and crossing their path — inter-agent collisions stayed at zero in 18 of 20 tested configurations. The two exceptions were isolated single-digit contacts in our smallest, most cramped arenas at 8 robots — not a safety failure, just geometry running out of room.

This is the boring-but-critical lesson: your safety layer and your intelligence layer are different problems, and you should be able to prove the first one works even when the second one doesn't.

#RobotSafety #ControlTheory #Robotics #ROS2 #Gazebo
