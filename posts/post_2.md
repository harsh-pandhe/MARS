# LinkedIn Post 2: The Decentralized Solution (MARL + ROS 2)

## 1. Post Narrative & Draft

**Hook:** How do you teach three independent robots to coordinate and share space without a central controller? 🤖🛰️

By treating the swarm as a Multi-Agent Reinforcement Learning (**MARL**) environment. 

In my **MARS** project, I wrapped the ROS 2 Jazzy and Gazebo Sim pipeline in a custom **PettingZoo ParallelEnv**. This allowed us to train the swarm using **MAPPO** (Multi-Agent PPO) under Ray RLlib. 

Here is how the training was structured:
* **The Observation Space (32-Dim):** Each robot receives a local 24-sector Lidar scan, relative goal vectors, command velocities, and crucially, relative coordinate offsets to its nearest neighbors.
* **Shared Policy Training:** Since the robots are homogeneous, they share weights in a single policy network, speeding up training times.
* **Cooperative Reward Shaping:** The swarm receives a collective reward for discovering new sectors on a $10 \times 10$ global grid, while being penalized for entering one another's proximity safety buffer.

The result? **Emergent Repulsion.** Without being explicitly coded to avoid each other, the robots learned to partition the room smoothly to maximize area coverage.

Check out the Gazebo physical simulation and the RViz Lidar projection running the policy!

#Robotics #ReinforcementLearning #ArtificialIntelligence #DeepLearning #ROS2 #Python
