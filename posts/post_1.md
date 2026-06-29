# LinkedIn Post 1: The Greedy Convergence Bottleneck

## 1. Post Narrative & Draft

**Hook:** Traditional robotics path planning works beautifully for a single robot. But when you scale to a swarm, classical math breaks down. Here is why. 👇

If you deploy three autonomous robots to cooperatively map or search an area, the most common trap is letting them plan independently. In my multi-agent robot swarm (**MARS**) project, I benchmarked a classical **Frontier Heuristic** (where each agent greedily drives to the nearest unvisited sector). 

The result? **Greedy Convergence.**

Because individual robots select targets based only on local distance:
1. They target the exact same nearby unvisited sectors.
2. They cluster together, leading to high path overlap and redundant sensor sweeps.
3. Their area coverage rate plateaus, wasting time and energy.

To solve this, we need the swarm to coordinate dynamically—not by centralizing decisions, but by training decentralized policies that learn cooperative spatio-temporal spacing.

In my next post, I'll share how I bridged **ROS 2** and **Ray RLlib** to train a decentralized policy using PyTorch and **MAPPO** that naturally coordinates space partitioning.

Check out the architecture and the failure mode below!

#Robotics #ReinforcementLearning #ROS2 #AI #AutonomousSystems #SwarmRobotics

---

## 2. Visual Asset Preparation Guide

### **Asset A: System Architecture Diagram**
* **Source:** The Mermaid diagram located in the root `README.md`.
* **Action:** Convert the Mermaid syntax to a high-resolution SVG or PNG using a Mermaid live editor or rendering tool. Use this as the first slide of the post carousel.

### **Asset B: Greedy Frontier Exploration Failure Video**
* **Action:** Run the Heuristic baseline in GUI mode and record the robots clustering and overlapping.
* **Execution Command:**
  ```bash
  ./run_swarm.sh --benchmark
  ```
* **Recording Instructions:**
  1. Set the Gazebo simulation GUI and RViz window side-by-side.
  2. Capture a 15-second screen recording showing `tb1`, `tb2`, and `tb3` steering towards the same sector, causing their paths to cross and cluster.
  3. Speed up the final video by 2x for a punchy, highly engaging visual.
