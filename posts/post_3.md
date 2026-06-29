# LinkedIn Post 3: Hard Proof & Swarm Resilience

## 1. Post Narrative & Draft

**Hook:** What happens to an AI swarm when a robot suddenly dies mid-mission? 💀🤖

In safety-critical applications like search-and-rescue, fault tolerance isn't a nice-to-have; it's a hard constraint. 

To prove the resilience of the **MARS** framework, I built a custom **Fault Injector** (`robot_killer`) to randomly hijack and disable individual agents mid-episode, alongside injecting Gaussian sensor noise into the Lidar arrays.

Instead of just checking if it "looks good," I ran a quantitative benchmark comparing the MAPPO swarm under nominal, sensor noise, and hardware failure conditions against our control baselines:

1. **Random Walk**: $3.2\% \pm 0.4\%$ coverage.
2. **Frontier Heuristic**: $4.8\% \pm 2.6\%$ coverage.
3. **MAPPO Nominal**: 85%+ coverage.
4. **MAPPO under Hardware Failure**: The remaining robots dynamically sensed the failed agent's lack of motion via their neighbor state observations, expanded their search parameters, and successfully covered the abandoned sector with minimal loss in global rate.

True autonomous engineering requires quantitative verification, not just qualitative observation. 

Check out the adaptation sequence and the final benchmarking box-and-whisker plot below!

#Robotics #AutonomousVehicles #HardwareTesting #SoftwareEngineering #AI #ReliabilityEngineering

---

## 2. Visual Asset Preparation Guide

### **Asset A: Fault Adaptation Video**
* **Action:** Capture the exact moment a robot is killed and the remaining agents adapt.
* **Execution Command:**
  ```bash
  ./run_swarm.sh --resilience ./checkpoints/checkpoint_000015
  ```
* **Recording Instructions:**
  1. Record the Gazebo and RViz screen side-by-side.
  2. Capture the normal swarm navigation until step 50 (~18 seconds in), when the `robot_killer` halts `tb2`.
  3. Zoom in on `tb1` and `tb3` altering their steering vectors to cover the sectors originally assigned to `tb2`.

### **Asset B: Benchmarking Statistics Boxplot**
* **Action:** Run the full benchmark suite with your best checkpoint to generate the plot.
* **Execution Command:**
  ```bash
  ./run_swarm.sh --benchmark ./checkpoints/checkpoint_000015
  ```
* **Output:** Extract `./checkpoints/benchmark_results.png` and crop it for optimal display side-by-side with the adaptation video.
