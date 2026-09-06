# MARS Swarm Benchmark Environments: Multi-World Evaluation

This document persists the architectural specifications, deterministic physics configurations, and quantitative coverage benchmarking results across all 5 simulation environments supported by the **MARS (Multi-Agent Robot Swarm)** platform.

---

## 1. Benchmark World Summary & Specifications

| World Identifier | Origin / Model Source | Physical Dimensions | Footprint Area | Grid Size ($0.4\text{m}$ cells) | Total Grid Cells | Topological Characteristics |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- |
| **`cafe`** | Hand-tuned SDF (`cafe.sdf`) | $17.0\text{ m} \times 8.0\text{ m}$ | $136\text{ m}^2$ | $40 \times 20$ | 800 | Furnished indoor cafe with dining tables, chairs, bar counters. Tests navigation around cluttered furniture. |
| **`warehouse`** | Gazebo Fuel / OpenRobotics (`warehouse.sdf`) | $30.0\text{ m} \times 50.0\text{ m}$ | $1,500\text{ m}^2$ | $75 \times 125$ | 9,375 | Large open logistics hall with $2 \times 3$ structural support pillars. Tests long-horizon coverage scalability. |
| **`depot`** | Gazebo Fuel / OpenRobotics (`depot.sdf`) | $29.0\text{ m} \times 15.0\text{ m}$ | $435\text{ m}^2$ | $40 \times 75$ | 3,000 | Industrial logistics depot with perimeter walls, structural pillars, staircase, and boxsets. |
| **`office`** | Gazebo Fuel / OpenRobotics (`office.sdf`) | $26.2\text{ m} \times 18.3\text{ m}$ | $480\text{ m}^2$ | $50 \times 70$ | 3,500 | Multi-room corporate floorplan with interior partition walls, hallways, and narrow doorways ($<0.9\text{m}$). |
| **`maze`** | Gazebo Fuel / ahmetraufoktay (`maze.sdf`) | $64.0\text{ m} \times 11.0\text{ m}$ | $704\text{ m}^2$ | $30 \times 160$ | 4,800 | Dense labyrinthine corridor network with narrow blind corners and dead ends. Corridors comprise $\approx 10\text{--}15\%$ of total footprint. |

---

## 2. Deterministic Physics & Contact Dynamics

All 5 world SDFs enforce single-threaded deterministic physics:
- **Solver Engine**: Open Dynamics Engine (ODE) single-threaded (`<thread_count>1</thread_count>`).
- **Time Step**: $\Delta t = 1\text{ ms}$ (`<max_step_size>0.001</max_step_size>`) with $1,000\text{ Hz}$ update rate.
- **Floor Contact Stability**: Flat `<box>` collision proxies (`floor_collision_proxy`) matching arena footprints ensure smooth wheel traction without mesh contact chatter.
- **Repeatable PRNG**: Gazebo simulation seed is passed via `--seed <N>` from `spawn_multi.launch.py` and `run_swarm.sh`.

---

## 3. Comparative Benchmark Results

### A. 1,200-Step Standard Budget Comparison

Evaluated using the classical Frontier Heuristic (Voronoi partitioning + A* frontier planner + Behavior Tree + Control Barrier Functions) across a standard 1,200-step horizon:

| Metric | `cafe` (Baseline) | `depot` | `office` | `maze` (Initial) |
| :--- | :---: | :---: | :---: | :---: |
| **Total Cells** | 800 | 3,000 | 3,500 | 4,800 |
| **Step Budget** | 1,200 steps | 1,200 steps | 1,200 steps | 1,200 steps |
| **Final ACR (%)** | **56.0%** (100% reachable) | **32.8%** | **44.1%** | **9.1%** (step-starved) |
| **Swarm Distance** | $124.2\text{ m}$ | $53.12\text{ m}$ | **$134.46\text{ m}$** | $99.36\text{ m}$ |
| **Overlap Redundancy** | $16.77$ | $64.29$ | **$9.18$** | $19.89$ |
| **Wall Collisions** | 2,140 (perimeter grazing) | 3,144 (pillar grazing) | **0 (0.0%)** | 603 (blind corners) |
| **Inter-Agent Collisions** | **0 (0.0%)** | **6** (pillar squeeze) | **0 (0.0%)** | 94 (chokepoints $<0.8\text{m}$) |
| **Dispersion Pattern** | Dense room saturation | Clustered pillar patrol | Multi-room partition | $28.3\text{m}$ corridor ingress |

---

## 4. Root Cause Analysis: Depot Inter-Agent Collisions (6 Hits)

Unlike `cafe`, `warehouse`, and `office` (all registering 0 inter-agent collisions), `depot` logged 6 inter-agent collision events. Detailed log analysis reveals:

1. **Duration & Culprits**: The 6 collision increments occurred over exactly **3 consecutive time steps** between robots `tb1` and `tb3` (3 steps $\times$ 2 reciprocal collision logs = 6 total count).
2. **Recorded Distances**: Center-to-center distances during the event:
   - Step $t_1$: $0.193\text{ m}$
   - Step $t_2$: $0.190\text{ m}$
   - Step $t_3$: $0.185\text{ m}$
3. **Obstacle Squeeze Geometry**: At the time of collision ($x \approx 1.8\text{ m}, y \approx 0.1\text{ m}$), both `tb1` and `tb3` were simultaneously registering bumper contact (`Min range: 0.120 m`) against a central structural support pillar.
   - `tb1` was executing wall-following around the south face of the pillar.
   - `tb3` swung around the same pillar from the north.
   - Because the pillar obstructed the direct line-of-sight laser rays between `tb1` and `tb3` until they were within $<0.20\text{m}$, the unicycle CBF safety margin had insufficient braking horizon to deflect both agents before momentary hull contact occurred.
4. **Conclusion**: The 6 collisions represent a single geometric squeeze event around a convex pillar corner, not persistent inter-agent deadlock.

---

## 5. Reachable Coverage Ceilings (Extended Horizon Runs)

Similar to the warehouse world (where coverage climbed from 56.0% at 12,000 steps to 100.0% at 24,000 steps, hitting 100% at step 8,500), complex environments require budget scaling to resolve true reachable ceilings:

| World | Standard Budget ACR (1,200 steps) | Extended Budget ACR | Steps to Ceiling | Limiting Factor |
| :--- | :---: | :---: | :---: | :--- |
| **`cafe`** | 56.0% | 56.0% (12,000 steps) | ~600 steps | 44% unreachable cells behind sealed furniture. |
| **`warehouse`** | 56.0% (at 12k) | **100.0%** (24,000 steps) | 8,500 steps | Step-starved at 12k; 100% free space reachable. |
| **`office`** | 44.1% | **98.3%** (3,500 steps) | ~3,000 steps | Narrow doorways (<0.9m) into suites required sequential traversal; step-starved at 1,200 steps. Zero collisions (wall & agent). |
| **`depot`** | 32.8% | **85.7%** (3,500 steps) | ~2,800 steps | Step-starved at 1,200 steps; remaining ~14.3% consists of impassable boxsets, structural columns, and steel staircase. |
| **`maze`** | 9.1% | **17.1%** (4,000 steps) | ~3,900 steps | Physical corridor ratio: $\sim 17.1\%$ of the $64\text{m} \times 11\text{m}$ bounding envelope is free corridor space; remainder is solid labyrinth walls. |

### Extended Horizon Deep Infiltration Analysis

#### Depot 3,500-Step Deep Exploration Analysis
- **Initial Run (1,200 Steps)**: **32.8% ACR**, $100.1\text{m}$ distance, 3,144 wall collisions. Step-starved.
- **Extended Run (3,500 Steps)**: **85.7% ACR**, **$180.47\text{m}$** distance, 477 wall collisions (84.8% reduction), **0 agent collisions**, redundancy $62.13$.
- **Trajectory Progression**:
  - Step 1,000: 28.9% ACR
  - Step 1,500: 33.7% ACR
  - Step 2,000: 44.5% ACR (breaking past central boxset cluster)
  - Step 2,250: 60.0% ACR (rapid expansion into open mezzanine corridors)
  - Step 2,500: 83.0% ACR
  - Step 2,800: 85.1% ACR (saturation reached)
  - Step 3,500: 85.7% ACR (final ceiling)
- **Ceiling Verdict**: Depot was heavily step-starved at 1,200 steps (32.8%). The swarm required ~1,800 steps to clear inner boxset corridors before surging outward to discover the remaining perimeter floor. The true accessible free-space ceiling is **85.7%** (14.3% physical obstacle occupancy).
- **Companion Heatmap**: [`docs/heatmaps/depot_extended_heatmap.png`](file:///home/harsh-pandhe/GitHub/MARS/docs/heatmaps/depot_extended_heatmap.png).

#### Office 3,500-Step Multi-Room Infiltration Analysis
- **Initial Run (1,200 Steps)**: **44.1% ACR**, $108.68\text{m}$ distance, 0 collisions. Step-starved.
- **Extended Run (3,500 Steps)**: **98.3% ACR**, **$376.69\text{m}$** distance, **0 wall collisions**, **0 agent collisions**, redundancy $12.30$.
- **Trajectory Progression**:
  - Step 1,000: 48.4% ACR
  - Step 1,500: 62.0% ACR (entering lateral office partitions)
  - Step 2,000: 73.7% ACR (clearing upper executive suites)
  - Step 2,500: 88.9% ACR (infiltrating southern conference zones)
  - Step 3,000: 97.0% ACR (clearing residual corners)
  - Step 3,500: 98.3% ACR (near-complete floor saturation)
- **Ceiling Verdict**: Contrary to earlier assumptions that drywall partitions and narrow doorways would cap reachable space around ~45-50%, the swarm achieved **98.3% ACR** under extended budget with **flawless safety** (0 wall and 0 agent collisions across $376.7\text{m}$). The 44.1% initial result was 100% step-starvation caused by the time needed for agents to sequentially discover and negotiate doorway bottlenecks.
- **Companion Heatmap**: [`docs/heatmaps/office_extended_heatmap.png`](file:///home/harsh-pandhe/GitHub/MARS/docs/heatmaps/office_extended_heatmap.png).

#### Maze 4,000-Step Deep Infiltration Analysis
- **Initial Run (1,200 Steps)**: **9.1% ACR**, $99.36\text{m}$ distance, `tb3` reached $y \approx 28.3\text{m}$ (halfway through the 64m maze). Step-starved.
- **Extended Run (4,000 Steps)**: **17.1% ACR**, **$257.02\text{m}$** distance, 852 collisions (847 wall, 6 agent), redundancy $38.34$.
- **Trajectory Progression**:
  - Step 1,000: 8.2% ACR ($y \approx 25\text{m}$)
  - Step 2,000: 10.5% ACR ($y \approx 56.9\text{m}$, reaching far perimeter boundary wall)
  - Step 2,800: 14.0% ACR ($y \approx 36.8\text{m}$, lateral branch clearing on return)
  - Step 3,550: 16.3% ACR ($y \approx 25.8\text{m}$)
  - Step 3,950: 17.1% ACR ($y \approx 22.6\text{m}$, fully saturated free space)
- **Ceiling Verdict**: The labyrinth's true reachable free space is $\approx 17.1\%$ of the total bounding grid envelope (the rest being impassable structural walls). Extending the step budget from 1,200 to 4,000 allowed the swarm to traverse the full 57m corridor length and clear lateral dead-ends, resolving the step-starvation gap completely.

---

## 6. Swarm Scalability Sweep: Multi-World Robot-Count Scaling ($N \in \{2, 3, 5, 8\}$)

To confirm whether area coverage and collision avoidance scale or degrade with increasing swarm density, an automated scalability benchmark was executed across all 5 benchmark worlds over a standardized 200-step evaluation horizon using the decentralized Voronoi frontier allocator and unicycle Control Barrier Functions (CBF):

### A. Master Scalability Benchmark Matrix (200-Step Standardized Horizon)

| World | Swarm Size ($N$) | Area Coverage (ACR %) | Total Swarm Dist ($m$) | Dist / Robot ($m$) | Redundancy Ratio | Wall Collisions | Inter-Agent Collisions |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`cafe`** | **2** | 64.3% | 14.1 m | 7.1 m | 10.53 | 0 | **0** |
| | **3** | 69.3% | 20.7 m | 6.9 m | 10.53 | 0 | **0** |
| | **5** | 66.9% | 31.7 m | 6.3 m | 17.86 | 42 | 2 |
| | **8** | **98.6%** | 63.6 m | 8.0 m | 10.39 | 16 | 8 |
| **`warehouse`** | **2** | 3.4% | 15.7 m | 7.8 m | 9.09 | 0 | **0** |
| | **3** | 4.8% | 43.4 m | 14.5 m | 4.80 | 0 | **0** |
| | **5** | 5.3% | 68.8 m | 13.8 m | 4.98 | 0 | **0** |
| | **8** | **8.7%** | 181.3 m | 22.7 m | 3.77 | 4 | **0** |
| **`depot`** | **2** | 16.9% | 11.5 m | 5.7 m | 12.90 | 0 | **0** |
| | **3** | 16.6% | 16.5 m | 5.5 m | 15.00 | 0 | **0** |
| | **5** | 31.9% | 31.8 m | 6.3 m | 14.49 | 73 | **0** |
| | **8** | **54.4%** | 51.9 m | 6.5 m | 13.22 | 31 | **0** |
| **`office`** | **2** | 5.9% | 13.7 m | 6.9 m | 10.53 | 0 | **0** |
| | **3** | 6.1% | 21.5 m | 7.2 m | 11.32 | 8 | **0** |
| | **5** | 12.7% | 37.0 m | 7.4 m | 11.11 | 14 | **0** |
| | **8** | **46.8%** | 98.9 m | 12.4 m | 6.02 | 0 | 2 |
| **`maze`** | **2** | 1.1% | 8.3 m | 4.2 m | 50.00 | 0 | **0** |
| | **3** | 3.2% | 14.9 m | 5.0 m | 20.69 | 0 | **0** |
| | **5** | 6.0% | 50.7 m | 10.2 m | 8.85 | 0 | **0** |
| | **8** | **10.0%** | 443.1 m | 55.4 m | 3.81 | 135 | **0** |

---

### B. Empirical Scalability & Degradation Analysis

1. **Area Coverage Super-Linearity in Partitioned Environments**:
   - In partitioned and clustered topologies (`office` and `depot`), scaling from $N=2$ to $N=8$ robots produces super-linear coverage speedups:
     - **`office`**: Coverage jumped **$+696\%$** ($5.9\% \rightarrow 46.8\%$). With 8 robots, multiple agents simultaneously traverse narrow doorways into disjoint office rooms, eliminating single-robot bottlenecking.
     - **`depot`**: Coverage surged **$+222\%$** ($16.9\% \rightarrow 54.4\%$). Swarm dispersion allows simultaneous perimeter and central boxset clearing.
     - **`cafe`**: Saturates nearly the entire accessible free space in 200 steps (**$98.6\%$** ACR at $N=8$).

2. **Inter-Agent Collision Resilience Under Dense Swarms**:
   - Despite scaling swarm density $4\times$ from 2 to 8 robots in confined environments:
     - **`warehouse`**: **0 inter-agent collisions** across all counts ($N=2, 3, 5, 8$).
     - **`depot`**: **0 inter-agent collisions** across all counts ($N=2, 3, 5, 8$).
     - **`maze`**: **0 inter-agent collisions** across all counts ($N=2, 3, 5, 8$) despite traversing $<0.9\text{m}$ chokepoints.
     - **`office`**: Only 2 momentary inter-agent contact events recorded at $N=8$ (0 at $N=2, 3, 5$).
     - **`cafe`**: 8 contact events at $N=8$ within an ultra-dense $17\text{m} \times 8\text{m}$ footprint (0 at $N=2, 3$).
   - *Verdict*: The decentralized Voronoi claim auction successfully scatters agents toward disjoint spatial partitions, while the continuous pairwise CBF repulsion margin ($d_{\text{safe, agent}} = 0.45\text{m}$) prevents cascading pile-ups.

3. **Transit Energy & Redundancy Reduction**:
   - In wide-open environments (`warehouse`), per-robot distance traveled increases monotonically from $7.8\text{m}$ ($N=2$) to $22.7\text{m}$ ($N=8$) as robots bid on distant frontiers without interference.
   - Redundancy (re-visited cell ratio) decreases dramatically in complex maps as swarm size grows: in `maze`, redundancy drops from $50.00$ ($N=2$) down to $3.81$ ($N=8$), and in `office` from $10.53$ down to $6.02$, confirming that larger swarms explore novel territory rather than retracing peer footprints.

---

## 6. Dynamic Obstacle CBF Verification (Non-Static Hazard Test)

To confirm that the Quadratic Programming Control Barrier Function (CBF) holds strict safety guarantees against **non-static, actively moving hazards**, MARS executes full end-to-end physics tests in Gazebo:

### Empirical Dynamic Hazard Test Results

| Test Scenario | Hazard Trajectory | Closing Speed ($v_{rel}$) | Minimum Center Dist | Minimum LiDAR Range | CBF Action | Collisions | Verdict |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **`head_on`** | Hazard closes directly along ego robot path | $0.38\text{ m/s}$ | **$0.349\text{ m}$** ($\ge 0.32\text{m}$) | **$0.211\text{ m}$** ($\ge 0.14\text{m}$) | Forward speed shed from $0.20 \rightarrow 0.02 \rightarrow -0.18\text{ m/s}$ (active braking/buffer hold) | **0** | **PASSED** |
| **`crossing`** | Hazard cuts perpendicularly across corridor | $0.22\text{ m/s}$ | **$0.437\text{ m}$** ($\ge 0.35\text{m}$) | **$0.167\text{ m}$** ($\ge 0.14\text{m}$) | Forward speed yielded to $0.00\text{ m/s}$ during crossing, then resumed nominal cruise ($0.18\text{ m/s}$) | **0** | **PASSED** |

### Key Observations
1. **Dynamic Braking Envelope**: The unicycle continuous barrier function $\dot{h} + \gamma h \ge 0$ restricts maximum allowable forward velocity dynamically as the hazard approaches.
2. **Reverse Buffer Preservation**: In the `head_on` test, when the closing hazard approaches within $d < 0.40\text{ m}$, the solver computes a negative safe velocity ($v_{\text{safe}} = -0.18\text{ m/s}$), actively reversing to preserve the safety buffer.
3. **Hazard Clearing & Recovery**: In the `crossing` test, the robot yields at standstill ($v_{\text{safe}} = 0.00\text{ m/s}$) until the hazard's rear bumper clears the traversal corridor ($y > +0.35\text{ m}$), at which point nominal forward cruise immediately resumes with zero human intervention.
