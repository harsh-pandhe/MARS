import os
import sys
import time
import math
import heapq
import signal
import argparse
import numpy as np
import matplotlib.pyplot as plt

# Ensure path includes workspace packages
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from multi_env_wrapper import PettingZooSwarmEnv
from train_multi import start_gazebo, kill_stale_processes, gazebo_process


def build_obstacle_grid(env):
    """Downsample env.viz_grid (high-res) to the coverage grid resolution.
    A coverage cell is an obstacle if any of its high-res sub-cells is a
    known wall (value 100). Computed once per step and shared across agents
    to avoid the O(cells * ratio_x * ratio_y) cost being paid per-candidate.
    """
    ratio_x = env.viz_resolution_x // env.grid_resolution_x
    ratio_y = env.viz_resolution_y // env.grid_resolution_y
    obstacle = np.zeros((env.grid_resolution_y, env.grid_resolution_x), dtype=bool)
    for r in range(env.grid_resolution_y):
        for c in range(env.grid_resolution_x):
            block = env.viz_grid[r*ratio_y:(r+1)*ratio_y, c*ratio_x:(c+1)*ratio_x]
            if np.any(block == 100):
                obstacle[r, c] = True
    return obstacle


def inflate_obstacles(obstacle_grid, radius=1):
    """Dilate obstacle cells by `radius` so A* paths keep a clearance margin
    instead of hugging wall cells (grid cells are ~0.4m, well above the 0.20m
    LiDAR collision threshold, so hugging a wall cell readily trips it)."""
    h, w = obstacle_grid.shape
    inflated = obstacle_grid.copy()
    obstacle_rc = np.argwhere(obstacle_grid)
    for r, c in obstacle_rc:
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w:
                    inflated[nr, nc] = True
    return inflated


def line_of_sight_clear(obstacle_grid, start_rc, goal_rc):
    """Sample points along the straight line between two cells; True if none
    of them fall on an obstacle cell. Used to decide whether A* re-routing is
    needed at all, so the common unobstructed case is untouched."""
    h, w = obstacle_grid.shape
    r0, c0 = start_rc
    r1, c1 = goal_rc
    n = max(abs(r1 - r0), abs(c1 - c0), 1) * 2
    for i in range(n + 1):
        t = i / n
        r = int(round(r0 + (r1 - r0) * t))
        c = int(round(c0 + (c1 - c0) * t))
        r = min(max(r, 0), h - 1)
        c = min(max(c, 0), w - 1)
        if obstacle_grid[r, c]:
            return False
    return True


def astar_path(obstacle_grid, start_rc, goal_rc):
    """8-connected A* over the coverage grid. Returns a list of (r, c) cells
    from start to goal inclusive, or None if no path exists. Grid is small
    (~800 cells) so this is cheap enough to run fresh every step per agent."""
    h, w = obstacle_grid.shape
    if obstacle_grid[goal_rc[0], goal_rc[1]]:
        return None

    def heuristic(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    neighbors = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
                 (-1,-1,1.41421356),(-1,1,1.41421356),(1,-1,1.41421356),(1,1,1.41421356)]
    open_set = [(heuristic(start_rc, goal_rc), 0.0, start_rc)]
    came_from = {}
    g_score = {start_rc: 0.0}
    visited = set()

    while open_set:
        _, g, current = heapq.heappop(open_set)
        if current == goal_rc:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        if current in visited:
            continue
        visited.add(current)
        for dr, dc, cost in neighbors:
            nr, nc = current[0]+dr, current[1]+dc
            if not (0 <= nr < h and 0 <= nc < w) or obstacle_grid[nr, nc]:
                continue
            neighbor = (nr, nc)
            tentative_g = g + cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                heapq.heappush(open_set, (tentative_g + heuristic(neighbor, goal_rc), tentative_g, neighbor))
    return None


def run_benchmark_episode(env, algo=None, policy=None, mode='random', inject_noise=False, inject_failure=False, verbose=False):
    obs_dict, infos = env.reset()
    active_agents = env.possible_agents[:]
    
    # Metrics tracking
    steps = 0
    total_cells = env.grid_resolution_x * env.grid_resolution_y
    initial_visited = np.sum(env.visited_grid)
    
    # Trajectory tracking for distance
    prev_poses = {agent: (0.0, 0.0) for agent in env.possible_agents}
    distances = {agent: 0.0 for agent in env.possible_agents}
    
    # Cell occupancy overlap counts
    cell_visit_counts = np.zeros((env.grid_resolution_y, env.grid_resolution_x))

    # Collision tracking: count unique agent-steps flagged FAILED (lidar/inter-agent)
    collision_count = 0
    
    # Pick a random agent to fail if failure injection is enabled
    failed_agent = 'tb2'
    failure_triggered = False
    
    # We track claimed targets in this step to avoid greedy multi-agent target deadlocks
    claimed_targets = set()
    
    active = True
    while active and steps < env.max_steps:
        actions = {}

        # Reset claims for the current step
        step_claims = set()

        # Shared obstacle grid for this step (used by 'heuristic' mode's A* routing).
        # Computed once here rather than per-agent/per-candidate.
        obstacle_grid = build_obstacle_grid(env) if mode == 'heuristic' else None
        # Inflated copy for path planning only, so routes keep a clearance margin
        # from walls (frontier target *selection* still uses the raw, uninflated
        # grid so legitimately-reachable floor cells near a wall stay valid targets).
        planning_grid = inflate_obstacles(obstacle_grid, radius=1) if mode == 'heuristic' else None

        # 1. Action Selection based on Mode
        for agent in env.agents:
            # Handle failed agent
            if inject_failure and agent == failed_agent and steps >= 50:
                actions[agent] = np.array([0.0, 0.0])
                continue
                
            if mode == 'random':
                actions[agent] = env.action_space(agent).sample()
                
            elif mode == 'heuristic':
                # Frontier-exploration: target the closest unvisited grid cell
                # (selection unchanged from the original heuristic, kept stable so
                # the target doesn't hop between candidates frame-to-frame). A* is
                # used only to STEER toward that same target: when a direct line is
                # blocked but a route around the obstacle exists, follow the route's
                # next waypoint instead of stalling against the wall in a straight
                # line. If no path exists, falls back to the original straight-line
                # behavior (no worse than before).
                state = env.last_poses[agent]  # x, y, yaw
                min_x, max_x, min_y, max_y = env.grid_bounds

                def world_to_cell(x, y):
                    c = int(np.clip((x - min_x) / (max_x - min_x) * env.grid_resolution_x, 0, env.grid_resolution_x - 1))
                    r = int(np.clip((y - min_y) / (max_y - min_y) * env.grid_resolution_y, 0, env.grid_resolution_y - 1))
                    return (r, c)

                agent_cell = world_to_cell(state[0], state[1])

                # Find unvisited, obstacle-free cells (same as original heuristic)
                unvisited_coords = []
                for r in range(env.grid_resolution_y):
                    for c in range(env.grid_resolution_x):
                        if not env.visited_grid[r, c] and not obstacle_grid[r, c]:
                            cx = min_x + (c + 0.5) * (max_x - min_x) / env.grid_resolution_x
                            cy = min_y + (r + 0.5) * (max_y - min_y) / env.grid_resolution_y
                            unvisited_coords.append((cx, cy))

                if len(unvisited_coords) > 0:
                    # Filter coordinates to pick ones not claimed in this step
                    dists = []
                    valid_coords = []
                    for cx, cy in unvisited_coords:
                        coord_key = (round(cx, 2), round(cy, 2))
                        if coord_key not in step_claims:
                            dists.append(math.hypot(cx - state[0], cy - state[1]))
                            valid_coords.append((cx, cy))

                    if len(valid_coords) == 0:
                        # Fallback if all remaining unvisited cells are already claimed
                        dists = [math.hypot(cx - state[0], cy - state[1]) for cx, cy in unvisited_coords]
                        closest_idx = np.argmin(dists)
                        tx, ty = unvisited_coords[closest_idx]
                    else:
                        closest_idx = np.argmin(dists)
                        tx, ty = valid_coords[closest_idx]
                        step_claims.add((round(tx, 2), round(ty, 2)))

                    # Only re-route via A* if the direct line to the target is
                    # actually obstructed; otherwise keep the original straight-line
                    # steering unchanged (avoids introducing waypoint jitter in the
                    # common unobstructed case).
                    target_cell = world_to_cell(tx, ty)
                    if not line_of_sight_clear(planning_grid, agent_cell, target_cell):
                        path = astar_path(planning_grid, agent_cell, target_cell)
                        if path is not None and len(path) > 1:
                            wr, wc = path[1]
                            tx = min_x + (wc + 0.5) * (max_x - min_x) / env.grid_resolution_x
                            ty = min_y + (wr + 0.5) * (max_y - min_y) / env.grid_resolution_y

                    # Reactive Obstacle Avoidance using Lidar (safety net for dynamic
                    # obstacles/teammates not captured by the static obstacle grid)
                    agent_obs = obs_dict[agent]
                    # Beams 10, 11, 12, 13, 14 are front-facing beams
                    front_beams = agent_obs[10:15]
                    min_front_dist = np.min(front_beams)

                    if min_front_dist < 0.45:
                        # Obstacle very close! Back up slightly and spin away
                        linear = -0.05
                        left_dist = np.min(agent_obs[14:18])
                        right_dist = np.min(agent_obs[6:10])
                        angular = 0.6 if left_dist > right_dist else -0.6
                        actions[agent] = np.array([linear, angular], dtype=np.float32)
                    elif min_front_dist < 0.7:
                        # Obstacle ahead! Slow down and steer away
                        linear = 0.05
                        left_dist = np.min(agent_obs[14:18])
                        right_dist = np.min(agent_obs[6:10])
                        angular = 0.5 if left_dist > right_dist else -0.5
                        actions[agent] = np.array([linear, angular], dtype=np.float32)
                    else:
                        # Path clear! Proportional control to next waypoint
                        goal_dist = math.hypot(tx - state[0], ty - state[1])
                        goal_angle = math.atan2(ty - state[1], tx - state[0]) - state[2]
                        goal_angle = math.atan2(math.sin(goal_angle), math.cos(goal_angle))
                        linear = 0.18 if goal_dist > 0.2 else 0.0
                        angular = np.clip(1.5 * goal_angle, -0.8, 0.8)
                        actions[agent] = np.array([linear, angular], dtype=np.float32)
                else:
                    # No reachable unvisited cells left: perform random wander
                    actions[agent] = np.array([0.1, 0.0])
                    
            elif mode == 'mappo' and policy is not None:
                agent_obs = obs_dict[agent]
                
                # Inject Gaussian sensor noise to Lidar inputs if requested
                if inject_noise:
                    # Lidar readings are the first 24 dimensions
                    lidar_noise = np.random.normal(0.0, 0.15, size=(24,))
                    agent_obs = agent_obs.copy()
                    agent_obs[:24] = np.clip(agent_obs[:24] + lidar_noise, 0.12, 3.5)
                    
                act_batch, _, _ = policy.compute_actions(
                    np.array([agent_obs]),
                    explore=False
                )
                actions[agent] = act_batch[0]
                
        # 2. Step the Environment
        obs_dict, rewards, terminations, truncations, infos = env.step(actions)
        
        # 3. Track Metrics
        for agent in env.possible_agents:
            if agent in infos:
                if infos[agent].get('status') == 'FAILED':
                    collision_count += 1
                cx, cy = infos[agent]['x'], infos[agent]['y']
                
                # Accumulate distance
                if steps > 0:
                    px, py = prev_poses[agent]
                    distances[agent] += math.hypot(cx - px, cy - py)
                prev_poses[agent] = (cx, cy)
                
                # Map to grid coordinate for visit counts
                min_x, max_x, min_y, max_y = env.grid_bounds
                x_clipped = np.clip(cx, min_x, max_x - 1e-5)
                y_clipped = np.clip(cy, min_y, max_y - 1e-5)
                col = int((x_clipped - min_x) / (max_x - min_x) * env.grid_resolution_x)
                row = int((y_clipped - min_y) / (max_y - min_y) * env.grid_resolution_y)
                cell_visit_counts[row, col] += 1
                
        steps += 1
        if len(env.agents) == 0:
            active = False

        if verbose and steps % 50 == 0:
            # Reachable-area ACR: exclude cells discovered to be obstacles from
            # the denominator, since they can never be marked visited regardless
            # of navigation quality. On a large, only-partially-explored map most
            # obstacles aren't discovered yet, so this stays close to the raw
            # total_cells figure early on and only diverges once real walls/
            # pillars are found - it does not inflate the score, it just stops
            # penalizing the swarm for floor space that was never coverable.
            # env.viz_grid is populated every step regardless of mode (random/
            # heuristic/mappo alike), so this must apply uniformly across all
            # methods - gating it to one mode would score that mode against a
            # smaller, more generous denominator than the others in the same
            # comparison table.
            known_obstacles = build_obstacle_grid(env).sum()
            live_acr = (np.sum(env.visited_grid) / max(1, total_cells - known_obstacles)) * 100.0
            print(f"    Step {steps:4d}/{env.max_steps} | Coverage: {live_acr:5.1f}% | Active robots: {len(env.agents)}")

    # Calculate final results
    final_visited = np.sum(env.visited_grid)
    final_known_obstacles = build_obstacle_grid(env).sum()
    acr = (final_visited / max(1, total_cells - final_known_obstacles)) * 100.0
    
    # Calculate overlap redundancy: average visits per visited cell (excluding zero visits)
    visited_mask = cell_visit_counts > 0
    redundancy = np.mean(cell_visit_counts[visited_mask]) if np.any(visited_mask) else 0.0
    
    # Total distance traveled by the entire swarm
    total_distance = sum(distances.values())
    
    return {
        'acr': acr,
        'steps': steps,
        'redundancy': redundancy,
        'distance': total_distance,
        'collisions': collision_count
    }

def run_coverage_demo(gui=True, max_steps=1200, world='cafe'):
    """
    Maximize area coverage using the Frontier Heuristic (no training required)
    with continuous_exploration=True so agents keep pursuing new frontier cells
    instead of terminating on their initial random goal. Runs with Gazebo GUI +
    RViz by default so coverage can be watched live.

    world='cafe' (default): furnished cafe, realistic obstacle course, plateaus
    around ~90% since some grid cells are physically inside furniture.
    world='warehouse': verified obstacle-free 12x12m region of a real Gazebo
    Fuel model, case study for genuinely-achievable ~100% coverage.
    """
    print("\n" + "="*50)
    print(f"      MARS SWARM COVERAGE DEMO (Frontier Heuristic, world={world})      ")
    print("="*50 + "\n")

    start_gazebo(headless=not gui, world=world)

    print("[coverage-demo] Initializing Swarm Environment...")
    env = PettingZooSwarmEnv(max_steps=max_steps, continuous_exploration=True, world=world)

    print(f"[coverage-demo] Running single {max_steps}-step episode with dynamic frontier targeting...")
    res = run_benchmark_episode(env, mode='heuristic', verbose=True)

    env.close()
    if gazebo_process:
        print("[coverage-demo] Stopping Gazebo...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()

    print("\n" + "="*50)
    print(f"FINAL COVERAGE: {res['acr']:.1f}%  | Steps: {res['steps']} | "
          f"Redundancy: {res['redundancy']:.2f} | Distance: {res['distance']:.2f}m | "
          f"Collisions: {res['collisions']}")
    print("="*50 + "\n")

def main():
    parser = argparse.ArgumentParser(description="MARS Swarm Quantitative Benchmarking Suite")
    parser.add_argument('--checkpoint', type=str, default="", help="Path to checkpoint directory for MAPPO policy")
    parser.add_argument('--episodes', type=int, default=5, help="Number of evaluation episodes per configuration")
    parser.add_argument('--gui', action='store_true', help="Run with Gazebo GUI enabled")
    parser.add_argument('--coverage-demo', action='store_true', help="Run a single long Frontier Heuristic episode to maximize area coverage (GUI+RViz by default)")
    parser.add_argument('--max-steps', type=int, default=1200, help="Max steps for --coverage-demo")
    parser.add_argument('--headless', action='store_true', help="Force headless for --coverage-demo (default is GUI)")
    parser.add_argument('--world', type=str, default='cafe', choices=['cafe', 'warehouse'], help="World for --coverage-demo: 'cafe' (furnished, ~90%% ceiling) or 'warehouse' (verified obstacle-free case study, ~100%% ceiling)")
    args = parser.parse_args()

    if args.coverage_demo:
        run_coverage_demo(gui=not args.headless, max_steps=args.max_steps, world=args.world)
        return

    print("\n" + "="*50)
    print("      MARS SWARM QUANTITATIVE BENCHMARKING SUITE      ")
    print("="*50 + "\n")

    # 1. Initialize Ray if MAPPO is requested
    policy = None
    if args.checkpoint:
        import ray
        from ray.rllib.models import ModelCatalog
        from ray.tune.registry import register_env
        from ray.rllib.env.wrappers.pettingzoo_env import ParallelPettingZooEnv
        from train_multi import TorchCentralizedCriticModel, CentralizedCritic
        
        # Register Custom Centralized Critic Model
        ModelCatalog.register_custom_model("cc_model", TorchCentralizedCriticModel)
        
        # Initialize Ray
        ray.init(ignore_reinit_error=True)
        
        # Register Environment
        def env_creator(config_dict):
            return ParallelPettingZooEnv(PettingZooSwarmEnv(max_steps=150))
        register_env("mars_swarm_v0", env_creator)
        
        print(f"[benchmark] Loading MAPPO checkpoint from {args.checkpoint}...")
        try:
            algo = CentralizedCritic.from_checkpoint(args.checkpoint)
            policy = algo.get_policy("shared_policy")
            print("[benchmark] MAPPO policy loaded successfully.")
        except Exception as e:
            print(f"[benchmark] ERROR: Failed to load policy: {e}")
            sys.exit(1)
            
    # Start Gazebo
    start_gazebo(headless=not args.gui)
    
    print("[benchmark] Initializing Swarm Environment...")
    env = PettingZooSwarmEnv(max_steps=150)
    
    # Benchmarking configurations
    scenarios = [
        {'name': 'Random Walk', 'mode': 'random', 'noise': False, 'failure': False},
        {'name': 'Frontier Heuristic', 'mode': 'heuristic', 'noise': False, 'failure': False},
    ]
    
    if policy is not None:
        scenarios.extend([
            {'name': 'MAPPO (Nominal)', 'mode': 'mappo', 'noise': False, 'failure': False},
            {'name': 'MAPPO (Sensor Noise)', 'mode': 'mappo', 'noise': True, 'failure': False},
            {'name': 'MAPPO (Agent Failure)', 'mode': 'mappo', 'noise': False, 'failure': True},
        ])
        
    results_data = {scen['name']: {'acr': [], 'redundancy': [], 'distance': [], 'collisions': []} for scen in scenarios}
    
    for scen in scenarios:
        name = scen['name']
        print(f"\n--- Running Evaluation: {name} ({args.episodes} Episodes) ---")
        for ep in range(1, args.episodes + 1):
            res = run_benchmark_episode(
                env, 
                policy=policy, 
                mode=scen['mode'], 
                inject_noise=scen['noise'], 
                inject_failure=scen['failure']
            )
            results_data[name]['acr'].append(res['acr'])
            results_data[name]['redundancy'].append(res['redundancy'])
            results_data[name]['distance'].append(res['distance'])
            results_data[name]['collisions'].append(res['collisions'])
            print(f"  Episode {ep:2d} | ACR: {res['acr']:5.1f}% | Redundancy: {res['redundancy']:.2f} | Distance: {res['distance']:.2f}m | Collisions: {res['collisions']}")
            
    env.close()
    if gazebo_process:
        print("[benchmark] Stopping Gazebo...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()
    
    # Save statistics and generate comparison plots
    print("\n" + "="*50)
    print("                 BENCHMARK SUMMARY                ")
    print("="*50)
    for name, metrics in results_data.items():
        mean_acr = np.mean(metrics['acr'])
        std_acr = np.std(metrics['acr'])
        mean_red = np.mean(metrics['redundancy'])
        mean_dist = np.mean(metrics['distance'])
        mean_coll = np.mean(metrics['collisions'])
        print(f"{name:25s} | ACR: {mean_acr:5.1f} ± {std_acr:3.1f}% | Overlap Redundancy: {mean_red:4.2f} | Energy: {mean_dist:5.1f}m | Collisions: {mean_coll:4.2f}")
        
    # Generate Box-and-Whisker Plots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    names = list(results_data.keys())
    acr_values = [results_data[name]['acr'] for name in names]
    red_values = [results_data[name]['redundancy'] for name in names]
    
    ax1.boxplot(acr_values, patch_artist=True, boxprops=dict(facecolor='lightblue', color='blue'))
    ax1.set_xticklabels(names, rotation=25, ha='right')
    ax1.set_title('Area Coverage Rate (ACR) %')
    ax1.set_ylabel('Coverage %')
    ax1.grid(True, linestyle='--', alpha=0.6)
    
    ax2.boxplot(red_values, patch_artist=True, boxprops=dict(facecolor='lightgreen', color='green'))
    ax2.set_xticklabels(names, rotation=25, ha='right')
    ax2.set_title('Cell Visit Overlap Redundancy')
    ax2.set_ylabel('Average Visits / Visited Cell')
    ax2.grid(True, linestyle='--', alpha=0.6)
    
    plt.tight_layout()
    plot_dir = "./checkpoints"
    os.makedirs(plot_dir, exist_ok=True)
    plot_path = os.path.join(plot_dir, "benchmark_results.png")
    plt.savefig(plot_path)
    print(f"\n[benchmark] Quantitative comparison plot successfully generated and saved to: {plot_path}")
    print("="*50 + "\n")

if __name__ == "__main__":
    main()
