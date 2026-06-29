import os
import sys
import time
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt

# Ensure path includes workspace packages
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from multi_env_wrapper import PettingZooSwarmEnv
from train_multi import start_gazebo, kill_stale_processes, gazebo_process

def run_benchmark_episode(env, algo=None, policy=None, mode='random', inject_noise=False, inject_failure=False):
    obs_dict, infos = env.reset()
    active_agents = env.possible_agents[:]
    
    # Metrics tracking
    steps = 0
    total_cells = env.grid_resolution * env.grid_resolution
    initial_visited = np.sum(env.visited_grid)
    
    # Trajectory tracking for distance
    prev_poses = {agent: (0.0, 0.0) for agent in env.possible_agents}
    distances = {agent: 0.0 for agent in env.possible_agents}
    
    # Cell occupancy overlap counts
    cell_visit_counts = np.zeros((env.grid_resolution, env.grid_resolution))
    
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
        
        # 1. Action Selection based on Mode
        for agent in env.agents:
            # Handle failed agent
            if inject_failure and agent == failed_agent and steps >= 50:
                actions[agent] = np.array([0.0, 0.0])
                continue
                
            if mode == 'random':
                actions[agent] = env.action_space(agent).sample()
                
            elif mode == 'heuristic':
                # Frontier-exploration: Target closest unvisited grid cell
                state = env.last_poses[agent]  # x, y, yaw
                min_x, max_x, min_y, max_y = env.grid_bounds
                
                # Find unvisited cells
                unvisited_coords = []
                for r in range(env.grid_resolution):
                    for c in range(env.grid_resolution):
                        if not env.visited_grid[r, c]:
                            # Skip if this cell is occupied by a known wall in the high-res map
                            is_obstacle = False
                            ratio = env.viz_resolution // env.grid_resolution
                            start_r = r * ratio
                            start_c = c * ratio
                            for dr in range(ratio):
                                for dc in range(ratio):
                                    if env.viz_grid[start_r + dr, start_c + dc] == 100:
                                        is_obstacle = True
                                        break
                                if is_obstacle:
                                    break
                            if is_obstacle:
                                continue
                                
                            # Map row/col back to global x, y
                            cx = min_x + (c + 0.5) * (max_x - min_x) / env.grid_resolution
                            cy = min_y + (r + 0.5) * (max_y - min_y) / env.grid_resolution
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
                    
                    # Reactive Obstacle Avoidance using Lidar
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
                        # Path clear! Proportional control to target cell
                        goal_dist = math.hypot(tx - state[0], ty - state[1])
                        goal_angle = math.atan2(ty - state[1], tx - state[0]) - state[2]
                        goal_angle = math.atan2(math.sin(goal_angle), math.cos(goal_angle))
                        linear = 0.18 if goal_dist > 0.2 else 0.0
                        angular = np.clip(1.5 * goal_angle, -0.8, 0.8)
                        actions[agent] = np.array([linear, angular], dtype=np.float32)
                else:
                    # No unvisited cells left: perform random wander
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
                col = int((x_clipped - min_x) / (max_x - min_x) * env.grid_resolution)
                row = int((y_clipped - min_y) / (max_y - min_y) * env.grid_resolution)
                cell_visit_counts[row, col] += 1
                
        steps += 1
        if len(env.agents) == 0:
            active = False
            
    # Calculate final results
    final_visited = np.sum(env.visited_grid)
    acr = (final_visited / total_cells) * 100.0
    
    # Calculate overlap redundancy: average visits per visited cell (excluding zero visits)
    visited_mask = cell_visit_counts > 0
    redundancy = np.mean(cell_visit_counts[visited_mask]) if np.any(visited_mask) else 0.0
    
    # Total distance traveled by the entire swarm
    total_distance = sum(distances.values())
    
    return {
        'acr': acr,
        'steps': steps,
        'redundancy': redundancy,
        'distance': total_distance
    }

def main():
    parser = argparse.ArgumentParser(description="MARS Swarm Quantitative Benchmarking Suite")
    parser.add_argument('--checkpoint', type=str, default="", help="Path to checkpoint directory for MAPPO policy")
    parser.add_argument('--episodes', type=int, default=5, help="Number of evaluation episodes per configuration")
    parser.add_argument('--gui', action='store_true', help="Run with Gazebo GUI enabled")
    args = parser.parse_args()
    
    print("\n" + "="*50)
    print("      MARS SWARM QUANTITATIVE BENCHMARKING SUITE      ")
    print("="*50 + "\n")
    
    # 1. Initialize Ray if MAPPO is requested
    policy = None
    if args.checkpoint:
        import ray
        from ray.rllib.algorithms.algorithm import Algorithm
        ray.init(ignore_reinit_error=True)
        print(f"[benchmark] Loading MAPPO checkpoint from {args.checkpoint}...")
        try:
            algo = Algorithm.from_checkpoint(args.checkpoint)
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
        
    results_data = {scen['name']: {'acr': [], 'redundancy': [], 'distance': []} for scen in scenarios}
    
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
            print(f"  Episode {ep:2d} | ACR: {res['acr']:5.1f}% | Redundancy: {res['redundancy']:.2f} | Distance: {res['distance']:.2f}m")
            
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
        print(f"{name:25s} | ACR: {mean_acr:5.1f} ± {std_acr:3.1f}% | Overlap Redundancy: {mean_red:4.2f} | Energy: {mean_dist:5.1f}m")
        
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
