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

try:
    from decentralized_coordinator import DecentralizedCoordinator
except ImportError:
    from mars_swarm.decentralized_coordinator import DecentralizedCoordinator

try:
    from mission_behavior_tree import SwarmMissionTree
except ImportError:
    from mars_swarm.mission_behavior_tree import SwarmMissionTree

try:
    from swarm_telemetry import SwarmTelemetryLogger
except ImportError:
    from mars_swarm.swarm_telemetry import SwarmTelemetryLogger




def build_obstacle_grid(env, agent=None):
    """Downsample high-res viz_grid to coverage grid resolution.
    If agent is provided, evaluates against agent's local belief map.
    """
    ratio_x = env.viz_resolution_x // env.grid_resolution_x
    ratio_y = env.viz_resolution_y // env.grid_resolution_y
    obstacle = np.zeros((env.grid_resolution_y, env.grid_resolution_x), dtype=bool)
    
    grid_source = env.local_viz_grids[agent] if (agent is not None and agent in env.local_viz_grids) else env.viz_grid
    for r in range(env.grid_resolution_y):
        for c in range(env.grid_resolution_x):
            block = grid_source[r*ratio_y:(r+1)*ratio_y, c*ratio_x:(c+1)*ratio_x]
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


def run_benchmark_episode(env, algo=None, policy=None, mode='random', inject_noise=False, inject_failure=False, verbose=False, telemetry_logger=None):
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
    wall_collision_count = 0
    agent_collision_count = 0
    
    # Pick a random agent to fail if failure injection is enabled
    failed_agent = 'tb2'
    failure_triggered = False
    
    # We track claimed targets in this step to avoid greedy multi-agent target deadlocks
    claimed_targets = set()

    # Reachability memory for 'heuristic' mode: frontier selection alone has no
    # concept of "unreachable" (only "unvisited" / "not yet discovered as
    # obstacle"), so a target cell that's flush against a wall the robot's own
    # SLAM-like grid hasn't registered from its approach angle gets re-picked
    # forever - the robot walks into the same wall every step. Track how long
    # each agent has been chasing the same target cell without closing the
    # distance; after too long, blacklist that cell from selection so the
    # agent moves on to a genuinely reachable frontier instead of stalling.
    STUCK_STEPS_THRESHOLD = 25
    STUCK_PROGRESS_EPS = 0.05  # meters closer required per STUCK window to not count as stuck
    agent_target_state = {agent: {'cell': None, 'best_dist': None, 'stuck_steps': 0} for agent in env.possible_agents}
    blacklisted_target_cells = set()

    # Position-based stuck detection: the target-cell tracker above resets
    # stuck_steps to 0 whenever the "closest unvisited cell" changes, which
    # happens even while an agent is physically frozen -- once wedged, ties
    # between near-equidistant frontier candidates flip the selected target
    # cell every step or two (especially as OTHER agents update the shared
    # visited_grid), so stuck_steps never accumulates and the 25-step
    # blacklist never fires. Confirmed in a 6000-step run: one agent sat at
    # the exact same (x, y) for 1350+ consecutive steps (pure in-place
    # rotation) while target-cell tracking kept "resetting" as if making
    # progress. This tracker watches raw position instead, so it can't be
    # fooled by target churn, and escalates to a real escape maneuver
    # (not just the tiny -0.05 m/step reactive backup) when truly wedged.
    mission_trees = {agent: SwarmMissionTree(agent) for agent in env.possible_agents}
    coordinator = DecentralizedCoordinator(d_comm=3.0, claim_timeout_steps=30)
    active = True
    while active and steps < env.max_steps:
        actions = {}

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
                state = env.last_poses[agent]  # x, y, yaw
                local_visited = env.local_visited_grids.get(agent, env.visited_grid)
                local_obs = build_obstacle_grid(env, agent=agent)
                local_planning_grid = inflate_obstacles(local_obs, radius=1)

                actions[agent] = mission_trees[agent].tick(
                    current_step=steps,
                    agent_pose=state,
                    agent_poses=env.last_poses,
                    obs_dict=obs_dict,
                    local_visited_grid=local_visited,
                    local_obstacle_grid=local_obs,
                    local_planning_grid=local_planning_grid,
                    grid_bounds=env.grid_bounds,
                    grid_res_x=env.grid_resolution_x,
                    grid_res_y=env.grid_resolution_y,
                    coordinator=coordinator,
                    line_of_sight_fn=line_of_sight_clear,
                    astar_fn=astar_path,
                    is_continuous_exploration=getattr(env, 'continuous_exploration', True)
                )
                    
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
                if infos[agent].get('wall_contact', False):
                    wall_collision_count += 1
                if infos[agent].get('agent_contact', False):
                    agent_collision_count += 1
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

        obs_grid = build_obstacle_grid(env)
        known_obstacles = int(np.count_nonzero(obs_grid))
        valid_visited = np.logical_and(env.visited_grid, np.logical_not(obs_grid))
        live_acr = (np.count_nonzero(valid_visited) / max(1, total_cells - known_obstacles)) * 100.0

        is_deadlock = any(getattr(mission_trees.get(a), 'escape_steps_left', 0) == 14 for a in env.agents)
        if telemetry_logger is not None:
            telemetry_logger.record_step(
                step=steps,
                acr=live_acr,
                actions_dict=actions,
                poses_dict=env.last_poses,
                is_deadlock_event=is_deadlock,
                collisions=collision_count,
                wall_collisions=wall_collision_count,
                agent_collisions=agent_collision_count
            )

        if verbose and steps % 50 == 0:
            print(f"    Step {steps:4d}/{env.max_steps} | Coverage: {live_acr:5.1f}% | Active robots: {len(env.agents)}")

    # Calculate final results
    final_obs_grid = build_obstacle_grid(env)
    final_known_obstacles = int(np.count_nonzero(final_obs_grid))
    final_valid_visited = np.logical_and(env.visited_grid, np.logical_not(final_obs_grid))
    acr = (np.count_nonzero(final_valid_visited) / max(1, total_cells - final_known_obstacles)) * 100.0
    
    # Calculate overlap redundancy: average visits per visited cell (excluding zero visits)
    visited_mask = cell_visit_counts > 0
    redundancy = np.mean(cell_visit_counts[visited_mask]) if np.any(visited_mask) else 0.0
    
    # Total distance traveled by the entire swarm
    total_distance = sum(distances.values())
    
    final_results = {
        'acr': acr,
        'steps': steps,
        'redundancy': redundancy,
        'distance': total_distance,
        'collisions': collision_count,
        'wall_collisions': wall_collision_count,
        'agent_collisions': agent_collision_count
    }
    
    if telemetry_logger is not None:
        telemetry_logger.finalize_and_export(final_results)
        
    return final_results
def run_coverage_demo(gui=True, max_steps=1200, world='cafe', seed=42, export_json=""):
    print("\n" + "="*50)
    print(f"      FRONTIER HEURISTIC COVERAGE RUN ({max_steps} STEPS, {world.upper()} WORLD, SEED={seed})      ")
    print("="*50 + "\n")

    start_gazebo(headless=not gui, world=world, seed=seed)

    print("[coverage-demo] Initializing Swarm Environment...")
    env = PettingZooSwarmEnv(max_steps=max_steps, continuous_exploration=True, world=world)

    telemetry_logger = None
    if export_json:
        telemetry_dir = os.path.dirname(export_json) or "checkpoints"
        telemetry_logger = SwarmTelemetryLogger(log_dir=telemetry_dir, enable_tensorboard=True)

    print(f"[coverage-demo] Running single {max_steps}-step episode with dynamic frontier targeting...")
    res = run_benchmark_episode(env, mode='heuristic', verbose=True, telemetry_logger=telemetry_logger)

    if telemetry_logger and export_json:
        telemetry_logger.finalize_and_export(res, export_path=export_json)

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
          f"Collisions: {res['collisions']} (Wall: {res['wall_collisions']}, Agent: {res['agent_collisions']})")
    print("="*50 + "\n")

def main():
    parser = argparse.ArgumentParser(description="MARS Swarm Quantitative Benchmarking Suite")
    parser.add_argument('--checkpoint', type=str, default="", help="Path to checkpoint directory for MAPPO policy")
    parser.add_argument('--episodes', type=int, default=5, help="Number of evaluation episodes per configuration")
    parser.add_argument('--gui', action='store_true', help="Run with Gazebo GUI enabled")
    parser.add_argument('--coverage-demo', action='store_true', help="Run a single long Frontier Heuristic episode to maximize area coverage (GUI+RViz by default)")
    parser.add_argument('--max-steps', type=int, default=1200, help="Max steps for --coverage-demo")
    parser.add_argument('--headless', action='store_true', help="Force headless for --coverage-demo (default is GUI)")
    parser.add_argument('--world', type=str, default='cafe', choices=['cafe', 'warehouse', 'depot', 'office', 'maze'], help="World for benchmarking/demo: 'cafe', 'warehouse', 'depot', 'office', 'maze'")
    parser.add_argument('--seed', type=int, default=42, help="Deterministic PRNG seed for Gazebo physics, sensors, and repeatable replay")
    parser.add_argument('--export-json', type=str, default="", help="Optional filepath for structured run telemetry JSON export")
    args = parser.parse_args()

    if args.coverage_demo:
        run_coverage_demo(gui=not args.headless, max_steps=args.max_steps, world=args.world, seed=args.seed, export_json=args.export_json)
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
        
        # Initialize Ray with localhost binding and memory protections
        os.environ["ENABLE_RAY_CLUSTER"] = "0"
        ray.init(
            _node_ip_address="127.0.0.1",
            object_store_memory=500 * 1024 * 1024,
            ignore_reinit_error=True
        )
        
        # Register Environment
        def env_creator(config_dict):
            return ParallelPettingZooEnv(PettingZooSwarmEnv(max_steps=150, world=args.world))
        register_env("mars_swarm_v0", env_creator)
        
        print(f"[benchmark] Loading MAPPO checkpoint from {args.checkpoint}...")
        try:
            ckpt_path = os.path.abspath(args.checkpoint)
            shared_policy_dir = os.path.join(ckpt_path, "policies", "shared_policy")
            if os.path.isdir(shared_policy_dir):
                from ray.rllib.policy.policy import Policy
                policy = Policy.from_checkpoint(shared_policy_dir)
            else:
                algo = CentralizedCritic.from_checkpoint(ckpt_path)
                policy = algo.get_policy("shared_policy")
            print("[benchmark] MAPPO policy loaded successfully.")
        except Exception as e:
            print(f"[benchmark] ERROR: Failed to load policy: {e}")
            sys.exit(1)
            
    # Start Gazebo
    start_gazebo(headless=not args.gui, world=args.world, seed=args.seed)
    
    print(f"[benchmark] Initializing Swarm Environment for world='{args.world}'...")
    env = PettingZooSwarmEnv(max_steps=150, world=args.world)
    
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
        median_acr = np.median(metrics['acr'])
        mean_red = np.mean(metrics['redundancy'])
        median_red = np.median(metrics['redundancy'])
        mean_dist = np.mean(metrics['distance'])
        mean_coll = np.mean(metrics['collisions'])
        print(f"{name:25s} | ACR: {mean_acr:5.1f} ± {std_acr:3.1f}% (med: {median_acr:5.1f}%) | Overlap Redundancy: {mean_red:4.2f} (med: {median_red:4.2f}) | Energy: {mean_dist:5.1f}m | Collisions: {mean_coll:4.2f}")
        
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
