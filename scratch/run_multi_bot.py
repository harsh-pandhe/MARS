import os
import sys
import time
import numpy as np
import math
import subprocess
import datetime

# Ensure path includes workspace packages
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/mars_swarm/mars_swarm')))
from multi_env_wrapper import PettingZooSwarmEnv
from train_multi import start_gazebo, kill_stale_processes

def get_reachable_unvisited_frontiers(env, state):
    """
    Finds all unvisited cells in env.visited_grid that are reachable from the robot's
    current position using a Breadth-First Search (BFS).
    Traversability is determined by checking if the cell is clear of obstacles (100) in viz_grid.
    """
    min_x, max_x, min_y, max_y = env.grid_bounds
    x, y, yaw = state
    
    # Map robot position to visited_grid cell
    c0 = int(np.clip((x - min_x) / (max_x - min_x) * env.grid_resolution_x, 0, env.grid_resolution_x - 1))
    r0 = int(np.clip((y - min_y) / (max_y - min_y) * env.grid_resolution_y, 0, env.grid_resolution_y - 1))
    
    # Check which grid cells contain obstacles in high-resolution viz_grid
    traversable = np.ones((env.grid_resolution_y, env.grid_resolution_x), dtype=bool)
    
    for r in range(env.grid_resolution_y):
        for c in range(env.grid_resolution_x):
            start_r = int(r * env.viz_resolution_y / env.grid_resolution_y)
            end_r = int((r + 1) * env.viz_resolution_y / env.grid_resolution_y)
            start_c = int(c * env.viz_resolution_x / env.grid_resolution_x)
            end_c = int((c + 1) * env.viz_resolution_x / env.grid_resolution_x)
            
            is_obstacle = False
            for vr in range(start_r, end_r):
                for vc in range(start_c, end_c):
                    if env.viz_grid[vr, vc] == 100:
                        is_obstacle = True
                        break
                if is_obstacle:
                    break
            if is_obstacle:
                traversable[r, c] = False

    # Run BFS
    queue = [(r0, c0)]
    visited = set()
    visited.add((r0, c0))
    
    reachable_unvisited = []
    
    # 8-connectivity directions
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    head = 0
    while head < len(queue):
        r, c = queue[head]
        head += 1
        
        # If this cell is unvisited and traversable, it is a frontier candidate
        if not env.visited_grid[r, c]:
            cx = min_x + (c + 0.5) * (max_x - min_x) / env.grid_resolution_x
            cy = min_y + (r + 0.5) * (max_y - min_y) / env.grid_resolution_y
            reachable_unvisited.append((cx, cy))
            
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < env.grid_resolution_y and 0 <= nc < env.grid_resolution_x:
                if (nr, nc) not in visited and traversable[nr, nc]:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
                    
    return reachable_unvisited

def save_coverage_plot(trajectory_history, coverage_history, env, timestamp):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors
        
        # Calculate distance traveled for each robot
        dist_traveled = {}
        for agent in ['tb1', 'tb2', 'tb3']:
            traj = trajectory_history.get(agent, [])
            if len(traj) > 1:
                diffs = np.diff(np.array(traj), axis=0)
                dist_traveled[agent] = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))
            else:
                dist_traveled[agent] = 0.0
                
        # Set dark theme style
        plt.style.use('dark_background')
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8.5), facecolor='#0f0f12')
        
        # 1. Plot Coverage over time
        steps = range(1, len(coverage_history) + 1)
        ax1.set_facecolor('#181820')
        ax1.plot(steps, coverage_history, color='#00adb5', linewidth=3.0, label='Coverage %')
        ax1.fill_between(steps, coverage_history, color='#00adb5', alpha=0.15)
        
        ax1.set_title('Swarm Area Coverage Over Time', fontsize=15, fontweight='bold', pad=15, color='#ffffff')
        ax1.set_xlabel('Simulation Step', fontsize=12, color='#cccccc')
        ax1.set_ylabel('Coverage Percentage (%)', fontsize=12, color='#cccccc')
        ax1.set_ylim(-2, 102)
        ax1.grid(True, linestyle='--', color='#333340', alpha=0.6)
        ax1.tick_params(colors='#bbbbbb')
        ax1.legend(loc='lower right', facecolor='#1e1e26', edgecolor='#3a3a4a', labelcolor='#ffffff')
        
        # 2. Plot 2D Occupancy Grid and Trajectories
        min_x, max_x, min_y, max_y = env.grid_bounds
        ax2.set_facecolor('#181820')
        
        # Custom premium colormap:
        # -1 (unexplored) = Dark Navy Charcoal (#14141e)
        # 0 (free/explored) = Clean Off-white (#e8eaf0)
        # 100 (obstacle/wall) = Slate/Iron Gray (#4a4a5a)
        cmap = mcolors.ListedColormap(['#14141e', '#e8eaf0', '#4a4a5a'])
        bounds = [-1.5, -0.5, 50.0, 100.5]
        norm = mcolors.BoundaryNorm(bounds, cmap.N)
        
        ax2.imshow(env.viz_grid, extent=[min_x, max_x, min_y, max_y], origin='lower', cmap=cmap, norm=norm, alpha=0.9)
        
        # Colors matching the paths in RViz
        agent_colors = {
            'tb1': '#1f77b4', # Neon Blue
            'tb2': '#e377c2', # Neon Pink
            'tb3': '#2ca02c'  # Neon Green
        }
        
        # Plot each agent's trajectory
        for agent in ['tb1', 'tb2', 'tb3']:
            traj = trajectory_history.get(agent, [])
            if len(traj) > 0:
                traj_np = np.array(traj)
                ax2.plot(traj_np[:, 0], traj_np[:, 1], color=agent_colors[agent], linewidth=2.5, 
                         label=f'{agent} Path ({dist_traveled[agent]:.2f} m)', zorder=5)
                # Mark start position
                ax2.scatter(traj_np[0, 0], traj_np[0, 1], color=agent_colors[agent], marker='o', s=100, 
                            edgecolors='white', linewidths=1.5, zorder=6)
                # Mark end position
                ax2.scatter(traj_np[-1, 0], traj_np[-1, 1], color=agent_colors[agent], marker='X', s=150, 
                            edgecolors='white', linewidths=1.5, zorder=6)
                
        ax2.set_title('Swarm Exploration & Coverage Map', fontsize=15, fontweight='bold', pad=15, color='#ffffff')
        ax2.set_xlabel('X Position (m)', fontsize=12, color='#cccccc')
        ax2.set_ylabel('Y Position (m)', fontsize=12, color='#cccccc')
        ax2.set_xlim(min_x - 0.5, max_x + 0.5)
        ax2.set_ylim(min_y - 0.5, max_y + 0.5)
        ax2.grid(True, linestyle=':', color='#333340', alpha=0.6)
        ax2.tick_params(colors='#bbbbbb')
        ax2.legend(loc='upper right', facecolor='#1e1e26', edgecolor='#3a3a4a', labelcolor='#ffffff')
        
        # Add watermark / stats text box at the bottom of the figure
        total_cells = env.grid_resolution_x * env.grid_resolution_y
        visited_count = np.sum(env.visited_grid)
        final_coverage = (visited_count / total_cells) * 100.0
        
        info_text = (
            f"Final Coverage: {final_coverage:.1f}%  |  "
            f"Total Steps: {len(coverage_history)}  |  "
            f"Total Trajectory Length: {sum(dist_traveled.values()):.2f} m"
        )
        fig.text(0.5, 0.02, info_text, ha='center', fontsize=12, color='#ffffff', 
                 bbox=dict(facecolor='#1e1e26', edgecolor='#3a3a4a', boxstyle='round,pad=0.5'))
        
        plt.tight_layout(rect=[0, 0.05, 1, 1])
        plot_path = f"ros_bags/coverage_plot_{timestamp}.png"
        plt.savefig(plot_path, dpi=150)
        plt.close()
        print(f"\n[demo] Saved matplotlib coverage plot to {plot_path}")
    except Exception as e:
        print(f"\n[demo] Error generating matplotlib plot: {e}")

def allocate_targets(env, active_targets, target_pursuit_ticks):
    agents_needing_targets = []
    min_x, max_x, min_y, max_y = env.grid_bounds
    
    for agent in env.agents:
        state = env.last_poses[agent]
        tx_ty = active_targets[agent]
        
        need_replan = False
        if tx_ty is None:
            need_replan = True
        else:
            tx, ty = tx_ty
            dist = math.hypot(tx - state[0], ty - state[1])
            # Map target to visited grid
            c = int(np.clip((tx - min_x) / (max_x - min_x) * env.grid_resolution_x, 0, env.grid_resolution_x - 1))
            r = int(np.clip((ty - min_y) / (max_y - min_y) * env.grid_resolution_y, 0, env.grid_resolution_y - 1))
            
            if dist < 0.45:
                need_replan = True
            elif env.visited_grid[r, c]:
                need_replan = True
            elif target_pursuit_ticks[agent] >= 40:
                need_replan = True
                
        if need_replan:
            agents_needing_targets.append(agent)
            active_targets[agent] = None
            target_pursuit_ticks[agent] = 0
        else:
            target_pursuit_ticks[agent] += 1
            
    # If no agents need new targets, we are done
    if not agents_needing_targets:
        return
        
    # Find all unvisited reachable frontier cells for all agents combined
    all_frontiers = []
    frontier_keys = set()
    for agent in env.agents:
        state = env.last_poses[agent]
        frontiers = get_reachable_unvisited_frontiers(env, state)
        for fx, fy in frontiers:
            key = (round(fx, 1), round(fy, 1))
            if key not in frontier_keys:
                frontier_keys.add(key)
                all_frontiers.append((fx, fy))
                
    if not all_frontiers:
        return
        
    # For each agent needing a target, compute cost for each frontier
    beta = 12.0
    for agent in agents_needing_targets:
        state = env.last_poses[agent]
        best_frontier = None
        best_score = -1e9
        
        for fx, fy in all_frontiers:
            # Distance from robot to frontier
            dist_to_robot = math.hypot(fx - state[0], fy - state[1])
            
            # Repulsion from other agents' active targets
            repulsion = 0.0
            for other_agent in env.agents:
                if other_agent != agent and active_targets[other_agent] is not None:
                    otx, oty = active_targets[other_agent]
                    dist_to_other = math.hypot(fx - otx, fy - oty)
                    repulsion += 1.0 / (dist_to_other + 0.5)
                    
            # Compute heading alignment penalty (to favor moving forward)
            dx = fx - state[0]
            dy = fy - state[1]
            target_yaw = np.arctan2(dy, dx)
            angle_diff = target_yaw - state[2]
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
            turn_penalty = 1.5 * (1.0 - math.cos(angle_diff))
            
            score = -dist_to_robot - beta * repulsion - turn_penalty
            if score > best_score:
                best_score = score
                best_frontier = (fx, fy)
                
        if best_frontier is not None:
            active_targets[agent] = best_frontier
            target_pursuit_ticks[agent] = 0

def main():
    # Kill any stale processes first
    kill_stale_processes()
    
    # Start Gazebo simulation with GUI (multi-robot mode)
    start_gazebo(headless=False, multi=True)
    
    # Initialize the environment with all three robots
    env = PettingZooSwarmEnv(agents=['tb1', 'tb2', 'tb3'], max_steps=1500)
    obs_dict, infos = env.reset()
    
    print("\n" + "="*60)
    print("  COORDINATED MULTI-ROBOT COVERAGE DEMONSTRATION")
    print("  tb1, tb2, and tb3 will collaboratively explore the arena.")
    print("  They will target unvisited areas, claim targets to avoid overlap,")
    print("  and use reactive obstacle avoidance to navigate without crashes.")
    print("="*60 + "\n")
    
    # Create ros_bags directory and generate timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs('ros_bags', exist_ok=True)
    bag_dir = f"ros_bags/swarm_bag_{timestamp}"
    
    # Start ROS2 bag record in background
    bag_cmd = [
        "ros2", "bag", "record",
        "-o", bag_dir,
        "/tf", "/tf_static",
        "/tb1/scan", "/tb2/scan", "/tb3/scan",
        "/tb1/odom", "/tb2/odom", "/tb3/odom",
        "/tb1/cmd_vel", "/tb2/cmd_vel", "/tb3/cmd_vel",
        "/map"
    ]
    print(f"[demo] Starting ROS bag recording to {bag_dir}...")
    bag_proc = subprocess.Popen(bag_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    steps = 0
    max_steps = 1500
    
    # History tracking for plotting
    trajectory_history = {agent: [] for agent in ['tb1', 'tb2', 'tb3']}
    coverage_history = []
    
    active_targets = {agent: None for agent in ['tb1', 'tb2', 'tb3']}
    target_pursuit_ticks = {agent: 0 for agent in ['tb1', 'tb2', 'tb3']}
    last_poses_watchdog = {agent: None for agent in ['tb1', 'tb2', 'tb3']}
    stagnant_ticks = {agent: 0 for agent in ['tb1', 'tb2', 'tb3']}
    last_actions = {agent: np.array([0.0, 0.0]) for agent in ['tb1', 'tb2', 'tb3']}
    
    # Keep running until step limit or all agents terminated
    try:
        while steps < max_steps and len(env.agents) > 0:
            actions = {}
            
            # Watchdog and failure injection
            agents_to_kill = []
            for agent in list(env.agents):
                # 1. Simulated Failure Injection: Kill tb3 at step 100
                if agent == 'tb3' and steps == 100:
                    print(f"\n[watchdog] CRITICAL FAILURE: Simulating sudden breakdown of {agent} at step {steps}!")
                    agents_to_kill.append(agent)
                    continue
                    
                # 2. Watchdog Heartbeat: Check if robot is commanded to move but is not changing position
                if agent in env.last_poses:
                    curr_pose = env.last_poses[agent]
                    if last_poses_watchdog[agent] is not None:
                        prev_pose = last_poses_watchdog[agent]
                        dist_moved = math.hypot(curr_pose[0] - prev_pose[0], curr_pose[1] - prev_pose[1])
                        
                        prev_action = last_actions.get(agent, np.array([0.0, 0.0]))
                        if dist_moved < 0.002 and (abs(prev_action[0]) > 0.02 or abs(prev_action[1]) > 0.05):
                            stagnant_ticks[agent] += 1
                        else:
                            stagnant_ticks[agent] = 0
                    else:
                        stagnant_ticks[agent] = 0
                    last_poses_watchdog[agent] = list(curr_pose)
                    
                    # If stagnant for more than 40 steps, declare dead
                    if stagnant_ticks[agent] >= 40:
                        print(f"\n[watchdog] WARNING: Heartbeat lost for {agent} (stuck/dead for {stagnant_ticks[agent]} steps). Killing agent...")
                        agents_to_kill.append(agent)
                        
            for agent in agents_to_kill:
                if agent in env.agents:
                    env.agents.remove(agent)
                active_targets[agent] = None
                # Command velocity to zero immediately
                try:
                    from geometry_msgs.msg import Twist
                    twist = Twist()
                    env.node.cmd_vel_pubs[agent].publish(twist)
                except Exception:
                    pass
            
            # Record current robot poses
            for agent in ['tb1', 'tb2', 'tb3']:
                if agent in env.agents and agent in env.last_poses:
                    trajectory_history[agent].append(list(env.last_poses[agent][:2]))
            
            # Coordinated target allocation
            allocate_targets(env, active_targets, target_pursuit_ticks)
            
            for agent in env.agents:
                state = env.last_poses[agent]  # x, y, yaw
                tx_ty = active_targets[agent]
                
                if tx_ty is not None:
                    tx, ty = tx_ty
                    
                    dx = tx - state[0]
                    dy = ty - state[1]
                    global_target_angle = np.arctan2(dy, dx)
                    local_target_angle = global_target_angle - state[2]
                    local_target_angle = (local_target_angle + np.pi) % (2 * np.pi) - np.pi
                    
                    # 1. Attractive force towards goal
                    F_x = math.cos(local_target_angle)
                    F_y = math.sin(local_target_angle)
                    
                    # 2. Repulsive force from obstacles
                    agent_obs = obs_dict[agent]
                    for i in range(24):
                        d_b = agent_obs[i]
                        if d_b < 0.7:
                            beam_angle = i * (2.0 * np.pi / 24.0)
                            beam_angle = (beam_angle + np.pi) % (2 * np.pi) - np.pi
                            F_rep_mag = 0.08 * (1.0 / d_b - 1.0 / 0.7) / (d_b**2)
                            F_x -= F_rep_mag * math.cos(beam_angle)
                            F_y -= F_rep_mag * math.sin(beam_angle)
                            
                    # 3. Repulsive force from other robots
                    for other_agent in env.agents:
                        if other_agent != agent:
                            other_state = env.last_poses[other_agent]
                            dx_nb = other_state[0] - state[0]
                            dy_nb = other_state[1] - state[1]
                            dist_nb = math.hypot(dx_nb, dy_nb)
                            if dist_nb < 1.0:
                                global_angle = np.arctan2(dy_nb, dx_nb)
                                local_angle = global_angle - state[2]
                                local_angle = (local_angle + np.pi) % (2 * np.pi) - np.pi
                                F_rep_mag = 0.5 * (1.0 / dist_nb - 1.0 / 1.0) / (dist_nb**2)
                                F_x -= F_rep_mag * math.cos(local_angle)
                                F_y -= F_rep_mag * math.sin(local_angle)
                                
                    # 4. Compute steering and velocities
                    steer_angle = math.atan2(F_y, F_x)
                    angular_vel = np.clip(1.5 * steer_angle, -0.8, 0.8)
                    
                    front_beams = agent_obs[[22, 23, 0, 1, 2]]
                    min_front = np.min(front_beams)
                    
                    if min_front < 0.35:
                        # Extremely close obstacle: back up slightly and spin away
                        linear_vel = -0.05
                        left_dist = np.min(agent_obs[[3, 4, 5, 6, 7]])
                        right_dist = np.min(agent_obs[[17, 18, 19, 20, 21]])
                        angular_vel = 0.8 if left_dist > right_dist else -0.8
                    elif abs(steer_angle) > 1.2:
                        # Turn in place for sharp steering changes
                        linear_vel = 0.02
                    else:
                        speed_factor = max(0.1, math.cos(steer_angle))
                        linear_vel = 0.18 * speed_factor * np.clip((min_front - 0.3) / 0.5, 0.0, 1.0)
                        
                    actions[agent] = np.array([linear_vel, angular_vel], dtype=np.float32)
                else:
                    actions[agent] = np.array([0.0, 0.0], dtype=np.float32)
            
            # Check if all active agents have finished their coverage work
            all_done = True
            for agent in env.agents:
                state = env.last_poses[agent]
                if len(get_reachable_unvisited_frontiers(env, state)) > 0:
                    all_done = False
                    break
            if all_done and len(env.agents) > 0:
                print("\n[demo] All reachable areas explored! Finishing early...")
                break
            
            # Take environment step
            obs_dict, rewards, terminations, truncations, infos = env.step(actions)
            
            # Save actions for watchdog check next step
            for agent in ['tb1', 'tb2', 'tb3']:
                if agent in actions:
                    last_actions[agent] = actions[agent]
                else:
                    last_actions[agent] = np.array([0.0, 0.0])
            
            steps += 1
            total_cells = env.grid_resolution_x * env.grid_resolution_y
            visited_count = np.sum(env.visited_grid)
            coverage_pct = (visited_count / total_cells) * 100.0
            coverage_history.append(coverage_pct)
            
            # Format positions string for logging
            pos_strings = []
            for agent in ['tb1', 'tb2', 'tb3']:
                if agent in env.last_poses:
                    pos = env.last_poses[agent]
                    pos_strings.append(f"{agent}: ({pos[0]:.2f}, {pos[1]:.2f})")
                else:
                    pos_strings.append(f"{agent}: offline")
                    
            print(f"Step {steps:03d} | {' | '.join(pos_strings)} | Visited Cells: {visited_count}/{total_cells} ({coverage_pct:.1f}%)")
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n[demo] Simulation interrupted by user.")
    finally:
        # Gracefully stop ROS bag recording
        if bag_proc:
            print("[demo] Stopping ROS bag recording...")
            bag_proc.terminate()
            try:
                bag_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bag_proc.kill()
            print(f"[demo] ROS bag saved in: {bag_dir}")
            
        # Generate and save coverage plot
        save_coverage_plot(trajectory_history, coverage_history, env, timestamp)
        
        print("\nDemonstration finished. Cleaning up...")
        env.close()
        kill_stale_processes()

if __name__ == '__main__':
    main()
