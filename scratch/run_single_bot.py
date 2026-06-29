import os
import sys
import time
import numpy as np

# Ensure path includes workspace packages
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/mars_swarm/mars_swarm')))
from multi_env_wrapper import PettingZooSwarmEnv
from train_multi import start_gazebo, kill_stale_processes

def main():
    # Kill any stale processes first
    kill_stale_processes()
    
    # Start Gazebo simulation with GUI
    start_gazebo(headless=False, multi=False)
    
    # Initialize the environment
    env = PettingZooSwarmEnv(agents=['tb1'])
    obs_dict, infos = env.reset()
    
    print("\n" + "="*50)
    print("  SINGLE ROBOT MOVEMENT DEMONSTRATION")
    print("  Only tb1 will explore the arena using the Frontier Heuristic.")
    print("  tb2 and tb3 will remain stationary.")
    print("="*50 + "\n")
    
    steps = 0
    max_steps = 150
    
    # Keep running until step limit or terminated
    while steps < max_steps:
        actions = {}
        
        # 1. Compute Frontier Heuristic action for tb1
        if 'tb1' in env.agents:
            state = env.last_poses['tb1']  # x, y, yaw
            min_x, max_x, min_y, max_y = env.grid_bounds
            
            # Find closest unvisited cell
            unvisited_coords = []
            for r in range(env.grid_resolution):
                for c in range(env.grid_resolution):
                    if not env.visited_grid[r, c]:
                        # Skip if this cell contains a known obstacle (wall) in the high-res map
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
                            
                        cx = min_x + (c + 0.5) * (max_x - min_x) / env.grid_resolution
                        cy = min_y + (r + 0.5) * (max_y - min_y) / env.grid_resolution
                        unvisited_coords.append((cx, cy))
            
            if len(unvisited_coords) > 0:
                dists = [np.hypot(cx - state[0], cy - state[1]) for cx, cy in unvisited_coords]
                best_idx = np.argmin(dists)
                tx, ty = unvisited_coords[best_idx]
                
                # Reactive Obstacle Avoidance using Lidar
                agent_obs = obs_dict['tb1']
                front_beams = agent_obs[10:15]
                min_front_dist = np.min(front_beams)
                
                if min_front_dist < 0.45:
                    # Obstacle very close! Back up slightly and spin away
                    linear_vel = -0.05
                    left_dist = np.min(agent_obs[14:18])
                    right_dist = np.min(agent_obs[6:10])
                    angular_vel = 0.6 if left_dist > right_dist else -0.6
                    actions['tb1'] = np.array([linear_vel, angular_vel])
                elif min_front_dist < 0.7:
                    # Obstacle ahead! Slow down and steer away
                    linear_vel = 0.05
                    left_dist = np.min(agent_obs[14:18])
                    right_dist = np.min(agent_obs[6:10])
                    angular_vel = 0.5 if left_dist > right_dist else -0.5
                    actions['tb1'] = np.array([linear_vel, angular_vel])
                else:
                    # Simple proportional heading control to target (tx, ty)
                    dx = tx - state[0]
                    dy = ty - state[1]
                    target_yaw = np.arctan2(dy, dx)
                    
                    angle_diff = target_yaw - state[2]
                    # Normalize angle to [-pi, pi]
                    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
                    
                    # Compute linear and angular velocities
                    linear_vel = 0.18 if abs(angle_diff) < 0.5 else 0.05
                    angular_vel = np.clip(1.5 * angle_diff, -0.8, 0.8)
                    actions['tb1'] = np.array([linear_vel, angular_vel])
            else:
                actions['tb1'] = np.array([0.0, 0.0])
        
        # 2. Command stationary zero velocity for tb2 and tb3
        for agent in ['tb2', 'tb3']:
            if agent in env.agents:
                actions[agent] = np.array([0.0, 0.0])
                
        # Take environment step
        obs_dict, rewards, terminations, truncations, infos = env.step(actions)
        
        steps += 1
        total_cells = env.grid_resolution * env.grid_resolution
        print(f"Step {steps:03d} | tb1 Position: x={env.last_poses['tb1'][0]:.2f}, y={env.last_poses['tb1'][1]:.2f} | Visited Cells: {np.sum(env.visited_grid)}/{total_cells}")
        
        # If tb1 got terminated, break
        if 'tb1' not in env.agents or terminations.get('tb1', False):
            print("\ntb1 reached its target or terminated.")
            break
            
        time.sleep(0.05)
        
    print("\nDemonstration finished. Cleaning up...")
    env.close()
    kill_stale_processes()

if __name__ == '__main__':
    main()
