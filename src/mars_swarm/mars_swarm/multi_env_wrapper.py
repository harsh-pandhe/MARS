import os
import math
import time
import threading
import subprocess
import numpy as np
import gymnasium as gym
from pettingzoo import ParallelEnv

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped

def euler_from_quaternion(x, y, z, w):
    """Convert a quaternion into euler angles (roll, pitch, yaw)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw

def bresenham_line(r0, c0, r1, c1, resolution_r, resolution_c):
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    
    r, c = r0, c0
    while True:
        if 0 <= r < resolution_r and 0 <= c < resolution_c:
            cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells

from rclpy.parameter import Parameter

class SwarmNode(Node):
    def __init__(self, node_name, agents=['tb1', 'tb2', 'tb3']):
        super().__init__(
            node_name,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )
        self.agents = agents
        
        self.scan_msgs = {agent: None for agent in agents}
        self.odom_msgs = {agent: None for agent in agents}
        self.lock = threading.Lock()
        
        self.scan_events = {agent: threading.Event() for agent in agents}
        self.odom_events = {agent: threading.Event() for agent in agents}
        
        # Subscriptions and Publishers
        self.scan_subs = {}
        self.odom_subs = {}
        self.cmd_vel_pubs = {}
        self.path_pubs = {}
        self.paths = {}
        
        # Create coverage map publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        
        for agent in agents:
            scan_topic = f"/{agent}/scan"
            odom_topic = f"/{agent}/odom"
            cmd_vel_topic = f"/{agent}/cmd_vel"
            path_topic = f"/{agent}/path"
            
            self.scan_subs[agent] = self.create_subscription(
                LaserScan, scan_topic, self._make_scan_callback(agent), 10
            )
            self.odom_subs[agent] = self.create_subscription(
                Odometry, odom_topic, self._make_odom_callback(agent), 10
            )
            self.cmd_vel_pubs[agent] = self.create_publisher(
                Twist, cmd_vel_topic, 10
            )
            self.path_pubs[agent] = self.create_publisher(
                Path, path_topic, 10
            )
            
            path = Path()
            path.header.frame_id = 'tb1/odom'
            self.paths[agent] = path
            
    def _make_scan_callback(self, agent):
        def callback(msg):
            with self.lock:
                self.scan_msgs[agent] = msg
                self.scan_events[agent].set()
        return callback
        
    def _make_odom_callback(self, agent):
        def callback(msg):
            with self.lock:
                self.odom_msgs[agent] = msg
                self.odom_events[agent].set()
        return callback
        
    def get_latest_data(self, timeout=10.0):
        for agent in self.agents:
            self.scan_events[agent].clear()
            self.odom_events[agent].clear()
            
        results = {}
        for agent in self.agents:
            scan_ok = self.scan_events[agent].wait(timeout)
            odom_ok = self.odom_events[agent].wait(timeout)
            
            if not scan_ok:
                print(f"[multi_env_wrapper] WARNING: LaserScan subscription timed out for {agent}!")
            if not odom_ok:
                print(f"[multi_env_wrapper] WARNING: Odometry subscription timed out for {agent}!")
                
            with self.lock:
                results[agent] = (self.scan_msgs[agent], self.odom_msgs[agent])
        return results

class PettingZooSwarmEnv(ParallelEnv):
    metadata = {'render_modes': ['human'], "name": "mars_swarm_v0"}

    def __init__(self, agents=['tb1', 'tb2', 'tb3'], max_steps=300):
        super().__init__()
        self.agents = agents
        self.possible_agents = agents[:]
        self.max_steps = max_steps
        self.step_count = 0
        
        self.num_lidar_beams = 24
        
        # 24 (lidar) + 2 (goal rel) + 2 (vel) + 18 (9 neighbors * 2) = 46
        self.observation_spaces = {
            agent: gym.spaces.Box(low=-np.inf, high=np.inf, shape=(46,), dtype=np.float32)
            for agent in agents
        }
        self.action_spaces = {
            agent: gym.spaces.Box(low=np.array([-0.22, -1.0]), high=np.array([0.22, 1.0]), dtype=np.float32)
            for agent in agents
        }
        
        self.safe_goals_world = [
            (-3.5, -8.0), (2.5, -8.0),
            (-3.5, -3.0), (0.0, -3.0), (2.5, -3.0),
            (-3.5, 2.0), (0.0, 2.0), (2.5, 2.0),
            (-2.0, 6.0), (0.0, 6.0), (2.0, 6.0)
        ]
        self.safe_goals = []
        for gx, gy in self.safe_goals_world:
            gx_local = -gy - 0.7113
            gy_local = gx
            self.safe_goals.append((gx_local, gy_local))
            
        self.goal_positions = {agent: np.zeros(2, dtype=np.float32) for agent in agents}
        self.prev_goal_dists = {agent: 10.0 for agent in agents}
        
        # Grid parameters for area coverage (in tb1/odom frame: x_local in [-7.7113, 9.2887], y_local in [-4.5, 3.5])
        self.grid_bounds = (-7.7113, 9.2887, -4.5, 3.5)
        self.grid_resolution_x = 40
        self.grid_resolution_y = 20
        self.visited_grid = np.zeros((self.grid_resolution_y, self.grid_resolution_x), dtype=bool)
        
        # High-resolution grid for SLAM-like OccupancyGrid visualization in RViz
        self.map_resolution = 0.1
        self.viz_resolution_x = int((self.grid_bounds[1] - self.grid_bounds[0]) / self.map_resolution) # 170
        self.viz_resolution_y = int((self.grid_bounds[3] - self.grid_bounds[2]) / self.map_resolution) # 80
        self.viz_grid = np.full((self.viz_resolution_y, self.viz_resolution_x), -1, dtype=np.int8)
        
        self.last_obs = {
            agent: np.zeros(32, dtype=np.float32) for agent in agents
        }
        self.spawn_poses = {
            'tb1': (0.0, 0.0, 0.0),
            'tb2': (0.433, -0.25, 2.0944),
            'tb3': (0.433, 0.25, -2.0944)
        }
        self.odom_offsets = {
            agent: (0.0, 0.0, 0.0) for agent in agents
        }
        self.last_poses = {
            agent: self.spawn_poses[agent] for agent in agents
        }
        self.last_dists = {
            agent: 10.0 for agent in agents
        }
        
        rclpy.init(args=None)
        node_name = f"mars_swarm_node_{int(time.time() * 1000) % 10000}"
        self.node = SwarmNode(node_name, agents=self.agents)
        
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()
        
    def _spin(self):
        try:
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor.spin()
        except Exception as e:
            print(f"[multi_env_wrapper] Executor Exception: {e}")
            
    def _reset_sim(self):
        cmd = [
            "gz", "service", "-s", "/world/default/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "reset: { model_only: true }"
        ]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        except Exception as e:
            self.node.get_logger().error(f"Failed to reset Gazebo simulation: {e}")
            
    def _get_obs_and_states(self):
        data = self.node.get_latest_data(timeout=10.0)
        
        poses = {}
        velocities = {}
        for agent in self.possible_agents:
            scan_msg, odom_msg = data[agent]
            if odom_msg is not None:
                pos = odom_msg.pose.pose.position
                orientation = odom_msg.pose.pose.orientation
                yaw = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
                
                # Calibrate offsets at step 0 (during env reset)
                if self.step_count == 0:
                    self.odom_offsets[agent] = (pos.x, pos.y, yaw)
                
                # Translate relative coordinates to absolute global coordinates with rotation
                local_x = pos.x - self.odom_offsets[agent][0]
                local_y = pos.y - self.odom_offsets[agent][1]
                local_yaw = yaw - self.odom_offsets[agent][2]
                
                spawn_x, spawn_y, spawn_yaw = self.spawn_poses[agent]
                global_x = spawn_x + local_x * math.cos(spawn_yaw) - local_y * math.sin(spawn_yaw)
                global_y = spawn_y + local_x * math.sin(spawn_yaw) + local_y * math.cos(spawn_yaw)
                global_yaw = local_yaw + spawn_yaw
                global_yaw = math.atan2(math.sin(global_yaw), math.cos(global_yaw))
                
                poses[agent] = (global_x, global_y, global_yaw)
                velocities[agent] = (odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z)
                
                if self.step_count % 10 == 0 or self.step_count == 1:
                    print(f"[DEBUG ODOM] {agent}: raw_x={pos.x:.3f}, raw_y={pos.y:.3f}, raw_yaw={yaw:.3f} | global_x={global_x:.3f}, global_y={global_y:.3f}, global_yaw={global_yaw:.3f}")
            else:
                poses[agent] = self.last_poses[agent]
                velocities[agent] = (0.0, 0.0)
                
        observations = {}
        states = {}
        collisions = {}
        dists = {}
        
        for agent in self.possible_agents:
            scan_msg, odom_msg = data[agent]
            if scan_msg is None or odom_msg is None:
                observations[agent] = self.last_obs[agent]
                states[agent] = self.last_poses[agent]
                collisions[agent] = False
                dists[agent] = self.last_dists[agent]
                continue
                
            x, y, yaw = poses[agent]
            linear_vel, angular_vel = velocities[agent]
            
            # Goal rel
            goal_x, goal_y = self.goal_positions[agent]
            goal_dist = math.hypot(goal_x - x, goal_y - y)
            goal_angle = math.atan2(goal_y - y, goal_x - x) - yaw
            goal_angle = math.atan2(math.sin(goal_angle), math.cos(goal_angle))
            
            # Process Lidar
            raw_ranges = np.array(scan_msg.ranges, dtype=np.float32)
            raw_ranges[raw_ranges <= 0.05] = scan_msg.range_max
            raw_ranges = np.nan_to_num(raw_ranges, nan=scan_msg.range_max, posinf=scan_msg.range_max, neginf=scan_msg.range_min)
            actual_min_range = max(scan_msg.range_min, 0.12)
            raw_ranges = np.clip(raw_ranges, actual_min_range, scan_msg.range_max)
            
            sector_size = len(raw_ranges) // self.num_lidar_beams
            lidar_obs = []
            for i in range(self.num_lidar_beams):
                sector = raw_ranges[i*sector_size : (i+1)*sector_size]
                lidar_obs.append(np.min(sector) if len(sector) > 0 else scan_msg.range_max)
            lidar_obs = np.array(lidar_obs, dtype=np.float32)
            
            collision = np.min(lidar_obs) < 0.20
            if collision:
                print(f"[DEBUG] {agent} LIDAR COLLISION! Min range: {np.min(lidar_obs):.3f} m (beam index: {np.argmin(lidar_obs)})")
            if self.step_count <= 15:
                collision = False
            
            # Neighbor features
            neighbor_feats = []
            for other in self.possible_agents:
                if other == agent:
                    continue
                # Only include active, alive neighbors
                if other in self.agents and other in poses:
                    ox, oy, _ = poses[other]
                    n_dist = math.hypot(ox - x, oy - y)
                    n_angle = math.atan2(oy - y, ox - x) - yaw
                    n_angle = math.atan2(math.sin(n_angle), math.cos(n_angle))
                    neighbor_feats.extend([n_dist, n_angle])
                
            # Pad neighbor features to support up to 9 neighbors (max 10 robots)
            while len(neighbor_feats) < 18:
                neighbor_feats.extend([10.0, 0.0])
                
            obs = np.zeros(46, dtype=np.float32)
            obs[0:24] = lidar_obs
            obs[24] = goal_dist
            obs[25] = goal_angle
            obs[26] = linear_vel
            obs[27] = angular_vel
            obs[28:46] = neighbor_feats[:18]
            
            observations[agent] = obs
            states[agent] = (x, y, yaw)
            collisions[agent] = collision
            dists[agent] = goal_dist
            
            self.last_obs[agent] = obs
            self.last_poses[agent] = (x, y, yaw)
            self.last_dists[agent] = goal_dist
            
        return observations, states, dists, collisions

    def _mark_visited(self, x, y):
        min_x, max_x, min_y, max_y = self.grid_bounds
        x_clipped = np.clip(x, min_x, max_x - 1e-5)
        y_clipped = np.clip(y, min_y, max_y - 1e-5)
        
        col = int((x_clipped - min_x) / (max_x - min_x) * self.grid_resolution_x)
        row = int((y_clipped - min_y) / (max_y - min_y) * self.grid_resolution_y)
        col = np.clip(col, 0, self.grid_resolution_x - 1)
        row = np.clip(row, 0, self.grid_resolution_y - 1)
        
        newly_visited = not self.visited_grid[row, col]
        self.visited_grid[row, col] = True
        return newly_visited

    def publish_coverage_map(self):
        try:
            # Update visualization grid using Lidar ray-tracing
            for agent in self.agents:
                scan_msg = self.node.scan_msgs.get(agent)
                if scan_msg is None:
                    continue
                pose = self.last_poses.get(agent)
                if pose is None:
                    continue
                
                x, y, yaw = pose
                
                # Append pose to path message
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
                pose_stamped.header.frame_id = 'tb1/odom'
                pose_stamped.pose.position.x = float(x)
                pose_stamped.pose.position.y = float(y)
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
                
                if agent in self.node.paths:
                    self.node.paths[agent].poses.append(pose_stamped)
                    if len(self.node.paths[agent].poses) > 800:
                        self.node.paths[agent].poses.pop(0)
                    self.node.paths[agent].header.stamp = self.node.get_clock().now().to_msg()
                    self.node.path_pubs[agent].publish(self.node.paths[agent])
                
                min_x, max_x, min_y, max_y = self.grid_bounds
                
                # Start cell indices for viz_grid
                c0 = int(np.clip((x - min_x) / (max_x - min_x) * self.viz_resolution_x, 0, self.viz_resolution_x - 1))
                r0 = int(np.clip((y - min_y) / (max_y - min_y) * self.viz_resolution_y, 0, self.viz_resolution_y - 1))
                
                # Set robot's own footprint area to free (0)
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        rr, cc = r0 + dr, c0 + dc
                        if 0 <= rr < self.viz_resolution_y and 0 <= cc < self.viz_resolution_x:
                            self.viz_grid[rr, cc] = 0
                            # Map back to visited grid
                            r_grid = int(rr * self.grid_resolution_y / self.viz_resolution_y)
                            c_grid = int(cc * self.grid_resolution_x / self.viz_resolution_x)
                            self.visited_grid[r_grid, c_grid] = True
                
                ranges = np.array(scan_msg.ranges, dtype=np.float32)
                angle_min = scan_msg.angle_min
                angle_inc = scan_msg.angle_increment
                range_max = scan_msg.range_max
                
                # Step by 4 to keep performance high (90 rays per robot)
                for i in range(0, len(ranges), 4):
                    r = ranges[i]
                    if np.isnan(r) or np.isinf(r) or r < 0.12:
                        continue
                    
                    angle = angle_min + i * angle_inc
                    global_angle = yaw + angle
                    
                    is_obstacle = r < (range_max - 0.2)
                    r_clipped = min(r, range_max)
                    
                    ex = x + r_clipped * math.cos(global_angle)
                    ey = y + r_clipped * math.sin(global_angle)
                    
                    c1 = int(np.clip((ex - min_x) / (max_x - min_x) * self.viz_resolution_x, 0, self.viz_resolution_x - 1))
                    r1 = int(np.clip((ey - min_y) / (max_y - min_y) * self.viz_resolution_y, 0, self.viz_resolution_y - 1))
                    
                    line_cells = bresenham_line(r0, c0, r1, c1, self.viz_resolution_y, self.viz_resolution_x)
                    
                    # Mark cells along the ray as free (0) if within coverage limit (2.0m)
                    for rr, cc in line_cells[:-1]:
                        cell_x = min_x + (cc + 0.5) * self.map_resolution
                        cell_y = min_y + (rr + 0.5) * self.map_resolution
                        if math.hypot(cell_x - x, cell_y - y) <= 2.0:
                            self.viz_grid[rr, cc] = 0
                            # Map back to visited grid
                            r_grid = int(rr * self.grid_resolution_y / self.viz_resolution_y)
                            c_grid = int(cc * self.grid_resolution_x / self.viz_resolution_x)
                            self.visited_grid[r_grid, c_grid] = True
                    
                    # Mark the last cell
                    if len(line_cells) > 0:
                        rr, cc = line_cells[-1]
                        cell_x = min_x + (cc + 0.5) * self.map_resolution
                        cell_y = min_y + (rr + 0.5) * self.map_resolution
                        dist = math.hypot(cell_x - x, cell_y - y)
                        if is_obstacle:
                            # Always mark obstacle cells to ensure collision avoidance uses mapped boundaries
                            self.viz_grid[rr, cc] = 100
                        else:
                            if dist <= 2.0:
                                self.viz_grid[rr, cc] = 0
                                # Map back to visited grid
                                r_grid = int(rr * self.grid_resolution_y / self.viz_resolution_y)
                                c_grid = int(cc * self.grid_resolution_x / self.viz_resolution_x)
                                self.visited_grid[r_grid, c_grid] = True
            
            # Now build and publish the OccupancyGrid message using self.viz_grid
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = self.node.get_clock().now().to_msg()
            grid_msg.header.frame_id = 'tb1/odom'
            
            grid_msg.info.resolution = self.map_resolution
            grid_msg.info.width = self.viz_resolution_x
            grid_msg.info.height = self.viz_resolution_y
            grid_msg.info.origin.position.x = float(self.grid_bounds[0])
            grid_msg.info.origin.position.y = float(self.grid_bounds[2])
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0
            
            grid_msg.data = self.viz_grid.flatten().tolist()
            
            self.node.map_pub.publish(grid_msg)
        except Exception as e:
            print(f"[multi_env_wrapper] Error publishing coverage map: {e}")

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.np_random = np.random.default_rng(seed)
        elif not hasattr(self, 'np_random') or self.np_random is None:
            self.np_random = np.random.default_rng()
            
        self.step_count = 0
        self.agents = self.possible_agents[:]
        self.visited_grid.fill(False)
        self.viz_grid.fill(-1)
        for agent in self.possible_agents:
            if agent in self.node.paths:
                self.node.paths[agent].poses = []
        
        # Shuffle goal positions from safe targets
        for agent in self.agents:
            goal_idx = self.np_random.integers(0, len(self.safe_goals))
            self.goal_positions[agent] = np.array(self.safe_goals[goal_idx], dtype=np.float32)
            
        self._reset_sim()
        time.sleep(2.0)
        
        # Clear transient messages received during reset settling time
        for agent in self.agents:
            self.node.scan_events[agent].clear()
            self.node.odom_events[agent].clear()
        
        obs_dict, state_dict, dist_dict, collision_dict = self._get_obs_and_states()
        
        for agent in self.agents:
            self.prev_goal_dists[agent] = dist_dict[agent]
            x, y, _ = state_dict[agent]
            self._mark_visited(x, y)
            
        self.publish_coverage_map()
        return obs_dict, {agent: {} for agent in self.agents}

    def step(self, actions):
        # Apply actions
        for agent in self.possible_agents:
            if agent in self.agents and agent in actions:
                action = actions[agent]
                twist = Twist()
                # Clamp velocities strictly to physical TurtleBot3 Waffle limits
                twist.linear.x = float(np.clip(action[0], -0.22, 0.22))
                twist.angular.z = float(np.clip(action[1], -1.0, 1.0))
                self.node.cmd_vel_pubs[agent].publish(twist)
            else:
                # Stop completed/inactive agents
                twist = Twist()
                self.node.cmd_vel_pubs[agent].publish(twist)
                
        time.sleep(0.05)
        self.step_count += 1
        
        obs_dict, state_dict, dist_dict, collision_dict = self._get_obs_and_states()
        
        # Check newly visited cells for cooperative swarm coverage reward
        new_cells_visited = 0
        for agent in self.agents:
            x, y, _ = state_dict[agent]
            if self._mark_visited(x, y):
                new_cells_visited += 1
        coverage_reward = float(new_cells_visited) * 2.0
        
        self.publish_coverage_map()
        
        rewards = {}
        terminations = {}
        truncations = {}
        infos = {}
        
        truncated = self.step_count >= self.max_steps
        
        for agent in self.possible_agents:
            if agent not in self.agents:
                # Agent already inactive, but PettingZoo API requires outputting dummy variables
                # for completeness if RLlib asks, though we typically remove them.
                continue
                
            dist = dist_dict[agent]
            collision = collision_dict[agent]
            
            progress = self.prev_goal_dists[agent] - dist
            self.prev_goal_dists[agent] = dist
            
            # Step penalty + progress reward + cooperative coverage reward
            reward = -0.01 + progress * 5.0 + coverage_reward
            
            goal_reached = dist < 0.3
            if goal_reached:
                reward += 20.0
                
            if collision:
                reward -= 20.0
                
            # Inter-agent proximity penalties
            x, y, _ = state_dict[agent]
            for other in self.possible_agents:
                if other == agent:
                    continue
                ox, oy, _ = state_dict[other]
                sep = math.hypot(ox - x, oy - y)
                if sep < 0.5:
                    reward -= 0.5
                    if sep < 0.20:
                        reward -= 10.0
                        if self.step_count > 15:
                            collision = True
                            print(f"[DEBUG] {agent} INTER-AGENT COLLISION with {other}! Distance: {sep:.3f} m")
                        
            rewards[agent] = reward
            
            terminated = collision or goal_reached
            terminations[agent] = terminated
            truncations[agent] = truncated
            infos[agent] = {
                'x': state_dict[agent][0],
                'y': state_dict[agent][1],
                'yaw': state_dict[agent][2],
                'goal_dist': dist,
                'status': 'SUCCESS' if goal_reached else ('FAILED' if collision else 'RUNNING')
            }
            
        # Filter obs to include only active agents for this step
        filtered_obs = {agent: obs_dict[agent] for agent in self.agents}
        
        # Update active agents list
        for agent in list(self.agents):
            if terminations[agent] or truncations[agent]:
                self.agents.remove(agent)
                
        return filtered_obs, rewards, terminations, truncations, infos

    def close(self):
        # Stop all robots
        for agent in self.possible_agents:
            try:
                self.node.cmd_vel_pubs[agent].publish(Twist())
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def observation_space(self, agent):
        return self.observation_spaces[agent]

    def action_space(self, agent):
        return self.action_spaces[agent]
