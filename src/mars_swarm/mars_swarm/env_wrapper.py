import os
import math
import time
import subprocess
import threading
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def euler_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class TurtleBot3Node(Node):
    def __init__(self, node_name, namespace=''):
        super().__init__(
            node_name,
            namespace=namespace,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )
        

        
        self.scan_msg = None
        self.odom_msg = None
        self.lock = threading.Lock()
        self.scan_event = threading.Event()
        self.odom_event = threading.Event()
        
        # Use absolute paths with optional namespace to prevent relative topic resolution issues
        scan_topic = f"/{namespace}/scan" if namespace else "/scan"
        odom_topic = f"/{namespace}/odom" if namespace else "/odom"
        cmd_vel_topic = f"/{namespace}/cmd_vel" if namespace else "/cmd_vel"
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )
        
    def scan_callback(self, msg):
        # print("[env_wrapper] scan_callback triggered")
        with self.lock:
            self.scan_msg = msg
            self.scan_event.set()
            
    def odom_callback(self, msg):
        with self.lock:
            self.odom_msg = msg
            self.odom_event.set()
            
    def get_latest_data(self, timeout=10.0):
        # Wait for new messages
        self.scan_event.clear()
        self.odom_event.clear()
        
        scan_ok = self.scan_event.wait(timeout)
        odom_ok = self.odom_event.wait(timeout)
        
        if not scan_ok:
            print("[env_wrapper] WARNING: LaserScan subscription timed out!")
        if not odom_ok:
            print("[env_wrapper] WARNING: Odometry subscription timed out!")
        
        if not (scan_ok and odom_ok):
            return None, None
            
        with self.lock:
            return self.scan_msg, self.odom_msg

class TurtleBot3Env(gym.Env):
    metadata = {'render_modes': ['human']}
    
    def __init__(self, namespace='', max_steps=300):
        super().__init__()
        self.namespace = namespace
        self.max_steps = max_steps
        self.step_count = 0
        
        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init()
            
        # Create ROS 2 Node
        node_name = f"mars_env_node_{int(time.time() * 1000) % 10000}"
        self.node = TurtleBot3Node(node_name, namespace=namespace)
        
        # Spin ROS 2 in a background thread
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()
        
        # Goal configuration
        # Predefined safe positions in cafe_world to avoid obstacles
        self.safe_goals = [
            (-3.5, -8.0), (2.5, -8.0),
            (-3.5, -3.0), (0.0, -3.0), (2.5, -3.0),
            (-3.5, 2.0), (0.0, 2.0), (2.5, 2.0),
            (-2.0, 6.0), (0.0, 6.0), (2.0, 6.0)
        ]
        self.goal_pos = np.array([0.0, 2.0], dtype=np.float32)
        
        # Spaces definition
        self.num_lidar_beams = 24
        
        # Observation space: 24 lidar beams + 4 robot state elements:
        # [lidar_1..24, relative_goal_dist, relative_goal_angle, linear_vel, angular_vel]
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.num_lidar_beams + 4,),
            dtype=np.float32
        )
        
        # Action space: [linear_vel_cmd, angular_vel_cmd] in range [-1.0, 1.0]
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )
        
        self.prev_goal_dist = 0.0
        
        # Last-state caching for robust timeout fallback
        self.last_obs = np.zeros(self.num_lidar_beams + 4, dtype=np.float32)
        self.last_pose = (-2.0, -0.5, 0.0)
        self.last_dist = 10.0
        
    def _spin(self):
        try:
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor.spin()
        except Exception as e:
            print(f"[env_wrapper] Executor Exception: {e}")
        
    def _reset_sim(self):
        # Call Gazebo simulation reset service
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
            
    def _get_obs_and_state(self):
        # Fetch latest LaserScan and Odometry with extended timeout for simulator reset lags
        scan_msg, odom_msg = self.node.get_latest_data(timeout=10.0)
        
        if scan_msg is None or odom_msg is None:
            # Fallback to last known state if messages are transiently missing
            return self.last_obs, self.last_pose, self.last_dist, False
            
        # Check if scan is uninitialized (e.g. all zeros or empty)
        if len(scan_msg.ranges) == 0 or np.all(np.array(scan_msg.ranges) == 0.0):
            # It's an uninitialized scan, use last valid state
            return self.last_obs, self.last_pose, self.last_dist, False
            
        # 1. Process Lidar Data (sub-sample from 360 to 24 beams)
        # LaserScan contains ranges. Waffle scan has 360 beams.
        raw_ranges = np.array(scan_msg.ranges, dtype=np.float32)
        
        # Replace zero or near-zero values (uninitialized / invalid) with range_max
        raw_ranges[raw_ranges <= 0.05] = scan_msg.range_max
        
        # Handle nan/inf
        raw_ranges = np.nan_to_num(raw_ranges, nan=scan_msg.range_max, posinf=scan_msg.range_max, neginf=scan_msg.range_min)
        
        # Min range threshold: Waffle LDS has a physical minimum range of 0.12m
        actual_min_range = max(scan_msg.range_min, 0.12)
        raw_ranges = np.clip(raw_ranges, actual_min_range, scan_msg.range_max)
        
        # Downsample by taking min values in 24 sectors to capture nearby obstacles reliably
        sector_size = len(raw_ranges) // self.num_lidar_beams
        lidar_obs = []
        for i in range(self.num_lidar_beams):
            sector = raw_ranges[i*sector_size : (i+1)*sector_size]
            lidar_obs.append(np.min(sector) if len(sector) > 0 else scan_msg.range_max)
        lidar_obs = np.array(lidar_obs, dtype=np.float32)
        
        # 2. Process Odometry State
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        yaw = euler_from_quaternion(ori)
        
        # Velocities
        linear_vel = odom_msg.twist.twist.linear.x
        angular_vel = odom_msg.twist.twist.angular.z
        
        # 3. Calculate Relative Goal State
        dx = self.goal_pos[0] - pos.x
        dy = self.goal_pos[1] - pos.y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Relative angle to goal in robot frame
        global_goal_angle = math.atan2(dy, dx)
        rel_angle = global_goal_angle - yaw
        # Normalize rel_angle to [-pi, pi]
        rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
        
        # 4. Check for Collisions
        # Waffle size is roughly 0.3m. A reading under 0.20m is a collision.
        collision = np.min(lidar_obs) < 0.20
        if self.step_count <= 5:
            collision = False
        
        # Construct final observation array
        obs = np.zeros(self.num_lidar_beams + 4, dtype=np.float32)
        obs[0:self.num_lidar_beams] = lidar_obs
        obs[self.num_lidar_beams] = dist
        obs[self.num_lidar_beams + 1] = rel_angle
        obs[self.num_lidar_beams + 2] = linear_vel
        obs[self.num_lidar_beams + 3] = angular_vel
        
        # Update cached state
        self.last_obs = obs
        self.last_pose = (pos.x, pos.y, yaw)
        self.last_dist = dist
        
        return obs, self.last_pose, dist, collision
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        
        # Select a random goal from safe locations
        goal_idx = self.np_random.integers(0, len(self.safe_goals))
        self.goal_pos = np.array(self.safe_goals[goal_idx], dtype=np.float32)
        
        # Reset the Gazebo simulation
        self._reset_sim()
        time.sleep(2.0)  # Wait for simulation to settle and publish first messages
        
        # Clear transient messages received during reset settling time
        self.node.scan_event.clear()
        self.node.odom_event.clear()
        
        # Get first observation
        obs, state, dist, collision = self._get_obs_and_state()
        self.prev_goal_dist = dist
        
        info = {
            'x': state[0],
            'y': state[1],
            'yaw': state[2],
            'goal_dist': dist
        }
        
        return obs, info
        
    def step(self, action):
        self.step_count += 1
        
        # Action space is normalized to [-1.0, 1.0]. Map to physical velocities:
        # Linear velocity: [0.0, 0.22] m/s (forward-only to make navigation stable)
        # Angular velocity: [-1.0, 1.0] rad/s
        linear_cmd = float(np.clip((action[0] + 1.0) / 2.0 * 0.22, 0.0, 0.22))
        angular_cmd = float(np.clip(action[1] * 1.0, -1.0, 1.0))
        
        # Publish Twist command
        twist = Twist()
        twist.linear.x = linear_cmd
        twist.angular.z = angular_cmd
        self.node.cmd_vel_pub.publish(twist)
        
        # Retrieve new state and sensor readings
        obs, state, dist, collision = self._get_obs_and_state()
        
        # Initialize rewards and check termination conditions
        reward = 0.0
        terminated = False
        truncated = False
        
        # 1. Goal reaching condition
        if dist < 0.35:
            reward = 20.0
            terminated = True
            self.node.get_logger().info("Goal Reached!")
            
        # 2. Collision condition
        elif collision:
            reward = -20.0
            terminated = True
            self.node.get_logger().info("Collision Detected!")
            
        # 3. Default shaping reward (progress + step penalty)
        else:
            # Progress reward: positive if getting closer, negative if moving away
            progress = self.prev_goal_dist - dist
            reward = progress * 10.0 - 0.02
            
        # 4. Truncation condition (timeout)
        if self.step_count >= self.max_steps:
            truncated = True
            
        self.prev_goal_dist = dist
        
        info = {
            'x': state[0],
            'y': state[1],
            'yaw': state[2],
            'goal_dist': dist,
            'collision': collision
        }
        
        # Publish zero velocity command on termination to stop the robot
        if terminated or truncated:
            stop_twist = Twist()
            self.node.cmd_vel_pub.publish(stop_twist)
            
        return obs, reward, terminated, truncated, info
        
    def close(self):
        # Stop publishing velocity
        try:
            stop_twist = Twist()
            self.node.cmd_vel_pub.publish(stop_twist)
        except Exception:
            pass
            
        # Shutdown executor
        if hasattr(self, 'executor'):
            try:
                self.executor.shutdown()
            except Exception:
                pass
            
        # Destroy node and clean up
        self.node.destroy_node()
