"""
Lightweight 2D Scan-Matching Localization & Odometry Drift Correction.

Eliminates dead-reckoning drift over long-duration swarm missions by aligning
2D LiDAR returns against confirmed static obstacles in the occupancy grid using
a Correlative Distance-Field Scan Matcher with EMA smoothing.
"""

import math
import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class ScanMatchingLocalizer:
    """
    Corrects wheel odometry drift using 2D scan-to-map correlative matching.
    """
    
    def __init__(self, search_dist=0.06, search_yaw=0.04, alpha=0.35, max_correction_m=0.5):
        self.search_dist = float(search_dist)
        self.search_yaw = float(search_yaw)
        self.alpha = float(alpha)
        self.max_correction_m = float(max_correction_m)
        
        # Per-agent cumulative correction offsets: [dx, dy, dyaw]
        self.corrections = {}
        
    def reset(self, agent=None):
        if agent is None:
            self.corrections.clear()
        elif agent in self.corrections:
            self.corrections[agent] = np.zeros(3, dtype=np.float32)

    def get_corrected_pose(
        self,
        agent,
        raw_odom_pose,
        scan_msg,
        viz_grid,
        grid_bounds,
        viz_res_x,
        viz_res_y
    ):
        """
        Computes the SLAM-corrected pose for an agent.
        
        Parameters:
            agent (str): Agent identifier (e.g. 'tb1')
            raw_odom_pose (tuple): (x, y, yaw) from calibrated dead reckoning
            scan_msg (LaserScan): Current 2D LiDAR scan
            viz_grid (ndarray): High-resolution occupancy grid (values: -1 unknown, 0 free, 100 wall)
            grid_bounds (tuple): (min_x, max_x, min_y, max_y)
            viz_res_x, viz_res_y (int): Visualization grid dimensions
            
        Returns:
            tuple: (corrected_x, corrected_y, corrected_yaw)
        """
        ox, oy, oyaw = raw_odom_pose
        if agent not in self.corrections:
            self.corrections[agent] = np.zeros(3, dtype=np.float32)
            
        cur_dx, cur_dy, cur_dyaw = self.corrections[agent]
        init_x = ox + cur_dx
        init_y = oy + cur_dy
        init_yaw = oyaw + cur_dyaw
        
        # If OpenCV is not available, or scan_msg is None, return smoothed odometry
        if not HAS_CV2 or scan_msg is None or viz_grid is None:
            return (init_x, init_y, init_yaw)
            
        # Check if enough confirmed static obstacle cells exist in the map
        obs_mask = (viz_grid == 100)
        if np.sum(obs_mask) < 20:
            return (init_x, init_y, init_yaw)
            
        # Compute distance transform field from obstacles in C++
        free_mask = (~obs_mask).astype(np.uint8)
        dist_field = cv2.distanceTransform(free_mask, cv2.DIST_L2, 3)
        
        # Extract valid obstacle return beams
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        step = 6  # 60 beams across 360 degrees
        indices = np.arange(0, len(ranges), step)
        r = ranges[indices]
        valid = ~np.isnan(r) & ~np.isinf(r) & (r >= 0.20) & (r < scan_msg.range_max - 0.20)
        
        if np.sum(valid) < 8:
            return (init_x, init_y, init_yaw)
            
        r_valid = r[valid]
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        beam_angles = init_yaw + angle_min + indices[valid] * angle_inc
        
        min_x, max_x, min_y, max_y = grid_bounds
        sx = viz_res_x / (max_x - min_x)
        sy = viz_res_y / (max_y - min_y)
        
        cand_dx = np.linspace(-self.search_dist, self.search_dist, 7)
        cand_dy = np.linspace(-self.search_dist, self.search_dist, 7)
        cand_dyaw = np.linspace(-self.search_yaw, self.search_yaw, 5)
        
        best_delta = (0.0, 0.0, 0.0)
        best_cost = float('inf')
        
        for dyaw in cand_dyaw:
            cos_a = np.cos(beam_angles + dyaw)
            sin_a = np.sin(beam_angles + dyaw)
            px = r_valid * cos_a
            py = r_valid * sin_a
            
            for dx in cand_dx:
                cols = np.clip((init_x + dx + px - min_x) * sx, 0, viz_res_x - 1).astype(np.int32)
                for dy in cand_dy:
                    rows = np.clip((init_y + dy + py - min_y) * sy, 0, viz_res_y - 1).astype(np.int32)
                    cost = np.sum(dist_field[rows, cols]**2)
                    if cost < best_cost:
                        best_cost = cost
                        best_delta = (dx, dy, dyaw)
                        
        # Apply EMA smoothing to the correction
        new_dx = cur_dx + self.alpha * best_delta[0]
        new_dy = cur_dy + self.alpha * best_delta[1]
        new_dyaw = cur_dyaw + self.alpha * best_delta[2]
        
        # Guard against unbounded divergence
        new_dx = float(np.clip(new_dx, -self.max_correction_m, self.max_correction_m))
        new_dy = float(np.clip(new_dy, -self.max_correction_m, self.max_correction_m))
        
        self.corrections[agent] = np.array([new_dx, new_dy, new_dyaw], dtype=np.float32)
        
        final_x = ox + new_dx
        final_y = oy + new_dy
        final_yaw = math.atan2(math.sin(oyaw + new_dyaw), math.cos(oyaw + new_dyaw))
        
        return (final_x, final_y, final_yaw)
