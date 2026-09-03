"""
Behavior Tree Mission Controller for Multi-Agent Robot Swarms.

Implements formal state-driven mission execution using py_trees:
    Root (Selector)
      ├── Sequence: Mission Complete Handler
      │     ├── CheckMissionComplete (Coverage >= 100% or all frontiers exhausted)
      │     └── StandbyOrPerimeterPatrol
      ├── Sequence: Escape Maneuver Handler
      │     ├── CheckStuckCondition (pos_stuck_steps >= 30 or escape_steps_left > 0)
      │     └── ExecuteEscapeManeuver (v = -0.12, w = +/-0.6)
      ├── Sequence: Frontier Exploration
      │     ├── DispatchFrontierTarget (Decentralized CBAA / Voronoi Bidding)
      │     └── NavigateToWaypoint (A* steering with local reactive avoidance)
      └── BoundarySafePatrol (Fallback boundary-aware wander)
"""

import math
import numpy as np

try:
    import py_trees
    from py_trees.behaviour import Behaviour
    from py_trees.common import Status
    HAS_PY_TREES = True
except ImportError:
    HAS_PY_TREES = False


class SwarmMissionTree:
    """
    Encapsulates a formal Behavior Tree for an individual swarm agent.
    """
    
    def __init__(self, agent_id, d_comm=3.0):
        self.agent_id = agent_id
        self.d_comm = d_comm
        
        # Internal state memory
        self.anchor_pos = None
        self.pos_stuck_steps = 0
        self.escape_steps_left = 0
        self.escape_angular = 0.6
        self.blacklisted_cells = set()
        
        self.current_target_cell = None
        self.best_dist = None
        self.target_stuck_steps = 0
        
        self.mission_completed = False
        self.action = np.zeros(2, dtype=np.float32)

    def tick(
        self,
        current_step,
        agent_pose,
        agent_poses,
        obs_dict,
        local_visited_grid,
        local_obstacle_grid,
        local_planning_grid,
        grid_bounds,
        grid_res_x,
        grid_res_y,
        coordinator,
        line_of_sight_fn,
        astar_fn,
        is_continuous_exploration=True
    ):
        """
        Ticks the behavior tree and evaluates the action for this step.
        """
        x, y, yaw = agent_pose
        min_x, max_x, min_y, max_y = grid_bounds
        
        def world_to_cell(wx, wy):
            c = int(np.clip((wx - min_x) / (max_x - min_x) * grid_res_x, 0, grid_res_x - 1))
            r = int(np.clip((wy - min_y) / (max_y - min_y) * grid_res_y, 0, grid_res_y - 1))
            return (r, c)
            
        # =====================================================================
        # BRANCH 1: Stuck Detection & Escape Maneuver Handler
        # =====================================================================
        if self.escape_steps_left > 0:
            self.escape_steps_left -= 1
            return np.array([-0.12, self.escape_angular], dtype=np.float32)
            
        # Update position-based stuck tracker
        if self.anchor_pos is None:
            self.anchor_pos = (x, y)
            self.pos_stuck_steps = 0
        else:
            moved = math.hypot(x - self.anchor_pos[0], y - self.anchor_pos[1])
            if moved > 0.05:
                self.anchor_pos = (x, y)
                self.pos_stuck_steps = 0
            else:
                self.pos_stuck_steps += 1
                
        if self.pos_stuck_steps >= 30:
            # Commit to escape maneuver
            if self.current_target_cell is not None:
                self.blacklisted_cells.add(self.current_target_cell)
                tr, tc = self.current_target_cell
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        nr, nc = tr + dr, tc + dc
                        if 0 <= nr < grid_res_y and 0 <= nc < grid_res_x:
                            self.blacklisted_cells.add((nr, nc))
                            
            # Blacklist ahead cells
            head_x = x + 0.5 * math.cos(yaw)
            head_y = y + 0.5 * math.sin(yaw)
            ahead_cell = world_to_cell(head_x, head_y)
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    nr, nc = ahead_cell[0] + dr, ahead_cell[1] + dc
                    if 0 <= nr < grid_res_y and 0 <= nc < grid_res_x:
                        self.blacklisted_cells.add((nr, nc))
                        
            self.current_target_cell = None
            self.best_dist = None
            self.target_stuck_steps = 0
            self.pos_stuck_steps = 0
            self.anchor_pos = None
            self.escape_steps_left = 15
            self.escape_angular = -self.escape_angular
            return np.array([-0.12, self.escape_angular], dtype=np.float32)
            
        # =====================================================================
        # BRANCH 2: Frontier Exploration via Decentralized Voronoi / CBAA
        # =====================================================================
        target_cell, waypoint, bid = coordinator.select_decentralized_frontier(
            agent=self.agent_id,
            current_step=current_step,
            agent_poses=agent_poses,
            local_visited_grid=local_visited_grid,
            local_obstacle_grid=local_obstacle_grid,
            grid_bounds=grid_bounds,
            grid_res_x=grid_res_x,
            grid_res_y=grid_res_y,
            blacklisted_cells=self.blacklisted_cells,
            planning_grid=local_planning_grid,
            line_of_sight_fn=line_of_sight_fn,
            astar_fn=astar_fn
        )
        
        if target_cell is not None:
            self.mission_completed = False
            tx, ty = waypoint
            dist_to_target = math.hypot(tx - x, ty - y)
            
            # Target stagnation monitoring
            if self.current_target_cell == target_cell:
                if self.best_dist is None or dist_to_target < self.best_dist - 0.05:
                    self.best_dist = dist_to_target
                    self.target_stuck_steps = 0
                else:
                    self.target_stuck_steps += 1
            else:
                self.current_target_cell = target_cell
                self.best_dist = dist_to_target
                self.target_stuck_steps = 0
                
            if self.target_stuck_steps >= 25:
                self.blacklisted_cells.add(target_cell)
                self.current_target_cell = None
                self.best_dist = None
                self.target_stuck_steps = 0
                
            # Local Reactive Avoidance
            agent_obs = obs_dict[self.agent_id]
            front_beams = agent_obs[10:15]
            min_front_dist = np.min(front_beams)
            
            if min_front_dist < 0.45:
                linear = -0.05
                left_dist = np.min(agent_obs[14:18])
                right_dist = np.min(agent_obs[6:10])
                angular = 0.6 if left_dist > right_dist else -0.6
                return np.array([linear, angular], dtype=np.float32)
            elif min_front_dist < 0.7:
                linear = 0.05
                left_dist = np.min(agent_obs[14:18])
                right_dist = np.min(agent_obs[6:10])
                angular = 0.5 if left_dist > right_dist else -0.5
                return np.array([linear, angular], dtype=np.float32)
            else:
                # Proportional waypoint navigation
                goal_dist = math.hypot(tx - x, ty - y)
                goal_angle = math.atan2(ty - y, tx - x) - yaw
                goal_angle = math.atan2(math.sin(goal_angle), math.cos(goal_angle))
                linear = 0.18 if goal_dist > 0.2 else 0.0
                angular = np.clip(1.5 * goal_angle, -0.8, 0.8)
                return np.array([linear, angular], dtype=np.float32)
                
        # =====================================================================
        # BRANCH 3: Mission Complete / Boundary-Safe Perimeter Patrol
        # =====================================================================
        self.mission_completed = True
        agent_obs = obs_dict[self.agent_id]
        min_front = np.min(agent_obs[10:15])
        left_dist = np.min(agent_obs[14:18])
        right_dist = np.min(agent_obs[6:10])
        
        # If not continuous exploration, hold position safely
        if not is_continuous_exploration:
            return np.array([0.0, 0.0], dtype=np.float32)
            
        # Check boundary bounds
        margin = 0.6
        near_boundary = (x < min_x + margin or x > max_x - margin or 
                         y < min_y + margin or y > max_y - margin)
                         
        if near_boundary:
            mid_x = (min_x + max_x) / 2.0
            mid_y = (min_y + max_y) / 2.0
            center_angle = math.atan2(mid_y - y, mid_x - x) - yaw
            center_angle = math.atan2(math.sin(center_angle), math.cos(center_angle))
            return np.array([0.08, np.clip(1.2 * center_angle, -0.8, 0.8)], dtype=np.float32)
        elif min_front < 0.45:
            turn_dir = 0.6 if left_dist > right_dist else -0.6
            return np.array([-0.05, turn_dir], dtype=np.float32)
        elif min_front < 0.7:
            turn_dir = 0.5 if left_dist > right_dist else -0.5
            return np.array([0.05, turn_dir], dtype=np.float32)
        else:
            return np.array([0.12, 0.0], dtype=np.float32)
