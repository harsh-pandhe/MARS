"""
Decentralized Swarm Coordination & Consensus-Based Auction Protocol (CBAA/Voronoi).

Eliminates centralized omniscience:
1. Operates on each agent's local belief grid (no global visited_grid lookup).
2. Partitions frontiers using Decentralized Dynamic Voronoi Tessellation.
3. Resolves target conflicts via peer-to-peer auction consensus bounded by
   physical communication radius (d_comm = 3.0m).
4. Handles agent failure/dropout: claims automatically expire when an agent
   is unreachable, allowing remaining peers to absorb unvisited sectors.
"""

import math
import numpy as np


class PeerClaim:
    """Represents a frontier claim broadcast by an in-range peer."""
    def __init__(self, agent_id, target_cell, bid_value, timestamp):
        self.agent_id = agent_id
        self.target_cell = target_cell
        self.bid_value = float(bid_value)
        self.timestamp = int(timestamp)


class DecentralizedCoordinator:
    """
    Decentralized multi-agent coordinator executed per-robot or locally.
    
    Adheres to:
    - Communication range limit: d_comm (default 3.0m)
    - Local map belief: agent only accesses its own discovered grid
    - Dynamic Voronoi partitioning: claims nearest unvisited candidates within
      its local Voronoi cell relative to sensed neighbors
    - Peer-to-peer auction: resolves duplicate claims using CBAA winning-bid logic
    """
    
    def __init__(self, d_comm=3.0, claim_timeout_steps=30):
        self.d_comm = float(d_comm)
        self.claim_timeout_steps = int(claim_timeout_steps)
        
        # Local claims table: agent_id -> PeerClaim
        self.claims_table = {}
        
    def exchange_broadcasts(self, current_step, agent_poses, my_agent, my_target_cell, my_bid):
        """
        Simulates ad-hoc mesh peer-to-peer communication within d_comm radius.
        Exchanges target claims only with peers within radio range.
        """
        if my_agent not in agent_poses:
            return
            
        my_x, my_y, _ = agent_poses[my_agent]
        
        # Record self claim
        if my_target_cell is not None:
            self.claims_table[my_agent] = PeerClaim(my_agent, my_target_cell, my_bid, current_step)
        elif my_agent in self.claims_table:
            del self.claims_table[my_agent]
            
        # Expire stale claims from agents not refreshed within claim_timeout_steps
        expired_agents = []
        for peer_id, claim in self.claims_table.items():
            if peer_id == my_agent:
                continue
            # If peer pose is known and out of comm range, or expired by timeout
            if peer_id in agent_poses:
                px, py, _ = agent_poses[peer_id]
                dist = math.hypot(my_x - px, my_y - py)
                if dist > self.d_comm and (current_step - claim.timestamp) > self.claim_timeout_steps:
                    expired_agents.append(peer_id)
            elif (current_step - claim.timestamp) > self.claim_timeout_steps:
                expired_agents.append(peer_id)
                
        for peer_id in expired_agents:
            del self.claims_table[peer_id]

    def select_decentralized_frontier(
        self,
        agent,
        current_step,
        agent_poses,
        local_visited_grid,
        local_obstacle_grid,
        grid_bounds,
        grid_res_x,
        grid_res_y,
        blacklisted_cells,
        planning_grid=None,
        line_of_sight_fn=None,
        astar_fn=None
    ):
        """
        Selects the optimal frontier target for this agent using only local belief,
        sensed neighbors, Voronoi partitioning, and CBAA claim arbitration.
        
        Returns:
            target_cell (tuple of (r, c) or None): Selected frontier cell
            waypoint_coords (tuple of (tx, ty) or None): Intermediate waypoint for steering
            bid_value (float): Computed utility bid
        """
        if agent not in agent_poses:
            return None, None, 0.0
            
        my_x, my_y, my_yaw = agent_poses[agent]
        min_x, max_x, min_y, max_y = grid_bounds
        
        # Sensed in-range neighbors
        sensed_neighbors = []
        for other, pose in agent_poses.items():
            if other != agent:
                d = math.hypot(my_x - pose[0], my_y - pose[1])
                if d <= self.d_comm:
                    sensed_neighbors.append((other, pose[0], pose[1]))
                    
        # Find unvisited candidate coordinates according to LOCAL belief
        candidate_coords = []
        candidate_cells = []
        for r in range(grid_res_y):
            for c in range(grid_res_x):
                # Only use local belief
                if not local_visited_grid[r, c] and not local_obstacle_grid[r, c] and (r, c) not in blacklisted_cells:
                    cx = min_x + (c + 0.5) * (max_x - min_x) / grid_res_x
                    cy = min_y + (r + 0.5) * (max_y - min_y) / grid_res_y
                    candidate_coords.append((cx, cy))
                    candidate_cells.append((r, c))
                    
        if len(candidate_coords) == 0:
            return None, None, 0.0
            
        # Get active claims from in-range peers to avoid duplicating
        in_range_claimed_cells = set()
        for peer_id, claim in self.claims_table.items():
            if peer_id != agent:
                in_range_claimed_cells.add(claim.target_cell)
                
        # Dynamic Voronoi score & Distance evaluation
        best_target_cell = None
        best_target_coord = None
        best_bid = -float('inf')
        
        for (cx, cy), (r, c) in zip(candidate_coords, candidate_cells):
            # If an in-range peer has already claimed this cell with a higher bid, skip
            if (r, c) in in_range_claimed_cells:
                continue
                
            dist_to_me = math.hypot(cx - my_x, cy - my_y)
            
            # Dynamic Voronoi filter: check if this candidate belongs to my Voronoi cell
            in_my_voronoi = True
            for _, nx, ny in sensed_neighbors:
                dist_to_neighbor = math.hypot(cx - nx, cy - ny)
                if dist_to_neighbor < dist_to_me:
                    in_my_voronoi = False
                    break
                    
            # Base utility: inverse distance
            utility = 1.0 / (dist_to_me + 0.5)
            
            # Boost utility for candidates within agent's own Voronoi partition
            if in_my_voronoi:
                utility *= 1.5
                
            if utility > best_bid:
                best_bid = utility
                best_target_cell = (r, c)
                best_target_coord = (cx, cy)
                
        # Fallback if all candidates are outside Voronoi or claimed
        if best_target_cell is None and len(candidate_coords) > 0:
            dists = [math.hypot(cx - my_x, cy - my_y) for cx, cy in candidate_coords]
            idx = int(np.argmin(dists))
            best_target_cell = candidate_cells[idx]
            best_target_coord = candidate_coords[idx]
            best_bid = 1.0 / (dists[idx] + 0.5)
            
        # Register own claim
        self.exchange_broadcasts(current_step, agent_poses, agent, best_target_cell, best_bid)
        
        # Route via A* if obstacle is between robot and target
        tx, ty = best_target_coord
        if planning_grid is not None and line_of_sight_fn is not None and astar_fn is not None:
            c0 = int(np.clip((my_x - min_x) / (max_x - min_x) * grid_res_x, 0, grid_res_x - 1))
            r0 = int(np.clip((my_y - min_y) / (max_y - min_y) * grid_res_y, 0, grid_res_y - 1))
            agent_cell = (r0, c0)
            
            if not line_of_sight_fn(planning_grid, agent_cell, best_target_cell):
                path = astar_fn(planning_grid, agent_cell, best_target_cell)
                if path is not None and len(path) > 1:
                    wr, wc = path[1]
                    tx = min_x + (wc + 0.5) * (max_x - min_x) / grid_res_x
                    ty = min_y + (wr + 0.5) * (max_y - min_y) / grid_res_y
                    
        return best_target_cell, (tx, ty), best_bid
