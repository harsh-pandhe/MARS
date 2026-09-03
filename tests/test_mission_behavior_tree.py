"""
Unit tests for SwarmMissionTree formal state transitions.
"""

import os
import sys
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm')))
from mars_swarm.mission_behavior_tree import SwarmMissionTree
from mars_swarm.decentralized_coordinator import DecentralizedCoordinator


def test_mission_tree_stuck_triggers_escape():
    """Verify that 30 consecutive zero-movement steps trigger escape maneuver."""
    tree = SwarmMissionTree('tb1')
    coordinator = DecentralizedCoordinator()
    
    poses = {'tb1': (0.0, 0.0, 0.0)}
    obs = {'tb1': np.ones(46, dtype=np.float32) * 2.0}
    visited = np.zeros((10, 10), dtype=bool)
    obs_grid = np.zeros((10, 10), dtype=bool)
    bounds = (-5.0, 5.0, -5.0, 5.0)
    
    # Simulate 30 steps with identical pose
    action = None
    for step in range(1, 32):
        action = tree.tick(
            current_step=step,
            agent_pose=(0.0, 0.0, 0.0),
            agent_poses=poses,
            obs_dict=obs,
            local_visited_grid=visited,
            local_obstacle_grid=obs_grid,
            local_planning_grid=obs_grid,
            grid_bounds=bounds,
            grid_res_x=10,
            grid_res_y=10,
            coordinator=coordinator,
            line_of_sight_fn=lambda *args: True,
            astar_fn=lambda *args: None
        )
        
    # After 30 stagnant steps, escape maneuver must trigger (negative linear velocity)
    assert action is not None
    assert action[0] == -0.12, f"Expected escape linear velocity -0.12, got {action[0]}"
    assert abs(action[1]) == 0.6, f"Expected escape angular velocity +/-0.6, got {action[1]}"


def test_mission_tree_all_frontiers_explored_holds_or_patrols():
    """Verify that when local belief is 100% explored, tree transitions to safe boundary patrol."""
    tree = SwarmMissionTree('tb1')
    coordinator = DecentralizedCoordinator()
    
    poses = {'tb1': (0.0, 0.0, 0.0)}
    obs = {'tb1': np.ones(46, dtype=np.float32) * 2.0}
    # Completely visited grid (no unvisited cells)
    visited = np.ones((10, 10), dtype=bool)
    obs_grid = np.zeros((10, 10), dtype=bool)
    bounds = (-5.0, 5.0, -5.0, 5.0)
    
    action = tree.tick(
        current_step=1,
        agent_pose=(0.0, 0.0, 0.0),
        agent_poses=poses,
        obs_dict=obs,
        local_visited_grid=visited,
        local_obstacle_grid=obs_grid,
        local_planning_grid=obs_grid,
        grid_bounds=bounds,
        grid_res_x=10,
        grid_res_y=10,
        coordinator=coordinator,
        line_of_sight_fn=lambda *args: True,
        astar_fn=lambda *args: None,
        is_continuous_exploration=False
    )
    
    assert tree.mission_completed is True
    # Non-continuous mode holds position at completion
    assert np.allclose(action, [0.0, 0.0])
